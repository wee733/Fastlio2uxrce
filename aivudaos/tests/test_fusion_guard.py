import os
from pathlib import Path
import sys
import tempfile
import subprocess
import time
import unittest
import yaml
from concurrent.futures import ThreadPoolExecutor
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import render_params
import fusion_guard
import locking

class FusionGuardTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
    def tearDown(self): self.temp.cleanup()

    def test_counter_is_serialized_and_persisted(self):
        (self.root / 'reset_counter').write_text('1\n')
        with ThreadPoolExecutor(max_workers=8) as pool:
            values = list(pool.map(lambda _: fusion_guard.next_counter(self.root), range(16)))
        self.assertEqual(set(values), set(range(2,18)))
        self.assertEqual((self.root/'reset_counter').read_text(), '17\n')
    def test_missing_counter_initializes_like_original_service(self):
        new_directory = self.root / 'new-state'
        self.assertEqual(fusion_guard.next_counter(new_directory), 2)
        self.assertEqual((new_directory / 'reset_counter').read_text(), '2\n')
        self.assertEqual(fusion_guard.next_counter(new_directory), 3)
    def test_invalid_counter_is_never_replaced(self):
        (self.root / 'reset_counter').write_text('broken')
        with self.assertRaises(ValueError): fusion_guard.next_counter(self.root)
        self.assertEqual((self.root / 'reset_counter').read_text(), 'broken')
        (self.root / 'reset_counter').write_text('256')
        with self.assertRaises(ValueError): fusion_guard.next_counter(self.root)
        self.assertEqual((self.root / 'reset_counter').read_text(), '256')
    def test_symlink_counter_is_never_followed(self):
        target=self.root / 'external'; target.write_text('10')
        (self.root / 'reset_counter').symlink_to(target)
        with self.assertRaises(ValueError): fusion_guard.next_counter(self.root)
        self.assertEqual(target.read_text(), '10')
    def test_uint8_wrap(self):
        (self.root / 'reset_counter').write_text('255\n')
        self.assertEqual(fusion_guard.next_counter(self.root), 0)
    def test_gate_config_and_executable_are_required(self):
        share = self.root/'share'; prefix = self.root/'prefix'
        (share/'launch').mkdir(parents=True); (share/'config').mkdir()
        (prefix/'lib/fastlio_px4_bridge').mkdir(parents=True)
        (share/'launch/fc_imu_lio.launch.py').write_text("OnProcessExit Shutdown output_topic reset_counter fastlio_params bridge_params input_world_is_level")
        config=share/'config/bridge.yaml'
        config.write_text('fastlio_to_px4_visual_odometry:\n  ros__parameters:\n    require_fresh_imu: true\n    max_sample_age: 0.3\n')
        binary=prefix/'lib/fastlio_px4_bridge/fastlio_to_px4_visual_odometry'
        binary.write_bytes(b'status_topic require_fresh_imu IMU interrupted Odometry interrupted Input stream stale; restart LIO on the ground No stable IMU/odometry before startup timeout; check timesync/PTP')
        with mock.patch.object(fusion_guard,'ros',side_effect=[str(share),str(prefix)]): fusion_guard.require_guard(config)
        config.write_text('fastlio_to_px4_visual_odometry:\n  ros__parameters:\n    require_fresh_imu: false\n')
        with mock.patch.object(fusion_guard,'ros',side_effect=[str(share),str(prefix)]):
            with self.assertRaises(RuntimeError): fusion_guard.require_guard(config)
        config.write_text('fastlio_to_px4_visual_odometry:\n  ros__parameters:\n    require_fresh_imu: true\n    max_sample_age: 3.0\n')
        with mock.patch.object(fusion_guard,'ros',side_effect=[str(share),str(prefix)]):
            with self.assertRaisesRegex(RuntimeError, 'max_sample_age'): fusion_guard.require_guard(config)
    def test_calibration_requires_flags_and_right_handed_rotation(self):
        share = self.root/'share'
        (share/'config').mkdir(parents=True)
        file=share/'config/mid360s_fc_imu.yaml'
        parameters={'common': {'lid_topic': '/livox/lidar', 'imu_topic': '/lio/imu'},
                    'mapping': {'require_calibrated_extrinsic': True, 'extrinsic_calibrated': True,
                                'gravity_align': True, 'extrinsic_T': [0.1, 0.2, 0.3],
                                'extrinsic_R': [1,0,0,0,1,0,0,0,1]}}
        def check():
            file.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            with mock.patch.object(fusion_guard, 'ros', return_value=str(share)):
                fusion_guard.require_calibration(file)
        check()
        parameters['mapping']['extrinsic_calibrated']=False
        with self.assertRaisesRegex(RuntimeError, 'not marked'): check()
        parameters['mapping']['extrinsic_calibrated']=True
        parameters['mapping']['extrinsic_R']=[-1,0,0,0,1,0,0,0,1]
        with self.assertRaisesRegex(RuntimeError, 'right-handed'): check()
        parameters['mapping']['extrinsic_R']=[1,0,0,0,1,0,0,0,float('nan')]
        with self.assertRaisesRegex(RuntimeError, 'invalid'): check()
    def guard_environment(self):
        config = self.root / 'active.yaml'
        config.write_text('px4_bridge:\n  output_topic: /custom/vision\ncommon:\n  imu_topic: /custom/imu\nmapping:\n  filter_size_surf: 1\n  gravity_align: false\n')
        return mock.patch.dict(os.environ, AIVUDA_APP_CONFIG_PATH=str(config),
                               AIVUDA_APP_RUNTIME_DATA_PATH=str(self.root))

    @staticmethod
    def pkg_prefix(*args):
        prefix = ROOT/'ros_ws/install'/args[-1]
        return str(prefix/'share'/args[-1] if '--share' in args else prefix)

    def test_main_launches_configured_topics_after_checks(self):
        with self.guard_environment(), mock.patch.object(fusion_guard, 'ros', side_effect=self.pkg_prefix), \
             mock.patch.object(fusion_guard, 'acquire'), \
             mock.patch.object(fusion_guard, 'require_no_existing_publisher') as preflight, \
             mock.patch.object(fusion_guard, 'next_counter', return_value=42), \
             mock.patch.object(fusion_guard.os, 'execvp') as execute:
            fusion_guard.main(str(self.root))
        preflight.assert_called_once_with('/custom/vision','/custom/imu')
        arguments = execute.call_args.args[1]
        self.assertIn('output_topic:=/custom/vision', arguments)
        self.assertIn('imu_topic:=/custom/imu', arguments)
        self.assertIn('input_world_is_level:=false', arguments)
        self.assertIn('reset_counter:=42', arguments)
        lio_path = self.root/'fastlio_params.yaml'
        self.assertIn(f'fastlio_params:={lio_path}', arguments)
        self.assertIn(f'bridge_params:={self.root/"bridge_params.yaml"}', arguments)
        lio = render_params.read_yaml(lio_path)['/**']['ros__parameters']
        self.assertIs(type(lio['filter_size_surf']),float)
        self.assertEqual(lio['filter_size_surf'],1.0)
        self.assertNotIn('filter_size_surf',lio['mapping'])
        self.assertEqual(lio['point_filter_num'],2)
        self.assertTrue(lio['publish']['dense_publish_en'])
        self.assertEqual(len(lio['mapping']['extrinsic_R']),9)
        bridge=render_params.read_yaml(self.root/'bridge_params.yaml')
        self.assertEqual(bridge['fastlio_to_px4_visual_odometry']['ros__parameters']['status_topic'],
                         '/fast_lio/bridge_status')

    def test_main_reports_unwritable_counter_and_never_launches(self):
        with self.guard_environment(), mock.patch.object(fusion_guard, 'ros', side_effect=self.pkg_prefix), \
             mock.patch.object(fusion_guard, 'acquire'), \
             mock.patch.object(fusion_guard, 'require_no_existing_publisher'), \
             mock.patch.object(fusion_guard, 'next_counter', side_effect=PermissionError('denied')), \
             mock.patch.object(fusion_guard.os, 'execvp') as execute:
            with self.assertRaisesRegex(RuntimeError, 'writable counter directory'):
                fusion_guard.main(str(self.root))
            execute.assert_not_called()

    def test_publisher_preflight(self):
        rclpy = mock.Mock()
        node = rclpy.create_node.return_value
        with mock.patch.dict(sys.modules, rclpy=rclpy), mock.patch.object(fusion_guard.time, 'sleep'):
            for conflict in ('/custom/vision', '/Odometry', '/custom/imu'):
                node.count_publishers.side_effect = lambda topic: int(topic == conflict)
                with self.assertRaisesRegex(RuntimeError, conflict):
                    fusion_guard.require_no_existing_publisher('/custom/vision', '/custom/imu')
            node.count_publishers.side_effect = None
            node.count_publishers.return_value = 0
            fusion_guard.require_no_existing_publisher('/custom/vision', '/custom/imu')
        self.assertEqual(node.destroy_node.call_count, 4)
        self.assertEqual(rclpy.shutdown.call_count, 4)

    def test_lock_survives_exec_and_releases_on_exit(self):
        code = """import os,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
import locking
locking.Path=lambda _: Path(sys.argv[2])
held=locking.acquire(['aivuda-external-odometry.lock'])
Path(sys.argv[2],'ready').write_text('ok')
os.execv('/bin/sleep',['sleep','10'])
"""
        process=subprocess.Popen([sys.executable,'-c',code,str(ROOT/'scripts'),str(self.root)])
        try:
            ready=self.root/'ready'
            deadline=time.monotonic()+3
            while not ready.exists() and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertTrue(ready.exists())
            with mock.patch.object(locking,'Path',return_value=self.root):
                with self.assertRaises(BlockingIOError): locking.acquire(['aivuda-external-odometry.lock'])
            process.terminate(); process.wait(timeout=3)
            with mock.patch.object(locking,'Path',return_value=self.root):
                held=locking.acquire(['aivuda-external-odometry.lock']); os.close(held[0])
        finally:
            if process.poll() is None:
                process.kill(); process.wait(timeout=3)
    def test_inherited_lock_excludes_competing_process(self):
        # Both opens use the same inode; no file is unlinked while it may be locked.
        lockroot=self.root / f'fastlio-px4-{os.getuid()}'
        with mock.patch.object(locking,'Path',return_value=self.root):
            held=locking.acquire(['aivuda-external-odometry.lock'])
            self.assertTrue(os.get_inheritable(held[0]))
            with self.assertRaises(BlockingIOError): locking.acquire(['aivuda-external-odometry.lock'])
            os.close(held[0]); held=locking.acquire(['aivuda-external-odometry.lock']); os.close(held[0])
        self.assertTrue(lockroot.exists())

if __name__=='__main__': unittest.main()
