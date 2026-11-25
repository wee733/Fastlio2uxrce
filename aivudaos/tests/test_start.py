import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER = '''aivuda_yaml_has() { [[ "$1" == "fastlio.ros_domain_id" || "$1" == "ros.domain_id" || ( "$1" == "fastlio.transport_mode" && -v TEST_TRANSPORT_MODE ) ]]; }
aivuda_yaml_get() {
 case "$1" in
 ros.setup_script) printf %s "$TEST_ROS_SETUP" ;;
 fastlio.ros_domain_id|ros.domain_id) printf %s "${TEST_DOMAIN:-0}" ;;
 workspaces.px4_msgs_setup|workspaces.livox_setup) printf %s "" ;;
 workspaces.fastlio_setup) printf %s "$TEST_LIO_SETUP" ;;
 fastlio.transport_mode) printf %s "$TEST_TRANSPORT_MODE" ;;
 state.counter_directory) printf %s "$TEST_COUNTER_DIR" ;;
 *) return 1 ;;
 esac
}
'''

class FusionStartTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.base=Path(self.tmp.name)
        helper=self.base/'helper.sh'; helper.write_text(HELPER)
        setup=self.base/'setup.bash'; setup.write_text('true\n')
        lio=self.base/'lio.bash'; lio.write_text('true\n')
        ros=self.base/'ros2'; ros.write_text('#!/bin/bash\nexit 0\n'); ros.chmod(0o755)
        python=self.base/'python3'; python.write_text('#!/bin/bash\nprintf "%s\\n" "$*" > "$TEST_LOG"\n'); python.chmod(0o755)
        self.log=self.base/'calls.log'
        self.env=os.environ.copy(); self.env.update(PATH=str(self.base)+':'+self.env['PATH'],
          AIVUDA_APP_RUNTIME_DATA_PATH=str(self.base), AIVUDA_APP_HELPERS_ENTRY_PATH=str(helper), AIVUDA_APP_CONFIG_PATH=str(self.base/'conf.yaml'),
          TEST_ROS_SETUP=str(setup),TEST_LIO_SETUP=str(lio),TEST_COUNTER_DIR=str(self.base),
          TEST_LOG=str(self.log), TEST_TRANSPORT_MODE='inherit')
    def tearDown(self): self.tmp.cleanup()
    def start(self): return subprocess.run(['bash',str(ROOT/'start.sh')],env=self.env,text=True,capture_output=True)
    def test_one_click_launches_guard_with_counter_directory(self):
        p=self.start(); self.assertEqual(p.returncode,0,p.stderr)
        self.assertIn('fusion_guard.py '+str(self.base),self.log.read_text())
    def test_default_counter_is_inside_runtime(self):
        self.env['TEST_COUNTER_DIR']=''
        result=self.start()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(self.base/'reset_counter'), self.log.read_text())

    def test_transport_env_for_legacy_and_explicit_config(self):
        python = self.base / 'python3'
        python.write_text('#!/bin/bash\nenv > "$TEST_LOG"\n')
        self.env.update(RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', ROS_LOCALHOST_ONLY='1',
                        ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET', TEST_DOMAIN='7')
        for mode in ('inherit', 'localhost_udp', None):
            with self.subTest(mode=mode):
                if mode is None:
                    self.env.pop('TEST_TRANSPORT_MODE')
                else:
                    self.env['TEST_TRANSPORT_MODE'] = mode
                result = self.start()
                if mode != 'inherit' and int(Path('/proc/sys/net/core/rmem_max').read_text()) < 4194304:
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('net.core.rmem_max >= 4194304', result.stderr)
                    continue
                self.assertEqual(result.returncode, 0, result.stderr)
                env = dict(line.split('=', 1) for line in self.log.read_text().splitlines())
                self.assertEqual(env['ROS_DOMAIN_ID'], '7')
                if mode == 'inherit':
                    self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_cyclonedds_cpp')
                    self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'SUBNET')
                    self.assertEqual(env['ROS_LOCALHOST_ONLY'], '1')
                else:
                    self.assertEqual(env['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
                    self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'SYSTEM_DEFAULT')
                    self.assertNotIn('ROS_LOCALHOST_ONLY', env)
                    profile = Path(env['FASTRTPS_DEFAULT_PROFILES_FILE'])
                    self.assertTrue(profile.is_file())
                    self.assertEqual(env['FASTDDS_DEFAULT_PROFILES_FILE'], str(profile))

    def test_rejects_unknown_transport(self):
        self.env['TEST_TRANSPORT_MODE'] = 'loopback'
        result = self.start()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Unknown fastlio.transport_mode', result.stderr)

    def test_missing_ros_setup_is_rejected(self):
        self.env['TEST_LIO_SETUP']=str(self.base/'missing')
        self.assertNotEqual(self.start().returncode,0)
        self.assertFalse(self.log.exists())
    def test_rejects_bad_domain(self):
        self.env['TEST_DOMAIN']='233'; self.assertNotEqual(self.start().returncode,0)
        self.assertFalse(self.log.exists())

if __name__=='__main__': unittest.main()
