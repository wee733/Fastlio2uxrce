#!/usr/bin/env python3
"""Check installed guard and exclusive ownership; increment original PX4 counter, then exec."""
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

from locking import acquire
from render_params import render
from reset_counter import next_counter


def ros(*args):
    p = subprocess.run(['ros2', *args], text=True, capture_output=True, timeout=15, check=False,
                       env=dict(os.environ, LC_ALL='C'))
    if p.returncode:
        raise RuntimeError(f"ros2 {' '.join(args)} failed: {p.stderr.strip() or p.stdout.strip()}")
    return p.stdout.strip()


def require_guard(config_path):
    share = Path(ros('pkg', 'prefix', '--share', 'fastlio_px4_bridge'))
    prefix = Path(ros('pkg', 'prefix', 'fastlio_px4_bridge'))
    launch = (share / 'launch/fc_imu_lio.launch.py').read_text(encoding='utf-8')
    if not all(token in launch for token in ('OnProcessExit', 'Shutdown', 'output_topic', 'reset_counter',
                                             'fastlio_params', 'bridge_params', 'input_world_is_level')):
        raise RuntimeError('Installed launch lacks the expected output argument or process-exit shutdown')
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise RuntimeError('Installed bridge configuration is invalid')
    node = config.get('fastlio_to_px4_visual_odometry')
    if not isinstance(node, dict) or not isinstance(node.get('ros__parameters'), dict):
        raise RuntimeError('Installed bridge parameters are invalid')
    params = node['ros__parameters']
    if params.get('require_fresh_imu') is not True:
        raise RuntimeError('Installed bridge configuration does not require fresh IMU')
    max_age = params.get('max_sample_age')
    if isinstance(max_age, bool) or not isinstance(max_age, (int, float)) or not math.isfinite(max_age) or not 0 < max_age <= 0.3:
        raise RuntimeError('Installed bridge max_sample_age must be finite and within (0, 0.3] seconds')
    binary = (prefix / 'lib/fastlio_px4_bridge/fastlio_to_px4_visual_odometry').read_bytes()
    markers = (b'status_topic', b'require_fresh_imu', b'IMU interrupted', b'Odometry interrupted',
               b'Input stream stale; restart LIO on the ground',
               b'No stable IMU/odometry before startup timeout; check timesync/PTP')
    if not all(token in binary for token in markers):
        raise RuntimeError('Installed bridge binary lacks expected health-gate markers')


def require_calibration(config_path):
    """Check the exact calibration file used by the installed launch.

    The calibrated flag and a proper rotation matrix cannot establish that the
    physical mount still matches the file; they only reject invalid metadata.
    """
    data = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise RuntimeError(f'Invalid FAST-LIO calibration file: {config_path}')
    node = data.get('/**')
    if not isinstance(node, dict) or not isinstance(node.get('ros__parameters'), dict):
        raise RuntimeError('FAST-LIO calibration parameters are missing')
    parameters = node['ros__parameters']
    mapping = parameters.get('mapping')
    common = parameters.get('common')
    if not isinstance(mapping, dict) or not isinstance(common, dict):
        raise RuntimeError('FAST-LIO calibration or input settings are missing')
    if mapping.get('require_calibrated_extrinsic') is not True or mapping.get('extrinsic_calibrated') is not True:
        raise RuntimeError('FAST-LIO LiDAR-to-IMU calibration is not marked as required and calibrated')
    translation = mapping.get('extrinsic_T')
    rotation = mapping.get('extrinsic_R')
    def valid_vector(value, size):
        return (isinstance(value, list) and len(value) == size and
                all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in value))
    if not valid_vector(translation, 3) or not valid_vector(rotation, 9):
        raise RuntimeError('FAST-LIO LiDAR-to-IMU translation or rotation is invalid')
    rows = [rotation[i:i + 3] for i in (0, 3, 6)]
    for i in range(3):
        for j in range(3):
            dot = sum(rows[i][k] * rows[j][k] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 0.001:
                raise RuntimeError('FAST-LIO calibration rotation is not orthonormal')
    a, b, c = rows
    determinant = (a[0] * (b[1]*c[2] - b[2]*c[1]) - a[1] * (b[0]*c[2] - b[2]*c[0]) +
                   a[2] * (b[0]*c[1] - b[1]*c[0]))
    if abs(determinant - 1.0) > 0.001:
        raise RuntimeError('FAST-LIO calibration rotation is not right-handed')


def require_no_existing_publisher(output_topic, imu_topic):
    # Use the same DDS profile as the three nodes, independent of the ros2cli daemon.
    import rclpy
    rclpy.init()
    try:
        node = rclpy.create_node('fastlio_publisher_preflight', enable_rosout=False,
                                 start_parameter_services=False)
        try:
            for attempt in range(21):
                for topic in (output_topic, '/Odometry', imu_topic):
                    if node.count_publishers(topic):
                        raise RuntimeError(f'{topic} already has a visible publisher')
                if attempt < 20:
                    time.sleep(0.1)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def main(counter_directory):
    held = acquire(['aivuda-lio.lock', 'aivuda-external-odometry.lock'])
    runtime = Path(os.environ['AIVUDA_APP_RUNTIME_DATA_PATH'])
    lio_share = Path(ros('pkg', 'prefix', '--share', 'fast_lio'))
    bridge_share = Path(ros('pkg', 'prefix', '--share', 'fastlio_px4_bridge'))
    lio_path, bridge_path = render(Path(os.environ['AIVUDA_APP_CONFIG_PATH']), runtime,
                                   lio_share / 'config/mid360s_fc_imu.yaml',
                                   bridge_share / 'config/bridge.yaml')
    require_guard(bridge_path)
    require_calibration(lio_path)
    lio = yaml.safe_load(lio_path.read_text())['/**']['ros__parameters']
    bridge = yaml.safe_load(bridge_path.read_text())['fastlio_to_px4_visual_odometry']['ros__parameters']
    imu_topic = lio['common']['imu_topic']
    output_topic = bridge['output_topic']
    require_no_existing_publisher(output_topic, imu_topic)
    try:
        counter = next_counter(counter_directory)
    except PermissionError as exc:
        raise RuntimeError(f'Cannot write reset counter in {counter_directory}; '
                           'configure a writable counter directory shared with mocap') from exc
    print(f'Starting PX4 visual odometry via uXRCE; reset_counter={counter}.', flush=True)
    # Both lock descriptors are explicitly inheritable and held by ros2 launch until exit.
    os.execvp('ros2', ['ros2', 'launch', 'fastlio_px4_bridge', 'fc_imu_lio.launch.py',
                       f'fastlio_params:={lio_path}', f'bridge_params:={bridge_path}',
                       f'imu_topic:={imu_topic}', f'output_topic:={output_topic}',
                       f"input_world_is_level:={str(lio['mapping']['gravity_align']).lower()}",
                       f'reset_counter:={counter}'])


if __name__ == '__main__':
    try:
        if len(sys.argv) != 2: raise ValueError('Provide the absolute counter directory')
        main(sys.argv[1])
    except (OSError, RuntimeError, ValueError, yaml.YAMLError, subprocess.TimeoutExpired) as error:
        print(f'FAST-LIO PX4 start refused: {error}', file=sys.stderr)
        sys.exit(1)
