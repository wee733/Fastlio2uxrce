"""Exercise the installed C++ bridges on an isolated DDS domain (no flight controller)."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleOdometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


@pytest.fixture(scope='module')
def system(request):
    context = rclpy.context.Context()
    context.init(domain_id=187)
    node = rclpy.create_node('bridge_behavior_test', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    binary = Path(get_package_prefix('fastlio_px4_bridge')) / 'lib/fastlio_px4_bridge'
    env = dict(os.environ, ROS_DOMAIN_ID='187', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    commands = [
        ['fastlio_to_px4_visual_odometry', '-p', 'input_world_is_level:=true',
         '-p', 'input_odom_topic:=/test/odom', '-p', 'output_topic:=/test/vision',
         '-p', 'reset_counter:=7'],
        ['fastlio_to_px4_visual_odometry', '-p', 'input_odom_topic:=/test/odom',
         '-p', 'output_topic:=/test/blocked'],
        ['px4_imu_to_ros', '-p', 'input_topic:=/test/combined',
         '-p', 'output_topic:=/test/imu'],
    ]
    processes = [subprocess.Popen([str(binary / cmd[0]), '--ros-args', *cmd[1:]], env=env)
                 for cmd in commands]
    received = {'vision': [], 'imu': [], 'blocked': []}
    subs = [node.create_subscription(cls, '/test/' + key, received[key].append,
                                     qos_profile_sensor_data)
            for key, cls in [('vision', VehicleOdometry), ('imu', Imu),
                             ('blocked', VehicleOdometry)]]
    pubs = {'odom': node.create_publisher(Odometry, '/test/odom', 10),
            'combined': node.create_publisher(SensorCombined, '/test/combined',
                                              qos_profile_sensor_data)}

    def spin(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def cleanup():
        for process in processes:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for sub in subs:
            node.destroy_subscription(sub)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        assert all(process.returncode == 0 for process in processes)

    request.addfinalizer(cleanup)
    spin(3.0)
    assert pubs['odom'].get_subscription_count() == 2
    assert pubs['combined'].get_subscription_count() == 1
    return node, pubs, received, spin


def odometry(node, delay=0.05):
    msg = Odometry()
    stamp = node.get_clock().now().nanoseconds - int(delay * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.header.frame_id = 'camera_init'
    msg.child_frame_id = 'body'
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 3.0
    msg.pose.pose.orientation.z = math.sqrt(0.5)
    msg.pose.pose.orientation.w = math.sqrt(0.5)
    return msg


def imu(node):
    msg = SensorCombined()
    msg.timestamp = node.get_clock().now().nanoseconds // 1000 - 2000
    msg.gyro_rad = [0.1, 0.2, 0.3]
    msg.accelerometer_m_s2 = [1.0, 2.0, -9.81]
    msg.gyro_integral_dt = 5000
    msg.accelerometer_integral_dt = 5000
    return msg


def test_pose_frames_orientation_and_time(system):
    node, pubs, received, spin = system
    msg = odometry(node)
    pubs['odom'].publish(msg)
    spin(0.3)
    assert len(received['vision']) == 1
    out = received['vision'][-1]
    assert list(out.position) == [1.0, -2.0, -3.0]
    assert list(out.q) == pytest.approx([math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)])
    assert out.pose_frame == VehicleOdometry.POSE_FRAME_FRD
    assert out.timestamp_sample == (msg.header.stamp.sec * 10**6 +
                                    msg.header.stamp.nanosec // 1000)
    assert 30000 < out.timestamp - out.timestamp_sample < 200000
    assert math.isnan(out.velocity[0])  # No finite-difference velocity by default.
    assert out.reset_counter == 7
    assert out.quality == 0  # Unknown; never fabricated confidence=100.
    assert not received['blocked']


@pytest.mark.parametrize('case', ['stale', 'future', 'zero_stamp', 'zero_quat',
                                  'nan_quat', 'nan_position', 'wrong_frame'])
def test_invalid_pose_is_rejected(system, case):
    node, pubs, received, spin = system
    msg = odometry(node)
    if case in ('stale', 'future'):
        msg = odometry(node, 1.0 if case == 'stale' else -0.5)
    elif case == 'zero_stamp':
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
    elif case == 'zero_quat':
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 0.0
    elif case == 'nan_quat':
        msg.pose.pose.orientation.x = math.nan
    elif case == 'nan_position':
        msg.pose.pose.position.x = math.nan
    else:
        msg.header.frame_id = 'map'
    before = len(received['vision'])
    pubs['odom'].publish(msg)
    spin(0.15)
    assert len(received['vision']) == before


def test_imu_units_axes_qos_and_time(system):
    node, pubs, received, spin = system
    msg = imu(node)
    pubs['combined'].publish(msg)
    spin(0.2)
    assert received['imu']
    out = received['imu'][-1]
    assert [out.angular_velocity.x, out.angular_velocity.y,
            out.angular_velocity.z] == pytest.approx([0.1, -0.2, -0.3])
    assert [out.linear_acceleration.x, out.linear_acceleration.y,
            out.linear_acceleration.z] == pytest.approx([1.0, -2.0, 9.81])
    assert out.orientation_covariance[0] == -1.0
    assert out.header.stamp.sec * 10**6 + out.header.stamp.nanosec // 1000 == msg.timestamp
    assert out.header.frame_id == 'px4_imu_flu'
    before = len(received['imu'])
    pubs['combined'].publish(msg)  # Duplicate sample must not be reintegrated.
    spin(0.15)
    assert len(received['imu']) == before


@pytest.mark.parametrize('case', ['future', 'stale', 'skew', 'invalid_relative',
                                  'clip', 'nan', 'zero_dt'])
def test_invalid_imu_is_rejected(system, case):
    node, pubs, received, spin = system
    msg = imu(node)
    if case == 'future':
        msg.timestamp += 500000
    elif case == 'stale':
        msg.timestamp -= 500000
    elif case == 'skew':
        msg.accelerometer_timestamp_relative = 2000
    elif case == 'invalid_relative':
        msg.accelerometer_timestamp_relative = SensorCombined.RELATIVE_TIMESTAMP_INVALID
    elif case == 'clip':
        msg.accelerometer_clipping = 1
    elif case == 'nan':
        msg.gyro_rad = [math.nan, 0.0, 0.0]
    else:
        msg.gyro_integral_dt = 0
    before = len(received['imu'])
    pubs['combined'].publish(msg)
    spin(0.15)
    assert len(received['imu']) == before
