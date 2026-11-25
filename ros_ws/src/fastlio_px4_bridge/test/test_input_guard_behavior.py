"""Exercise the installed fail-closed bridge on an isolated local DDS domain."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def test_warmup_and_imu_loss_stop_output():
    context = rclpy.context.Context()
    context.init(domain_id=188)
    node = rclpy.create_node('guard_probe', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    binary = Path(get_package_prefix('fastlio_px4_bridge')) / 'lib/fastlio_px4_bridge'
    process = subprocess.Popen([
        str(binary / 'fastlio_to_px4_visual_odometry'), '--ros-args',
        '-p', 'input_world_is_level:=true', '-p', 'require_fresh_imu:=true',
        '-p', 'status_topic:=/test/health',
        '-p', 'input_odom_topic:=/test/odom', '-p', 'output_topic:=/test/vision'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=dict(os.environ, ROS_DOMAIN_ID='188', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'))
    received = []
    node.create_subscription(VehicleOdometry, '/test/vision', received.append,
                             qos_profile_sensor_data)
    health = []
    node.create_subscription(DiagnosticArray, '/test/health', health.append, 10)
    imu_pub = node.create_publisher(Imu, '/lio/imu', 10)
    odom_pub = node.create_publisher(Odometry, '/test/odom', 10)

    def feed(duration, use_imu):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            if use_imu:
                imu = Imu()
                imu.header.stamp = stamp
                imu_pub.publish(imu)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'camera_init'
            odom.child_frame_id = 'body'
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            executor.spin_once(timeout_sec=0.01)
            time.sleep(0.005)
    try:
        until = time.monotonic() + 5
        while odom_pub.get_subscription_count() == 0 and time.monotonic() < until:
            executor.spin_once(timeout_sec=.05)
        assert odom_pub.get_subscription_count() == 1
        feed(.4, False)
        assert not received
        feed(1, True)
        assert not received
        feed(1.5, True)
        assert received and process.poll() is None
        assert health and health[-1].status[0].hardware_id == 'fastlio_px4_bridge'
        keys = {kv.key for kv in health[-1].status[0].values}
        assert {'imu_age_ms', 'since_fresh_imu_ms', 'since_fresh_odom_ms'} <= keys
        feed(.4, False)
        log, _ = process.communicate(timeout=3)
        assert process.returncode == 2
        assert 'IMU interrupted (time since last fresh input at failure:' in log, log
        count = len(received)
        feed(.3, True)
        assert len(received) == count  # A new IMU cannot silently revive the old estimator.
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
