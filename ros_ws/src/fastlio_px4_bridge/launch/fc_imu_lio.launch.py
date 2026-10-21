"""Run the calibrated FC-IMU LIO pipeline; preview output by default."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create nodes without starting the driver, agent, PTP, or arming."""
    lio_share = get_package_share_directory('fast_lio')
    bridge_share = get_package_share_directory('fastlio_px4_bridge')
    return LaunchDescription([
        DeclareLaunchArgument('output_topic', default_value='/lio/vehicle_visual_odometry'),
        DeclareLaunchArgument('reset_counter', default_value='0'),
        Node(package='fastlio_px4_bridge', executable='px4_imu_to_ros', output='screen'),
        Node(
            package='fast_lio', executable='fastlio_mapping', output='screen',
            parameters=[lio_share + '/config/mid360s_fc_imu.yaml']),
        Node(
            package='fastlio_px4_bridge', executable='fastlio_to_px4_visual_odometry',
            output='screen', parameters=[bridge_share + '/config/bridge.yaml', {
                'input_world_is_level': True,
                'output_topic': LaunchConfiguration('output_topic'),
                'reset_counter': ParameterValue(
                    LaunchConfiguration('reset_counter'), value_type=int),
            }]),
    ])
