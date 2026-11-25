"""Run the calibrated FC-IMU LIO pipeline; preview output by default."""

from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create nodes without starting the driver, agent, PTP, or arming."""
    lio_share = get_package_share_directory('fast_lio')
    bridge_share = get_package_share_directory('fastlio_px4_bridge')
    stop_pipeline = [EmitEvent(event=Shutdown(reason='LIO component exited; restart on ground'))]
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=stop_pipeline)),
        DeclareLaunchArgument('output_topic', default_value='/lio/vehicle_visual_odometry'),
        DeclareLaunchArgument('fastlio_params', default_value=lio_share + '/config/mid360s_fc_imu.yaml'),
        DeclareLaunchArgument('bridge_params', default_value=bridge_share + '/config/bridge.yaml'),
        DeclareLaunchArgument('imu_topic', default_value='/lio/imu'),
        DeclareLaunchArgument('input_world_is_level', default_value='true'),
        DeclareLaunchArgument('reset_counter', default_value='0'),
        DeclareLaunchArgument('transport_mode', default_value='inherit',
                              choices=['inherit', 'localhost_udp']),
        OpaqueFunction(function=_nodes),
    ])


def _nodes(context):
    options = {}
    if LaunchConfiguration('transport_mode').perform(context) == 'localhost_udp':
        wrapper = Path(__file__).resolve().with_name('with_localhost_udp.sh')
        options['prefix'] = 'bash ' + shlex.quote(str(wrapper))
    return [
        Node(package='fastlio_px4_bridge', executable='px4_imu_to_ros',
             exec_name='px4_imu_to_ros', output='screen', **options,
             parameters=[{'output_topic': LaunchConfiguration('imu_topic')}]),
        Node(
            package='fast_lio', executable='fastlio_mapping',
            exec_name='fastlio_mapping', output='screen', **options,
            parameters=[LaunchConfiguration('fastlio_params')]),
        Node(
            package='fastlio_px4_bridge', executable='fastlio_to_px4_visual_odometry',
            exec_name='fastlio_to_px4_visual_odometry', output='screen', **options,
            remappings=[('/lio/imu', LaunchConfiguration('imu_topic'))],
            parameters=[LaunchConfiguration('bridge_params'), {
                'input_world_is_level': ParameterValue(
                    LaunchConfiguration('input_world_is_level'), value_type=bool),
                'output_topic': LaunchConfiguration('output_topic'),
                'reset_counter': ParameterValue(
                    LaunchConfiguration('reset_counter'), value_type=int),
            }]),
    ]
