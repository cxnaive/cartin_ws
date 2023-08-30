import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def get_radar_container():
    params_file = os.path.join(
        get_package_share_directory('bring_up'), 'config', 'radar.yaml')
    radar_xml = os.path.join(
        get_package_share_directory('bring_up'), 'config', 'radar.xml')

    radar_container = ComposableNodeContainer(
        name='radar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='radar_ros2',
                plugin='radar_ros2::RadarNode',
                name='radar',
                parameters=[params_file, {
                        'config_path': radar_xml,
                        'use_sensor_data_qos': False,
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )
    return radar_container


def generate_launch_description():
    return LaunchDescription([
        get_radar_container()
    ])
