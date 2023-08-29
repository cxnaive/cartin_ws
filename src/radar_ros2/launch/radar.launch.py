import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('radar_ros2'), 'config', 'radar.yaml')
    radar_xml = os.path.join(
        get_package_share_directory('radar_ros2'), 'config', 'radar.xml')

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),

        Node(
            package='radar_ros2',
            executable='radar_ros2_node',
            output='both',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                'config_path': radar_xml,
            }],
        )
    ])
