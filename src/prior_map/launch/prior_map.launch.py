import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('prior_map'), 'config', 'prior_map.yaml')
    
    map_file = os.path.join(
        get_package_share_directory('prior_map'), 'config', 'GlobalMap.pcd')
    
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='global_map_path',
                              default_value=map_file),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),

        Node(
            package='prior_map',
            executable='prior_map_node',
            output='both',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'global_map_path': LaunchConfiguration('global_map_path'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        )
    ])
