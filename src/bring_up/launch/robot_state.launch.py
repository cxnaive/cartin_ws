import os
import yaml
import sys

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

def generate_launch_description():

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('bring_up'), 'urdf', 'sensor_suite.urdf.xacro')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
