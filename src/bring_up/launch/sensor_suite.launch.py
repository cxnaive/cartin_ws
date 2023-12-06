from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bring_up'),'launch','robot_state.launch.py')
        )
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bring_up'),'launch','cameras.launch.py')
        )
    )

    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bring_up'),'launch','radar.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bring_up'),'launch','lidar.launch.py')
        )
    )

    ld.add_action(robot_state_launch)
    ld.add_action(cameras_launch)
    ld.add_action(radar_launch)
    ld.add_action(lidar_launch)
    return ld