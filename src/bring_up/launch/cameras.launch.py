import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, Shutdown
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

sys.path.append(os.path.join(
    get_package_share_directory('bring_up'), 'launch'))


def get_cameras_container():
    rgb_params = os.path.join(get_package_share_directory(
        'bring_up'), 'config', 'rgb.yaml')
    thermal_params = os.path.join(get_package_share_directory(
        'bring_up'), 'config', 'thermal.yaml')

    rgb_caminfo = os.path.join('package://bring_up/config', 'caminfo_rgb.yaml')
    thermal_caminfo = os.path.join(
        'package://bring_up/config', 'caminfo_thermal.yaml')

    cameras_container = ComposableNodeContainer(
        name='cameras_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='opencv_cam',
                plugin='opencv_cam::OpencvCameraNode',
                name='cam_rgb',
                parameters=[rgb_params, {
                        'camera_info_url': rgb_caminfo
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='opencv_cam',
                plugin='opencv_cam::OpencvCameraNode',
                name='cam_thermal',
                parameters=[thermal_params, {
                        'camera_info_url': "none"
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='thermal_imgproc',
                plugin='thermal_imgproc::ThermalImageProcNode',
                name='thermal_imgproc',
                parameters=[thermal_params, {
                        'camera_info_url': thermal_caminfo
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )
    return cameras_container

def generate_launch_description():
    return LaunchDescription([
        get_cameras_container()
    ])