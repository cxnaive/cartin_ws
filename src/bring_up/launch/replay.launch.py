from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch.actions import TimerAction, Shutdown
from ament_index_python import get_package_share_directory
import os

# 此处修改bag路径
bag_path = '/home/cx/car_park_0'

# def get_republish_container():
#     return ComposableNodeContainer(
#         name='replay_container',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container_isolated',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='image_republish',
#                 plugin='image_republish::ImageRepublishNode',
#                 name='cam_rgb_republish',
#                 parameters=[{
#                     'publish_topic':'cam_rgb'
#                 }],
#                 extra_arguments=[{'use_intra_process_comms': True}]
#             ),
#             ComposableNode(
#                 package='image_republish',
#                 plugin='image_republish::ImageRepublishNode',
#                 name='cam_thermal_republish',
#                 parameters=[{
#                     'publish_topic':'cam_thermal'
#                 }],
#                 extra_arguments=[{'use_intra_process_comms': True}]
#             ),
#         ],
#         output='both',
#         emulate_tty=True,
#         #            ros_arguments=['--ros-args', '--log-level',
#         #                           'armor_detector:='+launch_params['detector_log_level']],
#         on_exit=Shutdown(),
#     )


def generate_launch_description():

    # replay_container = get_republish_container()

    rgb_republish = Node(
        package='image_transport',
        executable='republish',
        name='main_cam_rgb',
        namespace='sensor',
        arguments=['compressed'],
        remappings=[
            ('in/compressed', '/cam_rgb/compressed'),
            ('out', '/cam_rgb')]
    )

    thermal_republish = Node(
        package='image_transport',
        executable='republish',
        name='main_cam_thermal',
        namespace='sensor',
        arguments=['compressed'],
        remappings=[
            ('in/compressed', '/cam_thermal/compressed'),
            ('out', '/cam_thermal')]
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bring_up'),
                         'launch', 'robot_state.launch.py')
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'bring_up'), 'launch', 'rviz.launch.py')
        )
    )

    bag_replay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        robot_state_launch,
        rgb_republish,
        thermal_republish,
        rviz_launch,
        # TimerAction(period=2.0, actions=[bag_replay])
    ])
