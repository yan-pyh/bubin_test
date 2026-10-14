import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    params_dir = '/home/nvidia/vision_ws/config'

    video_player = ComposableNode(
        package='rm_video_player',
        plugin='fyt::camera_driver::VideoPlayerNode',
        name='video_player',
        parameters=[os.path.join(params_dir, 'video_player_params.yaml')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    armor_detector = ComposableNode(
        package='armor_detector',
        plugin='fyt::auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[os.path.join(params_dir, 'armor_detector_params.yaml')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[video_player, armor_detector],
        output='both',
        emulate_tty=True,
    )
    return LaunchDescription([container])
