"""Main launch file for SO-101 Follower Arm and Remote USB Camera."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('so101_lerobot_follower')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'follower.yaml'])
    default_calibration = PathJoinSubstitution([pkg_share, 'config', 'follower.json'])

    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=default_calibration,
                              description='Path to local follower calibration JSON'),
        DeclareLaunchArgument('serial_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114210-if00',
                              description='Serial port for follower Feetech arm'),
        DeclareLaunchArgument('baud_rate', default_value='1000000', description='Serial baud rate'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0', description='Control loop frequency'),
        DeclareLaunchArgument('max_action_age_ms', default_value='500', description='Stale action threshold'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Whether to start USB camera streaming node alongside follower'),
        DeclareLaunchArgument('camera_device',
                              default_value='/dev/v4l/by-id/usb-BC-241211-ZW_USB_Camera_01.00.00-video-index0',
                              description='Stable V4L2 path for arm camera 1'),
        DeclareLaunchArgument('camera_fps', default_value='30.0', description='Camera frame rate'),
        DeclareLaunchArgument('camera_quality', default_value='75', description='Camera JPEG quality'),
        DeclareLaunchArgument('camera2_device',
                              default_value='/dev/v4l/by-id/usb-WCM_USB_WEB_CAM-video-index0',
                              description='Stable V4L2 path for arm camera 2 and QR scanning'),
        DeclareLaunchArgument('enable_camera2', default_value='true', description='Start the second USB camera'),
        DeclareLaunchArgument('camera2_fps', default_value='15.0', description='Second camera frame rate'),
        DeclareLaunchArgument('camera2_quality', default_value='65', description='Second camera JPEG quality'),
        DeclareLaunchArgument('enable_audio', default_value='true', description='Stream USB microphone audio'),
        DeclareLaunchArgument('audio_device', default_value='plughw:CARD=CAM,DEV=0', description='ALSA capture device for the USB microphone'),
        DeclareLaunchArgument('audio_chunk_ms', default_value='10', description='USB microphone audio packet size in milliseconds'),
        DeclareLaunchArgument('audio_gain', default_value='3.0', description='Digital microphone gain multiplier'),

        Node(
            package='so101_lerobot_follower',
            executable='so101_lerobot_follower_node',
            name='follower',
            namespace='so101',
            output='screen',
            parameters=[default_params, {
                'calibration_file': LaunchConfiguration('calibration_file'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'max_action_age_ms': LaunchConfiguration('max_action_age_ms'),
            }],
        ),

        Node(
            package='so101_lerobot_follower',
            executable='action_feedback_to_joint_states.py',
            name='action_feedback_to_joint_states',
            namespace='so101',
            output='screen',
            parameters=[{
                'feedback_topic': '/so101/follower/action_feedback',
                'joint_states_topic': '/joint_states',
                'joint_prefix': 'arm_',
                'gripper_lower': -0.174533,
                'gripper_upper': 1.74533,
            }],
        ),

        Node(
            package='so101_lerobot_follower',
            executable='camera_streamer_node.py',
            name='camera_streamer',
            namespace='so101/follower',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_camera')),
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'fps': LaunchConfiguration('camera_fps'),
                'jpeg_quality': LaunchConfiguration('camera_quality'),
                'topic': '/so101/follower/camera/image/compressed',
                'frame_id': 'camera1_optical_frame',
            }],
        ),
        Node(
            package='so101_lerobot_follower',
            executable='camera_streamer_node.py',
            name='camera2_streamer',
            namespace='so101/follower',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_camera2')),
            parameters=[{
                'device': LaunchConfiguration('camera2_device'),
                'width': 640,
                'height': 480,
                'fps': LaunchConfiguration('camera2_fps'),
                'jpeg_quality': LaunchConfiguration('camera2_quality'),
                'topic': '/so101/follower/camera2/image/compressed',
                'frame_id': 'camera2_optical_frame',
            }],
        ),
        Node(
            package='so101_lerobot_follower',
            executable='audio_streamer_node.py',
            name='audio_streamer',
            namespace='so101/follower',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_audio')),
            parameters=[{
                'device': LaunchConfiguration('audio_device'),
                'chunk_ms': LaunchConfiguration('audio_chunk_ms'),
                'audio_gain': LaunchConfiguration('audio_gain'),
            }],
        ),
    ])
