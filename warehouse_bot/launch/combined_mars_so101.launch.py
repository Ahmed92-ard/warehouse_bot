"""Start the SO101 follower and MARS base, LiDAR, and self-filter bringup.

The follower is included from its own package. MARS is started with ``ros2
launch`` in a separate process, so the two launch trees remain independently
managed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    follower_share = get_package_share_directory("so101_lerobot_follower")
    follower_launch = os.path.join(follower_share, "launch", "follower.launch.py")

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(follower_launch),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "control_rate_hz": LaunchConfiguration("control_rate_hz"),
            "max_action_age_ms": LaunchConfiguration("max_action_age_ms"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "enable_camera": LaunchConfiguration("enable_camera"),
            "camera_device": LaunchConfiguration("camera_device"),
            "camera_fps": LaunchConfiguration("camera_fps"),
            "camera_quality": LaunchConfiguration("camera_quality"),
            "enable_camera2": LaunchConfiguration("enable_camera2"),
            "camera2_device": LaunchConfiguration("camera2_device"),
            "camera2_fps": LaunchConfiguration("camera2_fps"),
            "camera2_quality": LaunchConfiguration("camera2_quality"),
            "enable_audio": LaunchConfiguration("enable_audio"),
            "audio_device": LaunchConfiguration("audio_device"),
            "audio_chunk_ms": LaunchConfiguration("audio_chunk_ms"),
            "audio_gain": LaunchConfiguration("audio_gain"),
        }.items(),
    )

    # Do not resolve warehouse_bot's package share in this launch description.
    # ros2 launch can fail independently inside this child process while the
    # SO101 follower remains alive.
    mars = ExecuteProcess(
        cmd=["ros2", "launch", "warehouse_bot", LaunchConfiguration("mars_launch_file")],
        name="mars_bringup",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mars")),
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        DeclareLaunchArgument("start_mars", default_value="true",
                              description="Start MARS bringup in a separate process"),
        DeclareLaunchArgument("mars_launch_file", default_value="robot.launch.py",
                              description="Launch file in warehouse_bot (for example robot.launch.py or base.launch.py)"),
        DeclareLaunchArgument("serial_port", default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114210-if00"),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("max_action_age_ms", default_value="500"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("camera_device",
                              default_value="/dev/v4l/by-id/usb-BC-241211-ZW_USB_Camera_01.00.00-video-index0",
                              description="Stable V4L2 path for SO-101 arm camera 1"),
        DeclareLaunchArgument("camera_fps", default_value="30.0"),
        DeclareLaunchArgument("camera_quality", default_value="75"),
        DeclareLaunchArgument("enable_camera2", default_value="true",
                              description="Start the second camera through the follower launch"),
        DeclareLaunchArgument("camera2_device",
                              default_value="/dev/v4l/by-id/usb-WCM_USB_WEB_CAM-video-index0",
                              description="Stable V4L2 path for SO-101 arm camera 2 and QR scanning"),
        DeclareLaunchArgument("camera2_fps", default_value="15.0"),
        DeclareLaunchArgument("camera2_quality", default_value="65"),
        DeclareLaunchArgument("enable_rover_camera", default_value="true",
                              description="Start the iBall camera mounted on the MARS chassis"),
        DeclareLaunchArgument("rover_camera_device",
                              default_value="/dev/v4l/by-id/usb-iBall_Face2Face_CHD20.0_Webcam_iBall_Face2Face_CHD20.0_Webcam_20140113-video-index0",
                              description="Stable V4L2 path for the iBall chassis camera"),
        DeclareLaunchArgument("rover_camera_fps", default_value="15.0"),
        DeclareLaunchArgument("enable_audio", default_value="true"),
        DeclareLaunchArgument("audio_device", default_value="plughw:CARD=CAM,DEV=0"),
        DeclareLaunchArgument("audio_chunk_ms", default_value="10",
                              description="Arm microphone audio packet size in milliseconds"),
        DeclareLaunchArgument("audio_gain", default_value="3.0",
                              description="Arm microphone digital gain multiplier"),
        DeclareLaunchArgument("enable_remote_audio_playback", default_value="true",
                              description="Play laptop microphone PCM through the Pi audio output"),
        # Both actions are scheduled independently; neither depends on the
        # other's process reaching a ready state.
        arm,
        mars,
        Node(
            package="so101_lerobot_follower",
            executable="camera_streamer_node.py",
            name="rover_camera_streamer",
            namespace="mars",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_rover_camera")),
            parameters=[{
                "device": LaunchConfiguration("rover_camera_device"),
                "width": 640,
                "height": 480,
                "fps": LaunchConfiguration("rover_camera_fps"),
                "jpeg_quality": 65,
                "topic": "/mars/camera/image/compressed",
                "frame_id": "rover_camera_optical_frame",
            }],
        ),
        Node(
            package="so101_lerobot_follower",
            executable="audio_tcp_receiver_node.py",
            name="laptop_audio_tcp_receiver",
            namespace="mars",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_remote_audio_playback")),
            parameters=[{
                "port": 5004,
                "sample_rate": 16000,
                "channels": 1,
                "chunk_ms": 10,
                "speaker_mac": "91:93:5B:83:95:3A",
                "speaker_sink": "bluez_output.91_93_5B_83_95_3A.1",
            }],
        ),
    ])
