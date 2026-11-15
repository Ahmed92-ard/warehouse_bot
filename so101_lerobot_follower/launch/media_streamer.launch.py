"""Start camera 2 and microphone streaming alongside an existing rover launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera2_device",
                              default_value="/dev/v4l/by-id/usb-WCM_USB_WEB_CAM-video-index0",
                              description="Stable V4L2 path for SO-101 arm camera 2 and QR scanning"),
        DeclareLaunchArgument("camera_fps", default_value="15.0",
                              description="Second camera capture rate"),
        DeclareLaunchArgument("camera_quality", default_value="65",
                              description="JPEG quality for the second camera"),
        DeclareLaunchArgument("audio_device", default_value="plughw:CARD=CAM,DEV=0",
                              description="ALSA capture device for the USB microphone"),
        DeclareLaunchArgument("audio_gain", default_value="3.0",
                              description="Digital microphone gain multiplier"),
        Node(
            package="so101_lerobot_follower",
            executable="camera_streamer_node.py",
            name="camera2_streamer",
            namespace="so101/follower",
            output="screen",
            parameters=[{
                "device": LaunchConfiguration("camera2_device"),
                "width": 640,
                "height": 480,
                "fps": LaunchConfiguration("camera_fps"),
                "jpeg_quality": LaunchConfiguration("camera_quality"),
                "topic": "/so101/follower/camera2/image/compressed",
                "frame_id": "camera2_optical_frame",
            }],
        ),
        Node(
            package="so101_lerobot_follower",
            executable="audio_streamer_node.py",
            name="audio_streamer",
            namespace="so101/follower",
            output="screen",
            parameters=[{"device": LaunchConfiguration("audio_device"),
                         "audio_gain": LaunchConfiguration("audio_gain")}],
        ),
    ])
