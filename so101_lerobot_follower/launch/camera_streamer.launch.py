"""Standalone launch file for SO-101 Follower USB Camera Streaming."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device",
                              default_value="/dev/v4l/by-id/usb-BC-241211-ZW_USB_Camera_01.00.00-video-index0",
                              description="Stable V4L2 path for SO-101 arm camera 1"),
        DeclareLaunchArgument("width", default_value="640", description="Capture width in pixels"),
        DeclareLaunchArgument("height", default_value="480", description="Capture height in pixels"),
        DeclareLaunchArgument("fps", default_value="30.0", description="Capture frame rate"),
        DeclareLaunchArgument("jpeg_quality", default_value="75", description="JPEG compression quality (1-100)"),

        Node(
            package="so101_lerobot_follower",
            executable="camera_streamer_node.py",
            name="camera_streamer",
            namespace="so101/follower",
            output="screen",
            parameters=[{
                "device": LaunchConfiguration("device"),
                "width": LaunchConfiguration("width"),
                "height": LaunchConfiguration("height"),
                "fps": LaunchConfiguration("fps"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
            }],
        ),
    ])

