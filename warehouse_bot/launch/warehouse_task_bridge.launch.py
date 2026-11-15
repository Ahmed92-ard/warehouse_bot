"""Warehouse task-to-Nav2 bridge; disabled unless explicitly enabled."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        DeclareLaunchArgument(
            "enable",
            default_value="false",
            description="Enable API task polling and Nav2 goals (requires Nav2 and robot token).",
        ),
        DeclareLaunchArgument(
            "api_url",
            default_value=os.getenv("MARS_WAREHOUSE_API_URL", "http://127.0.0.1:8000"),
            description="Private warehouse API base URL.",
        ),
        Node(
            package="warehouse_bot",
            executable="warehouse_task_bridge",
            name="warehouse_task_bridge",
            output="screen",
            parameters=[{"enable": True, "api_url": LaunchConfiguration("api_url")}],
            condition=IfCondition(LaunchConfiguration("enable")),
        ),
    ])
