"""Warehouse task-to-Nav2 bridge; disabled unless explicitly enabled."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_backend_environment():
    candidates = (
        Path.home() / "ros2_ws/src/warehouse_bot/rover/warehouse_backend/.env",
        Path(__file__).resolve().parents[2] / "warehouse_backend/.env",
    )
    for env_file in candidates:
        if not env_file.is_file():
            continue
        for raw in env_file.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key, value = key.strip(), value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                value = value[1:-1]
            if key in ("MARS_ROBOT_API_TOKEN", "MARS_WAREHOUSE_API_URL"):
                os.environ.setdefault(key, value)
        return


_load_backend_environment()


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        SetEnvironmentVariable("MARS_ROBOT_API_TOKEN", os.getenv("MARS_ROBOT_API_TOKEN", "")),
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
            package="rover",
            executable="warehouse_task_bridge",
            name="warehouse_task_bridge",
            output="screen",
            parameters=[{"enable": True, "api_url": LaunchConfiguration("api_url")}],
            condition=IfCondition(LaunchConfiguration("enable")),
        ),
    ])
