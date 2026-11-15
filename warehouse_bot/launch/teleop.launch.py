"""Keyboard teleoperation with Nav2-matched speed limits."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        # The upstream keyboard node reads stdin with termios. ROS launch gives
        # regular Node processes a pipe, even with emulate_tty enabled, so open
        # the launcher's controlling terminal explicitly and pass it through.
        ExecuteProcess(
            cmd=["/bin/bash", "-c", "ros2 run warehouse_bot mars_keyboard_teleop < /dev/tty"],
            name="mars_keyboard_teleop",
            output="screen",
            emulate_tty=True,
        ),
    ])
