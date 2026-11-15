"""Interactive keyboard teleop with MARS Nav2-matched default speeds."""

import sys


def main():
    # teleop_twist_keyboard declares these parameters read-only, so provide
    # the ROS parameter overrides before its rclpy.init() call.
    sys.argv.extend([
        "--ros-args",
        "-p", "speed:=0.12",
        "-p", "turn:=0.35",
    ])

    from teleop_twist_keyboard import main as keyboard_main

    keyboard_main()
