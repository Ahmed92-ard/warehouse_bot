import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('warehouse_bot'), 'config', 'base.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        Node(package='warehouse_bot', executable='esp32_bridge', parameters=[cfg], output='screen'),
    ])
