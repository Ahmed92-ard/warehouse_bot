import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = get_package_share_directory('warehouse_bot')
    xacro_file = os.path.join(share, 'urdf', 'mars_rover.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    self_filter_config = os.path.join(share, 'config', 'self_filter.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='warehouse_bot', executable='esp32_bridge',
             parameters=[os.path.join(share, 'config', 'base.yaml')], output='screen'),
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar_node',
             parameters=[{'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 'serial_baudrate': 115200,
                          'frame_id': 'lidar_link', 'angle_compensate': True}],
             output='screen'),
        Node(package='warehouse_bot', executable='scan_to_pointcloud', name='scan_to_pointcloud',
             parameters=[{'input_topic': '/scan', 'output_topic': '/scan_points'}], output='screen'),
        Node(package='robot_self_filter', executable='self_filter', name='self_filter',
             parameters=[self_filter_config, {
                 'robot_description': robot_description,
                 'lidar_sensor_type': 0,
                 'use_sim_time': False,
                 'zero_for_removed_points': False,
             }],
             remappings=[('/cloud_in', '/scan_points'), ('/cloud_out', '/scan_self_filtered')],
             output='screen'),
        # SLAM Toolbox consumes LaserScan, whereas robot_self_filter consumes
        # PointCloud2. Convert the filtered cloud back so mapping never uses the
        # raw scan that contains returns from the rover/arm/camera mount.
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='filtered_cloud_to_scan',
             parameters=[{
                 'target_frame': 'lidar_link',
                 'transform_tolerance': 0.05,
                 'min_height': -0.02,
                 'max_height': 0.02,
                 'angle_min': -3.14159265,
                 'angle_max': 3.14159265,
                 'angle_increment': 0.0174532925,
                 'scan_time': 0.1,
                 'range_min': 0.15,
                 'range_max': 12.0,
                 'use_inf': True,
             }],
             remappings=[('cloud_in', '/scan_self_filtered'),
                         ('scan', '/scan_filtered')],
             output='screen'),
    ])
