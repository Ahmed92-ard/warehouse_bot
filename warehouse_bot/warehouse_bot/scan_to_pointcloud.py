"""Project a 2D LaserScan into a generic XYZ PointCloud2 for self-filtering."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_points')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(f'Projecting {input_topic} into XYZ cloud {output_topic}')

    def scan_callback(self, scan):
        points = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
            angle += scan.angle_increment
        self.publisher.publish(create_cloud_xyz32(scan.header, points))


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
