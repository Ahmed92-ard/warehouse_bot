#!/usr/bin/env python3
"""Convert SO101 follower feedback (degrees/percent) to sensor_msgs/JointState."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so101_lerobot_follower.msg import SO101Action


class ActionFeedbackToJointStates(Node):
    def __init__(self):
        super().__init__('action_feedback_to_joint_states')
        self.declare_parameter('feedback_topic', '/so101/follower/action_feedback')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_prefix', 'arm_')
        self.declare_parameter('gripper_lower', -0.174533)
        self.declare_parameter('gripper_upper', 1.74533)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        prefix = self.get_parameter('joint_prefix').value
        self.joint_names = [
            prefix + name for name in (
                'shoulder_pan', 'shoulder_lift', 'elbow_flex',
                'wrist_flex', 'wrist_roll', 'gripper')
        ]
        self.gripper_lower = float(self.get_parameter('gripper_lower').value)
        self.gripper_upper = float(self.get_parameter('gripper_upper').value)
        if self.gripper_upper <= self.gripper_lower:
            raise ValueError('gripper_upper must be greater than gripper_lower')

        self.publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.subscription = self.create_subscription(
            SO101Action, self.feedback_topic, self.feedback_callback, 10)
        self.get_logger().info(
            f'Converting {self.feedback_topic} to {joint_states_topic}; '
            f'joint prefix={prefix!r}')

    def feedback_callback(self, feedback):
        expected = (
            'shoulder_pan', 'shoulder_lift', 'elbow_flex',
            'wrist_flex', 'wrist_roll', 'gripper')
        values_by_name = dict(zip(feedback.joint_names, feedback.values))
        if any(name not in values_by_name for name in expected):
            self.get_logger().warning('Ignoring feedback with unexpected joint names')
            return

        values = [float(values_by_name[name]) for name in expected]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Ignoring feedback containing non-finite positions')
            return
        if not 0.0 <= values[5] <= 100.0:
            self.get_logger().warning('Ignoring gripper feedback outside [0, 100] percent')
            return

        state = JointState()
        state.header = feedback.header
        state.name = self.joint_names
        state.position = [math.radians(value) for value in values[:5]]
        gripper_fraction = values[5] / 100.0
        state.position.append(
            self.gripper_lower + gripper_fraction * (self.gripper_upper - self.gripper_lower))
        self.publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = ActionFeedbackToJointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
