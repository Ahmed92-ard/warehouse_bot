#!/usr/bin/env python3
"""ESP32 base bridge: /cmd_vel -> 'V <l_cps> <r_cps>', encoder counts -> /odom + TF."""
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameters('', [
            ('port', '/dev/esp32'), ('baud', 115200),
            ('counts_per_rev', 5060.0), ('wheel_radius', 0.076), ('wheel_separation', 0.445),
            ('max_wheel_cps', 2200.0), ('cmd_timeout', 0.5), ('cmd_rate', 20.0),
            ('radius_trim', 0.0),
            ('odom_frame', 'odom'), ('base_frame', 'base_footprint'), ('publish_tf', True),
            ('wheel_joints', ['wheel1_joint', 'wheel2_joint']),
        ])
        g = lambda n: self.get_parameter(n).value
        self.cpr, self.sep = g('counts_per_rev'), g('wheel_separation')
        self.m_per_count = 2.0 * math.pi * g('wheel_radius') / self.cpr
        trim = g('radius_trim')   # +ve = right wheel effectively bigger
        self.mpc_l, self.mpc_r = self.m_per_count * (1 - trim), self.m_per_count * (1 + trim)
        self.max_cps, self.timeout = g('max_wheel_cps'), g('cmd_timeout')
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.publish_tf, self.joints = g('publish_tf'), g('wheel_joints')

        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = g('port'), g('baud'), 0.1
        # Never let a non-responsive USB serial device block node construction
        # before its ROS publishers/subscribers have been created.
        self.ser.write_timeout = 1.0
        self.ser.dtr = False
        self.ser.rts = False          # keep the ESP32-C6 out of reset
        self.ser.open()
        time.sleep(0.5)
        try:
            self.ser.write(b'S\nZ\n')
        except serial.SerialException as e:
            self.get_logger().error(f'initial serial command failed: {e}')

        self.lock = threading.Lock()
        self.target = (0.0, 0.0)
        self.last_cmd = 0.0
        self.x = self.y = self.th = 0.0
        self.last_counts = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / g('cmd_rate'), self.send_cmd)
        threading.Thread(target=self.reader, daemon=True).start()
        self.get_logger().info(
            f"ESP32 bridge on {g('port')}: r={g('wheel_radius')} sep={self.sep} cpr={self.cpr} trim={trim}")

    def on_cmd_vel(self, msg):
        v, w = msg.linear.x, msg.angular.z
        l = (v - w * self.sep / 2.0) / self.mpc_l
        r = (v + w * self.sep / 2.0) / self.mpc_r
        peak = max(abs(l), abs(r))
        if peak > self.max_cps:                     # scale both, keep the curve shape
            l, r = l * self.max_cps / peak, r * self.max_cps / peak
        with self.lock:
            self.target, self.last_cmd = (l, r), time.time()

    def send_cmd(self):
        with self.lock:
            l, r = self.target if time.time() - self.last_cmd < self.timeout else (0.0, 0.0)
        try:
            self.ser.write(f"V {l:.0f} {r:.0f}\n".encode())
        except serial.SerialException as e:
            self.get_logger().error(f'serial write failed: {e}', throttle_duration_sec=2.0)

    def reader(self):
        while rclpy.ok():
            try:
                s = self.ser.readline().decode(errors='ignore').split()
            except serial.SerialException:
                time.sleep(0.5)
                continue
            if len(s) != 7 or s[0] != 'E':
                continue
            try:
                lc, rc, ls, rs = int(s[1]), int(s[2]), float(s[3]), float(s[4])
            except ValueError:
                continue
            self.update_odom(lc, rc, ls, rs)

    def update_odom(self, lc, rc, ls, rs):
        if self.last_counts is None:
            self.last_counts = (lc, rc)
            return
        dl_c, dr_c = lc - self.last_counts[0], rc - self.last_counts[1]
        self.last_counts = (lc, rc)
        if abs(dl_c) > self.cpr or abs(dr_c) > self.cpr:   # ESP32 reset / counter jump
            return
        dl, dr = dl_c * self.mpc_l, dr_c * self.mpc_r
        ds, dth = (dl + dr) / 2.0, (dr - dl) / self.sep
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = now, self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = (ls * self.mpc_l + rs * self.mpc_r) / 2.0
        odom.twist.twist.angular.z = (rs * self.mpc_r - ls * self.mpc_l) / self.sep
        for i, c in zip((0, 7, 35), (0.01, 0.01, 0.05)):
            odom.pose.covariance[i] = c
            odom.twist.covariance[i] = c
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id = now, self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(t)

        js = JointState()
        js.header.stamp, js.name = now, list(self.joints)
        js.position = [lc / self.cpr * 2 * math.pi, rc / self.cpr * 2 * math.pi]
        self.js_pub.publish(js)

    def destroy_node(self):
        try:
            self.ser.write(b'S\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
