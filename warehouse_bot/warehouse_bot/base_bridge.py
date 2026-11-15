#!/usr/bin/env python3
"""MARS base bridge: /cmd_vel -> ESP32 'V l r' (counts/s); ESP32 'E ...' -> /odom, TF, /joint_states."""
import math, threading, time
import rclpy, serial
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class BaseBridge(Node):
    def __init__(self):
        super().__init__('base_bridge')
        dp = self.declare_parameter
        self.port = dp('port', '/dev/esp32').value
        self.r = dp('wheel_radius', 0.076).value
        self.b = dp('wheel_separation', 0.445).value
        self.cpr = dp('counts_per_rev', 4992.0).value
        self.max_cps = dp('max_cps', 3500.0).value
        self.cmd_timeout = dp('cmd_timeout', 0.5).value
        self.publish_tf = dp('publish_tf', True).value
        self.odom_frame = dp('odom_frame', 'odom').value
        self.base_frame = dp('base_frame', 'base_footprint').value
        self.joints = dp('wheel_joints', ['wheel1_joint', 'wheel2_joint']).value  # [left, right]
        self.mpc = 2.0 * math.pi * self.r / self.cpr          # metres per count

        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = self.port, 115200, 0.2
        self.ser.dtr = False; self.ser.rts = False; self.ser.open()
        self.ser.rts = True; time.sleep(0.1); self.ser.rts = False; time.sleep(2.5)  # clean ESP32 reset
        self.ser.reset_input_buffer(); self.ser.write(b'S\nZ\n')
        self.wlock = threading.Lock()

        self.target = (0.0, 0.0); self.last_cmd = 0.0
        self.x = self.y = self.th = 0.0; self.prev = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.send_cmd)
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.get_logger().info(f'base_bridge up on {self.port}: r={self.r} b={self.b} cpr={self.cpr}')

    def write(self, s):
        try:
            with self.wlock: self.ser.write(s.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'serial write failed: {e}')

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        l = (v - w * self.b / 2.0) / self.mpc
        r = (v + w * self.b / 2.0) / self.mpc
        k = max(abs(l), abs(r)) / self.max_cps
        if k > 1.0: l, r = l / k, r / k                    # keep curvature when saturating
        self.target = (l, r); self.last_cmd = time.time()

    def send_cmd(self):
        if time.time() - self.last_cmd > self.cmd_timeout: self.target = (0.0, 0.0)
        self.write(f'V {self.target[0]:.0f} {self.target[1]:.0f}\n')

    def read_loop(self):
        while rclpy.ok():
            try:
                s = self.ser.readline().decode(errors='ignore').split()
            except serial.SerialException as e:
                self.get_logger().error(f'serial read failed: {e}'); time.sleep(1.0); continue
            if len(s) != 7 or s[0] != 'E': continue
            try:
                tl, tr, sl, sr = int(s[1]), int(s[2]), float(s[3]), float(s[4])
            except ValueError:
                continue
            self.update(tl, tr, sl, sr)

    def update(self, tl, tr, sl, sr):
        if self.prev is None or abs(tl - self.prev[0]) > 10000 or abs(tr - self.prev[1]) > 10000:
            if self.prev is not None: self.get_logger().warn('tick jump (ESP32 reset?) - re-syncing odometry')
            self.prev = (tl, tr); return
        dl = (tl - self.prev[0]) * self.mpc
        dr = (tr - self.prev[1]) * self.mpc
        self.prev = (tl, tr)
        ds, dth = (dl + dr) / 2.0, (dr - dl) / self.b
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
        vl, vr = sl * self.mpc, sr * self.mpc
        v, w = (vl + vr) / 2.0, (vr - vl) / self.b

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        for i, c in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            o.pose.covariance[i] = c; o.twist.covariance[i] = c
        self.odom_pub.publish(o)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, self.odom_frame, self.base_frame
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tfb.sendTransform(t)

        js = JointState()
        js.header.stamp, js.name = now, list(self.joints)
        js.position = [tl * 2 * math.pi / self.cpr, tr * 2 * math.pi / self.cpr]
        js.velocity = [sl * 2 * math.pi / self.cpr, sr * 2 * math.pi / self.cpr]
        self.js_pub.publish(js)

    def stop(self):
        self.write('S\n')


def main():
    rclpy.init()
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(); node.destroy_node(); rclpy.try_shutdown()
