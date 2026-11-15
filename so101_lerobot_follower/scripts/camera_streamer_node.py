#!/usr/bin/env python3
"""Asynchronous USB Camera Streamer for SO-101 Follower Raspberry Pi.

Captures frames from a V4L2 USB camera (e.g., Zebronics Zeb-Sharp Pro), encodes them as JPEG,
and publishes sensor_msgs/msg/CompressedImage on a dedicated low-latency topic.
Runs entirely independent of the motor-control node and never alters torque state.
"""

import sys
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

try:
    import cv2
except ImportError:
    cv2 = None


class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__("camera_streamer", namespace="so101/follower")

        # Declare configurable parameters
        self.device_path = self.declare_parameter("device", "/dev/video0").value
        self.width = int(self.declare_parameter("width", 640).value)
        self.height = int(self.declare_parameter("height", 480).value)
        self.fps = float(self.declare_parameter("fps", 30.0).value)
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 75).value)
        self.reconnect_delay = float(self.declare_parameter("reconnect_delay_sec", 2.0).value)
        self.topic = str(self.declare_parameter(
            "topic", "/so101/follower/camera/image/compressed").value)
        self.frame_id = str(self.declare_parameter("frame_id", "camera_optical_frame").value)

        # Topic QoS: Best effort, volatile, depth 1 to eliminate queue buildup
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(
            CompressedImage, self.topic, qos
        )

        self.get_logger().info(
            f"CameraStreamer initialized: device={self.device_path}, resolution={self.width}x{self.height}@{self.fps}fps, "
            f"jpeg_quality={self.jpeg_quality}, topic={self.topic}"
        )

        self.running = True
        self.cap = None
        self.capture_thread = threading.Thread(target=self._capture_loop, name="camera-capture-thread", daemon=True)
        self.capture_thread.start()

    def _open_camera(self) -> bool:
        if cv2 is None:
            self.get_logger().error("OpenCV (cv2) is not installed. Camera streaming disabled.")
            return False

        try:
            # Use V4L2 backend on Linux
            cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
            if not cap.isOpened():
                # Fallback to default backend
                cap = cv2.VideoCapture(self.device_path)

            if not cap.isOpened():
                return False

            # Set preferred video codec to MJPEG for high frame-rate USB capture
            try:
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            except Exception:
                pass

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)

            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(
                f"Camera device {self.device_path} opened: actual {actual_w}x{actual_h} @ {actual_fps:.1f} FPS"
            )
            self.cap = cap
            return True
        except Exception as e:
            self.get_logger().warn(f"Exception opening camera {self.device_path}: {e}")
            return False

    def _capture_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, self.jpeg_quality))]
        frame_interval = 1.0 / max(1.0, self.fps)

        last_warn_time = 0.0

        while self.running and rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                if not self._open_camera():
                    now = time.monotonic()
                    if now - last_warn_time > 5.0:
                        self.get_logger().warn(
                            f"Waiting for camera {self.device_path}... (retrying every {self.reconnect_delay}s)"
                        )
                        last_warn_time = now
                    time.sleep(self.reconnect_delay)
                    continue

            loop_start = time.monotonic()
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn(f"Failed to read frame from {self.device_path}. Reconnecting...")
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                time.sleep(self.reconnect_delay)
                continue

            # Compress to JPEG
            success, enc_img = cv2.imencode(".jpg", frame, encode_param)
            if not success:
                continue

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = enc_img.tobytes()

            self.image_pub.publish(msg)

            # Throttle to desired FPS
            elapsed = time.monotonic() - loop_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self.running = False
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

