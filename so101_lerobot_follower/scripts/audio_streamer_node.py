#!/usr/bin/env python3
"""Capture USB microphone PCM with arecord and publish bounded ROS audio chunks."""

import subprocess
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from so101_lerobot_follower.msg import AudioChunk


class AudioStreamerNode(Node):
    def __init__(self):
        super().__init__("audio_streamer", namespace="so101/follower")
        self.device = str(self.declare_parameter("device", "plughw:CARD=CAM,DEV=0").value)
        self.sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self.channels = int(self.declare_parameter("channels", 1).value)
        self.chunk_ms = int(self.declare_parameter("chunk_ms", 10).value)
        self.audio_gain = float(self.declare_parameter("audio_gain", 3.0).value)
        self.topic = str(self.declare_parameter("topic", "/so101/follower/audio/pcm").value)
        self.reconnect_delay = float(self.declare_parameter("reconnect_delay_sec", 2.0).value)
        if self.sample_rate <= 0 or self.channels <= 0 or self.chunk_ms <= 0:
            raise ValueError("sample_rate, channels, and chunk_ms must all be positive")
        if not math.isfinite(self.audio_gain) or not 0.1 <= self.audio_gain <= 8.0:
            raise ValueError("audio_gain must be finite and between 0.1 and 8.0")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(AudioChunk, self.topic, qos)
        self.running = True
        self.process = None
        self.thread = threading.Thread(target=self._capture_loop, name="usb-audio-capture", daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"USB microphone stream device={self.device} format=S16_LE "
            f"rate={self.sample_rate} channels={self.channels} gain={self.audio_gain:.1f}x "
            f"chunk={self.chunk_ms}ms topic={self.topic}"
        )

    def _capture_loop(self):
        chunk_bytes = self.sample_rate * self.channels * 2 * self.chunk_ms // 1000
        while self.running and rclpy.ok():
            try:
                self.process = subprocess.Popen(
                    ["arecord", "-q", "-D", self.device, "-t", "raw", "-f", "S16_LE",
                     "-r", str(self.sample_rate), "-c", str(self.channels),
                     "--period-time=10000", "--buffer-time=40000"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0,
                )
                self.get_logger().info(f"Microphone capture opened on {self.device}")
                while self.running and self.process and self.process.stdout:
                    data = bytearray()
                    while self.running and len(data) < chunk_bytes:
                        part = self.process.stdout.read(chunk_bytes - len(data))
                        if not part:
                            break
                        data.extend(part)
                    if len(data) != chunk_bytes:
                        break
                    if self.audio_gain != 1.0:
                        import numpy as np
                        samples = np.frombuffer(data, dtype="<i2").astype(np.int32)
                        samples = np.clip(samples * self.audio_gain, -32768, 32767).astype("<i2")
                        data = samples.tobytes()
                    msg = AudioChunk()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "usb_microphone"
                    msg.sample_rate = self.sample_rate
                    msg.channels = self.channels
                    msg.encoding = "S16_LE"
                    msg.data = list(data)
                    self.publisher.publish(msg)
            except FileNotFoundError:
                self.get_logger().error("arecord is missing; install the Pi package alsa-utils")
                return
            except Exception as error:
                if self.running:
                    self.get_logger().warning(f"Microphone capture failed: {error}")
            finally:
                process, self.process = self.process, None
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            if self.running:
                self.get_logger().warning(f"Microphone unavailable; retrying in {self.reconnect_delay}s")
                time.sleep(self.reconnect_delay)

    def destroy_node(self):
        self.running = False
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamerNode()
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
