#!/usr/bin/env python3
"""Capture the local PulseAudio/PipeWire default microphone and publish PCM chunks."""

import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from so101_lerobot_follower.msg import AudioChunk


class AudioMicStreamerNode(Node):
    def __init__(self):
        super().__init__("laptop_audio_mic_streamer")
        self.source = str(self.declare_parameter("source", "").value)
        self.topic = str(self.declare_parameter(
            "topic", "/mars/audio/laptop_mic/pcm").value)
        self.sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self.channels = int(self.declare_parameter("channels", 1).value)
        self.chunk_ms = int(self.declare_parameter("chunk_ms", 20).value)
        self.reconnect_delay = float(
            self.declare_parameter("reconnect_delay_sec", 2.0).value)
        if self.sample_rate <= 0 or self.channels <= 0 or self.chunk_ms <= 0:
            raise ValueError("sample_rate, channels, and chunk_ms must all be positive")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(AudioChunk, self.topic, qos)
        self.running = True
        self.process = None
        self.thread = threading.Thread(
            target=self._capture_loop, name="laptop-microphone-capture", daemon=True)
        self.thread.start()
        source_label = self.source or "default source"
        self.get_logger().info(
            f"Laptop microphone stream source={source_label} rate={self.sample_rate} "
            f"channels={self.channels} chunk={self.chunk_ms}ms topic={self.topic}")

    def _read_chunk(self, stream, size):
        data = bytearray()
        while self.running and len(data) < size:
            part = stream.read(size - len(data))
            if not part:
                break
            data.extend(part)
        return data

    def _capture_loop(self):
        chunk_bytes = self.sample_rate * self.channels * 2 * self.chunk_ms // 1000
        while self.running and rclpy.ok():
            try:
                command = [
                    "parec", "--raw", "--format=s16le",
                    "--rate", str(self.sample_rate),
                    "--channels", str(self.channels),
                ]
                if self.source:
                    command.extend(["--device", self.source])
                self.process = subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
                self.get_logger().info("Opened laptop microphone capture")
                while self.running and self.process and self.process.stdout:
                    data = self._read_chunk(self.process.stdout, chunk_bytes)
                    if len(data) != chunk_bytes:
                        break
                    msg = AudioChunk()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "laptop_microphone"
                    msg.sample_rate = self.sample_rate
                    msg.channels = self.channels
                    msg.encoding = "S16_LE"
                    msg.data = list(data)
                    self.publisher.publish(msg)
            except FileNotFoundError:
                self.get_logger().error("parec is missing; install pulseaudio-utils on the laptop")
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
                self.get_logger().warning(
                    f"Laptop microphone unavailable; retrying in {self.reconnect_delay}s")
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
    node = AudioMicStreamerNode()
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
