#!/usr/bin/env python3
"""Play AudioChunk PCM through the local PipeWire default output."""

import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from so101_lerobot_follower.msg import AudioChunk


class AudioPlaybackNode(Node):
    def __init__(self):
        super().__init__("audio_playback")
        self.topic = str(self.declare_parameter(
            "topic", "/mars/audio/laptop_mic/pcm").value)
        self.sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self.channels = int(self.declare_parameter("channels", 1).value)
        self.target = str(self.declare_parameter("target", "").value)
        self.process = None
        self.last_error_log = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            AudioChunk, self.topic, self._on_audio, qos)
        self.get_logger().info(
            f"PCM playback topic={self.topic} output="
            f"{self.target or 'PipeWire default sink'} at {self.sample_rate} Hz")

    def _start_player(self):
        command = [
            "pw-cat", "--playback", "--format=s16",
            "--rate", str(self.sample_rate), "--channels", str(self.channels),
        ]
        if self.target:
            command.extend(["--target", self.target])
        command.append("-")
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, bufsize=0)
        self.get_logger().info("PipeWire audio playback started")

    def _on_audio(self, msg):
        if msg.sample_rate != self.sample_rate or msg.channels != self.channels:
            return
        if msg.encoding != "S16_LE":
            return
        try:
            if self.process is None or self.process.poll() is not None:
                self._start_player()
            self.process.stdin.write(bytes(msg.data))
            self.process.stdin.flush()
        except FileNotFoundError:
            self.process = None
            now = time.monotonic()
            if now - self.last_error_log > 5.0:
                self.get_logger().error("pw-cat is missing; install pipewire-bin")
                self.last_error_log = now
        except (BrokenPipeError, OSError) as error:
            if self.process is not None:
                try:
                    self.process.terminate()
                except Exception:
                    pass
            self.process = None
            now = time.monotonic()
            if now - self.last_error_log > 5.0:
                self.get_logger().warning(f"PipeWire playback stopped: {error}")
                self.last_error_log = now

    def destroy_node(self):
        if self.process is not None:
            if self.process.stdin:
                self.process.stdin.close()
            if self.process.poll() is None:
                self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlaybackNode()
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
