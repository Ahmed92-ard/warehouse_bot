#!/usr/bin/env python3
"""Stream laptop microphone PCM to the rover over a paced low-latency TCP link."""

import ctypes
import os
import signal
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def set_parent_death_signal():
    """Exit the sender if its ros2 launch supervisor disappears."""
    parent_pid = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        return
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signal.SIGTERM)


class AudioTcpSenderNode(Node):
    def __init__(self):
        super().__init__("audio_tcp_sender")
        self.source = str(self.declare_parameter("source", "").value)
        self.host = str(self.declare_parameter("host", "192.168.0.113").value)
        self.port = int(self.declare_parameter("port", 5004).value)
        self.sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self.channels = int(self.declare_parameter("channels", 1).value)
        self.chunk_ms = int(self.declare_parameter("chunk_ms", 10).value)
        if self.sample_rate <= 0 or self.channels <= 0 or self.chunk_ms <= 0:
            raise ValueError("sample_rate, channels, and chunk_ms must be positive")

        self.chunk_bytes = self.sample_rate * self.channels * 2 * self.chunk_ms // 1000
        self.stop_event = threading.Event()
        self.capture_enabled = threading.Event()
        self.last_control_message = time.monotonic()
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.control_sub = self.create_subscription(
            Bool, "/mars/audio/laptop_mic_enabled", self._on_capture_control,
            control_qos)
        self.control_watchdog = self.create_timer(0.25, self._check_control_heartbeat)
        self.process = None
        self.sock = None
        self.thread = threading.Thread(target=self._run, name="audio-tcp-sender", daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"Laptop mic PCM TCP stream to {self.host}:{self.port}; "
            f"{self.sample_rate} Hz, {self.channels} channel(s), {self.chunk_ms} ms chunks; "
            "capture is gated by the leader GUI push-to-talk button")

    def _on_capture_control(self, msg: Bool):
        self.last_control_message = time.monotonic()
        if msg.data:
            if not self.capture_enabled.is_set():
                self.capture_enabled.set()
                self.get_logger().info("Push-to-talk active; opening laptop microphone stream")
            return

        if self.capture_enabled.is_set():
            self.capture_enabled.clear()
            process = self.process
            if process is not None and process.poll() is None:
                process.terminate()
            self.get_logger().info("Push-to-talk released; laptop microphone stream stopped")

    def _check_control_heartbeat(self):
        if (self.capture_enabled.is_set() and
                time.monotonic() - self.last_control_message > 1.0):
            self.capture_enabled.clear()
            process = self.process
            if process is not None and process.poll() is None:
                process.terminate()
            self.get_logger().warning(
                "Leader GUI control heartbeat lost; microphone capture stopped for safety")

    @staticmethod
    def _read_exact(stream, size):
        data = bytearray()
        while len(data) < size:
            part = stream.read(size - len(data))
            if not part:
                return None
            data.extend(part)
        return data

    def _start_capture(self):
        command = [
            "parec", "--raw", "--format=s16le",
            "--rate", str(self.sample_rate), "--channels", str(self.channels),
            "--latency-msec=20",
        ]
        if self.source:
            command.extend(["--device", self.source])
        self.process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=None, bufsize=0)
        self.get_logger().info("Opened laptop microphone capture for TCP audio")

    def _stop_capture(self):
        process, self.process = self.process, None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def _close_socket(self):
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def _run(self):
        last_retry_log = 0.0
        while not self.stop_event.is_set() and rclpy.ok():
            if not self.capture_enabled.wait(timeout=0.1):
                continue
            try:
                sock = socket.create_connection((self.host, self.port), timeout=1.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
                # Keep the stream socket blocking after connect. A short send
                # timeout can expire during brief receiver/audio scheduling
                # stalls, causing a full mic and TCP reconnect every few sec.
                sock.setblocking(True)
                self.sock = sock
                if not self.capture_enabled.is_set():
                    continue
                self.get_logger().info(f"Connected to rover audio receiver at {self.host}:{self.port}")
                self._start_capture()
                while (not self.stop_event.is_set() and self.capture_enabled.is_set()
                       and self.process and self.process.stdout):
                    chunk = self._read_exact(self.process.stdout, self.chunk_bytes)
                    if chunk is None:
                        if self.capture_enabled.is_set():
                            raise RuntimeError("microphone capture stream ended")
                        break
                    sock.sendall(chunk)
            except (OSError, RuntimeError, FileNotFoundError) as error:
                if not self.stop_event.is_set() and self.capture_enabled.is_set():
                    now = time.monotonic()
                    if now - last_retry_log >= 3.0:
                        self.get_logger().warning(f"Audio TCP/capture interrupted: {error}; reconnecting")
                        last_retry_log = now
            finally:
                self._close_socket()
                self._stop_capture()
            self.stop_event.wait(0.25)

    def destroy_node(self):
        self.stop_event.set()
        self.capture_enabled.clear()
        self._close_socket()
        self._stop_capture()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    set_parent_death_signal()
    rclpy.init(args=args)
    node = AudioTcpSenderNode()
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
