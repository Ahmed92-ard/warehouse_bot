#!/usr/bin/env python3
"""Receive laptop microphone PCM over TCP and play it on the rover's default sink."""

import socket
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class AudioTcpReceiverNode(Node):
    def __init__(self):
        super().__init__("audio_tcp_receiver")
        self.port = int(self.declare_parameter("port", 5004).value)
        self.sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self.channels = int(self.declare_parameter("channels", 1).value)
        self.chunk_ms = int(self.declare_parameter("chunk_ms", 10).value)
        self.speaker_mac = str(self.declare_parameter(
            "speaker_mac", "91:93:5B:83:95:3A").value)
        self.speaker_sink = str(self.declare_parameter(
            "speaker_sink", "bluez_output.91_93_5B_83_95_3A.1").value)
        if self.sample_rate <= 0 or self.channels <= 0 or self.chunk_ms <= 0:
            raise ValueError("sample_rate, channels, and chunk_ms must be positive")

        self.chunk_bytes = self.sample_rate * self.channels * 2 * self.chunk_ms // 1000
        self.stop_event = threading.Event()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)
        self.player = None
        self.connection = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.speaker_status_pub = self.create_publisher(
            Bool, "/mars/audio/speaker_connected", status_qos)
        self.last_speaker_connected = None
        self.speaker_status_timer = self.create_timer(1.0, self._check_speaker_connection)
        self.thread = threading.Thread(target=self._serve, name="audio-tcp-receiver", daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"Listening for laptop microphone PCM on TCP port {self.port}; "
            f"PipeWire output at {self.sample_rate} Hz")

    def _check_speaker_connection(self):
        try:
            result = subprocess.run(
                ["bluetoothctl", "info", self.speaker_mac],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return

        connected = any(
            line.strip() == "Connected: yes" for line in result.stdout.splitlines())
        self.speaker_status_pub.publish(Bool(data=connected))
        if connected != self.last_speaker_connected:
            if connected:
                self.get_logger().info(f"Bluetooth speaker connected: {self.speaker_mac}")
            else:
                self.get_logger().warning(f"Bluetooth speaker disconnected: {self.speaker_mac}")
            self.last_speaker_connected = connected

    @staticmethod
    def _read_exact(sock, size):
        data = bytearray()
        while len(data) < size:
            part = sock.recv(size - len(data))
            if not part:
                return None
            data.extend(part)
        return data

    def _start_player(self):
        command = [
            "pw-cat", "--playback", "--format=s16",
            "--rate", str(self.sample_rate), "--channels", str(self.channels),
            "--target", self.speaker_sink, "--latency=20ms", "-",
        ]
        self.player = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=None, bufsize=0)
        self.get_logger().info(
            f"PipeWire playback started for TCP audio on sink {self.speaker_sink}")

    def _stop_player(self):
        player, self.player = self.player, None
        if player is None:
            return
        if player.stdin:
            try:
                player.stdin.close()
            except OSError:
                pass
        if player.poll() is None:
            player.terminate()
        try:
            player.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            player.kill()
            player.wait()

    def _serve(self):
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                connection, address = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.connection = connection
            self.get_logger().info(f"Laptop audio connected from {address[0]}")
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            try:
                self._start_player()
                while not self.stop_event.is_set():
                    chunk = self._read_exact(connection, self.chunk_bytes)
                    if chunk is None:
                        break
                    if self.player is None or self.player.poll() is not None:
                        raise BrokenPipeError("pw-cat exited")
                    self.player.stdin.write(chunk)
                    self.player.stdin.flush()
            except (OSError, BrokenPipeError) as error:
                if not self.stop_event.is_set():
                    self.get_logger().warning(f"Audio playback connection ended: {error}")
            finally:
                connection.close()
                self.connection = None
                self._stop_player()

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.server.close()
        except OSError:
            pass
        if self.connection is not None:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.connection.close()
        self._stop_player()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioTcpReceiverNode()
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
