"""Modernized Tk teleoperation GUI and ROS 2 Jazzy node for the SO-101 leader arm.

Features:
- Completely independent of the official LeRobot Python package.
- Reads leader joint states from the shared feetech_ros2_driver ROS interface.
- Loads local package-managed calibration copied from existing Hugging Face data.
- Real-time 6-joint telemetry table displaying leader angles, follower angles, and tracking error.
- Fully preserved lifecycle callbacks, safety transitions, diagnostics, and teleoperation controls.
"""

from __future__ import annotations

import json
import ctypes
import cv2
import logging
import logging.handlers
import math
import os
import queue
import signal
import socket
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import urllib.error
import urllib.request
import numpy as np
from pathlib import Path
from tkinter import messagebox, ttk

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from so101_lerobot_follower.msg import SO101Action
from so101_lerobot_leader.calibration import (
    DEFAULT_FOLLOWER_CALIBRATION,
    DEFAULT_LEADER_CALIBRATION,
    JOINT_NAMES,
    leader_count_to_follower_action,
    resolve_leader_calibration,
    validate_leader_calibration,
    validate_follower_calibration,
)

FOLLOWER_STATUS_NAME = "/so101/follower"
DIAGNOSTIC_STALE_SECONDS = 3.0
# Live read-only measurement: 25 Hz publisher (39–41 ms source intervals),
# with GUI callback gaps up to 190.5 ms. Pause output after one longer gap.
DEFAULT_LEADER_FEEDBACK_PAUSE_SEC = 0.25
def _set_parent_death_signal() -> None:
    """Exit this ROS child if its ros2 launch supervisor disappears."""
    parent_pid = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        return
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signal.SIGTERM)


def parse_follower_diagnostics(msg: DiagnosticArray) -> tuple[bool, str]:
    """Return readiness only for a complete, healthy follower status record."""
    status = next((item for item in msg.status if item.name == FOLLOWER_STATUS_NAME), None)
    if status is None:
        names = [item.name for item in msg.status]
        return False, f"follower status '{FOLLOWER_STATUS_NAME}' missing (received {names})"

    fields = {item.key: item.value for item in status.values}
    required = ("ready", "armable", "calibration_valid", "position_mode_ready", "torque_enabled_all", "feedback_valid")
    missing = [key for key in required if key not in fields]
    if missing:
        return False, f"follower diagnostics missing fields: {', '.join(missing)}"
    invalid = [key for key in required if fields[key] not in ("true", "false")]
    if invalid:
        return False, f"follower diagnostics have invalid boolean fields: {', '.join(invalid)}"

    ready = (fields["armable"] == "true" and status.level <= DiagnosticStatus.WARN and
             all(fields[key] == "true" for key in ("calibration_valid", "position_mode_ready", "feedback_valid")))
    detail = status.message or ("ready" if ready else "follower reports not ready")
    if ready and fields.get("torque_enabled_all") != "true":
        detail = "safe disarmed state; explicit Start will request torque enable"
    if not ready and status.level == DiagnosticStatus.OK:
        failed = [key for key in required if fields[key] != "true"]
        if fields.get("torque_enabled_all") != "true" and fields.get("armable") == "true":
            detail = "safe disarmed state; explicit Start will request torque enable"
        else:
            detail += " (not ready: " + ", ".join(failed) + ")"
    elif status.level != DiagnosticStatus.OK:
        detail += f" (diagnostic level {status.level})"
    return ready, detail


class OperatorConsoleHandler(logging.StreamHandler):
    """Print only concise state transitions; the rotating file keeps full records."""

    def emit(self, record: logging.LogRecord) -> None:
        if not getattr(record, "operator_event", False):
            return
        self.stream.write(record.getMessage() + self.terminator)
        self.flush()


def configure_logging(log_path: str) -> logging.handlers.QueueListener:
    destination = Path(log_path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    records: queue.Queue[logging.LogRecord] = queue.Queue(maxsize=5000)
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s [%(name)s] [pid=%(process)d thread=%(threadName)s] %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S%z",
    )
    console = OperatorConsoleHandler()
    rotating = logging.handlers.RotatingFileHandler(destination, maxBytes=5 * 1024 * 1024, backupCount=3)
    rotating.setFormatter(formatter)
    listener = logging.handlers.QueueListener(records, console, rotating, respect_handler_level=True)
    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(logging.handlers.QueueHandler(records))
    root.setLevel(logging.INFO)
    listener.start()
    logging.getLogger("so101.leader").info("rotating log file=%s; rotation=5MiB x 3", destination)
    return listener


class LeaderNode(Node):
    def __init__(self, gui_events: queue.Queue[tuple[str, object]]):
        super().__init__("leader_gui", namespace="so101")
        self.events = gui_events
        self.log = logging.getLogger("so101.leader")
        self.leader_port = self.declare_parameter("leader_port", "/dev/ttyACM0").value

        configured_calibration_file = self.declare_parameter(
            "leader_calibration_file", str(DEFAULT_LEADER_CALIBRATION)
        ).value
        self.leader_calibration_file = str(resolve_leader_calibration(configured_calibration_file))
        configured_follower_calibration = self.declare_parameter(
            "follower_calibration_file", str(DEFAULT_FOLLOWER_CALIBRATION)
        ).value
        self.follower_calibration_file = str(Path(configured_follower_calibration).expanduser().resolve())
        self.follower_ip = self.declare_parameter("follower_ip", "10.104.219.64").value
        self.rate_hz = float(self.declare_parameter("publish_rate_hz", 30.0).value)
        self.feedback_pause_timeout_sec = float(self.declare_parameter(
            "leader_feedback_pause_timeout_sec", DEFAULT_LEADER_FEEDBACK_PAUSE_SEC).value)
        if not math.isfinite(self.feedback_pause_timeout_sec) or self.feedback_pause_timeout_sec <= 0.0:
            raise ValueError("leader feedback pause timeout must be finite and positive")
        self.debug = bool(self.declare_parameter("debug_logging", False).value)
        self.warehouse_api_url = str(self.declare_parameter(
            "warehouse_api_url", "http://127.0.0.1:8000").value).rstrip("/")
        default_env = str(Path.home() / "ros2_ws/src/warehouse_robot_project/warehouse_bot/warehouse_backend/.env")
        self.warehouse_env_file = Path(str(self.declare_parameter(
            "warehouse_env_file", default_env).value)).expanduser()
        self.qr_camera_topic = str(self.declare_parameter(
            "qr_camera_topic", "/so101/follower/camera2/image/compressed").value)
        self.warehouse_operator_token = os.getenv("MARS_OPERATOR_API_TOKEN", "").strip()
        if not self.warehouse_operator_token:
            self.warehouse_operator_token = self._read_env_value(
                self.warehouse_env_file, "MARS_OPERATOR_API_TOKEN")

        # Action and status publishers
        self.action_pub = self.create_publisher(SO101Action, "/so101/leader/action", 1)
        diagnostics_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/so101/leader/diagnostics", diagnostics_qos)
        session_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.session_pub = self.create_publisher(Bool, "/so101/leader/teleop_enabled", session_qos)
        mic_control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.laptop_mic_pub = self.create_publisher(
            Bool, "/mars/audio/laptop_mic_enabled", mic_control_qos)
        self.laptop_mic_enabled = False

        # Subscriptions
        self.diag_sub = self.create_subscription(
            DiagnosticArray, "/so101/follower/diagnostics", self._on_diagnostics, 10
        )
        self.follower_parameters = AsyncParameterClient(self, "/so101/follower")
        self.leader_reconnect_client = self.create_client(
            Trigger, "/so101/leader_state/reconnect")
        self.feedback_sub = self.create_subscription(
            SO101Action, "/so101/follower/action_feedback", self._on_follower_feedback, 1
        )

        # Hardware joint_states subscription for ros2_control integration
        self.joint_states_sub = self.create_subscription(
            JointState, "/so101/leader/joint_states", self._on_leader_joint_states, 1
        )
        self.driver_diagnostics_sub = self.create_subscription(
            DiagnosticArray, "/so101/leader/driver_diagnostics", self._on_driver_diagnostics, 10
        )
        self._last_pi_speaker_connected: bool | None = None
        speaker_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.speaker_status_sub = self.create_subscription(
            Bool, "/mars/audio/speaker_connected", self._on_pi_speaker_status,
            speaker_status_qos)
        self.lock = threading.RLock()
        self.connected = False
        self.connecting = False
        self.calibration = validate_leader_calibration(self.leader_calibration_file)
        self.follower_calibration = validate_follower_calibration(self.follower_calibration_file)
        self.raw_positions: dict[str, int] = {}
        self._last_out_of_calibration: tuple[str, ...] | None = None
        self.last_leader_state = 0.0
        self.last_leader_feedback_gap_sec = 0.0
        self.max_leader_feedback_gap_sec = 0.0
        self.last_action_timer_callback = 0.0
        self.last_action_timer_interval_sec = 0.0
        self.max_action_timer_interval_sec = 0.0
        self.last_timing_summary = time.monotonic()
        self.feedback_pause_reason = ""
        self.teleop = False
        self.sequence = 0
        self.last_publish = 0.0
        self.publish_count = 0
        self.last_error = ""
        self.last_follower_diag = 0.0
        self.last_follower_diagnostic_interval_sec = 0.0
        self.follower_diagnostics_age_sec = float("inf")
        self.follower_ready = False
        self.follower_message = "No follower diagnostic received"
        self.follower_diagnostics_stale = False
        self._leader_feedback_paused = False
        self._ready_announced = False
        self.last_diagnostic_issue_log = 0.0
        self.diagnostics_publish_count = 0
        self.diagnostics_publish_logged = False
        self.diagnostics_group = ReentrantCallbackGroup()
        self._warehouse_stop = threading.Event()
        self._warehouse_last_error = ""
        self._warehouse_last_tasks = None
        self._qr_scan_lock = threading.Lock()
        self._qr_scan_context: dict | None = None
        self._qr_scan_last_decode = 0.0
        self._qr_last_payload = ""
        self._qr_last_payload_time = 0.0
        self._qr_detector = cv2.QRCodeDetector()
        qr_scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.qr_scan_state_pub = self.create_publisher(
            Bool, "/mars/warehouse/qr_scan_active", qr_scan_qos)
        qr_camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.qr_camera_sub = self.create_subscription(
            CompressedImage, self.qr_camera_topic,
            self._on_rover_qr_camera, qr_camera_qos)

        # Telemetry storage
        self.leader_telemetry: list[float] = [0.0] * 6
        self.follower_telemetry: list[float] = [0.0] * 6

        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish_action)
        self.session_state_timer = self.create_timer(0.5, self._publish_session_state)
        self.laptop_mic_heartbeat_timer = self.create_timer(
            0.2, self._publish_laptop_mic_state)
        self.diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics,
                                                   callback_group=self.diagnostics_group)
        self._warehouse_thread = threading.Thread(
            target=self._warehouse_poll_loop, name="warehouse-task-poll", daemon=True)
        self._warehouse_thread.start()

        # Initial disarmed state publish
        self.session_pub.publish(Bool(data=False))
        self._publish_laptop_mic_state()

        self.log.info("node started port=%s rate=%.1fHz action_topic=/so101/leader/action", self.leader_port, self.rate_hz)
        self.log.info("leader calibration id=leader file=%s", self.leader_calibration_file)
        self.log.info("session topic=/so101/leader/teleop_enabled diagnostics=/so101/leader/diagnostics,/so101/follower/diagnostics")

    @staticmethod
    def _read_env_value(path: Path, key: str) -> str:
        try:
            for raw in path.read_text(encoding="utf-8").splitlines():
                line = raw.strip()
                if line.startswith(key + "="):
                    value = line.split("=", 1)[1].strip()
                    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                        value = value[1:-1]
                    return value
        except OSError:
            pass
        return ""

    def _warehouse_request(self, path: str, method: str = "GET", headers: dict | None = None,
                           body: dict | None = None):
        request_headers = {
            "Authorization": f"Bearer {self.warehouse_operator_token}",
            "Accept": "application/json",
        }
        request_headers.update(headers or {})
        payload = json.dumps(body).encode("utf-8") if body is not None else None
        if payload is not None:
            request_headers["Content-Type"] = "application/json"
        request = urllib.request.Request(
            self.warehouse_api_url + path, data=payload, headers=request_headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=3.0) as response:
                payload = response.read()
        except urllib.error.HTTPError as error:
            detail = error.read(1024).decode("utf-8", errors="replace")
            raise RuntimeError(f"Warehouse API returned {error.code}: {detail}") from error
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"Warehouse API unavailable: {error}") from error
        return json.loads(payload.decode("utf-8")) if payload else {}

    def _warehouse_poll_loop(self) -> None:
        if not self.warehouse_operator_token:
            self.events.put(("warehouse_status", "Operator key not loaded; check the warehouse .env file."))
            return
        while not self._warehouse_stop.is_set():
            try:
                tasks = self._warehouse_request("/api/v1/operator/tasks")
                if tasks != self._warehouse_last_tasks:
                    self._warehouse_last_tasks = tasks
                    self.events.put(("warehouse_tasks", tasks))
                if self._warehouse_last_error:
                    self._warehouse_last_error = ""
                    self.events.put(("warehouse_status", "Warehouse connected"))
            except Exception as error:
                detail = str(error)
                if detail != self._warehouse_last_error:
                    self._warehouse_last_error = detail
                    self.events.put(("warehouse_status", detail))
            self._warehouse_stop.wait(2.5)

    def _warehouse_action(self, path: str, headers: dict, success: str) -> None:
        try:
            self._warehouse_request(path, method="POST", headers=headers)
            self.events.put(("warehouse_action", (True, success)))
            self._warehouse_last_tasks = None
        except Exception as error:
            self.events.put(("warehouse_action", (False, str(error))))

    def confirm_warehouse_pick(self, order_id: str, line_id: str, description: str) -> None:
        threading.Thread(
            target=self._warehouse_action,
            args=(f"/api/v1/operator/tasks/{order_id}/picked",
                  {"X-Order-Line-Id": line_id}, description), daemon=True).start()

    def confirm_warehouse_drop(self, order_id: str, order_number: str) -> None:
        threading.Thread(
            target=self._warehouse_action,
            args=(f"/api/v1/operator/tasks/{order_id}/dropped", {},
                  f"Order {order_number} marked delivered."), daemon=True).start()

    def set_warehouse_qr_scan(self, active: bool) -> None:
        with self._qr_scan_lock:
            self._qr_scan_context = {} if active else None
            self._qr_scan_last_decode = 0.0
            self._qr_last_payload = ""
            self._qr_last_payload_time = 0.0
        status = ("Scanning Arm camera 2 for QR codes · align a tag inside the box."
                  if active else "QR scan stopped.")
        self.qr_scan_state_pub.publish(Bool(data=active))
        self.events.put(("qr_scan_status", (active, status)))

    def _on_rover_qr_camera(self, message: CompressedImage) -> None:
        now = time.monotonic()
        with self._qr_scan_lock:
            context = self._qr_scan_context
            if context is None or now - self._qr_scan_last_decode < 0.25:
                return
            self._qr_scan_last_decode = now
        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
            if frame is None:
                return
            # Match the visible aiming box so adjacent labels or other tags are not
            # considered. QRCodeDetector reads QR symbols only; it does not OCR text.
            height, width = frame.shape[:2]
            box_w, box_h = int(width * 0.62), int(height * 0.68)
            x1, y1 = (width - box_w) // 2, (height - box_h) // 2
            roi = frame[y1:y1 + box_h, x1:x1 + box_w]
            enlarged = cv2.resize(roi, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(enlarged)
            decoded = ""
            for candidate in (roi, enlarged, enhanced,
                              cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                   cv2.THRESH_BINARY, 31, 7)):
                decoded, _points, _straight = self._qr_detector.detectAndDecode(candidate)
                if decoded:
                    break
            if not decoded:
                return
            with self._qr_scan_lock:
                if self._qr_scan_context is None:
                    return
                if decoded == self._qr_last_payload and now - self._qr_last_payload_time < 2.0:
                    return
                self._qr_last_payload = decoded
                self._qr_last_payload_time = now
            threading.Thread(target=self._resolve_scanned_qr, args=(decoded,), daemon=True).start()
        except Exception as error:
            self.log.debug("Rover QR decode failed: %s", error)

    def _resolve_scanned_qr(self, payload: str) -> None:
        try:
            result = self._warehouse_request(
                "/api/v1/operator/qr/resolve", method="POST", body={"identifier": payload})
            self.events.put(("qr_scan_result", (bool(result.get("found")), str(result.get("name", "Unknown item")))))
        except Exception as error:
            self.events.put(("qr_scan_error", str(error)))

    def destroy_node(self):
        self._warehouse_stop.set()
        with self._qr_scan_lock:
            self._qr_scan_context = None
        self.qr_scan_state_pub.publish(Bool(data=False))
        if getattr(self, "_warehouse_thread", None) and self._warehouse_thread.is_alive():
            self._warehouse_thread.join(timeout=0.5)
        return super().destroy_node()

    def _on_leader_joint_states(self, msg: JointState) -> None:
        if (tuple(msg.name) != JOINT_NAMES or len(msg.position) != len(JOINT_NAMES) or
                any(not math.isfinite(value) or value != int(value) or value < 0 or value > 4095
                    for value in msg.position)):
            self._leader_feedback_fault(
                f"invalid leader feedback structure/counts: names={msg.name} positions={list(msg.position)}",
                callback="_on_leader_joint_states")
            return
        received_at = time.monotonic()
        if self.last_leader_state > 0.0:
            gap = received_at - self.last_leader_state
            self.last_leader_feedback_gap_sec = gap
            self.max_leader_feedback_gap_sec = max(self.max_leader_feedback_gap_sec, gap)
            if gap > self.feedback_pause_timeout_sec:
                self.log.warning("leader feedback callback gap=%.3fs (action pause=%.3fs)",
                                 gap, self.feedback_pause_timeout_sec)
        self.raw_positions = {name: int(value) for name, value in zip(msg.name, msg.position, strict=True)}
        self.last_leader_state = received_at
        self.connecting = False
        self.feedback_pause_reason = ""
        self._leader_feedback_paused = False
        out_of_calibration = tuple(
            name for name in JOINT_NAMES
            if not self.calibration[name]["range_min"] <= self.raw_positions[name] <= self.calibration[name]["range_max"]
        )
        if out_of_calibration != self._last_out_of_calibration:
            previous_out_of_calibration = self._last_out_of_calibration or ()
            mapped_actions = {
                name: leader_count_to_follower_action(
                    name, self.raw_positions[name], self.calibration, self.follower_calibration)
                for name in JOINT_NAMES
            }
            leader_bounds = {name: (self.calibration[name]["range_min"], self.calibration[name]["range_max"])
                             for name in JOINT_NAMES}
            follower_bounds = {
                name: (self.follower_calibration[name]["range_min"], self.follower_calibration[name]["range_max"])
                for name in JOINT_NAMES
            }
            self.log.info(
                "leader calibration-range transition out_of_range=%s raw_counts=%s leader_bounds=%s "
                "follower_bounds=%s mapped_follower_actions=%s feedback_age=0.000s session_active=%s",
                out_of_calibration, self.raw_positions, leader_bounds, follower_bounds, mapped_actions,
                self.teleop)
            self._last_out_of_calibration = out_of_calibration
            if out_of_calibration and not previous_out_of_calibration and self.teleop:
                self._operator_log("LIMITED — Follower target constrained to joint limits", logging.WARNING)
        if not self.connected:
            self.connected = True
            self.last_error = ""
            self.events.put(("leader_connected", self.leader_calibration_file))
            self.log.info("read-only C++ driver feedback detected")

    def _leader_feedback_fault(self, reason: str, callback: str = "_leader_feedback_fault") -> None:
        self.last_error = reason
        self.connected = False
        self.feedback_pause_reason = ""
        self.log.error("invalid leader feedback callback=%s detail=%s", callback, reason)
        self.stop_teleop(f"callback={callback}; leader feedback fault: {reason}")
        self.events.put(("leader_disconnected", None))
        self.events.put(("error", "Leader feedback fault: " + reason))

    def _on_driver_diagnostics(self, msg: DiagnosticArray) -> None:
        status = next((item for item in msg.status if item.name == "feetech_read_only_state"), None)
        if status is None:
            return
        if status.message.startswith("RECOVERABLE_LEADER_FEEDBACK_LOSS:") and status.level == DiagnosticStatus.WARN:
            self.feedback_pause_reason = "PAUSED — WAITING FOR LEADER FEEDBACK"
            self.log.warning("recoverable leader driver diagnostic: %s", status.message)
            self.events.put(("feedback_paused", status.message))
            return
        if status.message.startswith("FATAL_LEADER_DRIVER_FAULT:") or status.level >= DiagnosticStatus.ERROR:
            self.last_error = status.message
            self.connected = False
            self.connecting = False
            self.log.error("fatal leader driver diagnostic: %s", status.message)
            self.stop_teleop("callback=_on_driver_diagnostics; fatal driver diagnostic: " + status.message)
            self.events.put(("leader_disconnected", None))
            self.events.put(("error", "Leader driver: " + status.message))

    def _on_pi_speaker_status(self, msg: Bool) -> None:
        connected = bool(msg.data)
        if connected == self._last_pi_speaker_connected:
            return
        self._last_pi_speaker_connected = connected
        if connected:
            self._operator_log("Pi Bluetooth speaker connected", logging.INFO)
        else:
            self._operator_log(
                "ALERT — Pi Bluetooth speaker disconnected; rover audio playback is unavailable",
                logging.WARNING,
            )

    def set_laptop_mic_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self.laptop_mic_enabled:
            return
        self.laptop_mic_enabled = enabled
        self._publish_laptop_mic_state()
        self._operator_log(
            "Laptop mic to rover enabled — hold to talk" if enabled
            else "Laptop mic to rover disabled",
            logging.INFO,
        )

    def _publish_laptop_mic_state(self) -> None:
        self.laptop_mic_pub.publish(Bool(data=self.laptop_mic_enabled))

    def connect_leader(self, port: str) -> None:
        if self.connecting:
            return
        self.connecting = True
        self.leader_port = port
        self.log.info("requesting read-only C++ Feetech driver on %s", port)
        if self.last_leader_state and time.monotonic() - self.last_leader_state < 1.0:
            self.connected = True
            self.connecting = False
            self.events.put(("leader_connected", self.leader_calibration_file))
        else:
            if not self.leader_reconnect_client.service_is_ready():
                self.connecting = False
                self.events.put(("error", "Leader reconnect service is unavailable; restart the leader launch"))
                return
            future = self.leader_reconnect_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_reconnect_service_response)

    def _on_reconnect_service_response(self, future) -> None:
        try:
            response = future.result()
            if not response.success:
                self.connecting = False
                self.events.put(("reconnect_requested", (False, response.message)))
                return
            self.events.put(("reconnect_requested", (True, response.message)))
        except Exception as error:
            self.connecting = False
            self.events.put(("reconnect_requested", (False, str(error))))

    def disconnect_leader(self) -> None:
        self.stop_teleop("callback=disconnect_leader; leader disconnect requested")
        self.connected = False
        self.events.put(("leader_disconnected", None))

    def start_teleop(self) -> None:
        with self.lock:
            if not self.connected:
                self.events.put(("error", "Connect the leader hardware first"))
                return
            if time.monotonic() - self.last_leader_state > self.feedback_pause_timeout_sec:
                self.events.put(("error", "Leader feedback is not fresh; wait for a current joint state"))
                return
            if not self.follower_is_ready():
                self.events.put(("error", f"Follower is not ready: {self.follower_message}"))
                return
            if self.teleop:
                return
            self.teleop = True
            self.publish_count = 0
            self.last_publish = time.monotonic()
            self.feedback_pause_reason = ""
        self.session_pub.publish(Bool(data=True))
        self.log.info("teleoperation started; rate=%.1fHz", self.rate_hz)
        self._operator_log("READY — Initial checks passed")
        self._operator_log("STARTED — Teleoperation active")
        self.events.put(("teleop", True))

    def stop_teleop(self, reason: str) -> None:
        with self.lock:
            was_active = self.teleop
            self.teleop = False
        if was_active:
            out_of_range = {
                name: self.raw_positions[name]
                for name in JOINT_NAMES
                if name in self.raw_positions and not (
                    self.calibration[name]["range_min"] <= self.raw_positions[name] <=
                    self.calibration[name]["range_max"])
            }
            self.log.warning(
                "session transition caller=%s previous_session_active=true new_session_active=false "
                "leader_calibration_violation=%s raw_counts=%s publish_false=/so101/leader/teleop_enabled",
                reason, out_of_range, dict(self.raw_positions))
            self._operator_log(self._operator_stop_message(reason), logging.WARNING)
            self.session_pub.publish(Bool(data=False))
            feedback_age = max(0.0, time.monotonic() - self.last_leader_state)
            raw_positions = dict(self.raw_positions)
            leader_bounds = {
                name: (self.calibration[name]["range_min"], self.calibration[name]["range_max"])
                for name in JOINT_NAMES
            }
            follower_bounds = {
                name: (self.follower_calibration[name]["range_min"], self.follower_calibration[name]["range_max"])
                for name in JOINT_NAMES
            }
            try:
                mapped_actions = {
                    name: leader_count_to_follower_action(
                        name, raw_positions[name], self.calibration, self.follower_calibration)
                    for name in JOINT_NAMES
                }
            except (KeyError, ValueError):
                mapped_actions = "unavailable: raw feedback or calibration invalid"
            self.log.warning(
                "teleoperation stopped trigger=%s actions_published=%d feedback_age=%.3fs raw_counts=%s "
                "leader_bounds=%s follower_bounds=%s mapped_follower_actions=%s",
                reason, self.publish_count, feedback_age, raw_positions, leader_bounds,
                follower_bounds, mapped_actions)
        self.events.put(("teleop", False))

    def _publish_action(self) -> None:
        now = time.monotonic()
        if self.last_action_timer_callback > 0.0:
            interval = now - self.last_action_timer_callback
            self.last_action_timer_interval_sec = interval
            self.max_action_timer_interval_sec = max(self.max_action_timer_interval_sec, interval)
        self.last_action_timer_callback = now
        with self.lock:
            teleop_enabled = self.teleop
        feedback_age = now - self.last_leader_state
        if now - self.last_timing_summary >= 5.0:
            diagnostics_age = now - self.last_follower_diag if self.last_follower_diag else float("inf")
            self.log.info(
                "timing teleop_enabled=%s action_timer_target=%.1fHz interval=%.3fs(max=%.3fs) leader_feedback_age=%.3fs "
                "leader_feedback_gap=%.3fs(max=%.3fs) follower_diagnostics_age=%.3fs",
                teleop_enabled, self.rate_hz, self.last_action_timer_interval_sec, self.max_action_timer_interval_sec,
                feedback_age, self.last_leader_feedback_gap_sec, self.max_leader_feedback_gap_sec,
                diagnostics_age)
            self.last_timing_summary = now
        with self.lock:
            if not self.teleop or not self.connected:
                return
        if feedback_age > self.feedback_pause_timeout_sec:
            self.feedback_pause_reason = (
                f"PAUSED — WAITING FOR LEADER FEEDBACK (age {feedback_age:.2f}s)")
            if not getattr(self, "_leader_feedback_paused", False):
                self._leader_feedback_paused = True
                self._operator_log("PAUSED — Waiting for leader feedback", logging.WARNING)
            return
        if getattr(self, "_leader_feedback_paused", False):
            self._leader_feedback_paused = False
            self.feedback_pause_reason = ""
            self._operator_log("RESUMED — Leader feedback restored")
        else:
            self.feedback_pause_reason = ""
        started = time.perf_counter()
        try:
            values = []
            for name in JOINT_NAMES:
                value = self.raw_positions[name]
                values.append(leader_count_to_follower_action(
                    name, value, self.calibration, self.follower_calibration))
            if not all(math.isfinite(v) for v in values):
                raise ValueError("conversion returned a non-finite action")

            msg = SO101Action()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.sequence = self.sequence
            msg.joint_names = list(JOINT_NAMES)
            msg.values = values
            self.action_pub.publish(msg)

            with self.lock:
                self.leader_telemetry = list(values)
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            self.publish_count += 1

            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if self.debug:
                self.log.debug("action sequence=%d values=%s ms=%.2f", msg.sequence, values, elapsed_ms)
            if now - self.last_publish >= 5.0:
                if self.publish_count:
                    self.log.info("action publication rate=%.1fHz", self.publish_count / max(now - self.last_publish, 0.001))
                self.publish_count = 0
                self.last_publish = now
        except Exception as exc:
            self.last_error = str(exc)
            self.log.exception("teleoperation stopped due to read error: %s", exc)
            self.stop_teleop(f"callback=_publish_action; leader read failure: {exc}")
            self.events.put(("error", f"Teleoperation stopped: {exc}"))

    def _publish_session_state(self) -> None:
        with self.lock:
            enabled = self.teleop
        self.session_pub.publish(Bool(data=enabled))

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        receipt = time.monotonic()
        self.last_follower_diagnostic_interval_sec = (
            receipt - self.last_follower_diag if self.last_follower_diag else 0.0)
        self.last_follower_diag = receipt
        self.follower_diagnostics_age_sec = 0.0
        was_stale = getattr(self, "follower_diagnostics_stale", False)
        self.follower_diagnostics_stale = False
        ready, description = parse_follower_diagnostics(msg)
        self.follower_ready = ready
        self.follower_message = description
        if self.teleop and not ready:
            self.stop_teleop("callback=_on_diagnostics; follower readiness lost: " + description)
        elif not ready:
            self._log_diagnostic_issue(f"follower readiness is not confirmed: {description}")
        if was_stale:
            if ready:
                self._operator_log("RESUMED — Follower diagnostics restored")
            else:
                self._operator_log("WARNING — Follower diagnostics returned; readiness not confirmed", logging.WARNING)
        self.events.put(("follower", (ready, description)))

    def _on_follower_feedback(self, msg: SO101Action) -> None:
        with self.lock:
            if len(msg.values) == 6:
                self.follower_telemetry = list(msg.values)
        self.events.put(("follower_telemetry", list(msg.values)))

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "/so101/leader"
        status.hardware_id = str(self.leader_port)
        status.level = DiagnosticStatus.OK if self.connected else DiagnosticStatus.WARN
        if self.last_error:
            status.message = self.last_error
        elif self.teleop:
            status.message = "teleoperation active"
        else:
            status.message = "connected; teleoperation stopped" if self.connected else "leader disconnected"
        array.status.append(status)
        self.diagnostics_pub.publish(array)

    def set_debug(self, enabled: bool) -> None:
        self.debug = enabled
        self.log.info("debug logging %s", "enabled" if enabled else "disabled")

    def _on_follower_debug_update(self, future) -> None:
        pass

    def follower_is_ready(self) -> bool:
        age = time.monotonic() - self.last_follower_diag if self.last_follower_diag else float("inf")
        self.follower_diagnostics_age_sec = age
        stale = age > DIAGNOSTIC_STALE_SECONDS
        if stale and not self.follower_diagnostics_stale:
            self.follower_diagnostics_stale = True
            detail = f"follower diagnostics stale age={age:.1f}s limit={DIAGNOSTIC_STALE_SECONDS:.1f}s"
            self._log_diagnostic_issue(detail)
            if self.teleop:
                self.stop_teleop("callback=follower_is_ready; follower session heartbeat lost")
                self._operator_log("DISARMED — Follower session heartbeat lost", logging.ERROR)
        return self.follower_ready and not stale

    def _operator_log(self, message: str, level: int = logging.INFO) -> None:
        self.log.log(level, message, extra={"operator_event": True})

    @staticmethod
    def _operator_stop_message(reason: str) -> str:
        lower = reason.lower()
        if "stop button" in lower or "user pressed stop" in lower:
            return "STOPPED — User requested stop"
        if "heartbeat stale" in lower:
            return "DISARMED — Session heartbeat lost"
        if "fatal driver" in lower or "serial" in lower:
            return "DISARMED — Leader serial connection failed"
        if "invalid leader feedback" in lower or "feedback fault" in lower:
            return "DISARMED — Leader feedback invalid"
        if "follower" in lower or "torque" in lower:
            return "DISARMED — Follower hardware fault confirmed"
        return "STOPPED — Teleoperation session ended"

    def _log_diagnostic_issue(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_diagnostic_issue_log >= 5.0:
            self.log.warning(message)
            self.last_diagnostic_issue_log = now

    def ping_follower(self, ip: str) -> None:
        try:
            res = subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True, timeout=2.0)
            reachable = res.returncode == 0
        except Exception:
            reachable = False
        self.events.put(("ping", (ip, reachable)))

class LeaderGui:
    """Arm teleoperation and warehouse controls; cameras run in a separate viewer."""

    def __init__(self, root: tk.Tk, node: LeaderNode):
        self.root, self.node = root, node
        self.log = logging.getLogger("so101.leader.gui")
        self._last_stop_input_source = ""
        self._last_stop_input_time = 0.0
        self._warehouse_tasks: list[dict] = []
        self._qr_scan_active = False
        self._font_resize_after: str | None = None
        self.root.title("MARS | SO-101 Teleoperation")
        self.root.geometry("1180x860")
        self.root.minsize(980, 680)
        self.root.configure(bg="#101820")
        self.events: queue.Queue[tuple[str, object]] = node.events

        self.bg_main = "#101820"
        self.bg_card = "#182631"
        self.bg_tile = "#0c141b"
        self.bg_input = "#243744"
        self.fg_text = "#e5edf2"
        self.fg_muted = "#9eafb9"
        self.accent_blue = "#70c5d9"
        self.accent_green = "#6fce9a"
        self.accent_red = "#ff8585"
        self.accent_amber = "#efc875"

        self._build_modern_ui()
        self.root.bind("<Configure>", self._schedule_responsive_fonts, add="+")
        self._apply_responsive_fonts()
        self._poll_queues()
        self._refresh_status()
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _schedule_responsive_fonts(self, event: tk.Event) -> None:
        if event.widget is not self.root:
            return
        if self._font_resize_after is not None:
            try:
                self.root.after_cancel(self._font_resize_after)
            except tk.TclError:
                pass
        self._font_resize_after = self.root.after(100, self._apply_responsive_fonts)

    def _apply_responsive_fonts(self) -> None:
        self._font_resize_after = None
        width = max(1, self.root.winfo_width())
        height = max(1, self.root.winfo_height())
        scale = max(1.5456, min(2.2356, min(width / 1180, height / 860) * 1.725))
        self._responsive_scale = scale
        style = ttk.Style(self.root)
        default_font = tkfont.nametofont("TkDefaultFont", root=self.root)

        def apply(widget: tk.Misc) -> None:
            try:
                spec = widget.cget("font")
            except tk.TclError:
                spec = ""
            if not spec:
                try:
                    style_name = widget.cget("style") or widget.winfo_class()
                    spec = style.lookup(style_name, "font") or default_font
                except (tk.TclError, TypeError):
                    spec = default_font
            if not hasattr(widget, "_warehouse_bot_font"):
                try:
                    actual = tkfont.Font(root=self.root, font=spec).actual()
                    widget._warehouse_bot_font = (
                        actual["family"], max(8, abs(int(actual["size"]))),
                        actual["weight"], actual["slant"], actual["underline"],
                        actual["overstrike"],
                    )
                    wrap = widget.cget("wraplength")
                    widget._warehouse_bot_wraplength = int(wrap) if str(wrap).isdigit() and int(wrap) > 0 else None
                except (tk.TclError, TypeError, ValueError):
                    widget._warehouse_bot_font = None
                    widget._warehouse_bot_wraplength = None
            base = getattr(widget, "_warehouse_bot_font", None)
            if base:
                family, size, weight, slant, underline, overstrike = base
                # Apply the requested increase to the actual widget font size.
                # The responsive factor above only adapts it to window dimensions.
                scaled_size = max(10, round(size * scale * 1.2))
                try:
                    widget.configure(font=(family, scaled_size, weight, slant, underline, overstrike))
                    wrap = getattr(widget, "_warehouse_bot_wraplength", None)
                    if wrap:
                        widget.configure(wraplength=max(120, round(wrap * scale)))
                except tk.TclError:
                    pass
            for child in widget.winfo_children():
                apply(child)

        apply(self.root)

    def _build_modern_ui(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=self.bg_main)
        style.configure("Card.TFrame", background=self.bg_card, relief="flat")
        style.configure("TLabel", background=self.bg_main, foreground=self.fg_text, font=("Sans", 10))
        style.configure("Card.TLabel", background=self.bg_card, foreground=self.fg_text, font=("Sans", 10))
        style.configure("Title.TLabel", background=self.bg_card, foreground=self.accent_blue,
                        font=("Sans", 11, "bold"))
        style.configure("Muted.Card.TLabel", background=self.bg_card, foreground=self.fg_muted,
                        font=("Sans", 9))
        style.configure("Badge.TLabel", background="#203d37", foreground=self.accent_green,
                        font=("Sans", 9, "bold"))
        style.configure("ErrorBadge.TLabel", background="#472a30", foreground=self.accent_red,
                        font=("Sans", 9, "bold"))
        style.map("TButton", background=[("active", self.accent_blue), ("disabled", self.bg_input)])

        main_box = ttk.Frame(self.root, padding=14)
        main_box.pack(fill="both", expand=True)
        header = ttk.Frame(main_box)
        header.pack(fill="x", pady=(0, 12))
        ttk.Label(header, text="MARS  /  SO-101", font=("Sans", 18, "bold"),
                  foreground=self.accent_blue).pack(side="left")
        ttk.Label(header, text="TELEOPERATION CONSOLE", font=("Sans", 9, "bold"),
                  foreground=self.fg_muted).pack(side="left", padx=(14, 0), pady=(5, 0))
        self.open_warehouse_button = tk.Button(
            header, text="Open Warehouse", command=self._open_warehouse,
            bg=self.bg_input, fg=self.fg_text, activebackground=self.accent_blue,
            activeforeground=self.bg_main, font=("Sans", 9, "bold"), relief="flat", padx=12, pady=7)
        self.open_warehouse_button.pack(side="right", padx=(10, 0))
        self.system_status_label = ttk.Label(header, text="● DISARMED", style="ErrorBadge.TLabel",
                                             padding=(10, 6))
        self.system_status_label.pack(side="right", padx=4)
        self.leader_status_badge = ttk.Label(header, text="LEADER · DISCONNECTED",
                                             style="ErrorBadge.TLabel", padding=(10, 6))
        self.leader_status_badge.pack(side="right", padx=4)
        self.follower_status_badge = ttk.Label(header, text="FOLLOWER · NOT READY",
                                               style="ErrorBadge.TLabel", padding=(10, 6))
        self.follower_status_badge.pack(side="right", padx=4)

        content = ttk.Frame(main_box)
        content.pack(fill="both", expand=True)
        content.grid_columnconfigure(0, weight=5, minsize=420)
        content.grid_columnconfigure(1, weight=6, minsize=500)
        content.grid_rowconfigure(0, weight=1)
        left = ttk.Frame(content)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(content)
        right.grid(row=0, column=1, sticky="nsew")

        controls = ttk.Frame(left, style="Card.TFrame", padding=16)
        controls.pack(fill="x", pady=(0, 9))
        controls_top = ttk.Frame(controls, style="Card.TFrame")
        controls_top.pack(fill="x")
        ttk.Label(controls_top, text="Teleoperation", style="Title.TLabel").pack(side="left")
        self.follower_label = ttk.Label(controls, text="Waiting for follower diagnostics",
                                        style="Muted.Card.TLabel", wraplength=360)
        self.follower_label.pack(anchor="w", pady=(5, 8))
        buttons = ttk.Frame(controls, style="Card.TFrame")
        buttons.pack(fill="x")
        self.connect_button = tk.Button(
            buttons, text="Connect leader", command=self._connect,
            bg=self.bg_input, fg=self.fg_text, activebackground=self.accent_blue,
            font=("Sans", 9, "bold"), relief="flat", padx=10, pady=8)
        self.connect_button.pack(side="left", padx=(0, 6))
        self.start_button = tk.Button(
            buttons, text="Start teleoperation", command=self.node.start_teleop,
            bg=self.bg_input, fg=self.fg_muted, activebackground=self.accent_green,
            font=("Sans", 9, "bold"), relief="flat", padx=12, pady=8, state="disabled")
        self.start_button.pack(side="left", padx=4)
        self.stop_button = tk.Button(
            buttons, text="Stop", command=self._stop_from_gui,
            bg="#69343a", fg="#fff0f0", activebackground=self.accent_red,
            font=("Sans", 9, "bold"), relief="flat", padx=14, pady=8, state="disabled")
        self.stop_button.pack(side="left", padx=4)
        self.stop_button.bind("<ButtonPress-1>",
                              lambda event: self._record_stop_input("mouse", event), add="+")
        self.stop_button.bind("<KeyPress-space>",
                              lambda event: self._record_stop_input("keyboard-space", event), add="+")
        self.stop_button.bind("<KeyPress-Return>",
                              lambda event: self._record_stop_input("keyboard-return", event), add="+")
        self.mic_button = tk.Button(
            buttons, text="Hold to talk", command=None, bg=self.bg_input, fg=self.fg_text,
            activebackground=self.accent_blue, font=("Sans", 9, "bold"), relief="flat", padx=10, pady=8)
        self.mic_button.pack(side="right")
        self.mic_button.bind("<ButtonPress-1>", self._start_laptop_mic)
        self.mic_button.bind("<ButtonRelease-1>", self._stop_laptop_mic)
        self.mic_button.bind("<Leave>", self._stop_laptop_mic)
        self.mic_button.bind("<KeyPress-space>", self._start_laptop_mic)
        self.mic_button.bind("<KeyRelease-space>", self._stop_laptop_mic)
        self.leader_label = ttk.Label(controls, text="Leader feedback not connected",
                                      style="Muted.Card.TLabel")
        self.leader_label.pack(anchor="w", pady=(8, 0))

        camera_note = ttk.Frame(left, style="Card.TFrame", padding=16)
        camera_note.pack(fill="x", pady=(0, 9))
        ttk.Label(camera_note, text="Camera viewer", style="Title.TLabel").pack(anchor="w")
        ttk.Label(camera_note, text="The three live feeds open in a separate window, so video decoding stays out of the teleoperation controls.",
                  style="Muted.Card.TLabel", wraplength=360, justify="left").pack(anchor="w", pady=(7, 10))

        telemetry = ttk.Frame(left, style="Card.TFrame", padding=12)
        telemetry.pack(fill="both", expand=True)
        ttk.Label(telemetry, text="Joint tracking", style="Title.TLabel").pack(anchor="w", pady=(0, 5))

        warehouse = ttk.Frame(right, style="Card.TFrame", padding=14)
        warehouse.pack(fill="both", expand=True, pady=(0, 9))
        warehouse_header = ttk.Frame(warehouse, style="Card.TFrame")
        warehouse_header.pack(fill="x", pady=(0, 5))
        ttk.Label(warehouse_header, text="Warehouse tasks", style="Title.TLabel").pack(side="left")
        self.qr_scan_button = tk.Button(
            warehouse_header, text="Scan item QR", command=self._toggle_warehouse_qr_scan,
            bg=self.bg_input, fg=self.fg_text, activebackground=self.accent_blue,
            relief="flat", font=("Sans", 9, "bold"), padx=10, pady=5)
        self.qr_scan_button.pack(side="right", padx=(8, 0))
        self.warehouse_status = ttk.Label(warehouse_header, text="Connecting…", style="Muted.Card.TLabel")
        self.warehouse_status.pack(side="right")
        self.qr_scan_status = ttk.Label(
            warehouse, text="QR scanner idle · point Arm camera 2 at an item tag.",
            style="Muted.Card.TLabel", wraplength=520, justify="left")
        self.qr_scan_status.pack(fill="x", anchor="w", pady=(3, 8))
        self.task_canvas = tk.Canvas(warehouse, bg=self.bg_card, highlightthickness=0, borderwidth=0)
        task_scroll = ttk.Scrollbar(warehouse, orient="vertical", command=self.task_canvas.yview)
        self.task_canvas.configure(yscrollcommand=task_scroll.set)
        self.task_list = ttk.Frame(self.task_canvas, style="Card.TFrame")
        self.task_window = self.task_canvas.create_window((0, 0), window=self.task_list, anchor="nw")
        self.task_list.bind("<Configure>", lambda _e: self.task_canvas.configure(
            scrollregion=self.task_canvas.bbox("all")))
        self.task_canvas.bind("<Configure>", lambda event: self.task_canvas.itemconfigure(
            self.task_window, width=event.width))
        self.task_canvas.pack(side="left", fill="both", expand=True)
        task_scroll.pack(side="right", fill="y")
        self._render_warehouse_tasks([])

        heading = ttk.Frame(telemetry, style="Card.TFrame")
        heading.pack(fill="x")
        for label, width in (("Joint", 12), ("Leader", 10), ("Follower", 10), ("Δ", 8)):
            ttk.Label(heading, text=label, width=width, style="Muted.Card.TLabel").pack(side="left")
        self.joint_rows = {}
        for name in JOINT_NAMES:
            row = ttk.Frame(telemetry, style="Card.TFrame")
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=name.replace("_", " ").title(), width=12,
                      style="Muted.Card.TLabel").pack(side="left")
            leader = ttk.Label(row, text="--", width=10, style="Card.TLabel", font=("Monospace", 9))
            follower = ttk.Label(row, text="--", width=10, style="Card.TLabel", font=("Monospace", 9))
            error = ttk.Label(row, text="--", width=8, style="Card.TLabel", font=("Monospace", 9))
            leader.pack(side="left")
            follower.pack(side="left")
            error.pack(side="left")
            self.joint_rows[name] = {"leader": leader, "follower": follower, "error": error}

        footer = ttk.Frame(main_box, style="Card.TFrame", padding=(10, 7))
        footer.pack(fill="x", pady=(10, 0))
        self.status_var = tk.StringVar(value="Starting safely disarmed")
        ttk.Label(footer, textvariable=self.status_var, style="Card.TLabel",
                  font=("Sans", 9, "bold")).pack(side="left")
        ttk.Label(footer, text="Logs are saved to ~/.ros/so101_lerobot_leader.log",
                  style="Muted.Card.TLabel").pack(side="right")

    def _open_warehouse(self) -> None:
        script = Path.home() / "ros2_ws/src/warehouse_robot_project/warehouse_bot/warehouse_backend/run_server.py"
        python = Path.home() / "python_envs/general/bin/python"
        if not script.is_file():
            self.status_var.set(f"Warehouse startup script not found: {script}")
            return
        if not python.exists():
            python = Path(os.sys.executable)
        log_path = Path.home() / ".ros/mars_warehouse_startup.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with log_path.open("a", encoding="utf-8") as output:
                subprocess.Popen([str(python), str(script)], stdin=subprocess.DEVNULL,
                                 stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True, close_fds=True)
            self.status_var.set("Starting warehouse service and opening the shop…")
        except OSError as error:
            self.status_var.set(f"Could not start warehouse service: {error}")

    def _render_warehouse_tasks(self, tasks: list[dict]) -> None:
        for child in self.task_list.winfo_children():
            child.destroy()
        visible = [entry for entry in tasks if entry.get("task", {}).get("state") not in
                   ("delivered", "completed", "cancelled")]
        if not visible:
            ttk.Label(self.task_list, text="No open warehouse tasks.",
                      style="Muted.Card.TLabel").pack(anchor="w", padx=4, pady=8)
            self._apply_responsive_fonts()
            return
        labels = {
            "queued": "Queued", "navigating_to_pick": "Going to Pick",
            "at_pick": "At Pick · operator action needed", "scanning": "At Pick · operator action needed",
            "waiting_for_operator": "At Pick · operator action needed",
            "navigating_to_drop": "Going to Drop", "at_drop": "At Drop · confirm delivery",
            "returning_to_dock": "Returning to Dock", "assistance_required": "Needs assistance",
        }
        for entry in visible[:12]:
            task = entry.get("task") or {}
            card = ttk.Frame(self.task_list, style="Card.TFrame", padding=(7, 6))
            card.pack(fill="x", pady=3)
            row = ttk.Frame(card, style="Card.TFrame")
            row.pack(fill="x")
            number = entry.get("order_number") or "—"
            ttk.Label(row, text=f"Order {number}", style="Card.TLabel",
                      font=("Sans", 10, "bold")).pack(side="left")
            ttk.Label(row, text=labels.get(task.get("state"), task.get("state", "Unknown")),
                      style="Muted.Card.TLabel").pack(side="right")
            items = entry.get("items") or []
            for item in items:
                picked = "Picked" if item.get("picked") else "Pending"
                suffix = " · current" if item.get("line_id") == task.get("current_line_id") else ""
                ttk.Label(card, text=f"{item.get('name', 'Item')} × {item.get('quantity', 0)}  ·  {picked}{suffix}",
                          style="Muted.Card.TLabel").pack(anchor="w", pady=(3, 0))
            state = task.get("state")
            current = next((item for item in items if item.get("line_id") == task.get("current_line_id")
                            and not item.get("picked")), None)
            if state in ("at_pick", "scanning", "waiting_for_operator") and current:
                description = f"Picked {current.get('quantity')} × {current.get('name')} for order {number}."
                actions = ttk.Frame(card, style="Card.TFrame")
                actions.pack(anchor="w", pady=(6, 0))
                button = tk.Button(
                    actions, text=f"Confirm picked · {current.get('name')}",
                    bg=self.accent_green, fg="#0d2119", activebackground="#89dfae",
                    relief="flat", font=("Sans", 9, "bold"), padx=10, pady=6,
                    command=lambda e=entry, i=current: self._confirm_pick(e, i))
                button.pack(side="left")
            elif state == "at_drop":
                button = tk.Button(
                    card, text="Confirm dropped", bg=self.accent_green, fg="#0d2119",
                    activebackground="#89dfae", relief="flat", font=("Sans", 9, "bold"),
                    padx=10, pady=6,
                    command=lambda e=entry: self._confirm_drop(e))
                button.pack(anchor="w", pady=(6, 0))
            if task.get("detail"):
                ttk.Label(card, text=str(task["detail"]), style="Muted.Card.TLabel",
                          wraplength=350).pack(anchor="w", pady=(4, 0))
        self._apply_responsive_fonts()

    def _toggle_warehouse_qr_scan(self) -> None:
        self._qr_scan_active = not self._qr_scan_active
        self.node.set_warehouse_qr_scan(self._qr_scan_active)

    def _confirm_pick(self, entry: dict, item: dict) -> None:
        number = entry.get("order_number", "")
        if not messagebox.askyesno(
                "Confirm item picked",
                f"Confirm you physically picked {item.get('quantity')} × {item.get('name')} for order {number}?"):
            return
        self.node.confirm_warehouse_pick(
            entry["order_id"], item["line_id"],
            f"{item.get('name')} confirmed as picked for order {number}.")
        self.warehouse_status.configure(text="Saving picked confirmation…")

    def _confirm_drop(self, entry: dict) -> None:
        number = entry.get("order_number", "")
        if not messagebox.askyesno("Confirm delivery", f"Confirm order {number} has physically arrived at Drop?"):
            return
        self.node.confirm_warehouse_drop(entry["order_id"], str(number))
        self.warehouse_status.configure(text="Saving delivery confirmation…")

    def _connect(self) -> None:
        if self.node.connected:
            self.node.disconnect_leader()
            return
        self.connect_button.configure(state="disabled")
        self.status_var.set("Connecting to leader…")
        self.node.connect_leader(self.node.leader_port)

    def _check_follower(self) -> None:
        ip = self.node.follower_ip
        threading.Thread(target=self.node.ping_follower, args=(ip,), daemon=True).start()

    def _start_laptop_mic(self, _event=None) -> None:
        self.node.set_laptop_mic_enabled(True)
        self.mic_button.configure(text="TALKING · release to mute", bg=self.accent_red, fg="#261113")
        return "break"

    def _stop_laptop_mic(self, _event=None) -> None:
        self.node.set_laptop_mic_enabled(False)
        self.mic_button.configure(text="Hold to talk", bg=self.bg_input, fg=self.fg_text)
        return "break"

    def _update_joint_telemetry(self) -> None:
        with self.node.lock:
            lead = list(self.node.leader_telemetry)
            foll = list(self.node.follower_telemetry)
        for index, name in enumerate(JOINT_NAMES):
            leader = lead[index] if index < len(lead) else 0.0
            follower = foll[index] if index < len(foll) else 0.0
            difference = abs(leader - follower)
            unit = "%" if name == "gripper" else "°"
            self.joint_rows[name]["leader"].configure(text=f"{leader:5.1f}{unit}")
            self.joint_rows[name]["follower"].configure(text=f"{follower:5.1f}{unit}")
            color = self.accent_red if difference > 15.0 else self.accent_amber if difference > 5.0 else self.accent_green
            self.joint_rows[name]["error"].configure(text=f"{difference:4.1f}{unit}", foreground=color)

    def _poll_queues(self) -> None:
        for _ in range(100):
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "leader_connected":
                self.leader_label.configure(text="Leader feedback connected", foreground=self.accent_green)
                self.leader_status_badge.configure(text="LEADER · CONNECTED", style="Badge.TLabel")
                self.connect_button.configure(text="Disconnect leader", state="normal")
            elif event == "leader_disconnected":
                self.leader_label.configure(text="Leader feedback disconnected", foreground=self.accent_red)
                self.leader_status_badge.configure(text="LEADER · FAULT", style="ErrorBadge.TLabel")
                self.connect_button.configure(text="Connect leader", state="normal")
            elif event == "reconnect_requested":
                success, detail = payload
                if success:
                    self.connect_button.configure(text="Reconnecting…", state="disabled")
                    self.leader_label.configure(text="Serial reconnect requested; waiting for feedback",
                                                foreground=self.accent_amber)
                    self.status_var.set(str(detail))
                else:
                    self.connect_button.configure(text="Connect leader", state="normal")
                    self.status_var.set(f"Reconnect failed: {detail}")
            elif event == "teleop":
                if payload:
                    self.status_var.set("Teleoperation active")
                elif not self.node.last_error:
                    self.status_var.set("Teleoperation stopped")
            elif event == "follower":
                ready, description = payload
                self.follower_label.configure(
                    text=description, foreground=self.accent_green if ready else self.accent_amber)
                self.follower_status_badge.configure(
                    text="FOLLOWER · READY" if ready else "FOLLOWER · NOT READY",
                    style="Badge.TLabel" if ready else "ErrorBadge.TLabel")
            elif event == "ping":
                ip, reachable = payload
                self.status_var.set(f"Follower {ip} reachable" if reachable else
                                    f"Follower {ip} did not answer ping; checking ROS diagnostics")
            elif event == "error":
                self.status_var.set(str(payload))
                if not self.node.connected and not self.node.connecting:
                    self.connect_button.configure(text="Connect leader", state="normal")
            elif event == "warehouse_tasks":
                self._warehouse_tasks = payload
                self._render_warehouse_tasks(payload)
                count = len([t for t in payload if t.get("task", {}).get("state") not in
                             ("delivered", "completed", "cancelled")])
                self.warehouse_status.configure(text=f"{count} open task{'s' if count != 1 else ''}")
            elif event == "warehouse_status":
                self.warehouse_status.configure(text=str(payload)[:50])
            elif event == "warehouse_action":
                success, detail = payload
                self.warehouse_status.configure(text=detail if success else "Action failed · refresh pending")
                if not success:
                    self.status_var.set(detail)
            elif event == "qr_scan_status":
                self._qr_scan_active, status = payload
                self.qr_scan_button.configure(
                    text="Stop scan" if self._qr_scan_active else "Scan item QR",
                    bg="#69343a" if self._qr_scan_active else self.bg_input)
                self.qr_scan_status.configure(text=status)
            elif event == "qr_scan_result":
                found, item_name = payload
                status = f"QR scanned · {item_name}" if found else "QR detected · item is not in the catalogue"
                if self._qr_scan_active:
                    status += " · scanner remains on until Stop scan"
                self.qr_scan_status.configure(text=status)
                self.status_var.set(f"QR scanned: {item_name}" if found else "Unknown QR item")
            elif event == "qr_scan_error":
                self.qr_scan_status.configure(text=f"QR decoded, but item lookup failed: {payload}")

        self._update_joint_telemetry()
        self.root.after(100, self._poll_queues)

    def _refresh_status(self) -> None:
        now = time.monotonic()
        feedback_age = now - self.node.last_leader_state if self.node.last_leader_state else float("inf")
        fresh_leader = self.node.connected and feedback_age <= self.node.feedback_pause_timeout_sec
        follower_ready = self.node.follower_is_ready()
        can_start = fresh_leader and follower_ready and not self.node.teleop
        if self.node.teleop:
            self.start_button.configure(text="Teleoperation active", state="disabled",
                                        bg=self.accent_green, fg="#0d2119", activebackground=self.accent_green)
            self.stop_button.configure(state="normal")
            if self.node.feedback_pause_reason:
                self.system_status_label.configure(text="● PAUSED · LEADER FEEDBACK", style="ErrorBadge.TLabel")
                self.status_var.set(self.node.feedback_pause_reason)
            elif not follower_ready:
                self.system_status_label.configure(text="● ARMED · FOLLOWER STATUS UNKNOWN", style="ErrorBadge.TLabel")
                self.status_var.set(f"Follower readiness not confirmed: {self.node.follower_message}")
            else:
                self.system_status_label.configure(text="● RUNNING", style="Badge.TLabel")
        else:
            self.start_button.configure(
                text="Start teleoperation", state="normal" if can_start else "disabled",
                bg=self.accent_green if can_start else self.bg_input,
                fg="#0d2119" if can_start else self.fg_muted,
                activebackground="#89dfae" if can_start else self.bg_input)
            self.stop_button.configure(state="disabled")
            if not self.node.connected:
                self.system_status_label.configure(text="● DISARMED", style="ErrorBadge.TLabel")
            elif not fresh_leader:
                self.system_status_label.configure(text="● WAITING · LEADER FEEDBACK", style="ErrorBadge.TLabel")
                self.status_var.set("Waiting for fresh leader joint feedback")
            elif not follower_ready:
                self.system_status_label.configure(text="● WAITING · FOLLOWER READY", style="ErrorBadge.TLabel")
                self.status_var.set(f"Follower not ready: {self.node.follower_message}")
            else:
                self.system_status_label.configure(text="● READY", style="Badge.TLabel")
                self.status_var.set("Ready · press Start teleoperation")
        self.root.after(350, self._refresh_status)

    def _record_stop_input(self, source: str, event: tk.Event) -> None:
        self._last_stop_input_source = source
        self._last_stop_input_time = time.monotonic()
        self.log.info("GUI Stop input source=%s event_type=%s keysym=%s",
                      source, getattr(event, "type", ""), getattr(event, "keysym", ""))

    def _stop_from_gui(self) -> None:
        age = time.monotonic() - self._last_stop_input_time
        source = self._last_stop_input_source if self._last_stop_input_source and age <= 2.0 else "mouse/keyboard"
        self._last_stop_input_source = ""
        self.node.stop_teleop("GUI Stop button; source=" + source)

    def _close(self) -> None:
        self.node.set_laptop_mic_enabled(False)
        self.node.stop_teleop("leader GUI closed")
        self.root.after(50, self.root.destroy)

def main(args: list[str] | None = None) -> None:
    _set_parent_death_signal()
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    log_path = os.path.expanduser("~/.ros/so101_lerobot_leader.log")
    listener = configure_logging(log_path)
    log = logging.getLogger("so101.leader")
    log.info("application startup Python=%s ROS_DOMAIN_ID=%s", os.sys.version.split()[0], os.getenv("ROS_DOMAIN_ID", "42"))
    rclpy.init(args=args)
    events: queue.Queue[tuple[str, object]] = queue.Queue()
    node = LeaderNode(events)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="ros-executor", daemon=True)
    spin_thread.start()
    root = tk.Tk()
    LeaderGui(root, node)
    log.info("GUI started; follower IP defaults to %s", node.follower_ip)
    try:
        root.mainloop()
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        listener.stop()
        log.info("application shutdown complete")


if __name__ == "__main__":
    main()
