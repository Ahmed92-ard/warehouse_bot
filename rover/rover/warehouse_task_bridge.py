"""Opt-in ROS 2 bridge between the warehouse task API and Nav2."""
from __future__ import annotations

import json
import math
import os
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformException, TransformListener


ACTIVE_NAV_STATES = {"navigating_to_pick", "navigating_to_drop", "returning_to_dock"}
HOLD_STATES = {"at_pick", "scanning", "waiting_for_operator", "at_drop"}


class WarehouseTaskBridge(Node):
    def __init__(self) -> None:
        super().__init__("warehouse_task_bridge")
        self.declare_parameter("enable", False)
        self.declare_parameter("api_url", os.getenv("MARS_WAREHOUSE_API_URL", "http://127.0.0.1:8000"))
        self.declare_parameter("poses_file", "")
        self.declare_parameter("poll_interval_sec", 2.0)
        self.declare_parameter("pose_tolerance_m", 0.45)
        self.declare_parameter("yaw_tolerance_rad", 0.70)
        self.declare_parameter("retry_delay_sec", 2.0)
        self._enabled = bool(self.get_parameter("enable").value)
        self._api_url = str(self.get_parameter("api_url").value).rstrip("/")
        self._token = os.getenv("MARS_ROBOT_API_TOKEN", "")
        self._polling = False
        self._lock = threading.Lock()
        self._active_signature = None
        self._goal_signature = None
        self._goal_handle = None
        self._goal_attempt_id = 0
        self._retry_after = 0.0
        self._pending_transition = None
        self._last_wait_log_key = None
        self._last_wait_log_at = 0.0
        poses_path = str(self.get_parameter("poses_file").value)
        if not poses_path:
            poses_path = os.path.join(get_package_share_directory("rover"), "maps", "docking_poses.yaml")
        try:
            with open(poses_path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
            self._frame = str(data["frame_id"])
            self._poses = data["poses"]
            if set(self._poses) != {"dock", "pick", "drop"}:
                raise ValueError("pose file must contain exactly dock, pick and drop")
            for key, pose in self._poses.items():
                if not all(math.isfinite(float(pose[k])) for k in ("x", "y", "yaw")):
                    raise ValueError(f"invalid {key} pose")
        except Exception as exc:
            raise RuntimeError(f"Unable to load approved Dock/Pick/Drop poses: {exc}") from exc

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._timer = self.create_timer(max(0.5, float(self.get_parameter("poll_interval_sec").value)), self._poll)
        if not self._enabled:
            self.get_logger().warning("Disabled. Set enable:=true only when Nav2/localization and API are ready.")
        elif not self._token:
            self.get_logger().error("MARS_ROBOT_API_TOKEN is missing; task polling is disabled.")
            self._enabled = False
        else:
            self.get_logger().info(f"Warehouse task bridge enabled; API={self._api_url}, map frame={self._frame}")

    def _log_wait(self, key: str, message: str, *, warning: bool = False) -> None:
        now = time.monotonic()
        if key != self._last_wait_log_key or now - self._last_wait_log_at >= 10.0:
            if warning:
                self._log_wait_warning(message)
            else:
                self._log_wait_info(message)
            self._last_wait_log_key = key
            self._last_wait_log_at = now

    def _log_wait_warning(self, message: str) -> None:
        self.get_logger().warning(message)

    def _log_wait_info(self, message: str) -> None:
        self.get_logger().info(message)

    def _localization(self):
        # AMCL publishes /amcl_pose only when its motion/update thresholds
        # trigger. TF is the continuous current-pose source and must not gate
        # task dispatch or Nav2 goal retries when localization is unavailable.
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame, "base_footprint", Time())
        except TransformException as exc:
            self._log_wait("missing_localization_tf",
                           f"Current pose unavailable from TF {self._frame}->base_footprint: {exc}")
            return None
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(p.x), float(p.y), yaw

    def _known_location(self):
        pose = self._localization()
        if pose is None:
            return None
        x, y, yaw = pose
        for name, target in self._poses.items():
            distance = math.hypot(x - float(target["x"]), y - float(target["y"]))
            yaw_error = abs(math.atan2(math.sin(yaw - float(target["yaw"])), math.cos(yaw - float(target["yaw"]))))
            if distance <= float(self.get_parameter("pose_tolerance_m").value) and yaw_error <= float(self.get_parameter("yaw_tolerance_rad").value):
                return name
        return "unknown"

    def _api(self, path: str, body: dict | None = None, headers: dict | None = None):
        request_headers = {"Authorization": f"Bearer {self._token}", "Accept": "application/json"}
        if body is not None:
            request_headers["Content-Type"] = "application/json"
        if headers:
            request_headers.update(headers)
        payload = json.dumps(body).encode("utf-8") if body is not None else None
        request = Request(self._api_url + path, data=payload, headers=request_headers, method="POST")
        try:
            with urlopen(request, timeout=2.5) as response:
                raw = response.read()
                return json.loads(raw.decode("utf-8")) if raw else {}
        except HTTPError as exc:
            detail = exc.read(1024).decode("utf-8", errors="replace")
            raise RuntimeError(f"API returned HTTP {exc.code}: {detail}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise RuntimeError(f"API connection failed: {exc}") from exc

    def _poll(self) -> None:
        if not self._enabled or self._polling:
            return
        if not self._lock.acquire(blocking=False):
            return
        self._polling = True
        try:
            if self._pending_transition is not None:
                order_id, state, destination, detail = self._pending_transition
                if self._report_state(order_id, state, destination, detail):
                    self._pending_transition = None
                return
            # Current location is advisory. Task claiming and goal dispatch
            # continue even before map->base_footprint becomes available.
            location = self._known_location()
            response = self._api("/api/v1/robot/tasks/claim", headers={"X-Robot-Location": location or "unknown"})
            task = response.get("task")
            if not task:
                reason = str(response.get("reason", "no_task"))
                expected = response.get("expected_start")
                suffix = f"; expected_start={expected}" if expected else ""
                self._log_wait(f"api_idle:{reason}:{location}",
                               f"No warehouse task dispatched: {reason}; robot_location={location or 'unknown'}{suffix}")
                return
            self._last_wait_log_key = None
            order_id = str(task.get("order_id", ""))
            state = str(task.get("state", ""))
            destination = str(task.get("current_destination", ""))
            signature = (order_id, state, destination)
            if not order_id:
                self.get_logger().error("API task response has no order_id; holding")
                return
            self._active_signature = signature
            if state == "assistance_required":
                recovery_state = {"pick": "navigating_to_pick", "drop": "navigating_to_drop",
                                  "dock": "returning_to_dock"}.get(destination)
                if recovery_state is None:
                    self.get_logger().error(
                        f"Task {order_id} needs assistance but destination is invalid: {destination!r}")
                    return
                if self._report_state(order_id, recovery_state, destination,
                                      "Retrying goal after previous Nav2 interruption"):
                    self.get_logger().info(f"Task {order_id} restored to {recovery_state}; goal will be retried")
                return
            if state in HOLD_STATES:
                if self._goal_handle is not None:
                    self._goal_attempt_id += 1
                    self._goal_handle.cancel_goal_async()
                    self._goal_handle = None
                self._goal_signature = signature
                self.get_logger().info(f"Task {order_id} is {state}; waiting for scan/operator/API update")
                return
            if state not in ACTIVE_NAV_STATES:
                self.get_logger().warning(f"Task {order_id} state={state!r} is not auto-dispatched")
                return
            target = {"navigating_to_pick": "pick", "navigating_to_drop": "drop", "returning_to_dock": "dock"}[state]
            if destination != target:
                self.get_logger().error(f"Task destination mismatch ({state} requires {target}, API says {destination}); holding")
                return
            if self._goal_signature != signature:
                self._send_goal(order_id, state, target, signature)
        except Exception as exc:
            self.get_logger().warning(str(exc))
        finally:
            self._polling = False
            self._lock.release()

    def _send_goal(self, order_id: str, task_state: str, target_name: str, signature) -> None:
        if time.monotonic() < self._retry_after:
            return
        if not self._nav.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning("Nav2 navigate_to_pose action is unavailable; will retry")
            return
        pose = self._poses[target_name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose["x"])
        goal.pose.pose.position.y = float(pose["y"])
        goal.pose.pose.orientation.z = math.sin(float(pose["yaw"]) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(pose["yaw"]) / 2.0)
        self._goal_signature = signature
        self._goal_attempt_id += 1
        attempt_id = self._goal_attempt_id
        self.get_logger().info(f"Navigating order {order_id} to approved {target_name} pose")
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response(
            f, order_id, task_state, target_name, signature, attempt_id))

    def _goal_response(self, future, order_id, task_state, target_name, signature, attempt_id) -> None:
        try:
            handle = future.result()
            if attempt_id != self._goal_attempt_id or signature != self._active_signature:
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                self._goal_signature = None
                self._retry_after = time.monotonic() + float(self.get_parameter("retry_delay_sec").value)
                self.get_logger().warning(f"Nav2 rejected order {order_id} goal; retry scheduled")
                return
            self._goal_handle = handle
            self.get_logger().info(f"Nav2 accepted warehouse goal for order {order_id}")
            handle.get_result_async().add_done_callback(
                lambda result: self._goal_result(
                    result, order_id, task_state, target_name, signature, attempt_id))
        except Exception as exc:
            self._goal_signature = None
            self._retry_after = time.monotonic() + float(self.get_parameter("retry_delay_sec").value)
            self.get_logger().warning(f"Nav2 goal response failed for order {order_id}: {exc}; retry scheduled")

    def _goal_result(self, future, order_id, task_state, target_name, signature, attempt_id) -> None:
        if attempt_id != self._goal_attempt_id:
            return
        self._goal_handle = None
        self._goal_signature = signature
        try:
            result = future.result()
            status = int(result.status)
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._goal_signature = None
                self._retry_after = time.monotonic() + float(self.get_parameter("retry_delay_sec").value)
                self.get_logger().warning(
                    f"Nav2 ended order {order_id} goal with status {status}; retrying after "
                    f"{self.get_parameter('retry_delay_sec').value}s")
                return
            self.get_logger().info(f"Nav2 succeeded for order {order_id} at {target_name}")
            reached_state = {"navigating_to_pick": "at_pick", "navigating_to_drop": "at_drop",
                             "returning_to_dock": "completed"}[task_state]
            self._pending_transition = (order_id, reached_state, target_name, "Nav2 reached approved waypoint")
        except Exception as exc:
            self._goal_signature = None
            self._retry_after = time.monotonic() + float(self.get_parameter("retry_delay_sec").value)
            self.get_logger().warning(f"Nav2 result failed for order {order_id}: {exc}; retry scheduled")

    def _report_state(self, order_id: str, state: str, destination: str, detail: str) -> bool:
        try:
            response = self._api(f"/api/v1/robot/tasks/{order_id}/state",
                                 {"state": state, "destination": destination, "nav_status": state, "detail": detail})
            self.get_logger().info(f"Task {order_id} updated to {response.get('state', state)}")
            return True
        except Exception as exc:
            self.get_logger().error(f"Could not persist task state {state}: {exc}; will retry without resending navigation")
            return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WarehouseTaskBridge()
    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor(num_threads=3))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
