import unittest
import json
import io
import logging
import tempfile
import threading
import time
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import Mock

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from so101_lerobot_follower.msg import SO101Action
from so101_lerobot_leader import gui as leader_gui
from so101_lerobot_leader.calibration import (
    DEFAULT_FOLLOWER_CALIBRATION,
    DEFAULT_LEADER_CALIBRATION,
    CalibrationError,
    leader_count_to_follower_action,
    resolve_leader_calibration,
    validate_follower_calibration,
    validate_leader_calibration,
)
from so101_lerobot_leader.gui import (
    JOINT_NAMES, LeaderNode, OperatorConsoleHandler, follower_diagnostic_confirms_fault,
    parse_follower_diagnostics,
)


class ActionContractTest(unittest.TestCase):
    def test_action_has_fixed_order_and_mixed_native_units(self):
        msg = SO101Action()
        msg.joint_names = list(JOINT_NAMES)
        msg.values = [10.0, -5.0, 20.0, 2.0, 90.0, 50.0]
        self.assertEqual(tuple(msg.joint_names), (
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
        ))
        self.assertEqual(list(msg.values[:5]), [10.0, -5.0, 20.0, 2.0, 90.0])
        self.assertEqual(msg.values[5], 50.0)

    def test_leader_source_declares_six_leader_joints(self):
        self.assertEqual(len(JOINT_NAMES), 6)
        self.assertEqual(JOINT_NAMES[-1], "gripper")

    def test_start_does_not_reset_action_sequence_between_sessions(self):
        node = LeaderNode.__new__(LeaderNode)
        node.lock = threading.Lock()
        node.connected = True
        node.last_leader_state = time.monotonic()
        node.feedback_pause_timeout_sec = 0.25
        node.follower_is_ready = lambda: True
        node.rate_hz = 30.0
        node.teleop = False
        node.sequence = 41
        node.publish_count = 0
        node.session_pub = Mock()
        node.log = Mock()
        node.events = Mock()

        node.start_teleop()

        self.assertTrue(node.teleop)
        self.assertEqual(node.sequence, 41)
        self.assertGreater(node.last_publish, 0.0, "rate measurement window starts when Start is accepted")
        node.session_pub.publish.assert_called_once()

    def test_feedback_pause_suppresses_actions_without_ending_session(self):
        for age in (0.30, 0.80):
            with self.subTest(feedback_age=age):
                node = LeaderNode.__new__(LeaderNode)
                node.lock = threading.Lock()
                node.teleop = True
                node.connected = True
                node.last_leader_state = time.monotonic() - age
                node.feedback_pause_timeout_sec = 0.25
                node.last_action_timer_callback = 0.0
                node.last_timing_summary = time.monotonic()
                node.raw_positions = {}
                node.rate_hz = 30.0
                node.log = Mock()
                node.action_pub = Mock()
                node.session_pub = Mock()
                node.stop_teleop = Mock()
                node._publish_action()
                node.action_pub.publish.assert_not_called()
                node.stop_teleop.assert_not_called()
                self.assertTrue(node.teleop, "feedback timeout must not end the active session")
                node.session_pub.publish.assert_not_called()

    def test_start_requires_fresh_follower_diagnostics(self):
        node = self._active_node()
        node.follower_diagnostics_stale = False
        node.follower_is_ready = LeaderNode.follower_is_ready.__get__(node)
        node.follower_ready = True
        node.last_follower_diag = time.monotonic() - 4.0
        node.follower_message = "stale diagnostics"

        node.start_teleop()

        self.assertFalse(node.teleop)
        node.session_pub.publish.assert_not_called()

    def test_stale_follower_diagnostics_preserve_active_session_and_actions(self):
        node = self._active_node()
        node.start_teleop()
        self.assertTrue(node.teleop)
        node.last_follower_diag = time.monotonic() - 4.0
        node.follower_is_ready = LeaderNode.follower_is_ready.__get__(node)
        node.follower_ready = False
        node.follower_message = "follower diagnostics stale"

        self.assertFalse(node.follower_is_ready(), "stale diagnostics cannot authorize a new Start")
        node._publish_action()
        node._publish_session_state()

        self.assertTrue(node.teleop, "late remote diagnostics alone must not end the established session")
        self.assertEqual(node.action_pub.publish.call_count, 1)
        self.assertTrue(node.session_pub.publish.call_args_list[-1].args[0].data)

    def test_fresh_follower_diagnostics_return_without_new_session_edge(self):
        node = self._active_node()
        node.start_teleop()
        initial_session_messages = node.session_pub.publish.call_count
        node.last_follower_diag = time.monotonic() - 4.0
        node.follower_is_ready = LeaderNode.follower_is_ready.__get__(node)
        node.follower_ready = False
        node.follower_diagnostics_stale = False
        node.follower_is_ready()  # record the stale transition
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "/so101/follower"
        status.level = DiagnosticStatus.OK
        status.message = "ready; teleoperation active"
        status.values = [KeyValue(key=key, value=value) for key, value in {
            "ready": "true", "armable": "true", "calibration_valid": "true",
            "position_mode_ready": "true", "torque_enabled_all": "true", "feedback_valid": "true",
        }.items()]
        msg.status = [status]

        node._on_diagnostics(msg)

        self.assertTrue(node.teleop)
        self.assertTrue(node.follower_is_ready())
        self.assertEqual(node.session_pub.publish.call_count, initial_session_messages,
                         "diagnostic recovery does not create another session edge")

    def test_nonfatal_unknown_follower_status_does_not_claim_ready_or_disarm(self):
        node = self._active_node()
        node.start_teleop()
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "/so101/follower"
        status.level = DiagnosticStatus.WARN
        status.message = "incomplete remote status"
        msg.status = [status]

        node._on_diagnostics(msg)

        self.assertTrue(node.teleop)
        self.assertFalse(node.follower_ready)
        self.assertFalse(follower_diagnostic_confirms_fault(msg))

    def test_terminal_handler_prints_only_operator_transitions(self):
        stream = io.StringIO()
        handler = OperatorConsoleHandler(stream)
        detail = logging.LogRecord("test", logging.INFO, __file__, 1, "joint dump and counters", (), None)
        concise = logging.LogRecord("test", logging.INFO, __file__, 1, "PAUSED — Waiting for leader feedback", (), None)
        concise.operator_event = True

        handler.emit(detail)
        handler.emit(concise)

        self.assertEqual(stream.getvalue(), "PAUSED — Waiting for leader feedback\n")

    def test_explicit_stop_publishes_disabled_session(self):
        node = LeaderNode.__new__(LeaderNode)
        node.lock = threading.Lock()
        node.teleop = True
        node.session_pub = Mock()
        node.log = Mock()
        node.events = Mock()
        node.publish_count = 3
        node.last_leader_state = time.monotonic()
        node.raw_positions = {name: 500 for name in JOINT_NAMES}
        node.calibration = {name: {"range_min": 100, "range_max": 900} for name in JOINT_NAMES}
        node.follower_calibration = {name: {"range_min": 200, "range_max": 800} for name in JOINT_NAMES}

        node.stop_teleop("GUI Stop button callback; event_source=mouse")

        self.assertFalse(node.teleop)
        node.session_pub.publish.assert_called_once()
        self.assertFalse(node.session_pub.publish.call_args.args[0].data)
        transition = next(call.args for call in node.log.warning.call_args_list
                          if "session transition caller=%s" in call.args[0])
        self.assertIn("previous_session_active=true new_session_active=false", transition[0])
        self.assertIn("publish_false=/so101/leader/teleop_enabled", transition[0])

    def test_follower_readiness_loss_stops_active_session(self):
        node = LeaderNode.__new__(LeaderNode)
        node.lock = threading.RLock()
        node.teleop = True
        node.debug = False
        node.events = Mock()
        node.log = Mock()
        node.last_diagnostic_issue_log = 0.0
        node.last_follower_diag = 0.0
        node.follower_ready = True
        node.follower_message = "ready"
        node.publish_count = 0
        node.session_pub = Mock()
        node.last_leader_state = time.monotonic()
        node.raw_positions = {name: 500 for name in JOINT_NAMES}
        node.calibration = {name: {"range_min": 100, "range_max": 900} for name in JOINT_NAMES}
        node.follower_calibration = {name: {"range_min": 200, "range_max": 800} for name in JOINT_NAMES}
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "/so101/follower"
        status.level = DiagnosticStatus.ERROR
        status.message = "feedback invalid"
        status.values = [KeyValue(key=key, value=value) for key, value in {
            "ready": "false", "calibration_valid": "true", "position_mode_ready": "true",
            "torque_enabled_all": "true", "feedback_valid": "false", "teleop_enabled": "false",
        }.items()]
        msg.status = [status]

        node._on_diagnostics(msg)

        self.assertFalse(node.teleop)
        self.assertFalse(node.follower_ready)
        self.assertFalse(node.session_pub.publish.call_args.args[0].data)

    def test_leader_range_violation_continues_with_follower_bounded_actions(self):
        node = LeaderNode.__new__(LeaderNode)
        node.lock = threading.RLock()
        node.connected = True
        node.teleop = False
        node._last_out_of_calibration = None
        node.last_leader_state = time.monotonic()
        node.feedback_pause_timeout_sec = 0.25
        node.feedback_fault_timeout_sec = 0.75
        node.raw_positions = {name: 500 for name in JOINT_NAMES}
        node.calibration = validate_leader_calibration(DEFAULT_LEADER_CALIBRATION)
        node.follower_calibration = validate_follower_calibration(DEFAULT_FOLLOWER_CALIBRATION)
        node.sequence = 12
        node.publish_count = 0
        node.last_publish = time.monotonic()
        node.last_action_timer_callback = 0.0
        node.last_action_timer_interval_sec = 0.0
        node.max_action_timer_interval_sec = 0.0
        node.last_timing_summary = time.monotonic()
        node.last_leader_feedback_gap_sec = 0.0
        node.max_leader_feedback_gap_sec = 0.0
        node.debug = False
        node.rate_hz = 30.0
        node.log = Mock()
        node.events = Mock()
        node.action_pub = Mock()
        node.session_pub = Mock()
        node.last_error = ""
        node.follower_is_ready = lambda: True
        node.get_clock = Mock(return_value=Mock(now=Mock(return_value=Mock(to_msg=Mock(return_value=Time(sec=5))))))

        node.start_teleop()
        self.assertTrue(node.teleop, "fresh preflight accepts explicit Start even while output is range-paused")
        self.assertEqual(node.session_pub.publish.call_count, 1)
        self.assertTrue(node.session_pub.publish.call_args.args[0].data)
        leader = node.calibration["shoulder_pan"]
        follower = node.follower_calibration["shoulder_pan"]
        span = leader["range_max"] - leader["range_min"]
        sample_counts = (
            leader["range_min"] + span // 2,
            max(0, leader["range_min"] - 100),
            min(4095, leader["range_max"] + 100),
            leader["range_min"] + span // 2,
        )
        for count in sample_counts:
            feedback = JointState()
            feedback.name = list(JOINT_NAMES)
            feedback.position = [500] * len(JOINT_NAMES)
            feedback.position[JOINT_NAMES.index("shoulder_pan")] = count
            node._on_leader_joint_states(feedback)
            node._publish_action()

        self.assertTrue(node.teleop)
        self.assertEqual(node.action_pub.publish.call_count, len(sample_counts))
        self.assertEqual(node.session_pub.publish.call_count, 1)
        actions = [call.args[0] for call in node.action_pub.publish.call_args_list]
        for action in actions:
            self.assertEqual(action.joint_names, list(JOINT_NAMES))
            for joint, value in zip(JOINT_NAMES, action.values):
                target = action_to_follower_count(joint, value, node.follower_calibration[joint])
                calibration = node.follower_calibration[joint]
                self.assertGreaterEqual(target, calibration["range_min"] - 1e-8)
                self.assertLessEqual(target, calibration["range_max"] + 1e-8)
        pan_targets = [action_to_follower_count("shoulder_pan", action.values[0], follower)
                       for action in actions]
        expected_pan_targets = []
        for count in sample_counts:
            fraction = min(1.0, max(0.0, (count - leader["range_min"]) / span))
            expected_pan_targets.append(
                follower["range_min"] + fraction * (follower["range_max"] - follower["range_min"]))
        for actual, expected in zip(pan_targets, expected_pan_targets):
            self.assertAlmostEqual(actual, expected)
        self.assertGreater(pan_targets[0], follower["range_min"])
        self.assertLess(pan_targets[0], follower["range_max"])
        self.assertAlmostEqual(pan_targets[1], follower["range_min"])
        self.assertAlmostEqual(pan_targets[2], follower["range_max"])
        self.assertAlmostEqual(pan_targets[3], pan_targets[0])
        self.assertEqual(actions[-1].sequence, 15)
        self.assertEqual(node.session_pub.publish.call_count, 1,
                         "range return neither stops nor restarts the existing session")
        self.assertFalse(any(
            "teleoperation stopped" in str(call.args)
            for call in node.log.warning.call_args_list))

    def test_gui_stop_button_records_explicit_stop_source(self):
        gui = leader_gui.LeaderGui.__new__(leader_gui.LeaderGui)
        gui.node = Mock()
        gui.log = Mock()
        gui._last_stop_input_source = "mouse"
        gui._last_stop_input_time = time.monotonic()

        gui._stop_from_gui()

        gui.node.stop_teleop.assert_called_once_with(
            "GUI Stop button callback; event_source=mouse")
        gui.log.warning.assert_called_once()
        self.assertIn("event_source=%s", gui.log.warning.call_args.args[0])

    def test_gui_stop_mouse_diagnostic_records_event_origin_fields(self):
        gui = leader_gui.LeaderGui.__new__(leader_gui.LeaderGui)
        gui.log = Mock()
        event = SimpleNamespace(type="ButtonPress", num=1, keysym="??", state=16,
                                x=8, y=6, x_root=1328, y_root=806, send_event=False)

        gui._record_stop_input("mouse", event)

        self.assertIn("event_type=%s button=%s", gui.log.info.call_args.args[0])
        self.assertIn("send_event=%s", gui.log.info.call_args.args[0])
        self.assertEqual(gui.log.info.call_args.args[1:4], ("mouse", "ButtonPress", 1))

    def test_feedback_gap_pauses_then_resumes_only_from_new_current_state(self):
        node = self._active_node()
        clock = [100.0]
        node.last_leader_state = 100.0
        node.raw_positions["shoulder_pan"] = 450
        with unittest.mock.patch.object(leader_gui.time, "monotonic", side_effect=lambda: clock[0]):
            node.start_teleop()
            self.assertTrue(node.teleop)
            node.action_pub.reset_mock()

            clock[0] = 100.30
            node._publish_action()
            node._publish_action()
            self.assertTrue(node.teleop, "a transient gap must preserve the explicit session")
            self.assertEqual(node.action_pub.publish.call_count, 0)
            self.assertIn("PAUSED", node.feedback_pause_reason)
            pause_events = [call for call in node.log.log.call_args_list
                            if call.args[1] == "PAUSED — Waiting for leader feedback"]
            self.assertEqual(len(pause_events), 1, "pause is printed once for the transition")
            paused_sequence = node.sequence

            clock[0] = 100.31
            msg = JointState()
            msg.name = list(JOINT_NAMES)
            msg.position = [700, 600, 600, 600, 600, 500]
            node._on_leader_joint_states(msg)
            node._publish_action()

            self.assertTrue(node.teleop, "fresh valid feedback resumes without another Start")
            self.assertEqual(node.action_pub.publish.call_count, 1)
            self.assertEqual(node.sequence, paused_sequence + 1)
            published = node.action_pub.publish.call_args.args[0]
            expected = leader_count_to_follower_action(
                "shoulder_pan", 700, node.calibration, node.follower_calibration)
            self.assertAlmostEqual(published.values[0], expected)
            self.assertEqual(node.feedback_pause_reason, "")
            resume_events = [call for call in node.log.log.call_args_list
                             if call.args[1] == "RESUMED — Leader feedback restored"]
            self.assertEqual(len(resume_events), 1, "recovery is printed once for the transition")

    def test_sustained_feedback_loss_pauses_without_disarming(self):
        node = self._active_node()
        clock = [200.0]
        node.last_leader_state = 200.0
        with unittest.mock.patch.object(leader_gui.time, "monotonic", side_effect=lambda: clock[0]):
            node.start_teleop()
            self.assertTrue(node.teleop)
            node.action_pub.reset_mock()
            node.session_pub.reset_mock()

            clock[0] = 200.76
            node._publish_action()

            self.assertTrue(node.teleop)
            self.assertTrue(node.connected)
            node.session_pub.publish.assert_not_called()
            self.assertEqual(node.action_pub.publish.call_count, 0)
            self.assertIn("PAUSED", node.feedback_pause_reason)

    def test_invalid_joint_state_structure_is_a_fault_not_a_range_pause(self):
        node = self._active_node()
        node.start_teleop()
        msg = JointState()
        msg.name = list(JOINT_NAMES[:-1])
        msg.position = [500] * 5

        node._on_leader_joint_states(msg)

        self.assertFalse(node.teleop)
        self.assertFalse(node.connected)
        self.assertIn("invalid leader feedback structure", node.last_error)
        self.assertFalse(node.session_pub.publish.call_args.args[0].data)

    def test_explicit_stop_blocks_all_later_actions(self):
        node = self._active_node()
        node.last_leader_state = time.monotonic()
        node.start_teleop()
        node.action_pub.reset_mock()
        node.stop_teleop("user pressed Stop")
        node._publish_action()
        self.assertFalse(node.teleop)
        self.assertEqual(node.action_pub.publish.call_count, 0)

    def test_leader_feedback_gap_metric_records_receipt_interval(self):
        node = LeaderNode.__new__(LeaderNode)
        node.feedback_pause_timeout_sec = 0.25
        node.feedback_fault_timeout_sec = 0.75
        node.last_leader_state = 100.0
        node.last_leader_feedback_gap_sec = 0.0
        node.max_leader_feedback_gap_sec = 0.0
        node.connected = True
        node.raw_positions = {}
        node.calibration = {name: {"range_min": 100, "range_max": 900} for name in JOINT_NAMES}
        node.follower_calibration = {name: {"range_min": 200, "range_max": 800} for name in JOINT_NAMES}
        node._last_out_of_calibration = None
        node.teleop = False
        node.log = Mock()
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = [500] * 6

        with unittest.mock.patch.object(leader_gui.time, "monotonic", return_value=100.4):
            node._on_leader_joint_states(msg)

        self.assertAlmostEqual(node.last_leader_feedback_gap_sec, 0.4)
        self.assertAlmostEqual(node.max_leader_feedback_gap_sec, 0.4)
        node.log.warning.assert_called_once()

    def test_action_timer_timing_is_logged_while_teleoperation_is_disabled(self):
        node = LeaderNode.__new__(LeaderNode)
        node.last_action_timer_callback = 0.0
        node.last_action_timer_interval_sec = 0.0
        node.max_action_timer_interval_sec = 0.0
        node.last_timing_summary = 1.0
        node.last_leader_state = 10.0
        node.last_leader_feedback_gap_sec = 0.0
        node.max_leader_feedback_gap_sec = 0.0
        node.last_follower_diag = 0.0
        node.rate_hz = 30.0
        node.lock = threading.RLock()
        node.teleop = False
        node.connected = False
        node.log = Mock()

        with unittest.mock.patch.object(leader_gui.time, "monotonic", side_effect=(10.0, 10.04)):
            node._publish_action()
            node._publish_action()

        self.assertAlmostEqual(node.last_action_timer_interval_sec, 0.04)
        self.assertTrue(any("timing teleop_enabled=%s" in call.args[0]
                            for call in node.log.info.call_args_list))

    def _active_node(self):
        node = LeaderNode.__new__(LeaderNode)
        node.lock = threading.RLock()
        node.connected = True
        node.teleop = False
        node.feedback_pause_timeout_sec = 0.25
        node.feedback_fault_timeout_sec = 0.75
        node.last_leader_state = time.monotonic()
        node.raw_positions = {name: 500 for name in JOINT_NAMES}
        node._last_out_of_calibration = None
        node.calibration = {name: {"range_min": 100, "range_max": 900} for name in JOINT_NAMES}
        node.follower_calibration = {name: {"range_min": 200, "range_max": 800} for name in JOINT_NAMES}
        node.sequence = 0
        node.publish_count = 0
        node.last_publish = time.monotonic()
        node.last_action_timer_callback = 0.0
        node.last_action_timer_interval_sec = 0.0
        node.max_action_timer_interval_sec = 0.0
        node.last_timing_summary = time.monotonic()
        node.last_leader_feedback_gap_sec = 0.0
        node.max_leader_feedback_gap_sec = 0.0
        node.debug = False
        node.rate_hz = 30.0
        node.log = Mock()
        node.events = Mock()
        node.action_pub = Mock()
        node.session_pub = Mock()
        node.feedback_pause_reason = ""
        node.last_error = ""
        node.follower_is_ready = lambda: True
        node.follower_message = "ready"
        node.last_diagnostic_issue_log = 0.0
        node.last_follower_diag = time.monotonic()
        node.follower_diagnostics_stale = False
        node.get_clock = Mock(return_value=Mock(now=Mock(return_value=Mock(to_msg=Mock(return_value=Time(sec=5))))))
        return node


def action_to_follower_count(joint, action, calibration):
    """Invert the follower's public action units for boundedness assertions."""
    low, high = calibration["range_min"], calibration["range_max"]
    if joint == "gripper":
        return low + action * (high - low) / 100.0
    midpoint = (low + high) / 2.0
    return midpoint + action * 4095.0 / 360.0


class LeaderFollowerMappingTest(unittest.TestCase):
    def setUp(self):
        self.leader = validate_leader_calibration(DEFAULT_LEADER_CALIBRATION)
        self.follower = validate_follower_calibration(DEFAULT_FOLLOWER_CALIBRATION)

    def test_below_inside_and_above_leader_interval_map_to_bounded_follower(self):
        for joint in JOINT_NAMES:
            leader = self.leader[joint]
            follower = self.follower[joint]
            span = leader["range_max"] - leader["range_min"]
            test_counts = (max(0, leader["range_min"] - 100),
                           leader["range_min"] + span // 2,
                           min(4095, leader["range_max"] + 100))
            for count in test_counts:
                with self.subTest(joint=joint, count=count):
                    action = leader_count_to_follower_action(joint, count, self.leader, self.follower)
                    target = action_to_follower_count(joint, action, follower)
                    self.assertGreaterEqual(target, follower["range_min"] - 1e-8)
                    self.assertLessEqual(target, follower["range_max"] + 1e-8)
                    expected_fraction = min(1.0, max(0.0, (count - leader["range_min"]) / span))
                    self.assertAlmostEqual(
                        (target - follower["range_min"]) / (follower["range_max"] - follower["range_min"]),
                        expected_fraction,
                    )

    def test_distinct_arm_ranges_are_mapped_by_calibrated_fraction(self):
        joint = "shoulder_lift"
        self.assertNotEqual(self.leader[joint]["range_min"], self.follower[joint]["range_min"])
        fraction = 0.3
        leader_count = self.leader[joint]["range_min"] + fraction * (
            self.leader[joint]["range_max"] - self.leader[joint]["range_min"])
        action = leader_count_to_follower_action(joint, leader_count, self.leader, self.follower)
        target = action_to_follower_count(joint, action, self.follower[joint])
        expected = self.follower[joint]["range_min"] + fraction * (
            self.follower[joint]["range_max"] - self.follower[joint]["range_min"])
        self.assertAlmostEqual(target, expected)
        self.assertNotAlmostEqual(target, leader_count)


class FollowerDiagnosticsTest(unittest.TestCase):
    def message(self, name="/so101/follower", level=DiagnosticStatus.OK, fields=None):
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = name
        status.level = level
        status.message = "ready; teleoperation stopped"
        fields = fields or {
            "ready": "true",
            "armable": "true",
            "calibration_valid": "true",
            "position_mode_ready": "true",
            "torque_enabled_all": "true",
            "feedback_valid": "true",
        }
        status.values = [KeyValue(key=key, value=value) for key, value in fields.items()]
        msg.status = [status]
        return msg

    def test_accepts_complete_healthy_follower_status(self):
        self.assertEqual(parse_follower_diagnostics(self.message()), (True, "ready; teleoperation stopped"))

    def test_only_confirmed_follower_errors_are_fatal(self):
        self.assertFalse(follower_diagnostic_confirms_fault(self.message(level=DiagnosticStatus.WARN)))
        self.assertTrue(follower_diagnostic_confirms_fault(self.message(level=DiagnosticStatus.ERROR)))
        latched = self.message(fields={"session_fault_latched": "true"})
        self.assertTrue(follower_diagnostic_confirms_fault(latched))

    def test_rejects_wrong_status_name_even_if_ready_field_is_true(self):
        ready, detail = parse_follower_diagnostics(self.message(name="/so101/other"))
        self.assertFalse(ready)
        self.assertIn("status '/so101/follower' missing", detail)

    def test_rejects_incomplete_or_unhealthy_status(self):
        ready, detail = parse_follower_diagnostics(self.message(fields={"ready": "true", "feedback_valid": "true"}))
        self.assertFalse(ready)
        self.assertIn("missing fields", detail)

        ready, _ = parse_follower_diagnostics(self.message(level=DiagnosticStatus.ERROR))
        self.assertFalse(ready)

        fields = {item.key: item.value for item in self.message().status[0].values}
        fields["calibration_valid"] = "false"
        ready, detail = parse_follower_diagnostics(self.message(fields=fields))
        self.assertFalse(ready)
        self.assertIn("calibration_valid", detail)

    def test_armable_follower_can_start_with_torque_disabled_but_requires_valid_feedback(self):
        fields = {item.key: item.value for item in self.message().status[0].values}
        fields["torque_enabled_all"] = "false"
        ready, detail = parse_follower_diagnostics(self.message(fields=fields))
        self.assertTrue(ready)
        self.assertIn("safe disarmed state", detail)

        fields["torque_enabled_all"] = "true"
        fields["feedback_valid"] = "false"
        ready, detail = parse_follower_diagnostics(self.message(fields=fields))
        self.assertFalse(ready)
        self.assertIn("feedback_valid", detail)


class LeaderCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "leader.json"
        self.calibration = {
            name: {"id": index, "drive_mode": 0, "homing_offset": 0,
                   "range_min": 0, "range_max": 4095}
            for index, name in enumerate(JOINT_NAMES, 1)
        }

    def tearDown(self):
        self.directory.cleanup()

    def write(self, value):
        self.path.write_text(value if isinstance(value, str) else json.dumps(value), encoding="utf-8")

    def test_default_selects_the_named_leader_cache_file(self):
        self.assertTrue(DEFAULT_LEADER_CALIBRATION.name == "leader.json")
        self.assertTrue(DEFAULT_LEADER_CALIBRATION.is_file())
        self.assertEqual(resolve_leader_calibration(), DEFAULT_LEADER_CALIBRATION.resolve())
        with self.assertRaises(CalibrationError):
            resolve_leader_calibration(Path(self.directory.name) / "None.json")
        with self.assertRaises(CalibrationError):
            resolve_leader_calibration("invalid_name.json")

    def test_valid_named_calibration_is_loaded(self):
        self.write(self.calibration)
        loaded = validate_leader_calibration(self.path)
        self.assertEqual(tuple(loaded), JOINT_NAMES)
        self.assertEqual(loaded["gripper"]["id"], 6)

    def test_existing_leader_cache_file_validates(self):
        loaded = validate_leader_calibration(DEFAULT_LEADER_CALIBRATION)
        self.assertEqual(tuple(loaded), JOINT_NAMES)
        self.assertEqual(loaded["shoulder_pan"]["id"], 1)

    def test_missing_and_malformed_files_fail(self):
        with self.assertRaises(FileNotFoundError):
            validate_leader_calibration(self.path)
        self.write("{ malformed")
        with self.assertRaises(CalibrationError):
            validate_leader_calibration(self.path)

    def test_missing_joints_and_invalid_fields_fail(self):
        incomplete = dict(self.calibration)
        incomplete.pop("gripper")
        self.write(incomplete)
        with self.assertRaises(CalibrationError):
            validate_leader_calibration(self.path)

        mutations = (("id", 99), ("drive_mode", 1), ("homing_offset", 2048),
                     ("range_min", 4095), ("range_max", 4096), ("range_min", 2.5))
        for field, invalid in mutations:
            with self.subTest(field=field, value=invalid):
                data = json.loads(json.dumps(self.calibration))
                data["shoulder_pan"][field] = invalid
                self.write(data)
                with self.assertRaises(CalibrationError):
                    validate_leader_calibration(self.path)


if __name__ == "__main__":
    unittest.main()
