"""Strict access to the local package SO-101 leader calibration."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
MOTOR_IDS = (1, 2, 3, 4, 5, 6)


def get_default_leader_calibration() -> Path:
    """Find the package-local leader calibration file in share or source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("so101_lerobot_leader"))
        cand = share / "config" / "leader.json"
        if cand.is_file():
            return cand.resolve()
    except Exception:
        pass
    cand = Path(__file__).resolve().parent.parent / "config" / "leader.json"
    if cand.is_file():
        return cand.resolve()
    return (Path("config") / "leader.json").resolve()


def get_default_follower_calibration() -> Path:
    """Find the packaged follower calibration used by the paired arm."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("so101_lerobot_follower"))
        candidate = share / "config" / "follower.json"
        if candidate.is_file():
            return candidate.resolve()
    except Exception:
        pass
    source_tree = Path(__file__).resolve().parents[2] / "so101_lerobot_follower" / "config" / "follower.json"
    if source_tree.is_file():
        return source_tree.resolve()
    return (Path("config") / "follower.json").resolve()


DEFAULT_LEADER_CALIBRATION = get_default_leader_calibration()
DEFAULT_FOLLOWER_CALIBRATION = get_default_follower_calibration()


class CalibrationError(ValueError):
    """Raised when the named leader calibration is missing or invalid."""


def resolve_leader_calibration(path: str | Path | None = None) -> Path:
    """Resolve the selected file and require the leader's exact filename."""
    selected = get_default_leader_calibration() if (path is None or str(path).strip() == "") else Path(path).expanduser()
    selected = selected.resolve()
    if selected.name != "leader.json":
        raise CalibrationError(f"leader calibration must be the named leader.json file, got {selected}")
    return selected


def validate_leader_calibration(path: str | Path) -> dict[str, dict[str, int]]:
    """Read and validate leader MotorCalibration JSON without modifying it."""
    selected = resolve_leader_calibration(path)
    if not selected.is_file():
        raise FileNotFoundError(f"leader calibration file does not exist: {selected}")
    try:
        document: Any = json.loads(selected.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"cannot parse leader calibration {selected}: {exc}") from exc
    if not isinstance(document, dict):
        raise CalibrationError(f"leader calibration root must be an object: {selected}")
    expected = set(JOINT_NAMES)
    if set(document) != expected:
        raise CalibrationError(
            f"leader calibration must contain exactly {list(JOINT_NAMES)}; "
            f"missing={sorted(expected - set(document))}, extra={sorted(set(document) - expected)}"
        )

    result: dict[str, dict[str, int]] = {}
    fields = ("id", "drive_mode", "homing_offset", "range_min", "range_max")
    for joint, motor_id in zip(JOINT_NAMES, MOTOR_IDS, strict=True):
        record = document[joint]
        if not isinstance(record, dict):
            raise CalibrationError(f"leader calibration entry for {joint} must be an object")
        missing_fields = set(fields) - set(record)
        if missing_fields:
            raise CalibrationError(f"leader calibration {joint} missing fields: {sorted(missing_fields)}")
        values: dict[str, int] = {}
        for field in fields:
            value = record[field]
            if isinstance(value, bool) or not isinstance(value, int):
                raise CalibrationError(f"leader calibration {joint}.{field} must be an integer, got {value!r}")
            values[field] = value
        if values["id"] != motor_id:
            raise CalibrationError(f"leader calibration {joint}.id={values['id']} expected {motor_id}")
        if values["drive_mode"] != 0:
            raise CalibrationError(f"leader calibration {joint}.drive_mode={values['drive_mode']} expected 0")
        if not -2047 <= values["homing_offset"] <= 2047:
            raise CalibrationError(f"leader calibration {joint}.homing_offset outside [-2047,2047]")
        if not (0 <= values["range_min"] < values["range_max"] <= 4095):
            raise CalibrationError(f"leader calibration {joint} position limits invalid: "
                                   f"[{values['range_min']},{values['range_max']}] (expected 0 <= min < max <= 4095)")
        result[joint] = values
    return result


def validate_follower_calibration(path: str | Path) -> dict[str, dict[str, int]]:
    """Validate the local copy of the paired follower's motor calibration."""
    selected = Path(path).expanduser().resolve()
    if selected.name != "follower.json":
        raise CalibrationError(f"follower calibration must be named follower.json, got {selected}")
    if not selected.is_file():
        raise FileNotFoundError(f"follower calibration file does not exist: {selected}")
    try:
        document: Any = json.loads(selected.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"cannot parse follower calibration {selected}: {exc}") from exc
    if not isinstance(document, dict) or set(document) != set(JOINT_NAMES):
        actual = set(document) if isinstance(document, dict) else set()
        raise CalibrationError(
            f"follower calibration must contain exactly {list(JOINT_NAMES)}; "
            f"missing={sorted(set(JOINT_NAMES) - actual)}, extra={sorted(actual - set(JOINT_NAMES))}"
        )

    result: dict[str, dict[str, int]] = {}
    for joint, motor_id in zip(JOINT_NAMES, MOTOR_IDS, strict=True):
        record = document[joint]
        if not isinstance(record, dict):
            raise CalibrationError(f"follower calibration entry for {joint} must be an object")
        values: dict[str, int] = {}
        for field in ("id", "drive_mode", "homing_offset", "range_min", "range_max"):
            value = record.get(field)
            if isinstance(value, bool) or not isinstance(value, int):
                raise CalibrationError(f"follower calibration {joint}.{field} must be an integer, got {value!r}")
            values[field] = value
        if values["id"] != motor_id:
            raise CalibrationError(f"follower calibration {joint}.id={values['id']} expected {motor_id}")
        if values["drive_mode"] != 0:
            raise CalibrationError(f"follower calibration {joint}.drive_mode={values['drive_mode']} expected 0")
        if not -2047 <= values["homing_offset"] <= 2047:
            raise CalibrationError(f"follower calibration {joint}.homing_offset outside [-2047,2047]")
        if not 0 <= values["range_min"] < values["range_max"] <= 4095:
            raise CalibrationError(
                f"follower calibration {joint} position limits invalid: "
                f"[{values['range_min']},{values['range_max']}]")
        result[joint] = values
    return result


def leader_count_to_follower_action(
    joint: str,
    leader_count: int,
    leader_calibration: dict[str, dict[str, int]],
    follower_calibration: dict[str, dict[str, int]],
) -> float:
    """Map one offset-adjusted leader count to a bounded follower action value."""
    if joint not in leader_calibration or joint not in follower_calibration:
        raise CalibrationError(f"joint {joint!r} is missing from a calibration")
    if not 0 <= leader_count <= 4095:
        raise CalibrationError(f"{joint} leader count outside [0,4095]: {leader_count}")
    leader = leader_calibration[joint]
    follower = follower_calibration[joint]
    leader_span = leader["range_max"] - leader["range_min"]
    follower_span = follower["range_max"] - follower["range_min"]
    if leader_span <= 0 or follower_span <= 0:
        raise CalibrationError(f"{joint} calibration range must have positive width")

    fraction = (leader_count - leader["range_min"]) / leader_span
    follower_count = follower["range_min"] + fraction * follower_span
    follower_count = min(follower["range_max"], max(follower["range_min"], follower_count))
    if joint == "gripper":
        return 100.0 * (follower_count - follower["range_min"]) / follower_span
    follower_midpoint = (follower["range_min"] + follower["range_max"]) / 2.0
    return (follower_count - follower_midpoint) * 360.0 / 4095.0
