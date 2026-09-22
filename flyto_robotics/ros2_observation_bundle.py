"""Transport-neutral ROS 2 observations shared by simulation and hardware."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping
from datetime import datetime, timezone
from typing import Any

ROS2_OBSERVATION_BUNDLE_VERSION = "flyto.robotics.ros2-observation-bundle.v1"

_FIELDS = {
    "contract_version",
    "observation_id",
    "resource_id",
    "runtime_snapshot",
    "deployment_mode",
    "provider",
    "phase",
    "execution_id",
    "observed_at",
    "pose",
    "range",
    "camera",
    "map_tf_available",
    "snapshot",
}
_PHASES = {"preflight", "before", "after", "post_stop"}
_MODES = {"simulation", "hardware"}
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,191}$")
_DIGEST = re.compile(r"^[0-9a-f]{64}$")


class Ros2ObservationError(ValueError):
    """Raised when observation evidence is incomplete or internally inconsistent."""


def build_ros2_observation_bundle(
    *,
    resource_id: str,
    runtime_snapshot: str,
    deployment_mode: str,
    provider: str,
    phase: str,
    execution_id: str | None,
    pose: Mapping[str, Any] | None,
    range_observation: Mapping[str, Any] | None,
    camera: Mapping[str, Any] | None,
    map_tf_available: bool,
    observed_at: datetime | None = None,
) -> dict[str, Any]:
    """Build one content-addressed observation without exposing ROS graph details."""

    timestamp = _utc(observed_at or datetime.now(timezone.utc), "observed_at")
    payload: dict[str, Any] = {
        "contract_version": ROS2_OBSERVATION_BUNDLE_VERSION,
        "resource_id": resource_id,
        "runtime_snapshot": runtime_snapshot,
        "deployment_mode": deployment_mode,
        "provider": provider,
        "phase": phase,
        "execution_id": execution_id,
        "observed_at": _format_datetime(timestamp),
        "pose": dict(pose) if pose is not None else None,
        "range": dict(range_observation) if range_observation is not None else None,
        "camera": dict(camera) if camera is not None else None,
        "map_tf_available": map_tf_available,
    }
    observation_seed = _snapshot(payload)
    payload["observation_id"] = f"obs-{observation_seed[:24]}"
    payload["snapshot"] = _snapshot(payload)
    return parse_ros2_observation_bundle(payload)


def parse_ros2_observation_bundle(value: Any) -> dict[str, Any]:
    """Strictly validate one simulation-or-hardware observation bundle."""

    if not isinstance(value, Mapping):
        raise Ros2ObservationError("observation bundle must be an object")
    if set(value) != _FIELDS:
        raise Ros2ObservationError("observation bundle fields do not match the contract")
    if value["contract_version"] != ROS2_OBSERVATION_BUNDLE_VERSION:
        raise Ros2ObservationError("observation bundle version is unsupported")

    _identifier(value["observation_id"], "observation_id")
    _identifier(value["resource_id"], "resource_id")
    _digest(value["runtime_snapshot"], "runtime_snapshot")
    if value["deployment_mode"] not in _MODES:
        raise Ros2ObservationError("deployment_mode must be simulation or hardware")
    _identifier(value["provider"], "provider")

    phase = value["phase"]
    if phase not in _PHASES:
        raise Ros2ObservationError("phase is unsupported")
    execution_id = value["execution_id"]
    if execution_id is not None:
        _identifier(execution_id, "execution_id")
    if phase != "preflight" and execution_id is None:
        raise Ros2ObservationError("execution phase must be bound to execution_id")

    _utc_text(value["observed_at"], "observed_at")
    if value["pose"] is not None:
        _pose(value["pose"])
    if value["range"] is not None:
        _range(value["range"])
    if value["camera"] is not None:
        _camera(value["camera"])
    if type(value["map_tf_available"]) is not bool:
        raise Ros2ObservationError("map_tf_available must be boolean")

    if (
        value["pose"] is None
        and value["range"] is None
        and value["camera"] is None
        and value["map_tf_available"] is False
    ):
        raise Ros2ObservationError("observation bundle contains no observations")

    _digest(value["snapshot"], "snapshot")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2ObservationError("observation bundle snapshot does not match")
    return dict(value)


def evaluate_observation_bundle(value: Mapping[str, Any]) -> dict[str, Any]:
    """Describe which verification classes this observation can support."""

    bundle = parse_ros2_observation_bundle(value)
    camera = bundle["camera"]
    checks = {
        "pose_observed": bundle["pose"] is not None,
        "range_observed": bundle["range"] is not None,
        "raw_vision_observed": camera is not None,
        "metric_vision_ready": bool(camera and camera["calibrated"]),
        "map_tf_observed": bundle["map_tf_available"] is True,
        "execution_bound": bundle["phase"] == "preflight" or bundle["execution_id"] is not None,
    }
    return {
        "contract_version": "flyto.robotics.ros2-observation-verdict.v1",
        "deployment_mode": bundle["deployment_mode"],
        "observation_id": bundle["observation_id"],
        "passed": any(
            checks[name]
            for name in (
                "pose_observed",
                "range_observed",
                "raw_vision_observed",
                "map_tf_observed",
            )
        )
        and checks["execution_bound"],
        "checks": [
            {"code": code, "passed": passed}
            for code, passed in checks.items()
        ],
        "evidence_snapshot": bundle["snapshot"],
    }


def runtime_snapshot(interfaces: Any) -> str:
    """Hash a standard-interface inventory without depending on discovery order."""

    normalized = sorted(
        (str(item.kind), str(item.name), str(item.type))
        for item in interfaces
    )
    return _snapshot({"interfaces": normalized})


def _pose(value: Any) -> None:
    if not isinstance(value, Mapping) or set(value) != {"frame", "x", "y", "yaw"}:
        raise Ros2ObservationError("pose must contain frame, x, y, and yaw")
    if value["frame"] not in {"map", "odom"}:
        raise Ros2ObservationError("pose frame is unsupported")
    _number(value["x"], "pose.x", -1000.0, 1000.0)
    _number(value["y"], "pose.y", -1000.0, 1000.0)
    _number(value["yaw"], "pose.yaw", -math.pi, math.pi)


def _range(value: Any) -> None:
    if not isinstance(value, Mapping) or set(value) != {"minimum_range_m", "sample_count"}:
        raise Ros2ObservationError("range must contain minimum_range_m and sample_count")
    _number(value["minimum_range_m"], "range.minimum_range_m", 0.0, 1000.0)
    sample_count = value["sample_count"]
    if isinstance(sample_count, bool) or not isinstance(sample_count, int):
        raise Ros2ObservationError("range.sample_count must be integer")
    if not 1 <= sample_count <= 1_000_000:
        raise Ros2ObservationError("range.sample_count is outside its valid range")


def _camera(value: Any) -> None:
    fields = {
        "encoding",
        "width",
        "height",
        "calibrated",
        "calibration_snapshot",
        "frame_snapshot",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise Ros2ObservationError("camera fields do not match the contract")
    encoding = value["encoding"]
    if not isinstance(encoding, str) or not 1 <= len(encoding) <= 64:
        raise Ros2ObservationError("camera.encoding is invalid")
    for field in ("width", "height"):
        item = value[field]
        if isinstance(item, bool) or not isinstance(item, int) or not 1 <= item <= 16_384:
            raise Ros2ObservationError(f"camera.{field} is invalid")
    if type(value["calibrated"]) is not bool:
        raise Ros2ObservationError("camera.calibrated must be boolean")
    calibration = value["calibration_snapshot"]
    if value["calibrated"]:
        _digest(calibration, "camera.calibration_snapshot")
    elif calibration is not None:
        raise Ros2ObservationError(
            "uncalibrated camera must not claim a calibration snapshot"
        )
    _digest(value["frame_snapshot"], "camera.frame_snapshot")


def _identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or _ID.fullmatch(value) is None:
        raise Ros2ObservationError(f"{label} is invalid")
    return value


def _digest(value: Any, label: str) -> str:
    if not isinstance(value, str) or _DIGEST.fullmatch(value) is None:
        raise Ros2ObservationError(f"{label} must be a SHA-256 digest")
    return value


def _number(value: Any, label: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Ros2ObservationError(f"{label} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise Ros2ObservationError(f"{label} is outside its valid range")
    return parsed


def _utc_text(value: Any, label: str) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise Ros2ObservationError(f"{label} must be UTC text")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise Ros2ObservationError(f"{label} is invalid") from exc
    return _utc(parsed, label)


def _utc(value: datetime, label: str) -> datetime:
    if value.tzinfo is None:
        raise Ros2ObservationError(f"{label} must include a UTC offset")
    return value.astimezone(timezone.utc)


def _format_datetime(value: datetime) -> str:
    return value.isoformat(timespec="microseconds").replace("+00:00", "Z")


def _snapshot(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()
