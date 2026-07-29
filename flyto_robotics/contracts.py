"""Versioned, transport-neutral Flyto2 Robotics contracts."""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

JOB_CONTRACT_VERSION = "flyto.robotics.job.v1"
RESULT_CONTRACT_VERSION = "flyto.robotics.result.v1"
MAX_JOB_BYTES = 64 * 1024
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
PAYLOAD_CATEGORIES = frozenset({"medication", "specimen", "supplies", "documents"})


class JobValidationError(ValueError):
    """Raised when a mission job is unsafe or violates its versioned contract."""


@dataclass(frozen=True)
class StationPose:
    station_id: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Payload:
    payload_id: str
    category: str
    sealed: bool


@dataclass(frozen=True)
class SafetyLimits:
    max_linear_speed: float = 0.25
    max_angular_speed: float = 0.8
    obstacle_stop_distance: float = 0.55
    pose_tolerance: float = 0.25
    mission_timeout_seconds: float = 180.0
    pickup_dwell_seconds: float = 2.0
    dropoff_dwell_seconds: float = 2.0


@dataclass(frozen=True)
class DeliveryJob:
    contract_version: str
    job_id: str
    robot_id: str
    task_type: str
    pickup: StationPose
    dropoff: StationPose
    payload: Payload
    safety: SafetyLimits
    metadata: dict[str, str] = field(default_factory=dict)


def _expect_object(value: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise JobValidationError(f"{field_name} must be an object")
    return value


def _reject_unknown(data: dict[str, Any], allowed: set[str], field_name: str) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise JobValidationError(f"{field_name} contains unsupported fields: {', '.join(unknown)}")


def _identifier(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise JobValidationError(f"{field_name} must be a safe identifier")
    return value


def _number(
    value: Any,
    field_name: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise JobValidationError(f"{field_name} must be a number")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise JobValidationError(f"{field_name} must be between {minimum} and {maximum}")
    return parsed


def _station(value: Any, field_name: str) -> StationPose:
    data = _expect_object(value, field_name)
    _reject_unknown(data, {"station_id", "x", "y", "yaw"}, field_name)
    required = {"station_id", "x", "y"}
    missing = sorted(required - set(data))
    if missing:
        raise JobValidationError(f"{field_name} is missing: {', '.join(missing)}")
    return StationPose(
        station_id=_identifier(data["station_id"], f"{field_name}.station_id"),
        x=_number(data["x"], f"{field_name}.x", minimum=-1000.0, maximum=1000.0),
        y=_number(data["y"], f"{field_name}.y", minimum=-1000.0, maximum=1000.0),
        yaw=_number(data.get("yaw", 0.0), f"{field_name}.yaw", minimum=-math.pi, maximum=math.pi),
    )


def _payload(value: Any) -> Payload:
    data = _expect_object(value, "payload")
    _reject_unknown(data, {"payload_id", "category", "sealed"}, "payload")
    missing = sorted({"payload_id", "category", "sealed"} - set(data))
    if missing:
        raise JobValidationError(f"payload is missing: {', '.join(missing)}")
    category = data["category"]
    if category not in PAYLOAD_CATEGORIES:
        raise JobValidationError("payload.category is unsupported")
    if not isinstance(data["sealed"], bool):
        raise JobValidationError("payload.sealed must be a boolean")
    return Payload(
        payload_id=_identifier(data["payload_id"], "payload.payload_id"),
        category=category,
        sealed=data["sealed"],
    )


def _safety(value: Any) -> SafetyLimits:
    data = _expect_object(value, "safety")
    allowed = {
        "max_linear_speed",
        "max_angular_speed",
        "obstacle_stop_distance",
        "pose_tolerance",
        "mission_timeout_seconds",
        "pickup_dwell_seconds",
        "dropoff_dwell_seconds",
    }
    _reject_unknown(data, allowed, "safety")
    defaults = SafetyLimits()
    return SafetyLimits(
        max_linear_speed=_number(
            data.get("max_linear_speed", defaults.max_linear_speed),
            "safety.max_linear_speed",
            minimum=0.02,
            maximum=0.5,
        ),
        max_angular_speed=_number(
            data.get("max_angular_speed", defaults.max_angular_speed),
            "safety.max_angular_speed",
            minimum=0.05,
            maximum=2.0,
        ),
        obstacle_stop_distance=_number(
            data.get("obstacle_stop_distance", defaults.obstacle_stop_distance),
            "safety.obstacle_stop_distance",
            minimum=0.15,
            maximum=2.0,
        ),
        pose_tolerance=_number(
            data.get("pose_tolerance", defaults.pose_tolerance),
            "safety.pose_tolerance",
            minimum=0.05,
            maximum=1.0,
        ),
        mission_timeout_seconds=_number(
            data.get("mission_timeout_seconds", defaults.mission_timeout_seconds),
            "safety.mission_timeout_seconds",
            minimum=10.0,
            maximum=3600.0,
        ),
        pickup_dwell_seconds=_number(
            data.get("pickup_dwell_seconds", defaults.pickup_dwell_seconds),
            "safety.pickup_dwell_seconds",
            minimum=0.0,
            maximum=60.0,
        ),
        dropoff_dwell_seconds=_number(
            data.get("dropoff_dwell_seconds", defaults.dropoff_dwell_seconds),
            "safety.dropoff_dwell_seconds",
            minimum=0.0,
            maximum=60.0,
        ),
    )


def parse_job(value: Any) -> DeliveryJob:
    """Validate and normalize a decoded job document."""
    data = _expect_object(value, "job")
    allowed = {
        "contract_version",
        "job_id",
        "robot_id",
        "task_type",
        "pickup",
        "dropoff",
        "payload",
        "safety",
        "metadata",
    }
    _reject_unknown(data, allowed, "job")
    required = allowed - {"metadata"}
    missing = sorted(required - set(data))
    if missing:
        raise JobValidationError(f"job is missing: {', '.join(missing)}")
    if data["contract_version"] != JOB_CONTRACT_VERSION:
        raise JobValidationError(f"contract_version must be {JOB_CONTRACT_VERSION}")
    if data["task_type"] != "hospital_delivery":
        raise JobValidationError("task_type must be hospital_delivery")

    pickup = _station(data["pickup"], "pickup")
    dropoff = _station(data["dropoff"], "dropoff")
    if pickup.station_id == dropoff.station_id:
        raise JobValidationError("pickup and dropoff stations must be different")

    metadata_raw = _expect_object(data.get("metadata", {}), "metadata")
    if len(metadata_raw) > 16:
        raise JobValidationError("metadata may contain at most 16 entries")
    metadata: dict[str, str] = {}
    for key, item in metadata_raw.items():
        safe_key = _identifier(key, "metadata key")
        if not isinstance(item, str) or len(item) > 256:
            raise JobValidationError(f"metadata.{safe_key} must be a short string")
        metadata[safe_key] = item

    return DeliveryJob(
        contract_version=JOB_CONTRACT_VERSION,
        job_id=_identifier(data["job_id"], "job_id"),
        robot_id=_identifier(data["robot_id"], "robot_id"),
        task_type="hospital_delivery",
        pickup=pickup,
        dropoff=dropoff,
        payload=_payload(data["payload"]),
        safety=_safety(data["safety"]),
        metadata=metadata,
    )


def load_job(path: str | Path) -> DeliveryJob:
    """Load a bounded UTF-8 JSON job from disk."""
    job_path = Path(path)
    try:
        size = job_path.stat().st_size
    except OSError as exc:
        raise JobValidationError("job file is not readable") from exc
    if size > MAX_JOB_BYTES:
        raise JobValidationError(f"job file exceeds {MAX_JOB_BYTES} bytes")
    try:
        decoded = json.loads(job_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise JobValidationError("job file must contain valid UTF-8 JSON") from exc
    return parse_job(decoded)


def job_to_dict(job: DeliveryJob) -> dict[str, Any]:
    """Serialize a validated job for logging or deterministic tests."""
    return asdict(job)


def write_json_atomic(path: str | Path, value: dict[str, Any]) -> None:
    """Atomically replace a JSON evidence file without leaving partial output."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=destination.parent,
        prefix=f".{destination.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(destination)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
