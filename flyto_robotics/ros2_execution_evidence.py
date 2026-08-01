"""Strict redacted evidence for an authorized ROS 2 action closed loop."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping
from datetime import datetime, timedelta, timezone
from typing import Any

from .ros2_action_executor import NavigationOutcome, PreparedNavigation
from .ros2_execution import parse_ros2_execution_grant

ROS2_EXECUTION_EVIDENCE_VERSION = "flyto.robotics.ros2-execution-evidence.v1"
FAULT_EXPECTATIONS = {
    "lidar_dropout": "lidar_stale",
    "odometry_freeze": "odometry_stale",
    "nav2_lifecycle_failure": "command_stale",
}
_SCENARIOS = {"success", "cancel", "emergency_stop", *FAULT_EXPECTATIONS}
_SAFETY_REASONS = {"emergency_stop", *FAULT_EXPECTATIONS.values()}
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,191}$")
_FIELDS = {
    "contract_version",
    "execution_id",
    "scenario",
    "robot_id",
    "workflow_id",
    "target_space_id",
    "resource_id",
    "adapter_id",
    "capability_id",
    "semantic_map_id",
    "semantic_location_id",
    "goal_frame",
    "grant_snapshot",
    "resource_plan_snapshot",
    "profile_snapshot",
    "runtime_snapshot",
    "status",
    "result_code",
    "goal_accepted",
    "feedback_count",
    "initial_pose",
    "final_pose",
    "displacement_m",
    "goal_error_m",
    "post_stop_drift_m",
    "started_at",
    "finished_at",
    "duration_seconds",
    "cancel_requested",
    "cancel_reason",
    "safety_stop_observed",
    "safety_stop_reason",
    "fault_injection_observed",
    "safety_stop_latency_ms",
    "event_codes",
    "snapshot",
}
_STATUSES = {
    "succeeded",
    "canceled",
    "safety_stopped",
    "timed_out",
    "rejected",
    "aborted",
}


class Ros2ExecutionEvidenceError(ValueError):
    """Raised when closed-loop evidence is incomplete or internally inconsistent."""


def build_ros2_execution_evidence(
    grant: Mapping[str, Any],
    prepared: PreparedNavigation,
    outcome: NavigationOutcome,
    *,
    scenario: str,
    finished_at: datetime | None = None,
) -> dict[str, Any]:
    """Bind terminal action facts to the exact short-lived authority snapshot."""

    validated_grant = parse_ros2_execution_grant(grant)
    if scenario not in _SCENARIOS:
        raise Ros2ExecutionEvidenceError("scenario is unsupported")
    finished = _utc(finished_at or datetime.now(timezone.utc), "finished_at")
    started = finished - timedelta(seconds=outcome.duration_seconds)
    seed = {
        "grant_snapshot": validated_grant["snapshot"],
        "scenario": scenario,
        "location_id": prepared.location_id,
        "started_at": _format_datetime(started),
    }
    evidence: dict[str, Any] = {
        "contract_version": ROS2_EXECUTION_EVIDENCE_VERSION,
        "execution_id": "ros2-exec-" + _snapshot(seed)[:24],
        "scenario": scenario,
        "robot_id": validated_grant["robot_id"],
        "workflow_id": validated_grant["workflow_id"],
        "target_space_id": validated_grant["target_space_id"],
        "resource_id": validated_grant["resource_id"],
        "adapter_id": validated_grant["adapter_id"],
        "capability_id": validated_grant["capability_id"],
        "semantic_map_id": prepared.map_id,
        "semantic_location_id": prepared.location_id,
        "goal_frame": prepared.frame_id,
        "grant_snapshot": validated_grant["snapshot"],
        "resource_plan_snapshot": validated_grant["resource_plan_snapshot"],
        "profile_snapshot": validated_grant["profile_snapshot"],
        "runtime_snapshot": validated_grant["runtime_snapshot"],
        "status": outcome.status,
        "result_code": outcome.result_code,
        "goal_accepted": outcome.goal_accepted,
        "feedback_count": outcome.feedback_count,
        "initial_pose": _pose_dict(outcome.initial_pose),
        "final_pose": _pose_dict(outcome.final_pose),
        "displacement_m": outcome.displacement_m,
        "goal_error_m": outcome.goal_error_m,
        "post_stop_drift_m": outcome.post_stop_drift_m,
        "started_at": _format_datetime(started),
        "finished_at": _format_datetime(finished),
        "duration_seconds": outcome.duration_seconds,
        "cancel_requested": outcome.cancel_requested,
        "cancel_reason": outcome.cancel_reason,
        "safety_stop_observed": outcome.safety_stop_observed,
        "safety_stop_reason": outcome.safety_stop_reason,
        "fault_injection_observed": outcome.fault_injection_observed,
        "safety_stop_latency_ms": outcome.safety_stop_latency_ms,
        "event_codes": list(outcome.event_codes),
    }
    evidence["snapshot"] = _snapshot(evidence)
    return parse_ros2_execution_evidence(evidence)


def parse_ros2_execution_evidence(value: Any) -> dict[str, Any]:
    """Strictly validate one content-addressed action evidence document."""

    if not isinstance(value, Mapping):
        raise Ros2ExecutionEvidenceError("execution evidence must be an object")
    missing = sorted(_FIELDS - set(value))
    extra = sorted(set(value) - _FIELDS)
    if missing or extra:
        raise Ros2ExecutionEvidenceError(
            "execution evidence fields do not match the contract"
        )
    if value["contract_version"] != ROS2_EXECUTION_EVIDENCE_VERSION:
        raise Ros2ExecutionEvidenceError("execution evidence version is unsupported")
    for field in (
        "execution_id",
        "robot_id",
        "workflow_id",
        "target_space_id",
        "resource_id",
        "adapter_id",
        "capability_id",
        "semantic_map_id",
        "semantic_location_id",
    ):
        _identifier(value[field], field)
    if value["scenario"] not in _SCENARIOS:
        raise Ros2ExecutionEvidenceError("scenario is unsupported")
    if value["goal_frame"] not in {"map", "odom"}:
        raise Ros2ExecutionEvidenceError("goal_frame is unsupported")
    for field in (
        "grant_snapshot",
        "resource_plan_snapshot",
        "profile_snapshot",
        "runtime_snapshot",
        "snapshot",
    ):
        if not isinstance(value[field], str) or _DIGEST.fullmatch(value[field]) is None:
            raise Ros2ExecutionEvidenceError(f"{field} must be a SHA-256 digest")
    if value["status"] not in _STATUSES:
        raise Ros2ExecutionEvidenceError("status is unsupported")
    if value["result_code"] not in {"succeeded", "canceled", "rejected", "aborted"}:
        raise Ros2ExecutionEvidenceError("result_code is unsupported")
    for field in (
        "goal_accepted",
        "cancel_requested",
        "safety_stop_observed",
        "fault_injection_observed",
    ):
        if type(value[field]) is not bool:
            raise Ros2ExecutionEvidenceError(f"{field} must be boolean")
    if (
        isinstance(value["feedback_count"], bool)
        or not isinstance(value["feedback_count"], int)
        or not 0 <= value["feedback_count"] <= 1_000_000
    ):
        raise Ros2ExecutionEvidenceError("feedback_count is invalid")
    _pose(value["initial_pose"], "initial_pose")
    _pose(value["final_pose"], "final_pose")
    for field in (
        "displacement_m",
        "goal_error_m",
        "post_stop_drift_m",
        "duration_seconds",
    ):
        _number(value[field], field, minimum=0.0, maximum=3600.0)
    started = _utc_text(value["started_at"], "started_at")
    finished = _utc_text(value["finished_at"], "finished_at")
    duration = (finished - started).total_seconds()
    if abs(duration - float(value["duration_seconds"])) > 0.01:
        raise Ros2ExecutionEvidenceError("duration does not match timestamps")
    cancel_reason = value["cancel_reason"]
    if cancel_reason not in {
        None,
        "operator_cancel",
        "emergency_stop",
        "timeout",
        *FAULT_EXPECTATIONS.values(),
    }:
        raise Ros2ExecutionEvidenceError("cancel_reason is unsupported")
    if value["cancel_requested"] is not (cancel_reason is not None):
        raise Ros2ExecutionEvidenceError("cancel fields are inconsistent")
    safety_reason = value["safety_stop_reason"]
    if safety_reason not in {None, *_SAFETY_REASONS}:
        raise Ros2ExecutionEvidenceError("safety_stop_reason is unsupported")
    if value["safety_stop_observed"] is not (safety_reason is not None):
        raise Ros2ExecutionEvidenceError("safety stop fields are inconsistent")
    if cancel_reason in _SAFETY_REASONS and cancel_reason != safety_reason:
        raise Ros2ExecutionEvidenceError("cancel and safety reasons disagree")
    latency = value["safety_stop_latency_ms"]
    if latency is not None:
        _number(latency, "safety_stop_latency_ms", minimum=0.0, maximum=10_000.0)
    fault_scenario = value["scenario"] in FAULT_EXPECTATIONS
    if value["fault_injection_observed"] is not fault_scenario:
        raise Ros2ExecutionEvidenceError("fault observation does not match scenario")
    if fault_scenario and (safety_reason is None or latency is None):
        raise Ros2ExecutionEvidenceError("fault scenario lacks measured safety stop")
    if not fault_scenario and latency is not None:
        raise Ros2ExecutionEvidenceError("non-fault scenario has fault latency")
    event_codes = value["event_codes"]
    if not isinstance(event_codes, list) or not 1 <= len(event_codes) <= 64:
        raise Ros2ExecutionEvidenceError("event_codes must contain 1 to 64 items")
    for index, code in enumerate(event_codes):
        _identifier(code, f"event_codes[{index}]")
    if len(event_codes) != len(set(event_codes)):
        raise Ros2ExecutionEvidenceError("event_codes must be unique")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2ExecutionEvidenceError("execution evidence snapshot does not match")
    return dict(value)


def evaluate_closed_loop_evidence(
    value: Mapping[str, Any],
    *,
    expected_scenario: str,
) -> dict[str, Any]:
    """Return a deterministic verdict for success, cancel, or emergency-stop proof."""

    evidence = parse_ros2_execution_evidence(value)
    checks = {
        "scenario_matches": evidence["scenario"] == expected_scenario,
        "goal_accepted": evidence["goal_accepted"] is True,
        "feedback_observed": evidence["feedback_count"] >= 1,
        "physical_displacement": evidence["displacement_m"] >= 0.05,
        "post_stop_stability": evidence["post_stop_drift_m"] <= 0.05,
    }
    if expected_scenario == "success":
        checks.update(
            {
                "terminal_status": evidence["status"] == "succeeded",
                "goal_reached": evidence["goal_error_m"] <= 0.35,
                "not_canceled": evidence["cancel_requested"] is False,
                "no_safety_stop": evidence["safety_stop_observed"] is False,
            }
        )
    elif expected_scenario == "cancel":
        checks.update(
            {
                "terminal_status": evidence["status"] == "canceled",
                "cancel_requested": evidence["cancel_reason"] == "operator_cancel",
                "no_safety_stop": evidence["safety_stop_observed"] is False,
            }
        )
    elif expected_scenario == "emergency_stop":
        checks.update(
            {
                "terminal_status": evidence["status"] == "safety_stopped",
                "cancel_requested": evidence["cancel_reason"] == "emergency_stop",
                "safety_stop_observed": evidence["safety_stop_observed"] is True,
                "safety_reason": evidence["safety_stop_reason"] == "emergency_stop",
            }
        )
    elif expected_scenario in FAULT_EXPECTATIONS:
        checks.update(
            {
                "terminal_status": evidence["status"] == "safety_stopped",
                "fault_injected": evidence["fault_injection_observed"] is True,
                "safety_stop_observed": evidence["safety_stop_observed"] is True,
                "safety_reason": (
                    evidence["safety_stop_reason"]
                    == FAULT_EXPECTATIONS[expected_scenario]
                ),
                "bounded_stop_latency": (
                    evidence["safety_stop_latency_ms"] is not None
                    and evidence["safety_stop_latency_ms"] <= 750.0
                ),
            }
        )
    else:
        raise Ros2ExecutionEvidenceError("expected_scenario is unsupported")
    return {
        "contract_version": "flyto.robotics.ros2-closed-loop-verdict.v1",
        "scenario": expected_scenario,
        "passed": all(checks.values()),
        "checks": [
            {"code": code, "passed": passed}
            for code, passed in checks.items()
        ],
        "evidence_snapshot": evidence["snapshot"],
    }


def _pose_dict(value: Any) -> dict[str, float]:
    return {
        "x": round(float(value.x), 6),
        "y": round(float(value.y), 6),
        "yaw": round(float(value.yaw), 6),
    }


def _pose(value: Any, label: str) -> None:
    if not isinstance(value, Mapping) or set(value) != {"x", "y", "yaw"}:
        raise Ros2ExecutionEvidenceError(f"{label} must contain x, y, and yaw")
    _number(value["x"], f"{label}.x", minimum=-1000.0, maximum=1000.0)
    _number(value["y"], f"{label}.y", minimum=-1000.0, maximum=1000.0)
    _number(value["yaw"], f"{label}.yaw", minimum=-math.pi, maximum=math.pi)


def _number(value: Any, label: str, *, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Ros2ExecutionEvidenceError(f"{label} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise Ros2ExecutionEvidenceError(f"{label} is outside its valid range")
    return parsed


def _identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or _IDENTIFIER.fullmatch(value) is None:
        raise Ros2ExecutionEvidenceError(f"{label} is invalid")
    return value


def _utc_text(value: Any, label: str) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise Ros2ExecutionEvidenceError(f"{label} must be UTC text")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise Ros2ExecutionEvidenceError(f"{label} is invalid") from exc
    return _utc(parsed, label)


def _utc(value: datetime, label: str) -> datetime:
    if value.tzinfo is None:
        raise Ros2ExecutionEvidenceError(f"{label} must include a UTC offset")
    return value.astimezone(timezone.utc)


def _format_datetime(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def _snapshot(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()
