"""Content-addressed verdicts for ROS 2 fault injection and soak runs."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping, Sequence
from datetime import datetime, timedelta
from typing import Any

from .ros2_action_executor import (
    Ros2ActionExecutionError,
    prepare_authorized_navigation,
)
from .ros2_execution import Ros2ExecutionError, parse_ros2_execution_grant
from .ros2_execution_evidence import (
    FAULT_EXPECTATIONS,
    evaluate_closed_loop_evidence,
    parse_ros2_execution_evidence,
)

ROS2_GRANT_EXPIRY_PROBE_VERSION = "flyto.robotics.ros2-grant-expiry-probe.v1"
ROS2_STRESS_REPORT_VERSION = "flyto.robotics.ros2-stress-report.v1"
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_REPORT_FIELDS = {
    "contract_version",
    "report_id",
    "robot_id",
    "requested_soak_runs",
    "completed_soak_runs",
    "fault_scenarios",
    "evidence_snapshots",
    "grant_expiry_probe_snapshot",
    "grant_expiry_rejected",
    "max_safety_stop_latency_ms",
    "max_post_stop_drift_m",
    "checks",
    "passed",
    "snapshot",
}


class Ros2StressEvidenceError(ValueError):
    """Raised when stress evidence is incomplete, inconsistent, or tampered."""


def prove_expired_grant_rejected(
    *,
    grant: Mapping[str, Any],
    manifest: Mapping[str, Any],
    runtime: Mapping[str, Any],
    semantic_map: Mapping[str, Any],
    location_id: str,
    frame_id: str = "map",
) -> dict[str, Any]:
    """Prove an expired authority snapshot is rejected before action resolution."""

    validated = parse_ros2_execution_grant(grant)
    expires_at = datetime.fromisoformat(
        str(validated["expires_at"]).replace("Z", "+00:00")
    )
    checked_at = expires_at + timedelta(milliseconds=1)
    rejected = False
    try:
        prepare_authorized_navigation(
            grant=dict(validated),
            manifest=dict(manifest),
            runtime=dict(runtime),
            semantic_map=dict(semantic_map),
            location_id=location_id,
            frame_id=frame_id,
            observed_at=checked_at,
        )
    except (Ros2ExecutionError, Ros2ActionExecutionError) as exc:
        rejected = "expired" in str(exc).lower()
    probe: dict[str, Any] = {
        "contract_version": ROS2_GRANT_EXPIRY_PROBE_VERSION,
        "grant_snapshot": validated["snapshot"],
        "checked_at": checked_at.isoformat().replace("+00:00", "Z"),
        "rejected": rejected,
        "reason_code": "expired_grant_rejected" if rejected else "unexpected_accept",
    }
    probe["snapshot"] = _snapshot(probe)
    return parse_grant_expiry_probe(probe)


def parse_grant_expiry_probe(value: Any) -> dict[str, Any]:
    fields = {
        "contract_version",
        "grant_snapshot",
        "checked_at",
        "rejected",
        "reason_code",
        "snapshot",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise Ros2StressEvidenceError("grant expiry probe fields do not match")
    if value["contract_version"] != ROS2_GRANT_EXPIRY_PROBE_VERSION:
        raise Ros2StressEvidenceError("grant expiry probe version is unsupported")
    for field in ("grant_snapshot", "snapshot"):
        if not isinstance(value[field], str) or not _DIGEST.fullmatch(value[field]):
            raise Ros2StressEvidenceError(f"{field} must be a SHA-256 digest")
    if type(value["rejected"]) is not bool:
        raise Ros2StressEvidenceError("grant expiry rejection must be boolean")
    expected_reason = (
        "expired_grant_rejected" if value["rejected"] else "unexpected_accept"
    )
    if value["reason_code"] != expected_reason:
        raise Ros2StressEvidenceError("grant expiry reason is inconsistent")
    _utc_text(value["checked_at"], "checked_at")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2StressEvidenceError("grant expiry probe snapshot does not match")
    return dict(value)


def build_ros2_stress_report(
    executions: Sequence[Mapping[str, Any]],
    grant_expiry_probe: Mapping[str, Any],
    *,
    requested_soak_runs: int,
) -> dict[str, Any]:
    """Aggregate independent action evidence into one strict stress verdict."""

    if isinstance(requested_soak_runs, bool) or not 1 <= requested_soak_runs <= 100:
        raise Ros2StressEvidenceError("requested soak runs must be between 1 and 100")
    evidence = [parse_ros2_execution_evidence(item) for item in executions]
    probe = parse_grant_expiry_probe(grant_expiry_probe)
    successes = [item for item in evidence if item["scenario"] == "success"]
    faults = {
        scenario: [item for item in evidence if item["scenario"] == scenario]
        for scenario in FAULT_EXPECTATIONS
    }
    snapshots = [item["snapshot"] for item in evidence]
    success_passed = all(
        evaluate_closed_loop_evidence(item, expected_scenario="success")["passed"]
        for item in successes
    )
    fault_passed = all(
        len(items) == 1
        and evaluate_closed_loop_evidence(items[0], expected_scenario=scenario)[
            "passed"
        ]
        for scenario, items in faults.items()
    )
    fault_documents = [items[0] for items in faults.values() if len(items) == 1]
    latencies = [float(item["safety_stop_latency_ms"]) for item in fault_documents]
    drifts = [float(item["post_stop_drift_m"]) for item in evidence]
    checks = {
        "soak_count_complete": len(successes) == requested_soak_runs,
        "all_soak_runs_passed": success_passed and bool(successes),
        "fault_matrix_complete": all(len(items) == 1 for items in faults.values()),
        "all_fault_runs_passed": fault_passed,
        "execution_set_exact": len(evidence)
        == requested_soak_runs + len(FAULT_EXPECTATIONS),
        "grant_expiry_rejected": probe["rejected"] is True,
        "evidence_snapshots_unique": len(snapshots) == len(set(snapshots)),
        "stop_latency_bounded": bool(latencies) and max(latencies) <= 750.0,
        "post_stop_drift_bounded": bool(drifts) and max(drifts) <= 0.05,
    }
    robot_ids = {item["robot_id"] for item in evidence}
    checks["single_robot_identity"] = len(robot_ids) == 1
    report: dict[str, Any] = {
        "contract_version": ROS2_STRESS_REPORT_VERSION,
        "report_id": "ros2-stress-" + _snapshot(
            {"snapshots": snapshots, "grant_probe": probe["snapshot"]}
        )[:24],
        "robot_id": next(iter(robot_ids), "unknown"),
        "requested_soak_runs": requested_soak_runs,
        "completed_soak_runs": len(successes),
        "fault_scenarios": {
            scenario: items[0]["snapshot"] if len(items) == 1 else None
            for scenario, items in faults.items()
        },
        "evidence_snapshots": snapshots,
        "grant_expiry_probe_snapshot": probe["snapshot"],
        "grant_expiry_rejected": probe["rejected"],
        "max_safety_stop_latency_ms": round(max(latencies), 3) if latencies else 0.0,
        "max_post_stop_drift_m": round(max(drifts), 6) if drifts else 0.0,
        "checks": [
            {"code": code, "passed": passed} for code, passed in checks.items()
        ],
        "passed": all(checks.values()),
    }
    report["snapshot"] = _snapshot(report)
    return parse_ros2_stress_report(report)


def parse_ros2_stress_report(value: Any) -> dict[str, Any]:
    """Strictly parse one content-addressed stress report."""

    if not isinstance(value, Mapping) or set(value) != _REPORT_FIELDS:
        raise Ros2StressEvidenceError("stress report fields do not match")
    if value["contract_version"] != ROS2_STRESS_REPORT_VERSION:
        raise Ros2StressEvidenceError("stress report version is unsupported")
    if not isinstance(value["report_id"], str) or not value["report_id"].startswith(
        "ros2-stress-"
    ):
        raise Ros2StressEvidenceError("stress report id is invalid")
    if not isinstance(value["robot_id"], str) or not value["robot_id"]:
        raise Ros2StressEvidenceError("stress report robot identity is invalid")
    requested = value["requested_soak_runs"]
    completed = value["completed_soak_runs"]
    if isinstance(requested, bool) or not isinstance(requested, int):
        raise Ros2StressEvidenceError("requested_soak_runs is invalid")
    if not 1 <= requested <= 100:
        raise Ros2StressEvidenceError("requested_soak_runs is invalid")
    if isinstance(completed, bool) or not isinstance(completed, int):
        raise Ros2StressEvidenceError("completed_soak_runs is invalid")
    if not 0 <= completed <= 100:
        raise Ros2StressEvidenceError("completed_soak_runs is invalid")
    faults = value["fault_scenarios"]
    if not isinstance(faults, Mapping) or set(faults) != set(FAULT_EXPECTATIONS):
        raise Ros2StressEvidenceError("stress fault matrix is invalid")
    for snapshot in faults.values():
        if snapshot is not None and (
            not isinstance(snapshot, str) or not _DIGEST.fullmatch(snapshot)
        ):
            raise Ros2StressEvidenceError("fault snapshot is invalid")
    snapshots = value["evidence_snapshots"]
    if not isinstance(snapshots, list) or not 1 <= len(snapshots) <= 103:
        raise Ros2StressEvidenceError("evidence snapshots are invalid")
    if len(snapshots) != len(set(snapshots)) or any(
        not isinstance(item, str) or not _DIGEST.fullmatch(item)
        for item in snapshots
    ):
        raise Ros2StressEvidenceError("evidence snapshots are invalid")
    for field in ("grant_expiry_probe_snapshot", "snapshot"):
        if not isinstance(value[field], str) or not _DIGEST.fullmatch(value[field]):
            raise Ros2StressEvidenceError(f"{field} must be a SHA-256 digest")
    if type(value["grant_expiry_rejected"]) is not bool:
        raise Ros2StressEvidenceError("grant expiry verdict must be boolean")
    _number(value["max_safety_stop_latency_ms"], "stop latency", maximum=10_000.0)
    _number(value["max_post_stop_drift_m"], "post-stop drift", maximum=1000.0)
    checks = value["checks"]
    if not isinstance(checks, list) or not 1 <= len(checks) <= 32:
        raise Ros2StressEvidenceError("stress checks are invalid")
    codes: list[str] = []
    for check in checks:
        if not isinstance(check, Mapping) or set(check) != {"code", "passed"}:
            raise Ros2StressEvidenceError("stress check fields are invalid")
        if not isinstance(check["code"], str) or not check["code"]:
            raise Ros2StressEvidenceError("stress check code is invalid")
        if type(check["passed"]) is not bool:
            raise Ros2StressEvidenceError("stress check verdict must be boolean")
        codes.append(check["code"])
    if len(codes) != len(set(codes)):
        raise Ros2StressEvidenceError("stress check codes must be unique")
    if type(value["passed"]) is not bool or value["passed"] is not all(
        check["passed"] for check in checks
    ):
        raise Ros2StressEvidenceError("stress report verdict is inconsistent")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2StressEvidenceError("stress report snapshot does not match")
    return dict(value)


def _utc_text(value: Any, label: str) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise Ros2StressEvidenceError(f"{label} must be UTC text")
    try:
        return datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise Ros2StressEvidenceError(f"{label} is invalid") from exc


def _number(value: Any, label: str, *, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Ros2StressEvidenceError(f"{label} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not 0.0 <= parsed <= maximum:
        raise Ros2StressEvidenceError(f"{label} is outside its valid range")
    return parsed


def _snapshot(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()
