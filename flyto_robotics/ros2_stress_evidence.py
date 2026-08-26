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
ROS2_STRESS_CAMPAIGN_VERSION = "flyto.robotics.ros2-stress-campaign.v1"
ROS2_PRESSURE_REPORT_VERSION = "flyto.robotics.ros2-pressure-report.v1"
ROS2_RESILIENCE_REPORT_VERSION = "flyto.robotics.ros2-resilience-report.v1"
ROS2_RESILIENCE_SERIES_VERSION = "flyto.robotics.ros2-resilience-series.v1"
ROS2_STRESS_CAMPAIGN_PROFILES: dict[str, dict[str, Any]] = {
    "baseline-l1": {
        "test_type": "baseline",
        "pressure_level": 1,
        "required_rounds": 1,
        "required_success_runs": 5,
        "required_fault_runs": 3,
        "max_safety_stop_latency_ms": 750.0,
        "max_post_stop_drift_m": 0.05,
    },
    "load-l2": {
        "test_type": "mission-load",
        "pressure_level": 2,
        "required_rounds": 2,
        "required_success_runs": 20,
        "required_fault_runs": 6,
        "max_safety_stop_latency_ms": 700.0,
        "max_post_stop_drift_m": 0.05,
    },
    "fault-l3": {
        "test_type": "fault-repetition",
        "pressure_level": 3,
        "required_rounds": 3,
        "required_success_runs": 15,
        "required_fault_runs": 9,
        "max_safety_stop_latency_ms": 650.0,
        "max_post_stop_drift_m": 0.04,
    },
    "endurance-l4": {
        "test_type": "long-soak",
        "pressure_level": 4,
        "required_rounds": 5,
        "required_success_runs": 100,
        "required_fault_runs": 15,
        "max_safety_stop_latency_ms": 600.0,
        "max_post_stop_drift_m": 0.03,
    },
    "mixed-l5": {
        "test_type": "mixed-pressure",
        "pressure_level": 5,
        "required_rounds": 10,
        "required_success_runs": 200,
        "required_fault_runs": 30,
        "max_safety_stop_latency_ms": 500.0,
        "max_post_stop_drift_m": 0.02,
    },
}
ROS2_PRESSURE_PROFILES: dict[str, dict[str, Any]] = {
    "resource-r1": {
        "mode": "resource",
        "campaign_profile_id": "fault-l3",
        "cpu_limit_millicores": 1500,
        "memory_limit_mib": 2048,
        "network_delay_ms": 0,
        "network_jitter_ms": 0,
        "network_loss_percent": 0.0,
        "minimum_execution_runs": 24,
    },
    "network-n1": {
        "mode": "network",
        "campaign_profile_id": "fault-l3",
        "cpu_limit_millicores": 2000,
        "memory_limit_mib": 3072,
        "network_delay_ms": 100,
        "network_jitter_ms": 20,
        "network_loss_percent": 1.0,
        "minimum_execution_runs": 24,
    },
    "endurance-e1": {
        "mode": "endurance",
        "campaign_profile_id": "endurance-l4",
        "cpu_limit_millicores": 2000,
        "memory_limit_mib": 3072,
        "network_delay_ms": 0,
        "network_jitter_ms": 0,
        "network_loss_percent": 0.0,
        "minimum_execution_runs": 115,
    },
}
ROS2_RESILIENCE_PROFILES: dict[str, dict[str, Any]] = {
    "runtime-network-r2": {
        "episode": "#008",
        "title": "Runtime network fault and recovery",
        "minimum_runs": 2,
        "conditions": {
            "injection_phase": "after_motion_observed",
            "network_delay_ms": 200,
            "network_jitter_ms": 50,
            "network_loss_percent": 100.0,
            "sensor_timeout_seconds": 0.60,
        },
        "required_events": (
            "lifecycle_active",
            "mission_motion_observed",
            "network_injected",
            "safety_stop_observed",
            "network_removed",
            "recovery_mission_succeeded",
        ),
        "metrics": {
            "safety_stop_latency_ms": ("lte", 650.0),
            "post_stop_drift_m": ("lte", 0.04),
            "oom_kill_count": ("eq", 0.0),
            "unexpected_process_deaths": ("eq", 0.0),
        },
        "artifact_kinds": {"event_log", "raw_log", "recovery_evidence"},
    },
    "resource-cliff-r2": {
        "episode": "#009",
        "title": "CPU and memory resource cliff matrix",
        "minimum_runs": 12,
        "conditions": {
            "cells": [
                {"cpu_millicores": 1500, "memory_mib": 2048},
                {"cpu_millicores": 1000, "memory_mib": 1536},
                {"cpu_millicores": 750, "memory_mib": 1024},
                {"cpu_millicores": 500, "memory_mib": 768},
            ],
            "repetitions_per_cell": 3,
        },
        "required_events": ("matrix_started", "cell_completed", "matrix_completed"),
        "metrics": {
            "matrix_cells": ("gte", 4.0),
            "repetitions_per_cell": ("gte", 3.0),
            "safe_cells": ("gte", 1.0),
            "oom_kill_count": ("eq", 0.0),
            "unexpected_process_deaths": ("eq", 0.0),
        },
        "artifact_kinds": {"matrix", "raw_log"},
    },
    "compound-chaos-c1": {
        "episode": "#010",
        "title": "Compound CPU, network, and LiDAR chaos",
        "minimum_runs": 2,
        "conditions": {
            "injection_phase": "after_motion_observed",
            "cpu_millicores": 1000,
            "memory_mib": 1536,
            "network_delay_ms": 100,
            "network_jitter_ms": 20,
            "network_loss_percent": 1.0,
            "lidar_dropout": True,
            "lidar_dropout_delay_seconds": 1.25,
            "sensor_timeout_seconds": 0.60,
        },
        "required_events": (
            "lifecycle_active",
            "mission_motion_observed",
            "network_injected",
            "lidar_dropout_injected",
            "safety_stop_observed",
            "pressure_removed",
            "recovery_mission_succeeded",
        ),
        "metrics": {
            "safety_stop_latency_ms": ("lte", 650.0),
            "post_stop_drift_m": ("lte", 0.04),
            "oom_kill_count": ("eq", 0.0),
            "unexpected_process_deaths": ("eq", 0.0),
        },
        "artifact_kinds": {"event_log", "raw_log", "recovery_evidence"},
    },
    "gazebo-endurance-l4": {
        "episode": "#011",
        "title": "Gazebo endurance L4 trend run",
        "minimum_runs": 115,
        "conditions": {
            "campaign_profile_id": "endurance-l4",
            "trend_basis": "sequential_per_run_container_peak",
            "required_success_runs": 100,
            "required_fault_runs": 15,
        },
        "required_events": (
            "endurance_started",
            "first_fault_completed",
            "last_fault_completed",
            "endurance_completed",
        ),
        "metrics": {
            "execution_runs": ("gte", 115.0),
            "pass_rate": ("eq", 1.0),
            "memory_slope_mib_per_hour": ("lte", 64.0),
            "stop_latency_slope_ms_per_hour": ("lte", 50.0),
            "oom_kill_count": ("eq", 0.0),
            "unexpected_process_deaths": ("eq", 0.0),
        },
        "artifact_kinds": {"pressure_report", "raw_log", "trend"},
    },
    "cold-repro-b3": {
        "episode": "#012",
        "title": "Cold container reproducibility",
        "minimum_runs": 3,
        "conditions": {
            "cold_container_runs": 3,
            "image_policy": "one_content_addressed_image",
            "source_policy": "one_source_snapshot",
        },
        "required_events": ("cold_run_started", "cold_run_completed"),
        "metrics": {
            "cold_runs": ("eq", 3.0),
            "passing_runs": ("eq", 3.0),
            "unique_container_ids": ("eq", 3.0),
            "source_snapshot_count": ("eq", 1.0),
            "image_id_count": ("eq", 1.0),
            "unexpected_process_deaths": ("eq", 0.0),
        },
        "artifact_kinds": {"build_receipts", "cold_run_report", "raw_log"},
    },
}
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
_CAMPAIGN_FIELDS = {
    "contract_version",
    "campaign_id",
    "profile_id",
    "test_type",
    "pressure_level",
    "thresholds",
    "build_provenance",
    "runtime_hygiene",
    "robot_id",
    "round_count",
    "passed_rounds",
    "total_success_runs",
    "total_fault_runs",
    "total_execution_runs",
    "report_snapshots",
    "pass_rate",
    "max_safety_stop_latency_ms",
    "p95_round_stop_latency_ms",
    "max_post_stop_drift_m",
    "p95_round_post_stop_drift_m",
    "checks",
    "passed",
    "snapshot",
}
_PRESSURE_REPORT_FIELDS = {
    "contract_version",
    "report_id",
    "pressure_profile_id",
    "mode",
    "campaign_profile_id",
    "campaign_snapshot",
    "limits",
    "observations",
    "checks",
    "passed",
    "snapshot",
}
_RESILIENCE_REPORT_FIELDS = {
    "contract_version",
    "report_id",
    "test_id",
    "episode",
    "title",
    "execution_mode",
    "source_snapshot",
    "container_image_id",
    "started_at",
    "finished_at",
    "run_summaries",
    "timeline",
    "metrics",
    "thresholds",
    "conditions",
    "artifacts",
    "checks",
    "passed",
    "snapshot",
}
_RESILIENCE_SERIES_FIELDS = {
    "contract_version",
    "series_id",
    "episode_order",
    "report_snapshots",
    "passed_reports",
    "failed_reports",
    "complete",
    "all_passed",
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
    expires_at = datetime.fromisoformat(str(validated["expires_at"]).replace("Z", "+00:00"))
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
    expected_reason = "expired_grant_rejected" if value["rejected"] else "unexpected_accept"
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
        and evaluate_closed_loop_evidence(items[0], expected_scenario=scenario)["passed"]
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
        "execution_set_exact": len(evidence) == requested_soak_runs + len(FAULT_EXPECTATIONS),
        "grant_expiry_rejected": probe["rejected"] is True,
        "evidence_snapshots_unique": len(snapshots) == len(set(snapshots)),
        "stop_latency_bounded": bool(latencies) and max(latencies) <= 750.0,
        "post_stop_drift_bounded": bool(drifts) and max(drifts) <= 0.05,
    }
    robot_ids = {item["robot_id"] for item in evidence}
    checks["single_robot_identity"] = len(robot_ids) == 1
    report: dict[str, Any] = {
        "contract_version": ROS2_STRESS_REPORT_VERSION,
        "report_id": "ros2-stress-"
        + _snapshot({"snapshots": snapshots, "grant_probe": probe["snapshot"]})[:24],
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
        "checks": [{"code": code, "passed": passed} for code, passed in checks.items()],
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
    if not isinstance(value["report_id"], str) or not value["report_id"].startswith("ros2-stress-"):
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
        not isinstance(item, str) or not _DIGEST.fullmatch(item) for item in snapshots
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


def build_ros2_stress_campaign(
    reports: Sequence[Mapping[str, Any]],
    *,
    profile_id: str,
    build_provenance: Mapping[str, Any],
    runtime_hygiene: Mapping[str, Any],
) -> dict[str, Any]:
    """Aggregate strict stress rounds against a non-self-declared pressure level."""

    profile = ROS2_STRESS_CAMPAIGN_PROFILES.get(profile_id)
    if profile is None:
        raise Ros2StressEvidenceError("stress campaign profile is unsupported")
    if isinstance(reports, (str, bytes)) or not 1 <= len(reports) <= 64:
        raise Ros2StressEvidenceError("stress campaign must contain 1 to 64 rounds")
    provenance = _parse_build_provenance(build_provenance)
    hygiene = _parse_runtime_hygiene(runtime_hygiene)
    rounds = [parse_ros2_stress_report(item) for item in reports]
    report_snapshots = [item["snapshot"] for item in rounds]
    execution_snapshots = [snapshot for item in rounds for snapshot in item["evidence_snapshots"]]
    robot_ids = {str(item["robot_id"]) for item in rounds}
    success_runs = sum(int(item["completed_soak_runs"]) for item in rounds)
    fault_runs = sum(
        sum(snapshot is not None for snapshot in item["fault_scenarios"].values())
        for item in rounds
    )
    execution_runs = sum(len(item["evidence_snapshots"]) for item in rounds)
    passed_rounds = sum(item["passed"] is True for item in rounds)
    stop_latencies = [float(item["max_safety_stop_latency_ms"]) for item in rounds]
    stop_drifts = [float(item["max_post_stop_drift_m"]) for item in rounds]
    checks = {
        "round_volume_met": len(rounds) >= int(profile["required_rounds"]),
        "all_rounds_passed": passed_rounds == len(rounds),
        "report_snapshots_unique": len(report_snapshots) == len(set(report_snapshots)),
        "execution_snapshots_unique_across_rounds": len(execution_snapshots)
        == len(set(execution_snapshots)),
        "single_robot_identity": len(robot_ids) == 1,
        "success_volume_met": success_runs >= int(profile["required_success_runs"]),
        "fault_volume_met": fault_runs >= int(profile["required_fault_runs"]),
        "execution_volume_consistent": execution_runs == success_runs + fault_runs,
        "runtime_logs_complete": hygiene["scenario_log_count"] == execution_runs,
        "runtime_hygiene_clean": hygiene["unexpected_process_deaths"] == 0
        and not hygiene["unexpected_exit_codes"],
        "grant_expiry_rejected_every_round": all(
            item["grant_expiry_rejected"] is True for item in rounds
        ),
        "stop_latency_bounded": max(stop_latencies) <= float(profile["max_safety_stop_latency_ms"]),
        "post_stop_drift_bounded": max(stop_drifts) <= float(profile["max_post_stop_drift_m"]),
    }
    thresholds = {
        "required_rounds": int(profile["required_rounds"]),
        "required_success_runs": int(profile["required_success_runs"]),
        "required_fault_runs": int(profile["required_fault_runs"]),
        "max_safety_stop_latency_ms": float(profile["max_safety_stop_latency_ms"]),
        "max_post_stop_drift_m": float(profile["max_post_stop_drift_m"]),
    }
    campaign: dict[str, Any] = {
        "contract_version": ROS2_STRESS_CAMPAIGN_VERSION,
        "campaign_id": "ros2-campaign-"
        + _snapshot(
            {
                "profile_id": profile_id,
                "reports": report_snapshots,
                "build_provenance": provenance,
                "runtime_hygiene": hygiene,
            }
        )[:24],
        "profile_id": profile_id,
        "test_type": profile["test_type"],
        "pressure_level": profile["pressure_level"],
        "thresholds": thresholds,
        "build_provenance": provenance,
        "runtime_hygiene": hygiene,
        "robot_id": next(iter(robot_ids), "unknown"),
        "round_count": len(rounds),
        "passed_rounds": passed_rounds,
        "total_success_runs": success_runs,
        "total_fault_runs": fault_runs,
        "total_execution_runs": execution_runs,
        "report_snapshots": report_snapshots,
        "pass_rate": round(passed_rounds / len(rounds), 6),
        "max_safety_stop_latency_ms": round(max(stop_latencies), 3),
        "p95_round_stop_latency_ms": _percentile(stop_latencies, 0.95, 3),
        "max_post_stop_drift_m": round(max(stop_drifts), 6),
        "p95_round_post_stop_drift_m": _percentile(stop_drifts, 0.95, 6),
        "checks": [{"code": code, "passed": passed} for code, passed in checks.items()],
        "passed": all(checks.values()),
    }
    campaign["snapshot"] = _snapshot(campaign)
    return parse_ros2_stress_campaign(campaign)


def parse_ros2_stress_campaign(value: Any) -> dict[str, Any]:
    """Strictly parse a content-addressed multi-round stress campaign."""

    if not isinstance(value, Mapping) or set(value) != _CAMPAIGN_FIELDS:
        raise Ros2StressEvidenceError("stress campaign fields do not match")
    if value["contract_version"] != ROS2_STRESS_CAMPAIGN_VERSION:
        raise Ros2StressEvidenceError("stress campaign version is unsupported")
    profile_id = value["profile_id"]
    profile = ROS2_STRESS_CAMPAIGN_PROFILES.get(profile_id)
    if profile is None:
        raise Ros2StressEvidenceError("stress campaign profile is unsupported")
    if value["test_type"] != profile["test_type"]:
        raise Ros2StressEvidenceError("stress campaign test type is inconsistent")
    if value["pressure_level"] != profile["pressure_level"]:
        raise Ros2StressEvidenceError("stress campaign pressure level is inconsistent")
    expected_thresholds = {
        "required_rounds": int(profile["required_rounds"]),
        "required_success_runs": int(profile["required_success_runs"]),
        "required_fault_runs": int(profile["required_fault_runs"]),
        "max_safety_stop_latency_ms": float(profile["max_safety_stop_latency_ms"]),
        "max_post_stop_drift_m": float(profile["max_post_stop_drift_m"]),
    }
    if value["thresholds"] != expected_thresholds:
        raise Ros2StressEvidenceError("stress campaign thresholds are inconsistent")
    provenance = _parse_build_provenance(value["build_provenance"])
    hygiene = _parse_runtime_hygiene(value["runtime_hygiene"])
    if not isinstance(value["campaign_id"], str) or not value["campaign_id"].startswith(
        "ros2-campaign-"
    ):
        raise Ros2StressEvidenceError("stress campaign id is invalid")
    if not isinstance(value["robot_id"], str) or not value["robot_id"]:
        raise Ros2StressEvidenceError("stress campaign robot identity is invalid")
    integer_limits = {
        "round_count": (1, 64),
        "passed_rounds": (0, 64),
        "total_success_runs": (0, 6400),
        "total_fault_runs": (0, 192),
        "total_execution_runs": (1, 6592),
    }
    for field, (minimum, maximum) in integer_limits.items():
        item = value[field]
        if isinstance(item, bool) or not isinstance(item, int) or not minimum <= item <= maximum:
            raise Ros2StressEvidenceError(f"stress campaign {field} is invalid")
    if value["passed_rounds"] > value["round_count"]:
        raise Ros2StressEvidenceError("stress campaign passed rounds are invalid")
    snapshots = value["report_snapshots"]
    if (
        not isinstance(snapshots, list)
        or len(snapshots) != value["round_count"]
        or any(not isinstance(item, str) or not _DIGEST.fullmatch(item) for item in snapshots)
    ):
        raise Ros2StressEvidenceError("stress campaign report snapshots are invalid")
    expected_campaign_id = (
        "ros2-campaign-"
        + _snapshot(
            {
                "profile_id": profile_id,
                "reports": snapshots,
                "build_provenance": provenance,
                "runtime_hygiene": hygiene,
            }
        )[:24]
    )
    if value["campaign_id"] != expected_campaign_id:
        raise Ros2StressEvidenceError("stress campaign id is inconsistent")
    pass_rate = _number(value["pass_rate"], "campaign pass rate", maximum=1.0)
    if pass_rate != round(value["passed_rounds"] / value["round_count"], 6):
        raise Ros2StressEvidenceError("stress campaign pass rate is inconsistent")
    _number(
        value["max_safety_stop_latency_ms"],
        "campaign stop latency",
        maximum=10_000.0,
    )
    _number(
        value["p95_round_stop_latency_ms"],
        "campaign p95 stop latency",
        maximum=10_000.0,
    )
    _number(
        value["max_post_stop_drift_m"],
        "campaign post-stop drift",
        maximum=1000.0,
    )
    _number(
        value["p95_round_post_stop_drift_m"],
        "campaign p95 post-stop drift",
        maximum=1000.0,
    )
    checks = value["checks"]
    if not isinstance(checks, list) or not 1 <= len(checks) <= 32:
        raise Ros2StressEvidenceError("stress campaign checks are invalid")
    codes: list[str] = []
    for check in checks:
        if not isinstance(check, Mapping) or set(check) != {"code", "passed"}:
            raise Ros2StressEvidenceError("stress campaign check fields are invalid")
        if not isinstance(check["code"], str) or not check["code"]:
            raise Ros2StressEvidenceError("stress campaign check code is invalid")
        if type(check["passed"]) is not bool:
            raise Ros2StressEvidenceError("stress campaign check verdict is invalid")
        codes.append(check["code"])
    if len(codes) != len(set(codes)):
        raise Ros2StressEvidenceError("stress campaign check codes must be unique")
    if type(value["passed"]) is not bool or value["passed"] is not all(
        check["passed"] for check in checks
    ):
        raise Ros2StressEvidenceError("stress campaign verdict is inconsistent")
    if not isinstance(value["snapshot"], str) or not _DIGEST.fullmatch(value["snapshot"]):
        raise Ros2StressEvidenceError("stress campaign snapshot is invalid")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2StressEvidenceError("stress campaign snapshot does not match")
    return dict(value)


def build_ros2_pressure_report(
    campaign: Mapping[str, Any],
    *,
    pressure_profile_id: str,
    observations: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind one campaign verdict to independently observed pressure evidence."""

    verified_campaign = parse_ros2_stress_campaign(campaign)
    profile = ROS2_PRESSURE_PROFILES.get(pressure_profile_id)
    if profile is None:
        raise Ros2StressEvidenceError("pressure profile is unsupported")
    observed = _parse_pressure_observations(observations)
    limits = _pressure_limits(profile)
    checks = _pressure_checks(profile, verified_campaign, observed)
    report: dict[str, Any] = {
        "contract_version": ROS2_PRESSURE_REPORT_VERSION,
        "report_id": "ros2-pressure-"
        + _snapshot(
            {
                "profile": pressure_profile_id,
                "campaign": verified_campaign["snapshot"],
                "observations": observed,
            }
        )[:24],
        "pressure_profile_id": pressure_profile_id,
        "mode": profile["mode"],
        "campaign_profile_id": verified_campaign["profile_id"],
        "campaign_snapshot": verified_campaign["snapshot"],
        "limits": limits,
        "observations": observed,
        "checks": [{"code": code, "passed": passed} for code, passed in checks.items()],
        "passed": all(checks.values()),
    }
    report["snapshot"] = _snapshot(report)
    return parse_ros2_pressure_report(report)


def parse_ros2_pressure_report(value: Any) -> dict[str, Any]:
    """Strictly parse and recompute a resource, network, or endurance verdict."""

    if not isinstance(value, Mapping) or set(value) != _PRESSURE_REPORT_FIELDS:
        raise Ros2StressEvidenceError("pressure report fields do not match")
    if value["contract_version"] != ROS2_PRESSURE_REPORT_VERSION:
        raise Ros2StressEvidenceError("pressure report version is unsupported")
    profile_id = value["pressure_profile_id"]
    profile = ROS2_PRESSURE_PROFILES.get(profile_id)
    if profile is None:
        raise Ros2StressEvidenceError("pressure profile is unsupported")
    if value["mode"] != profile["mode"]:
        raise Ros2StressEvidenceError("pressure mode is inconsistent")
    if value["campaign_profile_id"] != profile["campaign_profile_id"]:
        raise Ros2StressEvidenceError("pressure campaign profile is inconsistent")
    campaign_snapshot = value["campaign_snapshot"]
    if not isinstance(campaign_snapshot, str) or not _DIGEST.fullmatch(campaign_snapshot):
        raise Ros2StressEvidenceError("pressure campaign snapshot is invalid")
    limits = _parse_pressure_limits(value["limits"])
    if limits != _pressure_limits(profile):
        raise Ros2StressEvidenceError("pressure limits are inconsistent")
    observations = _parse_pressure_observations(value["observations"])
    expected_checks = _pressure_checks(profile, None, observations)
    checks = value["checks"]
    if not isinstance(checks, list) or len(checks) != len(expected_checks):
        raise Ros2StressEvidenceError("pressure checks are invalid")
    parsed_checks: dict[str, bool] = {}
    for check in checks:
        if not isinstance(check, Mapping) or set(check) != {"code", "passed"}:
            raise Ros2StressEvidenceError("pressure check fields are invalid")
        if not isinstance(check["code"], str) or type(check["passed"]) is not bool:
            raise Ros2StressEvidenceError("pressure check is invalid")
        if check["code"] in parsed_checks:
            raise Ros2StressEvidenceError("pressure check codes must be unique")
        parsed_checks[check["code"]] = check["passed"]
    if parsed_checks != expected_checks:
        raise Ros2StressEvidenceError("pressure checks are inconsistent")
    if type(value["passed"]) is not bool or value["passed"] is not all(expected_checks.values()):
        raise Ros2StressEvidenceError("pressure report verdict is inconsistent")
    expected_report_id = (
        "ros2-pressure-"
        + _snapshot(
            {
                "profile": profile_id,
                "campaign": campaign_snapshot,
                "observations": observations,
            }
        )[:24]
    )
    if value["report_id"] != expected_report_id:
        raise Ros2StressEvidenceError("pressure report id is inconsistent")
    if not isinstance(value["snapshot"], str) or not _DIGEST.fullmatch(value["snapshot"]):
        raise Ros2StressEvidenceError("pressure report snapshot is invalid")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2StressEvidenceError("pressure report snapshot does not match")
    return dict(value)


def build_ros2_resilience_report(
    *,
    test_id: str,
    source_snapshot: str,
    container_image_id: str,
    started_at: str,
    finished_at: str,
    run_summaries: Sequence[Mapping[str, Any]],
    timeline: Sequence[Mapping[str, Any]],
    metrics: Mapping[str, Any],
    artifacts: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Build one strict, content-addressed #008-#012 simulation verdict."""

    profile = ROS2_RESILIENCE_PROFILES.get(test_id)
    if profile is None:
        raise Ros2StressEvidenceError("resilience test profile is unsupported")
    runs = _parse_resilience_runs(run_summaries)
    events = _parse_resilience_timeline(timeline)
    observed_metrics = _parse_resilience_metrics(profile, metrics)
    evidence_artifacts = _parse_resilience_artifacts(artifacts)
    started = _utc_text(started_at, "resilience started_at")
    finished = _utc_text(finished_at, "resilience finished_at")
    thresholds = _resilience_thresholds(profile)
    checks = _resilience_checks(
        profile,
        runs,
        events,
        observed_metrics,
        evidence_artifacts,
        started,
        finished,
    )
    _digest_text(source_snapshot, "resilience source snapshot")
    _image_digest_text(container_image_id, "resilience container image id")
    report: dict[str, Any] = {
        "contract_version": ROS2_RESILIENCE_REPORT_VERSION,
        "report_id": "ros2-resilience-"
        + _resilience_snapshot(
            {
                "test_id": test_id,
                "source_snapshot": source_snapshot,
                "container_image_id": container_image_id,
                "runs": runs,
                "timeline": events,
                "metrics": observed_metrics,
                "artifacts": evidence_artifacts,
            }
        )[:24],
        "test_id": test_id,
        "episode": profile["episode"],
        "title": profile["title"],
        "execution_mode": "simulation",
        "source_snapshot": source_snapshot,
        "container_image_id": container_image_id,
        "started_at": started_at,
        "finished_at": finished_at,
        "run_summaries": runs,
        "timeline": events,
        "metrics": observed_metrics,
        "thresholds": thresholds,
        "conditions": profile["conditions"],
        "artifacts": evidence_artifacts,
        "checks": [{"code": code, "passed": passed} for code, passed in checks.items()],
        "passed": all(checks.values()),
    }
    report["snapshot"] = _resilience_snapshot(report)
    return parse_ros2_resilience_report(report)


def parse_ros2_resilience_report(value: Any) -> dict[str, Any]:
    """Strictly parse and recompute one advanced ROS 2 pressure verdict."""

    if not isinstance(value, Mapping) or set(value) != _RESILIENCE_REPORT_FIELDS:
        raise Ros2StressEvidenceError("resilience report fields do not match")
    if value["contract_version"] != ROS2_RESILIENCE_REPORT_VERSION:
        raise Ros2StressEvidenceError("resilience report version is unsupported")
    test_id = value["test_id"]
    profile = ROS2_RESILIENCE_PROFILES.get(test_id)
    if profile is None:
        raise Ros2StressEvidenceError("resilience test profile is unsupported")
    if value["episode"] != profile["episode"] or value["title"] != profile["title"]:
        raise Ros2StressEvidenceError("resilience profile identity is inconsistent")
    if value["execution_mode"] != "simulation":
        raise Ros2StressEvidenceError("resilience execution mode is invalid")
    _digest_text(value["source_snapshot"], "resilience source snapshot")
    _image_digest_text(value["container_image_id"], "resilience container image id")
    started = _utc_text(value["started_at"], "resilience started_at")
    finished = _utc_text(value["finished_at"], "resilience finished_at")
    runs = _parse_resilience_runs(value["run_summaries"])
    events = _parse_resilience_timeline(value["timeline"])
    metrics = _parse_resilience_metrics(profile, value["metrics"])
    if value["thresholds"] != _resilience_thresholds(profile):
        raise Ros2StressEvidenceError("resilience thresholds are inconsistent")
    if value["conditions"] != profile["conditions"]:
        raise Ros2StressEvidenceError("resilience conditions are inconsistent")
    artifacts = _parse_resilience_artifacts(value["artifacts"])
    expected_checks = _resilience_checks(
        profile, runs, events, metrics, artifacts, started, finished
    )
    checks = value["checks"]
    if not isinstance(checks, list) or len(checks) != len(expected_checks):
        raise Ros2StressEvidenceError("resilience checks are invalid")
    parsed_checks: dict[str, bool] = {}
    for check in checks:
        if not isinstance(check, Mapping) or set(check) != {"code", "passed"}:
            raise Ros2StressEvidenceError("resilience check fields are invalid")
        if not isinstance(check["code"], str) or type(check["passed"]) is not bool:
            raise Ros2StressEvidenceError("resilience check is invalid")
        if check["code"] in parsed_checks:
            raise Ros2StressEvidenceError("resilience check codes must be unique")
        parsed_checks[check["code"]] = check["passed"]
    if parsed_checks != expected_checks:
        raise Ros2StressEvidenceError("resilience checks are inconsistent")
    if type(value["passed"]) is not bool or value["passed"] is not all(expected_checks.values()):
        raise Ros2StressEvidenceError("resilience verdict is inconsistent")
    expected_report_id = (
        "ros2-resilience-"
        + _resilience_snapshot(
            {
                "test_id": test_id,
                "source_snapshot": value["source_snapshot"],
                "container_image_id": value["container_image_id"],
                "runs": runs,
                "timeline": events,
                "metrics": metrics,
                "artifacts": artifacts,
            }
        )[:24]
    )
    if value["report_id"] != expected_report_id:
        raise Ros2StressEvidenceError("resilience report id is inconsistent")
    _digest_text(value["snapshot"], "resilience report snapshot")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _resilience_snapshot(unsigned):
        raise Ros2StressEvidenceError("resilience report snapshot does not match")
    return dict(value)


def build_ros2_resilience_series(
    reports: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Aggregate the complete #008-#012 set without hiding failed episodes."""

    if isinstance(reports, (str, bytes)):
        raise Ros2StressEvidenceError("resilience series reports are invalid")
    parsed = [parse_ros2_resilience_report(item) for item in reports]
    by_id = {item["test_id"]: item for item in parsed}
    expected_ids = list(ROS2_RESILIENCE_PROFILES)
    if len(parsed) != len(by_id) or set(by_id) != set(expected_ids):
        raise Ros2StressEvidenceError("resilience series must contain #008-#012 once")
    ordered = [by_id[test_id] for test_id in expected_ids]
    report_snapshots = {item["test_id"]: item["snapshot"] for item in ordered}
    passed_reports = [item["test_id"] for item in ordered if item["passed"] is True]
    failed_reports = [item["test_id"] for item in ordered if item["passed"] is False]
    series: dict[str, Any] = {
        "contract_version": ROS2_RESILIENCE_SERIES_VERSION,
        "series_id": "ros2-resilience-series-"
        + _resilience_snapshot({"reports": report_snapshots})[:24],
        "episode_order": [item["episode"] for item in ordered],
        "report_snapshots": report_snapshots,
        "passed_reports": passed_reports,
        "failed_reports": failed_reports,
        "complete": True,
        "all_passed": not failed_reports,
    }
    series["snapshot"] = _resilience_snapshot(series)
    return parse_ros2_resilience_series(series)


def parse_ros2_resilience_series(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != _RESILIENCE_SERIES_FIELDS:
        raise Ros2StressEvidenceError("resilience series fields do not match")
    if value["contract_version"] != ROS2_RESILIENCE_SERIES_VERSION:
        raise Ros2StressEvidenceError("resilience series version is unsupported")
    expected_ids = list(ROS2_RESILIENCE_PROFILES)
    if value["episode_order"] != [
        ROS2_RESILIENCE_PROFILES[test_id]["episode"] for test_id in expected_ids
    ]:
        raise Ros2StressEvidenceError("resilience series episode order is invalid")
    snapshots = value["report_snapshots"]
    if not isinstance(snapshots, Mapping) or list(snapshots) != expected_ids:
        raise Ros2StressEvidenceError("resilience series report set is invalid")
    for snapshot in snapshots.values():
        _digest_text(snapshot, "resilience series report snapshot")
    passed = value["passed_reports"]
    failed = value["failed_reports"]
    if not isinstance(passed, list) or not isinstance(failed, list):
        raise Ros2StressEvidenceError("resilience series verdict lists are invalid")
    if len(passed) != len(set(passed)) or len(failed) != len(set(failed)):
        raise Ros2StressEvidenceError("resilience series verdict lists are invalid")
    if set(passed).intersection(failed) or set(passed).union(failed) != set(expected_ids):
        raise Ros2StressEvidenceError("resilience series verdict partition is invalid")
    if type(value["complete"]) is not bool or value["complete"] is not True:
        raise Ros2StressEvidenceError("resilience series completeness is invalid")
    if type(value["all_passed"]) is not bool or value["all_passed"] is not (not failed):
        raise Ros2StressEvidenceError("resilience series pass state is inconsistent")
    expected_series_id = (
        "ros2-resilience-series-" + _resilience_snapshot({"reports": dict(snapshots)})[:24]
    )
    if value["series_id"] != expected_series_id:
        raise Ros2StressEvidenceError("resilience series id is inconsistent")
    _digest_text(value["snapshot"], "resilience series snapshot")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _resilience_snapshot(unsigned):
        raise Ros2StressEvidenceError("resilience series snapshot does not match")
    return dict(value)


def _parse_resilience_runs(value: Any) -> list[dict[str, Any]]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise Ros2StressEvidenceError("resilience run summaries are invalid")
    if not 1 <= len(value) <= 6592:
        raise Ros2StressEvidenceError("resilience run summaries are invalid")
    parsed: list[dict[str, Any]] = []
    for index, item in enumerate(value, start=1):
        fields = {"run", "run_id", "passed", "snapshot", "duration_seconds"}
        if not isinstance(item, Mapping) or set(item) != fields:
            raise Ros2StressEvidenceError("resilience run summary fields are invalid")
        if item["run"] != index:
            raise Ros2StressEvidenceError("resilience run sequence is invalid")
        if not isinstance(item["run_id"], str) or not 1 <= len(item["run_id"]) <= 128:
            raise Ros2StressEvidenceError("resilience run id is invalid")
        if type(item["passed"]) is not bool:
            raise Ros2StressEvidenceError("resilience run verdict is invalid")
        _digest_text(item["snapshot"], "resilience run snapshot")
        _number(item["duration_seconds"], "resilience run duration", maximum=604800.0)
        parsed.append(dict(item))
    snapshots = [item["snapshot"] for item in parsed]
    if len(snapshots) != len(set(snapshots)):
        raise Ros2StressEvidenceError("resilience run snapshots must be unique")
    return parsed


def _parse_resilience_timeline(value: Any) -> list[dict[str, Any]]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise Ros2StressEvidenceError("resilience timeline is invalid")
    if not 2 <= len(value) <= 4096:
        raise Ros2StressEvidenceError("resilience timeline is invalid")
    parsed: list[dict[str, Any]] = []
    for index, item in enumerate(value, start=1):
        if not isinstance(item, Mapping) or set(item) != {"sequence", "event", "at"}:
            raise Ros2StressEvidenceError("resilience timeline fields are invalid")
        if item["sequence"] != index:
            raise Ros2StressEvidenceError("resilience timeline sequence is invalid")
        if not isinstance(item["event"], str) or not re.fullmatch(
            r"[a-z][a-z0-9_]{1,63}", item["event"]
        ):
            raise Ros2StressEvidenceError("resilience timeline event is invalid")
        _utc_text(item["at"], "resilience timeline time")
        parsed.append(dict(item))
    return parsed


def _parse_resilience_metrics(profile: Mapping[str, Any], value: Any) -> dict[str, float]:
    expected = profile["metrics"]
    if not isinstance(value, Mapping) or set(value) != set(expected):
        raise Ros2StressEvidenceError("resilience metrics are invalid")
    return {name: _signed_number(item, f"resilience metric {name}") for name, item in value.items()}


def _parse_resilience_artifacts(value: Any) -> list[dict[str, Any]]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise Ros2StressEvidenceError("resilience artifacts are invalid")
    if not 1 <= len(value) <= 64:
        raise Ros2StressEvidenceError("resilience artifacts are invalid")
    parsed: list[dict[str, Any]] = []
    for item in value:
        if not isinstance(item, Mapping) or set(item) != {"kind", "path", "sha256", "bytes"}:
            raise Ros2StressEvidenceError("resilience artifact fields are invalid")
        kind = item["kind"]
        path = item["path"]
        if not isinstance(kind, str) or not re.fullmatch(r"[a-z][a-z0-9_]{1,63}", kind):
            raise Ros2StressEvidenceError("resilience artifact kind is invalid")
        if (
            not isinstance(path, str)
            or not 1 <= len(path) <= 512
            or path.startswith(("/", "~"))
            or "://" in path
            or ".." in path.split("/")
        ):
            raise Ros2StressEvidenceError("resilience artifact path is invalid")
        _digest_text(item["sha256"], "resilience artifact digest")
        size = item["bytes"]
        if isinstance(size, bool) or not isinstance(size, int) or not 1 <= size <= 1 << 40:
            raise Ros2StressEvidenceError("resilience artifact size is invalid")
        parsed.append(dict(item))
    paths = [item["path"] for item in parsed]
    if len(paths) != len(set(paths)):
        raise Ros2StressEvidenceError("resilience artifact paths must be unique")
    return parsed


def _resilience_thresholds(profile: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        name: {"comparator": comparator, "value": float(threshold)}
        for name, (comparator, threshold) in profile["metrics"].items()
    }


def _resilience_checks(
    profile: Mapping[str, Any],
    runs: Sequence[Mapping[str, Any]],
    events: Sequence[Mapping[str, Any]],
    metrics: Mapping[str, float],
    artifacts: Sequence[Mapping[str, Any]],
    started: datetime,
    finished: datetime,
) -> dict[str, bool]:
    required_events = list(profile["required_events"])
    observed_events = [item["event"] for item in events]
    event_positions = [
        observed_events.index(event) for event in required_events if event in observed_events
    ]
    event_times = [_utc_text(item["at"], "resilience timeline time") for item in events]
    metric_checks = [
        _metric_passes(metrics[name], comparator, float(threshold))
        for name, (comparator, threshold) in profile["metrics"].items()
    ]
    return {
        "run_volume_met": len(runs) >= int(profile["minimum_runs"]),
        "all_runs_completed_safely": all(item["passed"] is True for item in runs),
        "required_events_observed": len(event_positions) == len(required_events),
        "required_event_order_valid": len(event_positions) == len(required_events)
        and event_positions == sorted(event_positions),
        "timeline_monotonic": event_times == sorted(event_times),
        "timeline_within_run_window": started < finished
        and all(started <= item <= finished for item in event_times),
        "metric_thresholds_met": all(metric_checks),
        "required_artifacts_present": set(profile["artifact_kinds"]).issubset(
            {item["kind"] for item in artifacts}
        ),
    }


def _metric_passes(value: float, comparator: str, threshold: float) -> bool:
    if comparator == "lte":
        return value <= threshold
    if comparator == "gte":
        return value >= threshold
    if comparator == "eq":
        return value == threshold
    raise Ros2StressEvidenceError("resilience metric comparator is unsupported")


def _digest_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not _DIGEST.fullmatch(value):
        raise Ros2StressEvidenceError(f"{label} must be a SHA-256 digest")
    return value


def _image_digest_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.startswith("sha256:"):
        raise Ros2StressEvidenceError(f"{label} is invalid")
    _digest_text(value.removeprefix("sha256:"), label)
    return value


def _signed_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Ros2StressEvidenceError(f"{label} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not -1e12 <= parsed <= 1e12:
        raise Ros2StressEvidenceError(f"{label} is outside its valid range")
    return parsed


def _resilience_snapshot(value: Mapping[str, Any]) -> str:
    """Hash JSON in the same canonical number form used by JSON.stringify."""

    def normalize(item: Any) -> Any:
        if isinstance(item, Mapping):
            return {key: normalize(child) for key, child in item.items()}
        if isinstance(item, list):
            return [normalize(child) for child in item]
        if isinstance(item, float) and item.is_integer():
            return int(item)
        return item

    return hashlib.sha256(
        json.dumps(
            normalize(value),
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()


def _pressure_limits(profile: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "cpu_limit_millicores": int(profile["cpu_limit_millicores"]),
        "memory_limit_mib": int(profile["memory_limit_mib"]),
        "network_delay_ms": int(profile["network_delay_ms"]),
        "network_jitter_ms": int(profile["network_jitter_ms"]),
        "network_loss_percent": float(profile["network_loss_percent"]),
        "minimum_execution_runs": int(profile["minimum_execution_runs"]),
    }


def _parse_pressure_limits(value: Any) -> dict[str, Any]:
    fields = {
        "cpu_limit_millicores",
        "memory_limit_mib",
        "network_delay_ms",
        "network_jitter_ms",
        "network_loss_percent",
        "minimum_execution_runs",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise Ros2StressEvidenceError("pressure limits are invalid")
    integer_limits = {
        "cpu_limit_millicores": (100, 64_000),
        "memory_limit_mib": (256, 262_144),
        "network_delay_ms": (0, 60_000),
        "network_jitter_ms": (0, 60_000),
        "minimum_execution_runs": (1, 6592),
    }
    parsed: dict[str, Any] = {}
    for field, (minimum, maximum) in integer_limits.items():
        item = value[field]
        if isinstance(item, bool) or not isinstance(item, int) or not minimum <= item <= maximum:
            raise Ros2StressEvidenceError(f"pressure limit {field} is invalid")
        parsed[field] = item
    parsed["network_loss_percent"] = _number(
        value["network_loss_percent"],
        "pressure network loss",
        maximum=100.0,
    )
    return parsed


def _parse_pressure_observations(value: Any) -> dict[str, Any]:
    fields = {
        "campaign_passed",
        "campaign_execution_runs",
        "scenario_log_count",
        "completed_scenarios",
        "cpu_limit_verified",
        "memory_limit_verified",
        "max_memory_bytes",
        "cpu_usage_usec",
        "cpu_throttled_usec",
        "oom_kill_count",
        "unexpected_process_deaths",
        "network_injection_verified",
        "recovery_verified",
        "elapsed_seconds",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise Ros2StressEvidenceError("pressure observations are invalid")
    parsed = dict(value)
    for field in (
        "campaign_passed",
        "cpu_limit_verified",
        "memory_limit_verified",
        "network_injection_verified",
        "recovery_verified",
    ):
        if type(parsed[field]) is not bool:
            raise Ros2StressEvidenceError(f"pressure observation {field} is invalid")
    integer_limits = {
        "campaign_execution_runs": (1, 6592),
        "scenario_log_count": (0, 6592),
        "completed_scenarios": (0, 6592),
        "max_memory_bytes": (0, 1 << 50),
        "cpu_usage_usec": (0, 1 << 62),
        "cpu_throttled_usec": (0, 1 << 62),
        "oom_kill_count": (0, 1_000_000),
        "unexpected_process_deaths": (0, 6592),
    }
    for field, (minimum, maximum) in integer_limits.items():
        item = parsed[field]
        if isinstance(item, bool) or not isinstance(item, int) or not minimum <= item <= maximum:
            raise Ros2StressEvidenceError(f"pressure observation {field} is invalid")
    parsed["elapsed_seconds"] = _number(
        parsed["elapsed_seconds"],
        "pressure elapsed time",
        maximum=7 * 24 * 60 * 60,
    )
    return parsed


def _pressure_checks(
    profile: Mapping[str, Any],
    campaign: Mapping[str, Any] | None,
    observations: Mapping[str, Any],
) -> dict[str, bool]:
    campaign_passed = observations["campaign_passed"] is True
    campaign_runs = int(observations["campaign_execution_runs"])
    if campaign is not None:
        campaign_passed = campaign_passed and campaign["passed"] is True
        campaign_runs = int(campaign["total_execution_runs"])
    memory_limit_bytes = int(profile["memory_limit_mib"]) * 1024 * 1024
    network_expected = profile["mode"] == "network"
    return {
        "campaign_profile_matches": campaign is None
        or campaign["profile_id"] == profile["campaign_profile_id"],
        "campaign_passed": campaign_passed,
        "execution_volume_met": campaign_runs >= int(profile["minimum_execution_runs"]),
        "scenario_logs_complete": observations["scenario_log_count"] == campaign_runs,
        "all_scenarios_completed": observations["completed_scenarios"] == campaign_runs,
        "cpu_limit_verified": observations["cpu_limit_verified"] is True,
        "memory_limit_verified": observations["memory_limit_verified"] is True,
        "memory_observed": int(observations["max_memory_bytes"]) > 0,
        "memory_within_limit": int(observations["max_memory_bytes"]) <= memory_limit_bytes,
        "cpu_usage_observed": int(observations["cpu_usage_usec"]) > 0,
        "no_oom_kills": observations["oom_kill_count"] == 0,
        "no_unexpected_process_deaths": observations["unexpected_process_deaths"] == 0,
        "network_injection_state_matches": observations["network_injection_verified"]
        is network_expected,
        "recovery_verified": observations["recovery_verified"] is True,
        "elapsed_time_observed": float(observations["elapsed_seconds"]) > 0.0,
    }


def _parse_build_provenance(value: Any) -> dict[str, Any]:
    fields = {
        "source_snapshot",
        "container_image_id",
        "ros_distro",
        "simulator",
        "execution_mode",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise Ros2StressEvidenceError("stress campaign build provenance is invalid")
    if not isinstance(value["source_snapshot"], str) or not _DIGEST.fullmatch(
        value["source_snapshot"]
    ):
        raise Ros2StressEvidenceError("stress campaign source snapshot is invalid")
    image_id = value["container_image_id"]
    if (
        not isinstance(image_id, str)
        or not image_id.startswith("sha256:")
        or not _DIGEST.fullmatch(image_id.removeprefix("sha256:"))
    ):
        raise Ros2StressEvidenceError("stress campaign container image id is invalid")
    if value["ros_distro"] != "jazzy":
        raise Ros2StressEvidenceError("stress campaign ROS distribution is invalid")
    if value["simulator"] != "gazebo-harmonic":
        raise Ros2StressEvidenceError("stress campaign simulator is invalid")
    if value["execution_mode"] != "simulation":
        raise Ros2StressEvidenceError("stress campaign execution mode is invalid")
    return dict(value)


def _parse_runtime_hygiene(value: Any) -> dict[str, Any]:
    fields = {
        "scenario_log_count",
        "expected_forced_terminations",
        "unexpected_process_deaths",
        "unexpected_exit_codes",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise Ros2StressEvidenceError("stress campaign runtime hygiene is invalid")
    for field in (
        "scenario_log_count",
        "expected_forced_terminations",
        "unexpected_process_deaths",
    ):
        item = value[field]
        if isinstance(item, bool) or not isinstance(item, int) or not 0 <= item <= 6592:
            raise Ros2StressEvidenceError(f"stress campaign runtime hygiene {field} is invalid")
    codes = value["unexpected_exit_codes"]
    if (
        not isinstance(codes, list)
        or len(codes) > 64
        or len(codes) != len(set(codes))
        or any(isinstance(code, bool) or not isinstance(code, int) for code in codes)
        or any(not -255 <= code <= 255 for code in codes)
    ):
        raise Ros2StressEvidenceError("stress campaign runtime hygiene exit codes are invalid")
    if (value["unexpected_process_deaths"] == 0) is not (not codes):
        raise Ros2StressEvidenceError("stress campaign runtime hygiene verdict is inconsistent")
    return dict(value)


def _percentile(values: Sequence[float], ratio: float, digits: int) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(len(ordered) * ratio) - 1)
    return round(float(ordered[index]), digits)


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
