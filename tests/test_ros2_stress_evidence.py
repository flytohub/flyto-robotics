from __future__ import annotations

import copy
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from flyto_robotics.contracts import StationPose
from flyto_robotics.resource_binding import load_resource_plan
from flyto_robotics.ros2_action_executor import (
    NavigationExecutionMonitor,
    prepare_authorized_navigation,
)
from flyto_robotics.ros2_execution import authorize_ros2_execution
from flyto_robotics.ros2_execution_evidence import build_ros2_execution_evidence
from flyto_robotics.ros2_pairing import (
    load_ros2_adapter_manifest,
    load_ros2_runtime_snapshot,
)
from flyto_robotics.ros2_stress_evidence import (
    Ros2StressEvidenceError,
    build_ros2_stress_report,
    parse_ros2_stress_report,
    prove_expired_grant_rejected,
)

ROOT = Path(__file__).resolve().parents[1]
AT = datetime(2026, 8, 1, 10, 0, tzinfo=timezone.utc)
FAULT_REASONS = {
    "lidar_dropout": "lidar_stale",
    "odometry_freeze": "odometry_stale",
    "nav2_lifecycle_failure": "command_stale",
}


def _authority():
    manifest = load_ros2_adapter_manifest(
        ROOT / "examples/ros2-adapters/flyto2-standard.json"
    )
    runtime = load_ros2_runtime_snapshot(
        ROOT / "examples/ros2-runtime/ready-sim.json"
    )
    grant = authorize_ros2_execution(
        resource_plan=load_resource_plan(
            ROOT / "examples/resource-plans/nav2-hospital-delivery.json"
        ),
        manifest=manifest,
        runtime=runtime,
        workflow_id="hospital_delivery.v1",
        resource_id="flyto-rover-sim-001",
        capability_id="robotics.motion.navigate@1",
        target_space_id="gazebo-nav2-lab",
        observed_at=AT,
    )
    semantic_map = json.loads(
        (ROOT / "examples/maps/atomic-color-route.json").read_text()
    )
    return manifest, runtime, grant, semantic_map


def _execution(scenario: str, index: int) -> dict:
    manifest, runtime, grant, semantic_map = _authority()
    location = (
        "hospital.route.blue_end"
        if scenario == "success"
        else "hospital.route.yellow_end"
    )
    prepared = prepare_authorized_navigation(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id=location,
        frame_id="map",
        observed_at=AT,
    )
    started = AT + timedelta(seconds=index)
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=started,
    )
    monitor.accept_goal()
    monitor.feedback(1.0)
    if scenario == "success":
        result_code = "succeeded"
        terminal = StationPose("robot.current", 1.44, 0.0, 0.0)
    else:
        monitor.request_cancel(
            FAULT_REASONS[scenario],
            fault_injection_observed=True,
            safety_stop_latency_ms=350.0 + index,
        )
        result_code = "canceled"
        terminal = StationPose("robot.current", 0.2, 0.0, 0.0)
    finished = started + timedelta(seconds=2)
    outcome = monitor.finish(result_code, terminal, finished_at=finished)
    return build_ros2_execution_evidence(
        grant,
        prepared,
        outcome,
        scenario=scenario,
        finished_at=finished,
    )


def test_stress_report_closes_soak_faults_and_expired_grant() -> None:
    manifest, runtime, grant, semantic_map = _authority()
    probe = prove_expired_grant_rejected(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id="hospital.route.blue_end",
    )
    executions = [
        _execution("success", 1),
        _execution("success", 2),
        _execution("lidar_dropout", 3),
        _execution("odometry_freeze", 4),
        _execution("nav2_lifecycle_failure", 5),
    ]

    report = build_ros2_stress_report(executions, probe, requested_soak_runs=2)

    assert report["passed"] is True
    assert report["completed_soak_runs"] == 2
    assert report["grant_expiry_rejected"] is True
    assert report["max_safety_stop_latency_ms"] == 355.0
    assert len(set(report["evidence_snapshots"])) == 5


def test_stress_report_tampering_and_missing_fault_fail_closed() -> None:
    manifest, runtime, grant, semantic_map = _authority()
    probe = prove_expired_grant_rejected(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id="hospital.route.blue_end",
    )
    executions = [
        _execution("success", 1),
        _execution("lidar_dropout", 2),
        _execution("odometry_freeze", 3),
    ]
    report = build_ros2_stress_report(executions, probe, requested_soak_runs=1)
    assert report["passed"] is False
    assert next(
        check
        for check in report["checks"]
        if check["code"] == "fault_matrix_complete"
    )["passed"] is False

    tampered = copy.deepcopy(report)
    tampered["passed"] = True
    with pytest.raises(Ros2StressEvidenceError, match="verdict"):
        parse_ros2_stress_report(tampered)
