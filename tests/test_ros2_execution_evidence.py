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
from flyto_robotics.ros2_execution_evidence import (
    Ros2ExecutionEvidenceError,
    build_ros2_execution_evidence,
    evaluate_closed_loop_evidence,
    parse_ros2_execution_evidence,
)
from flyto_robotics.ros2_pairing import (
    load_ros2_adapter_manifest,
    load_ros2_runtime_snapshot,
)

ROOT = Path(__file__).resolve().parents[1]
AT = datetime(2026, 8, 1, 10, 0, tzinfo=timezone.utc)


def _success_evidence() -> dict:
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
    prepared = prepare_authorized_navigation(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id="hospital.route.blue_end",
        frame_id="map",
        observed_at=AT,
    )
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    monitor.accept_goal()
    monitor.feedback(1.0)
    outcome = monitor.finish(
        "succeeded",
        StationPose("robot.current", 1.44, 0.0, 0.0),
        finished_at=AT + timedelta(seconds=5),
    )
    return build_ros2_execution_evidence(
        grant,
        prepared,
        outcome,
        scenario="success",
        finished_at=AT + timedelta(seconds=5),
    )


def test_success_evidence_proves_authority_feedback_motion_and_goal() -> None:
    evidence = _success_evidence()
    verdict = evaluate_closed_loop_evidence(evidence, expected_scenario="success")

    assert verdict["passed"] is True
    assert evidence["displacement_m"] > 1.4
    assert evidence["goal_error_m"] < 0.02
    assert evidence["post_stop_drift_m"] == 0.0
    encoded = json.dumps(evidence, sort_keys=True)
    assert "/navigate_to_pose" not in encoded
    assert "nav2_msgs" not in encoded
    assert "cmd_vel" not in encoded


def test_evidence_tampering_and_false_success_fail_closed() -> None:
    evidence = _success_evidence()
    tampered = copy.deepcopy(evidence)
    tampered["feedback_count"] = 0
    with pytest.raises(Ros2ExecutionEvidenceError, match="snapshot"):
        parse_ros2_execution_evidence(tampered)

    resigned = copy.deepcopy(evidence)
    resigned["displacement_m"] = 0.0
    unsigned = {key: value for key, value in resigned.items() if key != "snapshot"}
    import hashlib

    resigned["snapshot"] = hashlib.sha256(
        json.dumps(unsigned, separators=(",", ":"), sort_keys=True).encode()
    ).hexdigest()
    verdict = evaluate_closed_loop_evidence(resigned, expected_scenario="success")
    assert verdict["passed"] is False
    assert next(
        check for check in verdict["checks"] if check["code"] == "physical_displacement"
    )["passed"] is False

    unstable = copy.deepcopy(evidence)
    unstable["post_stop_drift_m"] = 0.2
    unsigned = {key: value for key, value in unstable.items() if key != "snapshot"}
    unstable["snapshot"] = hashlib.sha256(
        json.dumps(unsigned, separators=(",", ":"), sort_keys=True).encode()
    ).hexdigest()
    verdict = evaluate_closed_loop_evidence(unstable, expected_scenario="success")
    assert verdict["passed"] is False
    assert next(
        check for check in verdict["checks"] if check["code"] == "post_stop_stability"
    )["passed"] is False
