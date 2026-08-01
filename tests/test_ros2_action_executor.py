from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from flyto_robotics.contracts import StationPose
from flyto_robotics.resource_binding import load_resource_plan
from flyto_robotics.ros2_action_executor import (
    NavigationExecutionMonitor,
    Ros2ActionExecutionError,
    prepare_authorized_navigation,
)
from flyto_robotics.ros2_execution import authorize_ros2_execution
from flyto_robotics.ros2_pairing import (
    load_ros2_adapter_manifest,
    load_ros2_runtime_snapshot,
)

ROOT = Path(__file__).resolve().parents[1]
AT = datetime(2026, 8, 1, 10, 0, tzinfo=timezone.utc)


def _prepared():
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
    semantic_map = __import__("json").loads(
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
    return prepared, grant


def test_semantic_goal_is_resolved_only_behind_the_exact_grant() -> None:
    prepared, _grant = _prepared()

    assert prepared.location_id == "hospital.route.blue_end"
    assert prepared.pose.x == 1.45
    assert prepared.target.interface_type == "nav2_msgs/action/NavigateToPose"


def test_monitor_requires_acceptance_feedback_and_physical_terminal_pose() -> None:
    prepared, _grant = _prepared()
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    monitor.accept_goal()
    monitor.feedback(1.2)
    monitor.feedback(0.1)
    outcome = monitor.finish(
        "succeeded",
        StationPose("robot.current", 1.42, 0.01, 0.0),
        finished_at=AT + timedelta(seconds=8),
    )

    assert outcome.status == "succeeded"
    assert outcome.feedback_count == 2
    assert outcome.displacement_m > 1.4
    assert outcome.goal_error_m < 0.04
    assert outcome.post_stop_drift_m == 0.0

    no_feedback = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    no_feedback.accept_goal()
    with pytest.raises(Ros2ActionExecutionError, match="feedback"):
        no_feedback.finish(
            "succeeded",
            StationPose("robot.current", 1.45, 0.0, 0.0),
            finished_at=AT + timedelta(seconds=1),
        )


def test_feedback_before_local_acceptance_is_bounded_and_replayed_in_order() -> None:
    prepared, _grant = _prepared()
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    monitor.begin_goal_submission()
    monitor.feedback(1.4)

    assert monitor.feedback_count == 0
    assert monitor.event_codes[-1] == "server_available"

    monitor.accept_goal()

    assert monitor.feedback_count == 1
    assert monitor.event_codes[-2:] == ["goal_accepted", "feedback_observed"]


@pytest.mark.parametrize(
    ("reason", "expected_status", "safety"),
    [
        ("operator_cancel", "canceled", False),
        ("emergency_stop", "safety_stopped", True),
        ("timeout", "timed_out", False),
    ],
)
def test_cancel_paths_are_explicit_and_fail_closed(
    reason: str,
    expected_status: str,
    safety: bool,
) -> None:
    prepared, _grant = _prepared()
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    monitor.accept_goal()
    monitor.feedback(4.0)
    monitor.request_cancel(reason)
    outcome = monitor.finish(
        "canceled",
        StationPose("robot.current", 0.25, 0.0, 0.0),
        settled_pose=StationPose("robot.current", 0.26, 0.0, 0.0),
        finished_at=AT + timedelta(seconds=2),
    )

    assert outcome.status == expected_status
    assert outcome.cancel_requested is True
    assert outcome.safety_stop_observed is safety
    assert outcome.safety_stop_reason == ("emergency_stop" if safety else None)
    assert outcome.post_stop_drift_m == 0.01


def test_fault_stop_records_exact_reason_latency_and_independent_abort() -> None:
    prepared, _grant = _prepared()
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    monitor.accept_goal()
    monitor.feedback(4.0)
    monitor.observe_safety_stop(
        "lidar_stale",
        fault_injection_observed=True,
        safety_stop_latency_ms=412.5,
    )
    outcome = monitor.finish(
        "aborted",
        StationPose("robot.current", 0.2, 0.0, 0.0),
        settled_pose=StationPose("robot.current", 0.201, 0.0, 0.0),
        finished_at=AT + timedelta(seconds=2),
    )

    assert outcome.status == "safety_stopped"
    assert outcome.result_code == "aborted"
    assert outcome.cancel_requested is False
    assert outcome.safety_stop_reason == "lidar_stale"
    assert outcome.fault_injection_observed is True
    assert outcome.safety_stop_latency_ms == 412.5


def test_invalid_goal_frame_and_out_of_order_cancel_are_rejected() -> None:
    prepared, grant = _prepared()
    manifest = load_ros2_adapter_manifest(
        ROOT / "examples/ros2-adapters/flyto2-standard.json"
    )
    runtime = load_ros2_runtime_snapshot(
        ROOT / "examples/ros2-runtime/ready-sim.json"
    )
    semantic_map = __import__("json").loads(
        (ROOT / "examples/maps/atomic-color-route.json").read_text()
    )
    with pytest.raises(Ros2ActionExecutionError, match="goal frame"):
        prepare_authorized_navigation(
            grant=grant,
            manifest=manifest,
            runtime=runtime,
            semantic_map=semantic_map,
            location_id="hospital.route.blue_end",
            frame_id="base_link",
            observed_at=AT,
        )

    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=AT,
    )
    with pytest.raises(Ros2ActionExecutionError, match="executing"):
        monitor.request_cancel("operator_cancel")
