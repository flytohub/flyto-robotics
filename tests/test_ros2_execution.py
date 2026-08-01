from __future__ import annotations

import copy
import json
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from flyto_robotics.resource_binding import load_resource_plan
from flyto_robotics.ros2_execution import (
    Ros2ExecutionError,
    authorize_ros2_execution,
    parse_ros2_execution_grant,
    resolve_ros2_execution_target,
)
from flyto_robotics.ros2_pairing import (
    build_ros2_runtime_snapshot,
    load_ros2_adapter_manifest,
    load_ros2_runtime_snapshot,
    standard_ros2_adapter_manifest,
)

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "examples/ros2-adapters/flyto2-standard.json"
RUNTIME_PATH = ROOT / "examples/ros2-runtime/ready-sim.json"
RESOURCE_PLAN_PATH = ROOT / "examples/resource-plans/nav2-hospital-delivery.json"
OBSERVED_AT = datetime(2026, 8, 1, 10, 0, tzinfo=timezone.utc)


def _authorize() -> tuple[dict, dict, dict]:
    manifest = load_ros2_adapter_manifest(MANIFEST_PATH)
    runtime = load_ros2_runtime_snapshot(RUNTIME_PATH)
    grant = authorize_ros2_execution(
        resource_plan=load_resource_plan(RESOURCE_PLAN_PATH),
        manifest=manifest,
        runtime=runtime,
        workflow_id="hospital_delivery.v1",
        resource_id="flyto-rover-sim-001",
        capability_id="robotics.motion.navigate@1",
        target_space_id="gazebo-nav2-lab",
        observed_at=OBSERVED_AT,
    )
    return grant, manifest, runtime


def test_execution_grant_binds_every_authority_without_graph_details() -> None:
    grant, manifest, runtime = _authorize()

    assert grant["adapter_id"] == "ros2.nav2.navigate_to_pose.v1"
    assert grant["resource_plan_snapshot"] == (
        "43a9edfecd3c45fa20e0b73fd965d2f860b6ffc6ae5a8991eaec60c57ea3931f"
    )
    assert grant["profile_snapshot"] == manifest["snapshot"]
    assert grant["runtime_snapshot"] == runtime["snapshot"]
    encoded = json.dumps(grant, sort_keys=True)
    assert "/navigate_to_pose" not in encoded
    assert "nav2_msgs" not in encoded

    target = resolve_ros2_execution_target(
        grant,
        manifest,
        runtime,
        observed_at=OBSERVED_AT,
    )
    assert target.interface_name == "/navigate_to_pose"
    assert target.interface_type == "nav2_msgs/action/NavigateToPose"
    assert target.grant_snapshot == grant["snapshot"]


def test_grant_tampering_expiry_and_cross_context_reuse_fail_closed() -> None:
    grant, manifest, runtime = _authorize()
    tampered = copy.deepcopy(grant)
    tampered["resource_id"] = "attacker-robot"
    with pytest.raises(Ros2ExecutionError, match="snapshot does not match"):
        parse_ros2_execution_grant(tampered)

    with pytest.raises(Ros2ExecutionError, match="expired"):
        resolve_ros2_execution_target(
            grant,
            manifest,
            runtime,
            observed_at=OBSERVED_AT + timedelta(seconds=61),
        )

    other_robot = standard_ros2_adapter_manifest("another-robot")
    with pytest.raises(Ros2ExecutionError, match="robot_id does not match"):
        resolve_ros2_execution_target(
            grant,
            other_robot,
            runtime,
            observed_at=OBSERVED_AT,
        )


def test_unready_graph_or_wrong_resource_binding_cannot_issue_a_grant() -> None:
    manifest = standard_ros2_adapter_manifest("flyto-rover-sim-001")
    runtime = build_ros2_runtime_snapshot(
        manifest,
        deployment_mode="simulation",
        emergency_stop_ready=False,
        adapter_states=[
            {
                "adapter_id": "ros2.nav2.navigate_to_pose.v1",
                "status": "ready",
                "interface_available": True,
                "lifecycle_state": "active",
                "observation_sequence": "test:ready:1",
            }
        ],
        observed_at=OBSERVED_AT,
    )
    arguments = {
        "resource_plan": load_resource_plan(RESOURCE_PLAN_PATH),
        "manifest": manifest,
        "runtime": runtime,
        "workflow_id": "hospital_delivery.v1",
        "resource_id": "flyto-rover-sim-001",
        "capability_id": "robotics.motion.navigate@1",
        "target_space_id": "gazebo-nav2-lab",
        "observed_at": OBSERVED_AT,
    }
    with pytest.raises(Ros2ExecutionError, match="emergency_stop_ready"):
        authorize_ros2_execution(**arguments)

    ready_runtime = load_ros2_runtime_snapshot(RUNTIME_PATH)
    arguments["runtime"] = ready_runtime
    arguments["resource_id"] = "another-robot"
    with pytest.raises(Ros2ExecutionError, match="exactly one matching"):
        authorize_ros2_execution(**arguments)


def test_cli_replays_and_prints_the_redacted_execution_grant() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "flyto_robotics.cli",
            "authorize-ros2-execution",
            "--manifest",
            str(MANIFEST_PATH),
            "--runtime",
            str(RUNTIME_PATH),
            "--resource-plan",
            str(RESOURCE_PLAN_PATH),
            "--workflow",
            "hospital_delivery.v1",
            "--resource",
            "flyto-rover-sim-001",
            "--capability",
            "robotics.motion.navigate@1",
            "--space",
            "gazebo-nav2-lab",
            "--at",
            "2026-08-01T10:00:00Z",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )

    assert completed.returncode == 0, completed.stderr
    grant = json.loads(completed.stdout)
    assert grant["contract_version"] == "flyto.robotics.ros2-execution-grant.v1"
    assert "/navigate_to_pose" not in completed.stdout
