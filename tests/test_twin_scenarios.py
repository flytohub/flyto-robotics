"""Twin scenarios are runnable as written, and their verdict cannot be faked or loosened."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TWIN = ROOT / "sim" / "twin"
SCENARIOS = sorted((ROOT / "scenarios" / "twin").glob("*.json"))
OBSTACLE_STOP = ROOT / "scenarios" / "twin" / "demo-room-obstacle-stop.json"
WORKFLOW = ROOT / ".github" / "workflows" / "simulator.yml"


def _module():
    spec = importlib.util.spec_from_file_location(
        "twin_scenario_check", TWIN / "tools" / "scenario_check.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


check = _module()


def _scenario() -> dict:
    return json.loads(OBSTACLE_STOP.read_text(encoding="utf-8"))


def _passing_run() -> dict:
    """What a correct stop at the demo box looks like to the recorder."""
    return {
        "outcome": "failed",
        "evidence": {
            "motion_outcome": {
                "reason": "obstacle_blocked",
                "clearance_floor_m": 0.35,
                "start_pose": {"x": -1.8, "y": 0.0, "yaw": 0.0},
                "final_pose": {"x": -1.42, "y": 0.004, "yaw": 0.01},
                "stop_clearance": {
                    "floor_m": 0.35,
                    "nearest_range_m": 0.39,
                    "floor_held": True,
                    "travel_floor_held": True,
                },
            }
        },
        "goals": [
            {
                "action": "/drive_on_heading",
                "action_type": "nav2_msgs/action/DriveOnHeading",
                "after_result": False,
            }
        ],
        "motion_commands_after_result": [],
        "poses_after_result": [{"x": -1.42, "y": 0.004}, {"x": -1.421, "y": 0.004}],
    }


def _failed_checks(observed: dict) -> set[str]:
    verdict = check.evaluate(check.validate_scenario(_scenario(), twin_root=TWIN), observed)
    return {item["code"] for item in verdict["checks"] if not item["passed"]}


def test_every_twin_scenario_validates() -> None:
    assert SCENARIOS, "scenarios/twin holds at least the obstacle-stop scenario"
    for path in SCENARIOS:
        check.load_scenario(path, twin_root=TWIN)


def test_obstacle_stop_scenario_is_the_demo_box_with_the_fixed_floor() -> None:
    scenario = check.load_scenario(OBSTACLE_STOP, twin_root=TWIN)
    assert scenario["twin"]["world"].endswith("/flyto_demo_room.world")
    assert scenario["twin"]["start_pose"] == {"x": -1.8, "y": 0.0, "yaw": 0.0}
    assert scenario["call"]["capability_id"] == "motion.advance"
    # The request reaches past the box (near face 0.75 m ahead), so only a stop ends it.
    assert scenario["call"]["arguments"]["distance_m"] > 0.75
    assert scenario["expectations"]["clearance_floor_m"] == 0.35
    assert scenario["expectations"]["allowed_goal_action_types"] == [
        "nav2_msgs/action/DriveOnHeading"
    ]


def test_scenario_floor_matches_the_adapter_default(monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(ROOT))
    from flyto_robotics import generic_ros2_adapter

    monkeypatch.delenv("FLYTO_ROS2_MIN_CLEARANCE_M", raising=False)
    assert generic_ros2_adapter._clearance_floor() == check.SAFETY_FLOOR_M == 0.35


@pytest.mark.parametrize("floor", [0.3, 0.1, 0.0, 0.5])
def test_a_scenario_cannot_name_another_floor(floor) -> None:
    scenario = _scenario()
    scenario["expectations"]["clearance_floor_m"] = floor
    with pytest.raises(check.ScenarioError, match="never lowered"):
        check.validate_scenario(scenario, twin_root=TWIN)


@pytest.mark.parametrize(
    "action_type",
    sorted(check.DETOUR_OR_RECOVERY_ACTION_TYPES),
)
def test_a_scenario_cannot_allow_a_detour_or_recovery_goal(action_type) -> None:
    scenario = _scenario()
    scenario["expectations"]["allowed_goal_action_types"].append(action_type)
    with pytest.raises(check.ScenarioError, match="no-detour"):
        check.validate_scenario(scenario, twin_root=TWIN)


def test_a_scenario_world_must_ship_with_the_twin() -> None:
    scenario = _scenario()
    scenario["twin"]["world"] = "/opt/flyto-twin/worlds/missing.world"
    with pytest.raises(check.ScenarioError, match="sim/twin/worlds"):
        check.validate_scenario(scenario, twin_root=TWIN)


def test_a_correct_stop_passes() -> None:
    assert _failed_checks(_passing_run()) == set()


def test_a_stop_reported_as_completed_fails() -> None:
    run = _passing_run()
    run["outcome"] = "completed"
    run["evidence"]["motion_outcome"]["reason"] = "completed"
    assert _failed_checks(run) == {"outcome", "motion_reason"}


def test_an_automatic_detour_fails() -> None:
    run = _passing_run()
    run["goals"].append(
        {
            "action": "/navigate_through_poses",
            "action_type": "nav2_msgs/action/NavigateThroughPoses",
            "after_result": False,
        }
    )
    assert _failed_checks(run) == {"only_requested_goals", "goal_count"}


def test_a_goal_after_the_result_fails() -> None:
    run = _passing_run()
    run["goals"].append(
        {
            "action": "/drive_on_heading",
            "action_type": "nav2_msgs/action/DriveOnHeading",
            "after_result": True,
        }
    )
    assert "no_goal_after_result" in _failed_checks(run)


def test_motion_after_the_result_fails() -> None:
    run = _passing_run()
    run["motion_commands_after_result"] = [{"topic": "/cmd_vel", "msg": {"linear": {"x": 0.1}}}]
    run["poses_after_result"].append({"x": -1.30, "y": 0.0})
    assert _failed_checks(run) == {"no_motion_command_after_result", "held_still_after_result"}


def test_a_breached_or_lowered_floor_fails() -> None:
    breached = _passing_run()
    clearance = breached["evidence"]["motion_outcome"]["stop_clearance"]
    clearance.update({"nearest_range_m": 0.2, "floor_held": False, "travel_floor_held": False})
    assert _failed_checks(breached) == {"stopped_outside_floor"}

    lowered = _passing_run()
    lowered["evidence"]["motion_outcome"]["clearance_floor_m"] = 0.2
    lowered["evidence"]["motion_outcome"]["stop_clearance"]["floor_m"] = 0.2
    assert _failed_checks(lowered) == {"floor_not_lowered"}


def test_a_refusal_without_motion_fails() -> None:
    run = copy.deepcopy(_passing_run())
    run["outcome"] = "refused"
    run["evidence"] = {"reason_code": "path_blocked"}
    run["goals"] = []
    failed = _failed_checks(run)
    assert {"moved_toward_obstacle", "only_requested_goals", "goal_count"} <= failed


def test_simulator_workflow_runs_both_closed_loops_on_hosted_runners() -> None:
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "simulator-closed-loop:" in text
    assert "runs-on: ubuntu-24.04" in text
    assert "self-hosted" not in text
    assert "timeout-minutes:" in text
    assert "./scripts/run_nav2_closed_loop.sh" in text
    assert "./scripts/run_twin_scenario.sh scenarios/twin/demo-room-obstacle-stop.json" in text
    assert "FLYTO_ROS2_MIN_CLEARANCE_M" not in text
