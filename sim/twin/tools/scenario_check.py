"""Load a twin scenario (`scenarios/twin/*.json`) and judge what a run observed.

Pure functions, no ROS: `run_scenario.py` drives the twin and records what
happened; this module decides whether that passes. The unit tests feed it
synthetic observations, so the verdict logic is checked without a simulator.

The 0.35 m LiDAR clearance floor is fixed here. A scenario that names any
other floor is refused, and a run whose adapter reported any other floor
fails, so neither a scenario file nor an environment variable can lower it.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

CONTRACT_VERSION = "flyto.robotics.twin-scenario.v1"
VERDICT_VERSION = "flyto.robotics.twin-scenario-verdict.v1"
SAFETY_FLOOR_M = 0.35
WORLD_PREFIX = "/opt/flyto-twin/worlds/"
# Goals that move the robot somewhere other than straight along the request:
# a navigation (a detour) or a Nav2 behaviour (a recovery). A scenario that
# asserts "no detour" may never list them as allowed.
DETOUR_OR_RECOVERY_ACTION_TYPES = frozenset(
    {
        "nav2_msgs/action/NavigateToPose",
        "nav2_msgs/action/NavigateThroughPoses",
        "nav2_msgs/action/BackUp",
        "nav2_msgs/action/Spin",
    }
)
_EXPECTATION_KEYS = {
    "outcome",
    "motion_reason",
    "clearance_floor_m",
    "stop_floor_held",
    "min_travelled_m",
    "max_travelled_m",
    "allowed_goal_action_types",
    "max_goals_sent",
    "quiet_after_seconds",
    "max_drift_after_m",
}


class ScenarioError(ValueError):
    """A scenario file that cannot be run as written."""


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{name} must be a number")
    if not math.isfinite(float(value)):
        raise ScenarioError(f"{name} must be finite")
    return float(value)


def validate_scenario(value: Mapping[str, Any], *, twin_root: Path) -> dict[str, Any]:
    """Return the scenario if it is runnable against the twin in `twin_root`."""

    if value.get("contract_version") != CONTRACT_VERSION:
        raise ScenarioError("contract_version is unsupported")
    for key in ("scenario_id", "title", "description"):
        if not isinstance(value.get(key), str) or not value[key].strip():
            raise ScenarioError(f"{key} is required")
    twin = value.get("twin")
    if not isinstance(twin, Mapping):
        raise ScenarioError("twin is required")
    world = twin.get("world")
    if not isinstance(world, str) or not world.startswith(WORLD_PREFIX):
        raise ScenarioError(f"twin.world must be a world under {WORLD_PREFIX}")
    # Self-contained: the world ships in sim/twin/worlds (mounted read-only).
    if not (twin_root / "worlds" / world[len(WORLD_PREFIX):]).is_file():
        raise ScenarioError("twin.world is not in sim/twin/worlds")
    pose = twin.get("start_pose")
    if not isinstance(pose, Mapping):
        raise ScenarioError("twin.start_pose is required")
    for key in ("x", "y", "yaw"):
        _number(pose.get(key), f"twin.start_pose.{key}")
    call = value.get("call")
    if not isinstance(call, Mapping) or call.get("capability_id") not in {
        "motion.advance",
        "motion.retreat",
    }:
        raise ScenarioError("call.capability_id must be motion.advance or motion.retreat")
    arguments = call.get("arguments")
    if not isinstance(arguments, Mapping):
        raise ScenarioError("call.arguments is required")
    if _number(arguments.get("distance_m"), "call.arguments.distance_m") <= 0:
        raise ScenarioError("call.arguments.distance_m must be positive")
    if _number(arguments.get("speed_mps"), "call.arguments.speed_mps") <= 0:
        raise ScenarioError("call.arguments.speed_mps must be positive")
    if _number(call.get("deadline_seconds"), "call.deadline_seconds") <= 0:
        raise ScenarioError("call.deadline_seconds must be positive")
    expectations = value.get("expectations")
    if not isinstance(expectations, Mapping) or set(expectations) != _EXPECTATION_KEYS:
        raise ScenarioError(f"expectations must name exactly {sorted(_EXPECTATION_KEYS)}")
    if _number(expectations["clearance_floor_m"], "clearance_floor_m") != SAFETY_FLOOR_M:
        raise ScenarioError(f"clearance_floor_m must be {SAFETY_FLOOR_M}; it is never lowered")
    if expectations["stop_floor_held"] is not True:
        raise ScenarioError("stop_floor_held must be true")
    low = _number(expectations["min_travelled_m"], "min_travelled_m")
    high = _number(expectations["max_travelled_m"], "max_travelled_m")
    if not 0 <= low <= high:
        raise ScenarioError("min_travelled_m must be between 0 and max_travelled_m")
    allowed = expectations["allowed_goal_action_types"]
    if (
        not isinstance(allowed, list)
        or not allowed
        or not all(isinstance(item, str) for item in allowed)
    ):
        raise ScenarioError("allowed_goal_action_types must be a non-empty list")
    leaked = DETOUR_OR_RECOVERY_ACTION_TYPES.intersection(allowed)
    if leaked:
        raise ScenarioError(f"a no-detour scenario cannot allow {sorted(leaked)}")
    goals = expectations["max_goals_sent"]
    if isinstance(goals, bool) or not isinstance(goals, int) or goals < 1:
        raise ScenarioError("max_goals_sent must be a positive integer")
    if _number(expectations["quiet_after_seconds"], "quiet_after_seconds") <= 0:
        raise ScenarioError("quiet_after_seconds must be positive")
    if _number(expectations["max_drift_after_m"], "max_drift_after_m") < 0:
        raise ScenarioError("max_drift_after_m must not be negative")
    return dict(value)


def load_scenario(path: Path, *, twin_root: Path) -> dict[str, Any]:
    return validate_scenario(json.loads(path.read_text(encoding="utf-8")), twin_root=twin_root)


def _travelled(summary: Mapping[str, Any]) -> float | None:
    start, end = summary.get("start_pose"), summary.get("final_pose")
    if not isinstance(start, Mapping) or not isinstance(end, Mapping):
        return None
    try:
        return math.hypot(float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"]))
    except (KeyError, TypeError, ValueError):
        return None


def _drift(poses: Sequence[Mapping[str, Any]]) -> float | None:
    points = []
    for pose in poses:
        try:
            points.append((float(pose["x"]), float(pose["y"])))
        except (KeyError, TypeError, ValueError):
            continue
    if len(points) < 2:
        return None
    return max(math.hypot(x - points[0][0], y - points[0][1]) for x, y in points[1:])


def evaluate(scenario: Mapping[str, Any], observed: Mapping[str, Any]) -> dict[str, Any]:
    """Judge one run.

    `observed` carries the call's `outcome` and `evidence`, every action goal
    the adapter sent (`goals`: action, action_type, `after_result`), every
    non-zero velocity command (`motion_commands_after_result`) and odometry
    poses sampled over the quiet window after the call returned
    (`poses_after_result`).
    """

    expected = scenario["expectations"]
    evidence = observed.get("evidence") or {}
    summary = evidence.get("motion_outcome") or {}
    clearance = summary.get("stop_clearance") or {}
    goals = list(observed.get("goals") or [])
    travelled = _travelled(summary)
    drift = _drift(observed.get("poses_after_result") or [])
    allowed = set(expected["allowed_goal_action_types"])
    checks = {
        "outcome": observed.get("outcome") == expected["outcome"],
        "motion_reason": summary.get("reason") == expected["motion_reason"],
        # The floor the adapter applied is the 0.35 m floor, not a lowered one.
        "floor_not_lowered": (
            summary.get("clearance_floor_m") == SAFETY_FLOOR_M
            and clearance.get("floor_m") == SAFETY_FLOOR_M
        ),
        # It stopped with every return, and the path ahead, outside the floor.
        "stopped_outside_floor": (
            clearance.get("floor_held") is expected["stop_floor_held"]
            and clearance.get("travel_floor_held") is expected["stop_floor_held"]
        ),
        "moved_toward_obstacle": (
            travelled is not None
            and expected["min_travelled_m"] <= travelled <= expected["max_travelled_m"]
        ),
        # No detour and no recovery: only the requested drive was ever sent.
        "only_requested_goals": bool(goals)
        and all(goal.get("action_type") in allowed for goal in goals),
        "goal_count": 0 < len(goals) <= expected["max_goals_sent"],
        "no_goal_after_result": not any(goal.get("after_result") for goal in goals),
        "no_motion_command_after_result": not observed.get("motion_commands_after_result"),
        "held_still_after_result": drift is not None and drift <= expected["max_drift_after_m"],
    }
    return {
        "contract_version": VERDICT_VERSION,
        "scenario_id": scenario["scenario_id"],
        "passed": all(checks.values()),
        "checks": [{"code": code, "passed": passed} for code, passed in checks.items()],
        "measured": {
            "outcome": observed.get("outcome"),
            "motion_reason": summary.get("reason"),
            "clearance_floor_m": summary.get("clearance_floor_m"),
            "stop_clearance": clearance or None,
            "travelled_m": round(travelled, 4) if travelled is not None else None,
            "goals": goals,
            "motion_commands_after_result": list(
                observed.get("motion_commands_after_result") or []
            ),
            "drift_after_result_m": round(drift, 4) if drift is not None else None,
        },
    }
