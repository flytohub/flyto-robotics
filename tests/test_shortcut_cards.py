"""The four keyboard-shortcut workflow cards and the turn_relative primitive."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from flyto_robotics.ai_planner import PlanValidationError, compile_workflow, load_plan, parse_plan
from flyto_robotics.contracts import load_job
from flyto_robotics.input_runtime import InputValidationError, ValidatedWorkflowCatalog
from flyto_robotics.mission import MissionController, Pose2D, normalize_angle
from flyto_robotics.workflow import PrimitiveKind

ROOT = Path(__file__).resolve().parents[1]
JOB = ROOT / "examples/jobs/tb3-lab-shortcut.json"
CARDS = (
    "shortcut-forward-40cm",
    "shortcut-backward-40cm",
    "shortcut-turn-left-90deg",
    "shortcut-turn-right-90deg",
)


def payload(name: str) -> dict:
    return json.loads((ROOT / f"examples/plans/{name}.json").read_text(encoding="utf-8"))


def run_card(name: str, *, obstacle_range: float | None = None, obstacle_ticks=(20, 40)):
    job = load_job(JOB)
    workflow = compile_workflow(load_plan(ROOT / f"examples/plans/{name}.json"))
    controller = MissionController(job, workflow=workflow)
    pose, now, timestep = Pose2D(0.0, 0.0, 0.0), 0.0, 0.05
    peak_linear = 0.0
    for index in range(1200):
        obstructed = obstacle_range is not None and obstacle_ticks[0] <= index < obstacle_ticks[1]
        command = controller.tick(
            pose,
            minimum_range=obstacle_range if obstructed else math.inf,
            now=now,
        )
        peak_linear = max(peak_linear, abs(command.linear_x))
        pose = Pose2D(
            pose.x + command.linear_x * math.cos(pose.yaw) * timestep,
            pose.y + command.linear_x * math.sin(pose.yaw) * timestep,
            normalize_angle(pose.yaw + command.angular_z * timestep),
        )
        now += timestep
        if controller.terminal:
            break
    return controller, pose, now, peak_linear


def test_every_card_parses_compiles_and_matches_the_job_robot() -> None:
    job = load_job(JOB)
    for name in CARDS:
        plan = load_plan(ROOT / f"examples/plans/{name}.json")
        assert plan.robot_id == job.robot_id
        steps = compile_workflow(plan).steps
        assert steps[-1].kind is PrimitiveKind.SAFE_STOP


def test_catalog_admits_all_four_cards() -> None:
    catalog = ValidatedWorkflowCatalog.from_plan_payloads(
        tuple(payload(name) for name in CARDS)
    )
    assert catalog is not None


@pytest.mark.parametrize(
    ("name", "expected_rad"),
    (("shortcut-turn-left-90deg", 1.5708), ("shortcut-turn-right-90deg", -1.5708)),
)
def test_turn_card_reaches_its_angle_without_ever_translating(
    name: str, expected_rad: float
) -> None:
    controller, pose, elapsed, peak_linear = run_card(name)
    assert controller.state.value == "completed", controller.failure_reason
    assert abs(normalize_angle(pose.yaw) - expected_rad) < 0.06
    # A rotation that drifts forward would creep toward obstacles the operator
    # cannot see; the primitive must hold linear velocity at exactly zero.
    assert peak_linear == 0.0
    # The card has to finish inside the client's 5 s press-and-hold window.
    assert elapsed < 5.0


@pytest.mark.parametrize(
    ("name", "sign"), (("shortcut-forward-40cm", 1.0), ("shortcut-backward-40cm", -1.0))
)
def test_move_card_travels_the_signed_distance(name: str, sign: float) -> None:
    controller, pose, elapsed, _ = run_card(name)
    assert controller.state.value == "completed", controller.failure_reason
    assert math.copysign(1.0, pose.x) == sign
    assert 0.35 <= abs(pose.x) <= 0.45
    assert elapsed < 5.0


def test_obstacle_during_a_turn_records_a_stop() -> None:
    controller, _, _, _ = run_card("shortcut-turn-left-90deg", obstacle_range=0.2)
    assert "obstacle_stop" in {event.kind for event in controller.events}


def test_turn_beyond_the_bound_is_rejected() -> None:
    broken = payload("shortcut-turn-left-90deg")
    broken["steps"][0]["arguments"]["yaw_delta_rad"] = 3.2
    with pytest.raises(PlanValidationError):
        parse_plan(broken)


def test_zero_turn_is_rejected() -> None:
    broken = payload("shortcut-turn-left-90deg")
    broken["steps"][0]["arguments"]["yaw_delta_rad"] = 0.0
    plan = parse_plan(broken)
    with pytest.raises(ValueError, match="yaw_delta_rad"):
        compile_workflow(plan)


def test_turn_card_without_safe_stop_is_rejected_twice() -> None:
    broken = payload("shortcut-turn-left-90deg")
    broken["steps"] = broken["steps"][:1]
    with pytest.raises(PlanValidationError, match="safe_stop"):
        parse_plan(broken)
    # Independently enforced again when the catalog admits a card.
    good = payload("shortcut-turn-left-90deg")
    good["steps"][-1]["capability"] = "dwell"
    good["steps"][-1]["arguments"] = {"seconds": 0.0}
    with pytest.raises((PlanValidationError, InputValidationError)):
        ValidatedWorkflowCatalog.from_plan_payloads((good,))


def test_unknown_turn_argument_is_rejected() -> None:
    broken = payload("shortcut-turn-left-90deg")
    broken["steps"][0]["arguments"]["linear_x"] = 0.5
    with pytest.raises(PlanValidationError):
        parse_plan(broken)
