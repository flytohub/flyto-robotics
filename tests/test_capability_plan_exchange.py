"""The planner wire contract flyto-robotics shares with flyto-ai.

``tests/fixtures/capability-plan-exchange.v1.json`` is byte-identical to the
copy in flyto-ai's ``tests/fixtures/``. Both repos pin its digest, so a change
to what robotics sends or accepts fails here and there until both move.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from flyto_robotics.ai_planner import (
    PLAN_CONTRACT_VERSION,
    PLANNER_REQUEST_CONTRACT,
    CallablePlannerTransport,
    PlanValidationError,
    parse_plan,
    plan_to_dict,
    planner_request,
    request_ai_plan,
)
from flyto_robotics.cli import _load_goal_frame, _semantic_map_store

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/fixtures/capability-plan-exchange.v1.json"
# Must equal EXCHANGE_FIXTURE_SHA256 in flyto-ai tests/test_capability_plan_exchange.py.
EXCHANGE_FIXTURE_SHA256 = (
    "a891da42ce4f5af2fb57abc441185de8f8e511be20cec810dcfc4f293beae877"
)


def _fixture() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _request_from_sources(fixture: dict) -> dict:
    sources = fixture["sources"]
    plan = json.loads((ROOT / sources["plan"]).read_text(encoding="utf-8"))
    return planner_request(
        goal=plan["goal"],
        resource_id=plan["resource_id"],
        goal_frame=_load_goal_frame(ROOT / sources["goal_frame"]),
        semantic_map=_semantic_map_store(
            ROOT / sources["semantic_map"],
            sources["semantic_map_id"],
        ),
    )


def test_fixture_is_the_pinned_shared_copy() -> None:
    digest = hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
    assert digest == EXCHANGE_FIXTURE_SHA256


def test_robotics_sends_exactly_the_fixture_request() -> None:
    fixture = _fixture()
    request = _request_from_sources(fixture)
    assert request == fixture["request"]
    assert request["planner_contract"] == PLANNER_REQUEST_CONTRACT
    assert request["planner_contract"] == "flyto.robotics.planner-request.v2"
    assert "robot_id" not in request
    assert PLAN_CONTRACT_VERSION in request["instructions"]


def test_robotics_accepts_the_fixture_plan_unchanged() -> None:
    fixture = _fixture()
    plan = parse_plan(fixture["plan"])
    assert plan.contract_version == "flyto.capability-plan.v1"
    assert plan.resource_id == fixture["request"]["resource_id"]
    assert plan_to_dict(plan) == fixture["plan"]


def test_robotics_round_trips_the_fixture_through_a_transport() -> None:
    fixture = _fixture()
    sent: list[dict] = []

    def planner(request: dict) -> object:
        sent.append(request)
        return fixture["plan"]

    sources = fixture["sources"]
    plan = request_ai_plan(
        CallablePlannerTransport(planner),
        goal=fixture["request"]["goal"],
        resource_id=fixture["request"]["resource_id"],
        goal_frame=_load_goal_frame(ROOT / sources["goal_frame"]),
        semantic_map=_semantic_map_store(
            ROOT / sources["semantic_map"],
            sources["semantic_map_id"],
        ),
    )
    assert sent == [fixture["request"]]
    assert plan.resource_id == fixture["request"]["resource_id"]


@pytest.mark.parametrize(
    "mutation",
    [
        {"contract_version": "flyto.robotics.plan.v1"},
        {"robot_id": "flyto-rover-sim-001"},
    ],
    ids=["retired-contract-version", "retired-robot-id-field"],
)
def test_robotics_refuses_the_retired_plan_shape(mutation: dict) -> None:
    plan = dict(_fixture()["plan"])
    if "robot_id" in mutation:
        plan["robot_id"] = plan.pop("resource_id")
    else:
        plan.update(mutation)
    with pytest.raises(PlanValidationError):
        parse_plan(plan)
