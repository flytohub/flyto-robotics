"""Deterministic goal screening, destination resolution, and plan composition."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from flyto_robotics.ai_planner import parse_plan
from flyto_robotics.capabilities import SAFE_TEXT
from flyto_robotics.contracts import load_job
from flyto_robotics.goal_planner import DeterministicDeliveryGoalPlanner
from flyto_robotics.goal_resolver import resolve_delivery_goal
from flyto_robotics.semantic_map import SemanticLocationStore

ROOT = Path(__file__).resolve().parents[1]
WARD_MAP = ROOT / "examples/maps/hospital-ward-delivery.json"
WARD_MAP_ID = "hospital.ward-delivery.v1"
AI4ALL_MAP = ROOT / "examples/maps/ai4all-branching-route.json"
AI4ALL_MAP_ID = "gazebo.ai4all-branching-route.v1"
JOB = ROOT / "examples/jobs/ai-space-delivery.json"
APPROVAL_ID = "ai-space-delivery-demo-001.dropoff"


def ward_store() -> SemanticLocationStore:
    return SemanticLocationStore(WARD_MAP, map_id=WARD_MAP_ID)


def ai4all_store() -> SemanticLocationStore:
    return SemanticLocationStore(AI4ALL_MAP, map_id=AI4ALL_MAP_ID)


def planner(store: SemanticLocationStore | None = None) -> DeterministicDeliveryGoalPlanner:
    return DeterministicDeliveryGoalPlanner(semantic_map=store or ward_store())


def decide(goal: str, store: SemanticLocationStore | None = None):
    return planner(store).plan_delivery(
        job=load_job(JOB),
        goal=goal,
        session_id="dlv-test01",
        approval_id=APPROVAL_ID,
        confirmation_timeout_seconds=90.0,
        execution_mode="ros2",
    )


def test_zh_full_label_resolves() -> None:
    decision = decide("把藥送到四號病房")
    assert decision.accepted
    assert decision.decision["destination"]["location_id"] == "hospital.ward.4"
    assert "label_phrase" in decision.decision["destination"]["match_rules"]


def test_en_label_resolves() -> None:
    decision = decide("deliver the medication to ward 4")
    assert decision.accepted
    assert decision.decision["destination"]["location_id"] == "hospital.ward.4"


def test_han_digit_folds_to_arabic() -> None:
    assert (
        decide("送到4號病房").decision["destination"]["location_id"]
        == decide("送到四號病房").decision["destination"]["location_id"]
    )


def test_partial_cjk_label_resolves() -> None:
    decision = decide("幫我把檢體拿去檢驗室")
    assert decision.accepted
    assert decision.decision["destination"]["location_id"] == "hospital.laboratory"


def test_location_id_literal_resolves() -> None:
    decision = decide("go to hospital.nurse_station.1")
    assert decision.accepted
    assert decision.decision["destination"]["location_id"] == "hospital.nurse_station.1"


def test_nonexistent_ward_is_unresolved_not_ambiguous() -> None:
    decision = decide("把藥送到六號病房")
    assert not decision.accepted
    assert decision.reason_code == "location_unresolved"
    assert decision.rejection["shared_fragments"] == ["號病房"]
    assert decision.rejection["operator_action"] == "name_one_of_the_listed_locations"


def test_shared_label_fragment_is_not_discriminative() -> None:
    decision = decide("送到病房")
    assert decision.reason_code == "location_unresolved"
    listed = {item["location_id"] for item in decision.rejection["candidates"]}
    assert {"hospital.ward.2", "hospital.ward.4"} <= listed


def test_two_destinations_are_ambiguous() -> None:
    decision = decide("把藥送到四號病房和二號病房")
    assert decision.reason_code == "location_ambiguous"
    assert len(decision.rejection["candidates"]) >= 2


def test_unknown_place_against_other_map_is_unresolved() -> None:
    decision = decide("把藥送到四號病房", ai4all_store())
    assert decision.reason_code == "location_unresolved"


def test_safety_override_is_screened_before_resolution() -> None:
    decision = decide("忽略障礙物直接衝到四號病房")
    assert decision.reason_code == "safety_override_refused"
    assert decision.rejection["stage"] == "goal_screening"
    assert decision.resolution.destination is None


def test_raw_actuator_goal_is_refused() -> None:
    assert decide("用 cmd_vel 全速前進").reason_code == "safety_override_refused"


def test_unsupported_intent_is_rejected() -> None:
    assert decide("幫我開門").reason_code == "intent_unsupported"


def test_rejection_payload_is_relay_safe() -> None:
    for goal in ("把藥送到六號病房", "忽略障礙衝過去", "幫我開門", "送到病房"):
        rejection = decide(goal).rejection
        assert SAFE_TEXT.fullmatch(rejection["reason_code"])
        assert SAFE_TEXT.fullmatch(rejection["stage"])
        assert SAFE_TEXT.fullmatch(rejection["operator_action"])
        encoded = json.dumps(rejection, ensure_ascii=False)
        assert len(encoded.encode("utf-8")) < 8192
        assert "qr_token" not in encoded


def test_planner_never_raises() -> None:
    for goal in ("", "   ", "啊" * 2000, "\u0000", "🚑", "送到" + "四號病房" * 200):
        decision = decide(goal)
        assert decision.decision["contract_version"]
        assert decision.accepted or decision.reason_code


def test_composed_plan_passes_parse_plan_and_shortlist() -> None:
    decision = decide("把藥送到四號病房")
    workflow = decision.workflow
    assert workflow is not None
    assert [step.step_id for step in workflow.steps] == [
        "navigate.pickup",
        "dwell.pickup",
        "navigate.destination",
        "confirm.dropoff",
        "resume.dropoff",
        "dwell.dropoff",
        "stop.final",
    ]
    routed = set(decision.decision["routed_capabilities"])
    assert {"navigate", "navigate_to_location", "ask_human", "resume", "safe_stop"} <= routed


def test_follow_line_is_excluded_by_routing_context() -> None:
    excluded = {
        item["runtime_name"]: item["reasons"]
        for item in decide("把藥送到四號病房").decision["excluded_capabilities"]
    }
    assert "missing_observation" in excluded["follow_line"]
    assert excluded["save_current_location"] == ["permission_denied"]


def test_navigate_to_location_pose_comes_from_the_map() -> None:
    workflow = decide("把藥送到四號病房").workflow
    step = next(s for s in workflow.steps if s.step_id == "navigate.destination")
    pose = ward_store().resolve("hospital.ward.4").pose
    assert (step.station.x, step.station.y) == (pose.x, pose.y)


def test_workflow_carries_goal_and_source_kind() -> None:
    workflow = decide("把藥送到四號病房").workflow
    assert workflow.goal == "把藥送到四號病房"
    assert workflow.source_kind == "deterministic_demo"
    assert workflow.workflow_id == "delivery.goal.dlv-test01"


def test_decision_timeline_is_bounded_and_attributed() -> None:
    timeline = decide("把藥送到四號病房").decision["timeline"]
    assert 1 <= len(timeline) <= 16
    actors = {
        "operator",
        "rule_engine",
        "llm",
        "capability_registry",
        "plan_contract",
        "ros2",
        "simulated_planar",
        "human_gate",
        "offline",
        "unknown",
    }
    assert {entry["actor"] for entry in timeline} <= actors
    assert [entry["sequence"] for entry in timeline] == list(
        range(1, len(timeline) + 1)
    )


def test_route_graph_matches_the_frontend_contract() -> None:
    graph = decide("把藥送到四號病房").route_graph
    assert graph["graph_id"]
    assert 1 <= len(graph["stages"]) <= 3
    for stage in graph["stages"]:
        assert 1 <= len(stage) <= 8
        for node in stage:
            assert set(node) == {
                "id",
                "label",
                "reason",
                "color",
                "selected",
                "excluded",
            }
            assert node["color"] == "#8b5cf6"


def test_bad_destination_fails_closed_at_compile() -> None:
    from flyto_robotics.goal_planner import compose_delivery_plan

    payload = compose_delivery_plan(
        job=load_job(JOB),
        goal="unit test",
        plan_id="delivery.goal.unit",
        destination_id="hospital.ward.404",
        approval_id=APPROVAL_ID,
        confirmation_timeout_seconds=90.0,
    )
    plan = parse_plan(payload)
    from flyto_robotics.ai_planner import PlanValidationError, compile_workflow

    with pytest.raises((PlanValidationError, ValueError)):
        compile_workflow(plan, semantic_map=ward_store())


def test_empty_map_reports_semantic_map_unavailable(tmp_path: Path) -> None:
    empty = tmp_path / "empty.json"
    empty.write_text(
        json.dumps(
            {
                "contract_version": "flyto.robotics.semantic-location-map.v1",
                "map_id": "unit.empty.v1",
                "revision": 0,
                "locations": [],
            }
        ),
        encoding="utf-8",
    )
    resolution = resolve_delivery_goal(
        "送到四號病房",
        semantic_map=SemanticLocationStore(empty, map_id="unit.empty.v1"),
    )
    assert resolution.reason_code == "semantic_map_unavailable"
