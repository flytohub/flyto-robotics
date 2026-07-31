"""Verify truthful live-planning evidence for the multi-device showcase."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from typing import Any

from .ai_planner import PlanValidationError, parse_plan
from .capabilities import GoalFrame, default_capability_registry

SHOWCASE_PLANNING_CONTRACT_VERSION = (
    "flyto.robotics.showcase-planning-evidence.v2"
)
PLANNING_SESSION_CONTRACT_VERSION = "flyto.robotics.planning-session.v1"
AI_RESPONSE_CONTRACT_VERSION = "flyto.ai.robotics-plan-response.v1"
AI_ATTESTATION_CONTRACT_VERSION = (
    "flyto.ai.robotics-planning-attestation.v1"
)


def _object(value: object, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise PlanValidationError(f"{field} must be an object")
    return dict(value)


def _canonical(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _snapshot(value: object) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _verify_snapshot(
    value: Mapping[str, Any],
    *,
    field: str,
) -> None:
    supplied = value.get("snapshot")
    if not isinstance(supplied, str) or len(supplied) != 64:
        raise PlanValidationError(f"{field}.snapshot must be a SHA-256 digest")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if supplied != _snapshot(unsigned):
        raise PlanValidationError(f"{field}.snapshot does not match")


def _route_candidates(request: Mapping[str, Any]) -> list[dict[str, Any]]:
    observations = request.get("observations", {})
    if not isinstance(observations, Mapping):
        raise PlanValidationError("planner request observations must be an object")
    raw = observations.get("route_candidates", [])
    if not isinstance(raw, list) or len(raw) > 32:
        raise PlanValidationError(
            "planner request route_candidates must be a bounded array"
        )
    result = []
    for index, item in enumerate(raw):
        candidate = _object(item, f"route_candidates[{index}]")
        route_id = candidate.get("route_id")
        locations = candidate.get("location_ids")
        if not isinstance(route_id, str) or not route_id:
            raise PlanValidationError(
                f"route_candidates[{index}].route_id is invalid"
            )
        if (
            not isinstance(locations, list)
            or not locations
            or any(not isinstance(location, str) for location in locations)
        ):
            raise PlanValidationError(
                f"route_candidates[{index}].location_ids is invalid"
            )
        result.append(candidate)
    return result


def build_showcase_planning_evidence(
    *,
    session: Mapping[str, Any],
    goal_frame: Mapping[str, object],
    executed_plan: Mapping[str, Any],
    robot_id: str,
) -> dict[str, Any]:
    """Verify that one executed plan came from a truthful, attested session."""

    session_data = dict(session)
    if (
        session_data.get("contract_version")
        != PLANNING_SESSION_CONTRACT_VERSION
    ):
        raise PlanValidationError(
            f"planning session must use {PLANNING_SESSION_CONTRACT_VERSION}"
        )
    _verify_snapshot(session_data, field="planning_session")
    planning_mode = session_data.get("planning_mode")
    if planning_mode not in {"live_llm", "deterministic_fixture"}:
        raise PlanValidationError("planning_mode is unsupported")
    if session_data.get("robot_id") != robot_id:
        raise PlanValidationError(
            "planning session robot_id does not match the executor"
        )
    goal = session_data.get("goal")
    if not isinstance(goal, str) or not goal:
        raise PlanValidationError("planning session goal is invalid")
    parsed_frame = GoalFrame.from_mapping(goal_frame)
    rounds = session_data.get("rounds")
    if not isinstance(rounds, list) or not 1 <= len(rounds) <= 8:
        raise PlanValidationError("planning session rounds are invalid")
    final_round_number = session_data.get("final_round")
    if (
        isinstance(final_round_number, bool)
        or not isinstance(final_round_number, int)
        or not 1 <= final_round_number <= len(rounds)
    ):
        raise PlanValidationError("planning session final_round is invalid")
    final_round = _object(
        rounds[final_round_number - 1],
        "planning_session.final_round",
    )
    if final_round.get("sequence") != final_round_number:
        raise PlanValidationError("planning session round sequence is invalid")
    request = _object(final_round.get("request"), "final_round.request")
    if (
        request.get("planner_contract")
        != "flyto.robotics.planner-request.v1"
    ):
        raise PlanValidationError("final planner request contract is invalid")
    if request.get("goal") != goal or request.get("robot_id") != robot_id:
        raise PlanValidationError(
            "final planner request does not match the planning session"
        )
    if request.get("goal_frame") != parsed_frame.to_dict():
        raise PlanValidationError(
            "showcase goal frame does not match the attested planner request"
        )

    response = _object(final_round.get("response"), "final_round.response")
    if response.get("contract_version") != AI_RESPONSE_CONTRACT_VERSION:
        raise PlanValidationError("final Flyto AI response contract is invalid")
    plan_payload = _object(response.get("plan"), "final_round.response.plan")
    attestation = _object(
        response.get("attestation"),
        "final_round.response.attestation",
    )
    if (
        attestation.get("contract_version")
        != AI_ATTESTATION_CONTRACT_VERSION
    ):
        raise PlanValidationError("Flyto AI attestation contract is invalid")
    _verify_snapshot(attestation, field="Flyto AI attestation")
    if attestation.get("mode") != planning_mode:
        raise PlanValidationError(
            "attestation mode does not match planning session"
        )
    if attestation.get("request_sha256") != _snapshot(request):
        raise PlanValidationError(
            "attestation request digest does not match final request"
        )
    if attestation.get("plan_sha256") != _snapshot(plan_payload):
        raise PlanValidationError(
            "attestation plan digest does not match final plan"
        )
    if _canonical(plan_payload) != _canonical(dict(executed_plan)):
        raise PlanValidationError(
            "Gazebo executed plan does not match the attested final plan"
        )

    registry = default_capability_registry()
    plan = parse_plan(plan_payload, registry=registry)
    if plan.robot_id != robot_id or plan.goal != goal:
        raise PlanValidationError(
            "validated plan does not match the requested robot and goal"
        )
    route = _object(request.get("capability_route"), "capability_route")
    raw_candidates = route.get("candidates", [])
    if not isinstance(raw_candidates, list):
        raise PlanValidationError("capability_route candidates are invalid")
    candidates = [
        str(candidate["runtime_name"])
        for candidate in raw_candidates
        if isinstance(candidate, Mapping) and "runtime_name" in candidate
    ]
    selected = [step.capability for step in plan.steps]
    outside_shortlist = sorted(set(selected) - set(candidates))
    if outside_shortlist:
        raise PlanValidationError(
            "validated plan selected capabilities outside the routed shortlist: "
            + ", ".join(outside_shortlist)
        )

    route_candidates = _route_candidates(request)
    selected_locations = [
        str(step.arguments["location_id"])
        for step in plan.steps
        if step.capability == "navigate_to_location"
    ]
    matching_routes = [
        candidate
        for candidate in route_candidates
        if candidate["location_ids"] == selected_locations
    ]
    selected_route_id = attestation.get("selected_route_id")
    if route_candidates and (
        len(matching_routes) != 1
        or matching_routes[0]["route_id"] != selected_route_id
    ):
        raise PlanValidationError(
            "attested route does not match the executed semantic location sequence"
        )
    expected_source_kind = (
        "llm" if planning_mode == "live_llm" else "deterministic_demo"
    )
    if plan.generated_by.kind != expected_source_kind:
        raise PlanValidationError(
            "plan source kind does not match truthful planning mode"
        )
    if planning_mode == "live_llm" and len(rounds) < 2:
        raise PlanValidationError(
            "live branching showcase must include initial planning and replan"
        )

    return {
        "contract_version": SHOWCASE_PLANNING_CONTRACT_VERSION,
        "session_id": session_data.get("session_id"),
        "planning_mode": planning_mode,
        "goal": goal,
        "goal_frame": parsed_frame.to_dict(),
        "robot_id": robot_id,
        "round_count": len(rounds),
        "replan_count": max(0, len(rounds) - 1),
        "planning_rounds": [
            {
                "sequence": item.get("sequence"),
                "status": item.get("status"),
                "trigger": item.get("trigger"),
                "selected_route_id": (
                    item.get("response", {})
                    .get("attestation", {})
                    .get("selected_route_id")
                    if isinstance(item, Mapping)
                    and isinstance(item.get("response"), Mapping)
                    and isinstance(
                        item.get("response", {}).get("attestation"),
                        Mapping,
                    )
                    else None
                ),
                "route_evaluation": item.get("route_evaluation"),
            }
            for item in rounds
            if isinstance(item, Mapping)
        ],
        "capability_routing": {
            "registry_snapshot": route.get("registry_snapshot"),
            "confidence": route.get("confidence"),
            "needs_clarification": route.get("needs_clarification"),
            "shortlist": candidates,
            "shortlist_size": len(candidates),
        },
        "selected_route": {
            "route_id": selected_route_id,
            "location_ids": selected_locations,
        },
        "validated_plan": {
            "plan_id": plan.plan_id,
            "source": {
                "kind": plan.generated_by.kind,
                "provider": plan.generated_by.provider,
                "model": plan.generated_by.model,
            },
            "attestation": {
                "run_id": attestation.get("run_id"),
                "request_sha256": attestation.get("request_sha256"),
                "plan_sha256": attestation.get("plan_sha256"),
                "schema_sha256": attestation.get("schema_sha256"),
                "attempt_count": attestation.get("attempt_count"),
                "latency_ms": attestation.get("latency_ms"),
                "snapshot": attestation.get("snapshot"),
            },
            "steps": [
                {
                    "step_id": step.step_id,
                    "capability": step.capability,
                    "on_failure": step.on_failure,
                }
                for step in plan.steps
            ],
            "selected_capabilities": list(dict.fromkeys(selected)),
            "step_count": len(plan.steps),
            "strict_validation_passed": True,
            "executed_plan_matches_attestation": True,
            "direct_motor_commands_allowed": False,
        },
    }
