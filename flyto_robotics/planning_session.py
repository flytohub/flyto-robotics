"""Create one attested initial-plan/replan session before physical execution."""

from __future__ import annotations

import argparse
import hashlib
import json
import uuid
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Protocol

from .ai_planner import (
    HTTPJsonPlannerTransport,
    PlanValidationError,
    parse_plan,
    planner_request,
)
from .capabilities import CapabilityRoutingContext, GoalFrame
from .contracts import write_json_atomic
from .route_graph import RouteGraph, RouteGraphError
from .semantic_map import SemanticLocationStore

SCENARIO_CONTRACT = "flyto.robotics.route-scenario.v1"
SESSION_CONTRACT = "flyto.robotics.planning-session.v1"
AI_RESPONSE_CONTRACT = "flyto.ai.robotics-plan-response.v1"
AI_ATTESTATION_CONTRACT = "flyto.ai.robotics-planning-attestation.v1"
MAX_INPUT_BYTES = 512 * 1024


class PlanningSessionError(ValueError):
    """Raised when route selection, provenance, or replanning fails closed."""


class AttestedPlannerTransport(Protocol):
    """Transport boundary required by the session orchestrator."""

    def complete_attested(
        self,
        request: dict[str, Any],
    ) -> tuple[object, dict[str, Any] | None]:
        """Return a candidate plan and its provider attestation."""


def _canonical(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _snapshot(value: object) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _mapping(value: object, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise PlanningSessionError(f"{field} must be an object")
    return dict(value)


def _load_json(path: str | Path, field: str) -> dict[str, Any]:
    source = Path(path)
    try:
        if source.stat().st_size > MAX_INPUT_BYTES:
            raise PlanningSessionError(f"{field} exceeds the byte limit")
        decoded = json.loads(source.read_text(encoding="utf-8"))
    except PlanningSessionError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PlanningSessionError(f"{field} must contain UTF-8 JSON") from exc
    return _mapping(decoded, field)


def _verify_attestation(
    *,
    request: Mapping[str, Any],
    plan_payload: object,
    attestation: object,
    expected_route_ids: set[str],
    robot_id: str,
    goal: str,
) -> tuple[dict[str, Any], str]:
    plan_data = _mapping(plan_payload, "planner plan")
    proof = _mapping(attestation, "planner attestation")
    if proof.get("contract_version") != AI_ATTESTATION_CONTRACT:
        raise PlanningSessionError("planner attestation contract is invalid")
    if proof.get("mode") != "live_llm":
        raise PlanningSessionError("live planning requires a live_llm attestation")
    unsigned_proof = {
        key: value for key, value in proof.items() if key != "snapshot"
    }
    if proof.get("snapshot") != _snapshot(unsigned_proof):
        raise PlanningSessionError("planner attestation snapshot does not match")
    if proof.get("request_sha256") != _snapshot(request):
        raise PlanningSessionError("planner attestation request digest does not match")
    if proof.get("plan_sha256") != _snapshot(plan_data):
        raise PlanningSessionError("planner attestation plan digest does not match")
    selected_route_id = proof.get("selected_route_id")
    if (
        not isinstance(selected_route_id, str)
        or selected_route_id not in expected_route_ids
    ):
        raise PlanningSessionError(
            "planner selected a route outside the executable candidate set"
        )
    try:
        parsed = parse_plan(plan_data)
    except PlanValidationError as exc:
        raise PlanningSessionError(str(exc)) from exc
    if parsed.robot_id != robot_id or parsed.goal != goal:
        raise PlanningSessionError(
            "planner plan does not match the requested robot and goal"
        )
    if parsed.generated_by.kind != "llm":
        raise PlanningSessionError("live planning plan source must be llm")
    selected_locations = [
        str(step.arguments["location_id"])
        for step in parsed.steps
        if step.capability == "navigate_to_location"
    ]
    route_candidates = request.get("observations", {}).get(
        "route_candidates",
        [],
    )
    exact_matches = [
        candidate
        for candidate in route_candidates
        if isinstance(candidate, Mapping)
        and candidate.get("route_id") == selected_route_id
        and candidate.get("location_ids") == selected_locations
    ]
    if len(exact_matches) != 1:
        raise PlanningSessionError(
            "planner semantic locations do not exactly match the attested route"
        )
    return plan_data, selected_route_id


def _planner_round(
    *,
    sequence: int,
    trigger: str,
    goal: str,
    robot_id: str,
    goal_frame: GoalFrame,
    routing_context: CapabilityRoutingContext,
    semantic_map: SemanticLocationStore,
    evaluation: Mapping[str, Any],
    transport: AttestedPlannerTransport,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    candidates = evaluation.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise PlanningSessionError("route evaluation has no executable candidates")
    request = planner_request(
        goal=goal,
        robot_id=robot_id,
        goal_frame=goal_frame,
        routing_context=routing_context,
        semantic_map=semantic_map,
        observations={
            "route_candidates": candidates,
            "route_selection_policy": {
                "kind": "ranked_safe_candidates",
                "instruction": (
                    "Choose the highest-scoring candidate unless the goal explicitly "
                    "requires a different executable candidate."
                ),
            },
            "planning_round": sequence,
            "planning_trigger": trigger,
        },
        route_limit=8,
    )
    plan_payload, attestation = transport.complete_attested(request)
    if attestation is None:
        raise PlanningSessionError(
            "planner returned no attestation; fixture output cannot be called live AI"
        )
    route_ids = {
        str(candidate["route_id"])
        for candidate in candidates
        if isinstance(candidate, Mapping) and "route_id" in candidate
    }
    plan_data, selected_route_id = _verify_attestation(
        request=request,
        plan_payload=plan_payload,
        attestation=attestation,
        expected_route_ids=route_ids,
        robot_id=robot_id,
        goal=goal,
    )
    response = {
        "contract_version": AI_RESPONSE_CONTRACT,
        "plan": plan_data,
        "attestation": attestation,
    }
    return request, response, selected_route_id


def _selected_route_dependency_ids(
    evaluation: Mapping[str, Any],
    route_id: str,
) -> set[str]:
    candidates = evaluation.get("candidates", [])
    selected = next(
        (
            candidate
            for candidate in candidates
            if isinstance(candidate, Mapping)
            and candidate.get("route_id") == route_id
        ),
        None,
    )
    if not isinstance(selected, Mapping):
        raise PlanningSessionError("selected route is absent from route evaluation")
    dependencies = selected.get("dependencies", [])
    return {
        str(item["resource_id"])
        for item in dependencies
        if isinstance(item, Mapping) and "resource_id" in item
    }


def run_planning_session(
    *,
    scenario: Mapping[str, Any],
    goal: str,
    robot_id: str,
    semantic_map: SemanticLocationStore,
    transport: AttestedPlannerTransport,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Run initial selection, inject one declared dependency change, and replan."""

    data = dict(scenario)
    allowed = {
        "contract_version",
        "scenario_id",
        "goal_frame",
        "routing_context",
        "graph",
        "initial_resource_observations",
        "preflight_change",
    }
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise PlanningSessionError(
            "route scenario contains unsupported fields: " + ", ".join(unknown)
        )
    if data.get("contract_version") != SCENARIO_CONTRACT:
        raise PlanningSessionError(
            f"route scenario contract must be {SCENARIO_CONTRACT}"
        )
    scenario_id = data.get("scenario_id")
    if not isinstance(scenario_id, str) or not scenario_id:
        raise PlanningSessionError("scenario_id must be non-empty text")
    try:
        graph = RouteGraph.from_mapping(data.get("graph"))
        goal_frame = GoalFrame.from_mapping(
            _mapping(data.get("goal_frame"), "goal_frame")
        )
        routing_context = CapabilityRoutingContext.from_mapping(
            _mapping(data.get("routing_context"), "routing_context")
        )
    except (RouteGraphError, ValueError) as exc:
        raise PlanningSessionError(str(exc)) from exc
    observations = _mapping(
        data.get("initial_resource_observations"),
        "initial_resource_observations",
    )
    change = _mapping(data.get("preflight_change"), "preflight_change")
    if set(change) != {"resource_id", "reason", "observation"}:
        raise PlanningSessionError(
            "preflight_change requires resource_id, reason, and observation"
        )
    changed_resource_id = change.get("resource_id")
    if not isinstance(changed_resource_id, str) or not changed_resource_id:
        raise PlanningSessionError("preflight_change.resource_id is invalid")
    changed_observation = _mapping(
        change.get("observation"),
        "preflight_change.observation",
    )

    initial_evaluation = graph.evaluate(observations, phase="preflight")
    request_one, response_one, route_one = _planner_round(
        sequence=1,
        trigger="initial_goal",
        goal=goal,
        robot_id=robot_id,
        goal_frame=goal_frame,
        routing_context=routing_context,
        semantic_map=semantic_map,
        evaluation=initial_evaluation,
        transport=transport,
    )
    if changed_resource_id not in _selected_route_dependency_ids(
        initial_evaluation,
        route_one,
    ):
        raise PlanningSessionError(
            "declared preflight change does not affect the AI-selected route"
        )

    revised_observations = dict(observations)
    revised_observations[changed_resource_id] = changed_observation
    revised_evaluation = graph.evaluate(
        revised_observations,
        phase="preflight",
    )
    revised_route_ids = {
        str(candidate["route_id"])
        for candidate in revised_evaluation["candidates"]
        if isinstance(candidate, Mapping)
    }
    if route_one in revised_route_ids:
        raise PlanningSessionError(
            "dependency change did not exclude the invalidated selected route"
        )
    request_two, response_two, route_two = _planner_round(
        sequence=2,
        trigger="resource_dependency_changed",
        goal=goal,
        robot_id=robot_id,
        goal_frame=goal_frame,
        routing_context=routing_context,
        semantic_map=semantic_map,
        evaluation=revised_evaluation,
        transport=transport,
    )
    if route_two == route_one:
        raise PlanningSessionError("replanning did not select a different route")

    session: dict[str, Any] = {
        "contract_version": SESSION_CONTRACT,
        "session_id": f"planning-session-{uuid.uuid4().hex}",
        "planning_mode": "live_llm",
        "scenario_id": scenario_id,
        "goal": goal,
        "robot_id": robot_id,
        "resource_change": {
            "resource_id": changed_resource_id,
            "reason": change["reason"],
            "before": observations.get(changed_resource_id),
            "after": changed_observation,
        },
        "rounds": [
            {
                "sequence": 1,
                "status": "superseded",
                "trigger": "initial_goal",
                "request": request_one,
                "response": response_one,
                "route_evaluation": initial_evaluation,
            },
            {
                "sequence": 2,
                "status": "selected",
                "trigger": "resource_dependency_changed",
                "request": request_two,
                "response": response_two,
                "route_evaluation": revised_evaluation,
            },
        ],
        "final_round": 2,
    }
    session["snapshot"] = _snapshot(session)
    final_plan = _mapping(response_two["plan"], "final plan")
    return session, final_plan


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate an attested Flyto2 branching-route planning session."
    )
    parser.add_argument("--scenario-file", required=True)
    parser.add_argument("--semantic-map-file", required=True)
    parser.add_argument("--semantic-map-id", required=True)
    parser.add_argument("--planner-url", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--goal", required=True)
    parser.add_argument("--robot-id", default="flyto-rover-sim-001")
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    return parser


def main() -> int:
    args = _parser().parse_args()
    scenario = _load_json(args.scenario_file, "route scenario")
    semantic_map = SemanticLocationStore(
        args.semantic_map_file,
        map_id=args.semantic_map_id,
    )
    transport = HTTPJsonPlannerTransport(
        args.planner_url,
        timeout_seconds=args.timeout_seconds,
    )
    session, final_plan = run_planning_session(
        scenario=scenario,
        goal=args.goal,
        robot_id=args.robot_id,
        semantic_map=semantic_map,
        transport=transport,
    )
    output_dir = Path(args.output_dir)
    write_json_atomic(output_dir / "planning-session.json", session)
    write_json_atomic(output_dir / "validated-plan.json", final_plan)
    print(
        json.dumps(
            {
                "planning_session": str(output_dir / "planning-session.json"),
                "validated_plan": str(output_dir / "validated-plan.json"),
                "selected_route_id": session["rounds"][1]["response"][
                    "attestation"
                ]["selected_route_id"],
                "rounds": 2,
                "planning_mode": "live_llm",
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
