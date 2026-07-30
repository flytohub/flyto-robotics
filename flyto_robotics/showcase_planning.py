"""Build auditable AI-planning evidence for the multi-device showcase."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .ai_planner import PlanValidationError, load_plan, planner_request
from .capabilities import GoalFrame, default_capability_registry

SHOWCASE_PLANNING_CONTRACT_VERSION = "flyto.robotics.showcase-planning-evidence.v1"


def build_showcase_planning_evidence(
    *,
    goal: str,
    goal_frame: dict[str, object],
    plan_file: str | Path,
    robot_id: str,
    route_limit: int = 8,
) -> dict[str, Any]:
    """Prove registry routing, LLM output validation, and selected atomic steps."""
    registry = default_capability_registry()
    parsed_frame = GoalFrame.from_mapping(goal_frame)
    request = planner_request(
        goal=goal,
        goal_frame=parsed_frame,
        robot_id=robot_id,
        registry=registry,
        route_limit=route_limit,
    )
    plan = load_plan(plan_file, registry=registry)
    if plan.robot_id != robot_id:
        raise PlanValidationError("showcase plan robot_id does not match the request")
    route = request["capability_route"]
    if not isinstance(route, dict):
        raise RuntimeError("planner_request returned an invalid capability route")
    raw_candidates = route.get("candidates", [])
    if not isinstance(raw_candidates, list):
        raise RuntimeError("planner_request returned invalid candidates")
    candidates = [
        str(candidate["runtime_name"])
        for candidate in raw_candidates
        if isinstance(candidate, dict) and "runtime_name" in candidate
    ]
    selected = [step.capability for step in plan.steps]
    outside_shortlist = sorted(set(selected) - set(candidates))
    if outside_shortlist:
        raise PlanValidationError(
            "validated plan selected capabilities outside the routed shortlist: "
            + ", ".join(outside_shortlist)
        )
    return {
        "contract_version": SHOWCASE_PLANNING_CONTRACT_VERSION,
        "goal": goal,
        "goal_frame": parsed_frame.to_dict(),
        "robot_id": robot_id,
        "capability_routing": {
            "registry_snapshot": route.get("registry_snapshot"),
            "confidence": route.get("confidence"),
            "needs_clarification": route.get("needs_clarification"),
            "shortlist": candidates,
            "shortlist_size": len(candidates),
        },
        "validated_plan": {
            "plan_id": plan.plan_id,
            "source": {
                "kind": plan.generated_by.kind,
                "provider": plan.generated_by.provider,
                "model": plan.generated_by.model,
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
            "direct_motor_commands_allowed": False,
        },
    }
