"""Compose validated delivery workflows from screened operator goals."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any

from .ai_planner import PlanValidationError, compile_workflow, parse_plan, plan_to_dict
from .capabilities import (
    CapabilityRoutingContext,
    CapabilityValidationError,
    GoalFrame,
    default_capability_registry,
)
from .contracts import DeliveryJob
from .goal_resolver import (
    GOAL_MATCHER_ID,
    DecisionTimeline,
    GoalResolution,
    GoalResolutionError,
    rejection_payload,
    resolve_delivery_goal,
)
from .semantic_map import (
    SemanticLocationMap,
    SemanticLocationStore,
    SemanticMapValidationError,
)
from .workflow import WorkflowPlan

DECISION_CONTRACT_VERSION = "flyto.robotics.delivery-decision.v1"
GOAL_PLANNER_ID = "flyto.robotics.goal-planner.deterministic.v1"
GOAL_PLAN_TEMPLATE = "delivery.goal.qr_confirmed.v1"
FIXED_PLAN_TEMPLATE = "hospital_delivery.qr_confirmed.v1"
RESUME_TIMEOUT_SECONDS = 30.0
SAFE_STOP_TIMEOUT_SECONDS = 5.0
MAX_ROUTE_STAGE_NODES = 8
ROUTE_NODE_COLOR = "#8b5cf6"

# Phase-1 execution envelope, expressed declaratively: omitting
# camera.line_scene makes follow_line structurally unroutable, and withholding
# location.write blocks goal-driven map writes.
DELIVERY_OBSERVATIONS = ("odometry", "minimum_range", "human_decision")
DELIVERY_RESOURCES = (
    "base_controller",
    "semantic_map",
    "operator_channel",
    "range_sensor",
)
DELIVERY_PERMISSIONS = ("location.read",)

DELIVERY_GOAL_FRAME = GoalFrame(
    intent_ids=(
        "route.navigate.pose",
        "route.navigate.location",
        "time.dwell",
        "human.approval.request",
        "human.approval.resume",
        "safety.stop",
    ),
    required_affordances=(
        "motion.navigate.pose",
        "motion.navigate.semantic_location",
        "time.wait.bounded",
        "human.request_decision",
        "human.resume_after_approval",
        "safety.stop.motion",
    ),
    desired_effects=(
        "robot.pose.reached",
        "robot.location.reached",
        "time.elapsed",
        "human.decision.requested",
        "workflow.resumed",
        "robot.motion.stopped",
    ),
)


@dataclass(frozen=True)
class GoalDecision:
    """Outcome of turning one goal into an executable workflow, or refusing it."""

    accepted: bool
    workflow: WorkflowPlan | None
    decision: dict[str, Any]
    rejection: dict[str, Any] | None
    route_graph: dict[str, Any] | None
    resolution: GoalResolution | None

    @property
    def reason_code(self) -> str:
        if self.rejection is None:
            return ""
        return str(self.rejection["reason_code"])


def _sha256_of(value: object) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _routing_context(job: DeliveryJob) -> CapabilityRoutingContext:
    return CapabilityRoutingContext(
        robot_model=job.robot_id,
        available_observations=frozenset(DELIVERY_OBSERVATIONS),
        available_resources=frozenset(DELIVERY_RESOURCES),
        granted_permissions=frozenset(DELIVERY_PERMISSIONS),
    )


def compose_delivery_plan(
    *,
    job: DeliveryJob,
    goal: str,
    plan_id: str,
    destination_id: str,
    approval_id: str,
    confirmation_timeout_seconds: float,
) -> dict[str, Any]:
    """Build one flyto.robotics.plan.v1 payload for a resolved destination."""
    pickup_dwell = max(0.0, job.safety.pickup_dwell_seconds)
    dropoff_dwell = max(0.0, job.safety.dropoff_dwell_seconds)
    mission_timeout = job.safety.mission_timeout_seconds
    return {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": plan_id,
        "robot_id": job.robot_id,
        "goal": goal,
        "generated_by": {
            "kind": "deterministic_demo",
            "provider": "flyto.robotics",
            "model": "goal-resolver.v1",
        },
        "steps": [
            {
                "step_id": "navigate.pickup",
                "capability": "navigate",
                "arguments": {
                    "station_id": job.pickup.station_id,
                    "x": job.pickup.x,
                    "y": job.pickup.y,
                    "yaw": job.pickup.yaw,
                },
                "timeout_seconds": mission_timeout,
                "on_failure": "abort",
            },
            {
                "step_id": "dwell.pickup",
                "capability": "dwell",
                "arguments": {"seconds": pickup_dwell},
                "timeout_seconds": max(1.0, pickup_dwell + 1.0),
                "on_failure": "abort",
            },
            {
                "step_id": "navigate.destination",
                "capability": "navigate_to_location",
                "arguments": {"location_id": destination_id},
                "timeout_seconds": mission_timeout,
                "on_failure": "abort",
            },
            {
                "step_id": "confirm.dropoff",
                "capability": "ask_human",
                "arguments": {
                    "approval_id": approval_id,
                    "prompt_key": "delivery.qr_confirmation",
                },
                "timeout_seconds": confirmation_timeout_seconds,
                "on_failure": "abort",
            },
            {
                "step_id": "resume.dropoff",
                "capability": "resume",
                "arguments": {"approval_id": approval_id},
                "timeout_seconds": RESUME_TIMEOUT_SECONDS,
                "on_failure": "abort",
            },
            {
                "step_id": "dwell.dropoff",
                "capability": "dwell",
                "arguments": {"seconds": dropoff_dwell},
                "timeout_seconds": max(1.0, dropoff_dwell + 1.0),
                "on_failure": "abort",
            },
            {
                "step_id": "stop.final",
                "capability": "safe_stop",
                "arguments": {"seconds": 0.0},
                "timeout_seconds": SAFE_STOP_TIMEOUT_SECONDS,
                "on_failure": "abort",
            },
        ],
    }


def _decision_steps(
    plan_payload: dict[str, Any], *, execution_mode: str
) -> list[dict[str, Any]]:
    human_gated = {"ask_human", "resume"}
    steps: list[dict[str, Any]] = []
    for index, step in enumerate(plan_payload["steps"], start=1):
        capability = str(step["capability"])
        steps.append(
            {
                "index": index,
                "step_id": step["step_id"],
                "capability": capability,
                "decided_by": "rule_engine",
                "executed_by": (
                    "human_gate" if capability in human_gated else execution_mode
                ),
            }
        )
    return steps


def _route_graph(
    resolution: GoalResolution,
    *,
    routed: tuple[str, ...],
    excluded: tuple[tuple[str, tuple[str, ...]], ...],
) -> dict[str, Any]:
    def node(
        node_id: str, label: str, reason: str, *, selected: bool, excluded_flag: bool
    ) -> dict[str, Any]:
        return {
            "id": node_id[:128],
            "label": label[:128],
            "reason": reason[:128],
            "color": ROUTE_NODE_COLOR,
            "selected": selected,
            "excluded": excluded_flag,
        }

    goal_stage = [
        node(
            "goal",
            resolution.goal or "goal",
            "operator_goal",
            selected=True,
            excluded_flag=False,
        )
    ]
    location_stage = [
        node(
            candidate.location_id,
            candidate.label,
            (
                ", ".join(candidate.match_rules) + f" score {candidate.score:.1f}"
                if candidate.match_rules
                else "shared_fragment"
            ),
            selected=candidate.selected,
            excluded_flag=not candidate.selected,
        )
        for candidate in resolution.candidates[:MAX_ROUTE_STAGE_NODES]
    ]
    capability_stage = [
        node(name, name, "affordance_match", selected=True, excluded_flag=False)
        for name in routed[:MAX_ROUTE_STAGE_NODES]
    ]
    capability_stage.extend(
        node(name, name, ", ".join(reasons), selected=False, excluded_flag=True)
        for name, reasons in excluded[: MAX_ROUTE_STAGE_NODES - len(capability_stage)]
    )
    stages = [stage for stage in (goal_stage, location_stage, capability_stage) if stage]
    return {"graph_id": "delivery.goal.route.v1", "stages": stages}


class FixedTemplateGoalPlanner:
    """Legacy planner: ignores goal text and runs the fixed delivery template."""

    planner_kind = "fixed_template"

    def plan_delivery(
        self,
        *,
        job: DeliveryJob,
        goal: str,
        session_id: str,
        approval_id: str,
        confirmation_timeout_seconds: float,
        execution_mode: str = "unknown",
    ) -> GoalDecision:
        timeline = DecisionTimeline()
        timeline.record(
            stage="goal_received",
            actor="operator",
            detail=f"{len(goal.encode('utf-8'))} bytes accepted by delivery-request.v1",
        )
        timeline.record(
            stage="plan_selected",
            actor="rule_engine",
            detail="fixed delivery template; goal text is not interpreted",
        )
        decision = {
            "contract_version": DECISION_CONTRACT_VERSION,
            "outcome": "accepted",
            "planner_kind": self.planner_kind,
            "planner_id": "flyto.robotics.goal-planner.fixed.v1",
            "plan_template": FIXED_PLAN_TEMPLATE,
            "llm_consulted": False,
            "execution_mode": execution_mode,
            "timeline": timeline.to_list(),
        }
        return GoalDecision(
            accepted=True,
            workflow=None,
            decision=decision,
            rejection=None,
            route_graph=None,
            resolution=None,
        )


class DeterministicDeliveryGoalPlanner:
    """Resolve a goal to one destination and compile a validated workflow.

    Never raises: every failure becomes a structured rejection so the gateway
    can fail closed with an auditable reason instead of a stack trace.
    """

    planner_kind = "deterministic_rule_engine"

    def __init__(
        self,
        *,
        semantic_map: SemanticLocationMap | SemanticLocationStore,
    ) -> None:
        self._semantic_map = semantic_map
        self._registry = default_capability_registry()

    def plan_delivery(
        self,
        *,
        job: DeliveryJob,
        goal: str,
        session_id: str,
        approval_id: str,
        confirmation_timeout_seconds: float,
        execution_mode: str = "unknown",
    ) -> GoalDecision:
        timeline = DecisionTimeline()
        timeline.record(
            stage="goal_received",
            actor="operator",
            detail=f"{len(goal.encode('utf-8'))} bytes accepted by delivery-request.v1",
        )
        try:
            return self._plan(
                job=job,
                goal=goal,
                session_id=session_id,
                approval_id=approval_id,
                confirmation_timeout_seconds=confirmation_timeout_seconds,
                execution_mode=execution_mode,
                timeline=timeline,
            )
        except (
            GoalResolutionError,
            SemanticMapValidationError,
            CapabilityValidationError,
            PlanValidationError,
            OSError,
            ValueError,
        ) as exc:
            return self._rejected(
                resolution=GoalResolution(
                    goal=goal[:2000],
                    normalized_goal="",
                    map_id="",
                    map_revision=0,
                    reason_code="plan_not_executable",
                    stage="workflow_compilation",
                    detail=str(exc)[:160],
                    operator_action="contact_operator",
                ),
                execution_mode=execution_mode,
                timeline=timeline,
            )

    def _rejected(
        self,
        *,
        resolution: GoalResolution,
        execution_mode: str,
        timeline: DecisionTimeline,
        routed: tuple[str, ...] = (),
        excluded: tuple[tuple[str, tuple[str, ...]], ...] = (),
    ) -> GoalDecision:
        timeline.record(
            stage="goal_rejected",
            actor="rule_engine",
            detail=f"{resolution.reason_code}: {resolution.detail}",
        )
        decision = {
            "contract_version": DECISION_CONTRACT_VERSION,
            "outcome": "rejected",
            "planner_kind": self.planner_kind,
            "planner_id": GOAL_PLANNER_ID,
            "plan_template": GOAL_PLAN_TEMPLATE,
            "matcher_id": GOAL_MATCHER_ID,
            "llm_consulted": False,
            "execution_mode": execution_mode,
            "map_id": resolution.map_id,
            "map_revision": resolution.map_revision,
            "reason_code": resolution.reason_code,
            "stage": resolution.stage,
            "timeline": timeline.to_list(),
        }
        return GoalDecision(
            accepted=False,
            workflow=None,
            decision=decision,
            rejection=rejection_payload(resolution),
            route_graph=_route_graph(resolution, routed=routed, excluded=excluded),
            resolution=resolution,
        )

    def _plan(
        self,
        *,
        job: DeliveryJob,
        goal: str,
        session_id: str,
        approval_id: str,
        confirmation_timeout_seconds: float,
        execution_mode: str,
        timeline: DecisionTimeline,
    ) -> GoalDecision:
        resolution = resolve_delivery_goal(goal, semantic_map=self._semantic_map)
        if not resolution.resolved:
            return self._rejected(
                resolution=resolution,
                execution_mode=execution_mode,
                timeline=timeline,
            )
        timeline.record(
            stage="goal_screened",
            actor="rule_engine",
            detail="no safety-override or raw-actuator phrase",
        )
        destination = resolution.destination
        assert destination is not None
        timeline.record(
            stage="location_resolved",
            actor="rule_engine",
            detail=(
                f"{destination.location_id} via "
                f"{','.join(destination.match_rules) or 'label'}, "
                f"score {destination.score:.1f}"
            ),
        )

        route = self._registry.route(
            goal,
            goal_frame=DELIVERY_GOAL_FRAME,
            context=_routing_context(job),
            limit=8,
        )
        routed = route.names
        timeline.record(
            stage="capability_routed",
            actor="capability_registry",
            detail=(
                f"{len(routed)} candidates, confidence {route.confidence:.2f}, "
                f"{len(route.excluded)} excluded"
            ),
        )
        if route.needs_clarification or route.semantic_missing:
            return self._rejected(
                resolution=GoalResolution(
                    goal=resolution.goal,
                    normalized_goal=resolution.normalized_goal,
                    map_id=resolution.map_id,
                    map_revision=resolution.map_revision,
                    candidates=resolution.candidates,
                    reason_code="capability_route_ambiguous",
                    stage="capability_routing",
                    detail="capability routing needs operator clarification",
                    operator_action="restate_goal",
                ),
                execution_mode=execution_mode,
                timeline=timeline,
                routed=routed,
                excluded=route.excluded,
            )

        plan_payload = compose_delivery_plan(
            job=job,
            goal=resolution.goal,
            plan_id=f"delivery.goal.{session_id}",
            destination_id=destination.location_id,
            approval_id=approval_id,
            confirmation_timeout_seconds=confirmation_timeout_seconds,
        )
        outside = sorted(
            {str(step["capability"]) for step in plan_payload["steps"]} - set(routed)
        )
        if outside:
            return self._rejected(
                resolution=GoalResolution(
                    goal=resolution.goal,
                    normalized_goal=resolution.normalized_goal,
                    map_id=resolution.map_id,
                    map_revision=resolution.map_revision,
                    reason_code="capability_outside_shortlist",
                    stage="plan_composition",
                    detail="composed step outside the routed shortlist: "
                    + ", ".join(outside),
                    operator_action="contact_operator",
                ),
                execution_mode=execution_mode,
                timeline=timeline,
                routed=routed,
                excluded=route.excluded,
            )

        plan = parse_plan(plan_payload, registry=self._registry)
        timeline.record(
            stage="plan_validated",
            actor="plan_contract",
            detail=f"{len(plan.steps)} steps, flyto.robotics.plan.v1",
        )
        workflow = compile_workflow(plan, semantic_map=self._semantic_map)
        timeline.record(
            stage="workflow_compiled",
            actor="plan_contract",
            detail=f"pose resolved from map revision {resolution.map_revision}",
        )
        timeline.record(
            stage="execution_started",
            actor=execution_mode,
            detail=f"runner mode {execution_mode}",
        )

        decision = {
            "contract_version": DECISION_CONTRACT_VERSION,
            "outcome": "accepted",
            "planner_kind": self.planner_kind,
            "planner_id": GOAL_PLANNER_ID,
            "plan_template": GOAL_PLAN_TEMPLATE,
            "matcher_id": GOAL_MATCHER_ID,
            "source_kind": workflow.source_kind,
            "llm_consulted": False,
            "llm_provider": None,
            "llm_model": None,
            "fallback_reason": None,
            "registry_snapshot": route.registry_snapshot,
            "routing_confidence": round(route.confidence, 3),
            "needs_clarification": False,
            "routed_capabilities": list(routed),
            "excluded_capabilities": [
                {"runtime_name": name, "reasons": list(reasons)}
                for name, reasons in route.excluded[:10]
            ],
            "map_id": resolution.map_id,
            "map_revision": resolution.map_revision,
            "destination": destination.to_dict(),
            "plan_sha256": _sha256_of(plan_to_dict(plan)),
            "execution_mode": execution_mode,
            "steps": _decision_steps(plan_payload, execution_mode=execution_mode),
            "timeline": timeline.to_list(),
        }
        return GoalDecision(
            accepted=True,
            workflow=workflow,
            decision=decision,
            rejection=None,
            route_graph=_route_graph(
                resolution, routed=routed, excluded=route.excluded
            ),
            resolution=resolution,
        )
