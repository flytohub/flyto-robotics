"""Untrusted AI-plan boundary and compilation into executable robot workflows."""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from .capabilities import (
    SAFE_TEXT,
    CapabilityRegistry,
    CapabilityRoutingContext,
    CapabilityValidationError,
    GoalFrame,
    default_capability_registry,
)
from .contracts import StationPose
from .semantic_map import (
    SemanticLocationMap,
    SemanticLocationStore,
    SemanticMapValidationError,
)
from .workflow import (
    MissionState,
    PrimitiveKind,
    WorkflowPlan,
    WorkflowStep,
)

PLAN_CONTRACT_VERSION = "flyto.robotics.plan.v1"
MAX_PLAN_BYTES = 128 * 1024
MAX_PLAN_STEPS = 64
ALLOWED_FAILURE_POLICIES = frozenset({"abort", "request_replan"})
MOTION_CAPABILITIES = frozenset(
    {"navigate", "navigate_to_location", "move_relative", "follow_line"}
)


class PlanValidationError(ValueError):
    """Raised when a planner response cannot safely become an executable plan."""


@dataclass(frozen=True)
class PlanSource:
    kind: str
    provider: str
    model: str


@dataclass(frozen=True)
class CapabilityCall:
    step_id: str
    capability: str
    arguments: dict[str, object]
    timeout_seconds: float
    on_failure: str


@dataclass(frozen=True)
class RobotPlan:
    contract_version: str
    plan_id: str
    robot_id: str
    goal: str
    generated_by: PlanSource
    steps: tuple[CapabilityCall, ...]
    registry_snapshot: str = ""
    allowed_capabilities: tuple[str, ...] = ()
    routing_confidence: float = 0.0


class PlannerTransport(Protocol):
    """Provider-neutral boundary; network and model SDKs live behind this protocol."""

    def complete(self, request: dict[str, Any]) -> object:
        """Return a decoded JSON plan or a JSON string containing one."""


@dataclass(frozen=True)
class CallablePlannerTransport:
    """Small adapter useful for Flyto functions, local models, and deterministic tests."""

    callback: Callable[[dict[str, Any]], object]

    def complete(self, request: dict[str, Any]) -> object:
        return self.callback(request)


@dataclass(frozen=True)
class HTTPJsonPlannerTransport:
    """POST the provider-neutral request to Flyto Cloud or a local planner service."""

    url: str
    bearer_token: str | None = None
    timeout_seconds: float = 30.0

    def complete(self, request: dict[str, Any]) -> object:
        plan, _ = self.complete_attested(request)
        return plan

    def complete_attested(
        self,
        request: dict[str, Any],
    ) -> tuple[object, dict[str, Any] | None]:
        """Return the candidate plus optional Flyto AI provenance envelope."""

        parsed = urlparse(self.url)
        local_hosts = {"localhost", "127.0.0.1", "::1"}
        if parsed.scheme != "https" and not (
            parsed.scheme == "http" and parsed.hostname in local_hosts
        ):
            raise PlanValidationError(
                "planner URL must use HTTPS, except HTTP loopback development"
            )
        if not parsed.hostname:
            raise PlanValidationError("planner URL requires a host")
        if not 0.1 <= self.timeout_seconds <= 120.0:
            raise PlanValidationError("planner timeout must be between 0.1 and 120 seconds")
        headers = {
            "Accept": "application/json",
            "Content-Type": "application/json",
        }
        if self.bearer_token:
            headers["Authorization"] = f"Bearer {self.bearer_token}"
        http_request = Request(
            self.url,
            data=json.dumps(request, ensure_ascii=False).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with urlopen(http_request, timeout=self.timeout_seconds) as response:
                payload = response.read(MAX_PLAN_BYTES + 1)
        except HTTPError as exc:
            detail = ""
            try:
                error_payload = exc.read(4097)
                if len(error_payload) <= 4096:
                    decoded_error = json.loads(error_payload.decode("utf-8"))
                    if isinstance(decoded_error, dict) and isinstance(
                        decoded_error.get("detail"),
                        str,
                    ):
                        detail = decoded_error["detail"][:1000]
            except (UnicodeError, json.JSONDecodeError, OSError):
                pass
            message = "planner service rejected the request"
            if detail:
                message = f"{message}: {detail}"
            raise PlanValidationError(message) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise PlanValidationError("planner service request failed") from exc
        if len(payload) > MAX_PLAN_BYTES:
            raise PlanValidationError("planner service response is too large")
        try:
            decoded = json.loads(payload.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise PlanValidationError("planner service must return UTF-8 JSON") from exc
        if isinstance(decoded, dict) and set(decoded) == {"plan"}:
            return decoded["plan"], None
        if isinstance(decoded, dict) and set(decoded) == {
            "contract_version",
            "plan",
            "attestation",
        }:
            if (
                decoded["contract_version"]
                != "flyto.ai.robotics-plan-response.v1"
            ):
                raise PlanValidationError(
                    "planner service returned an unsupported response contract"
                )
            attestation = decoded["attestation"]
            if not isinstance(attestation, dict):
                raise PlanValidationError(
                    "planner service attestation must be an object"
                )
            return decoded["plan"], attestation
        return decoded, None


def _object(value: object, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PlanValidationError(f"{field_name} must be an object")
    return value


def _reject_unknown(data: dict[str, Any], allowed: set[str], field_name: str) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise PlanValidationError(
            f"{field_name} contains unsupported fields: {', '.join(unknown)}"
        )


def _identifier(value: object, field_name: str) -> str:
    if not isinstance(value, str) or not SAFE_TEXT.fullmatch(value):
        raise PlanValidationError(f"{field_name} must be a safe identifier")
    return value


def _bounded_text(value: object, field_name: str, maximum: int) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise PlanValidationError(f"{field_name} must be 1 to {maximum} characters")
    return value.strip()


def _number(value: object, field_name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlanValidationError(f"{field_name} must be a number")
    parsed = float(value)
    if not minimum <= parsed <= maximum:
        raise PlanValidationError(f"{field_name} must be between {minimum} and {maximum}")
    return parsed


def _validate_plan_policy(steps: list[CapabilityCall]) -> None:
    """Apply cross-step safety rules that a per-capability schema cannot express."""
    if (
        any(step.capability in MOTION_CAPABILITIES for step in steps)
        and steps[-1].capability != "safe_stop"
    ):
        raise PlanValidationError("a plan containing motion must end with safe_stop")

    for index, step in enumerate(steps):
        if step.capability != "follow_line":
            continue
        color = str(step.arguments["color"])
        if step.arguments.get("completion") != "next_color":
            continue
        next_color = str(step.arguments["next_color"])
        if next_color == color:
            raise PlanValidationError(
                f"{step.step_id} cannot transition from {color} to the same color"
            )
        next_line = next(
            (
                candidate
                for candidate in steps[index + 1 :]
                if candidate.capability in MOTION_CAPABILITIES
            ),
            None,
        )
        if next_line is None or next_line.capability != "follow_line":
            raise PlanValidationError(
                f"{step.step_id} declares next_color {next_color} without a later "
                "follow_line step"
            )
        if next_line.arguments.get("color") != next_color:
            raise PlanValidationError(
                f"{step.step_id} declares next_color {next_color}, but the next "
                f"line step follows {next_line.arguments.get('color')}"
            )

    pending_approvals: set[str] = set()
    completed_approvals: set[str] = set()
    for step in steps:
        if step.capability == "ask_human":
            approval_id = str(step.arguments["approval_id"])
            if approval_id in pending_approvals or approval_id in completed_approvals:
                raise PlanValidationError(
                    f"approval_id {approval_id} must be unique within a plan"
                )
            pending_approvals.add(approval_id)
        elif step.capability == "resume":
            approval_id = str(step.arguments["approval_id"])
            if approval_id not in pending_approvals:
                raise PlanValidationError(
                    f"resume {step.step_id} has no preceding ask_human for {approval_id}"
                )
            pending_approvals.remove(approval_id)
            completed_approvals.add(approval_id)
        elif step.capability in MOTION_CAPABILITIES and pending_approvals:
            unresolved = ", ".join(sorted(pending_approvals))
            raise PlanValidationError(
                f"motion cannot continue before resume resolves approval: {unresolved}"
            )
    if pending_approvals:
        unresolved = ", ".join(sorted(pending_approvals))
        raise PlanValidationError(
            f"ask_human requires a matching later resume: {unresolved}"
        )


def parse_plan(
    value: object,
    *,
    registry: CapabilityRegistry | None = None,
) -> RobotPlan:
    """Treat AI output as hostile input and normalize only registered capability calls."""
    active_registry = registry or default_capability_registry()
    data = _object(value, "plan")
    allowed = {
        "contract_version",
        "plan_id",
        "robot_id",
        "goal",
        "generated_by",
        "steps",
    }
    _reject_unknown(data, allowed, "plan")
    missing = sorted(allowed - set(data))
    if missing:
        raise PlanValidationError(f"plan is missing: {', '.join(missing)}")
    if data["contract_version"] != PLAN_CONTRACT_VERSION:
        raise PlanValidationError(f"contract_version must be {PLAN_CONTRACT_VERSION}")

    source_data = _object(data["generated_by"], "generated_by")
    _reject_unknown(source_data, {"kind", "provider", "model"}, "generated_by")
    if set(source_data) != {"kind", "provider", "model"}:
        raise PlanValidationError("generated_by requires kind, provider, and model")
    source_kind = source_data["kind"]
    if source_kind not in {"llm", "human", "deterministic_demo"}:
        raise PlanValidationError("generated_by.kind is unsupported")
    source = PlanSource(
        kind=source_kind,
        provider=_identifier(source_data["provider"], "generated_by.provider"),
        model=_identifier(source_data["model"], "generated_by.model"),
    )

    raw_steps = data["steps"]
    if not isinstance(raw_steps, list) or not 1 <= len(raw_steps) <= MAX_PLAN_STEPS:
        raise PlanValidationError(f"steps must contain 1 to {MAX_PLAN_STEPS} items")
    steps: list[CapabilityCall] = []
    for index, raw_step in enumerate(raw_steps):
        field_name = f"steps[{index}]"
        step = _object(raw_step, field_name)
        step_allowed = {
            "step_id",
            "capability",
            "arguments",
            "timeout_seconds",
            "on_failure",
        }
        _reject_unknown(step, step_allowed, field_name)
        if set(step) != step_allowed:
            missing_step = sorted(step_allowed - set(step))
            raise PlanValidationError(
                f"{field_name} is missing: {', '.join(missing_step)}"
            )
        capability = _identifier(step["capability"], f"{field_name}.capability")
        try:
            arguments = active_registry.validate_call(capability, step["arguments"])
        except CapabilityValidationError as exc:
            raise PlanValidationError(str(exc)) from exc
        policy = step["on_failure"]
        if policy not in ALLOWED_FAILURE_POLICIES:
            raise PlanValidationError(
                f"{field_name}.on_failure must be abort or request_replan"
            )
        steps.append(
            CapabilityCall(
                step_id=_identifier(step["step_id"], f"{field_name}.step_id"),
                capability=capability,
                arguments=arguments,
                timeout_seconds=_number(
                    step["timeout_seconds"],
                    f"{field_name}.timeout_seconds",
                    0.1,
                    3600.0,
                ),
                on_failure=policy,
            )
        )

    identifiers = [step.step_id for step in steps]
    if len(identifiers) != len(set(identifiers)):
        raise PlanValidationError("step_id values must be unique")
    _validate_plan_policy(steps)
    return RobotPlan(
        contract_version=PLAN_CONTRACT_VERSION,
        plan_id=_identifier(data["plan_id"], "plan_id"),
        robot_id=_identifier(data["robot_id"], "robot_id"),
        goal=_bounded_text(data["goal"], "goal", 2000),
        generated_by=source,
        steps=tuple(steps),
    )


def parse_planner_response(
    response: object,
    *,
    registry: CapabilityRegistry | None = None,
) -> RobotPlan:
    """Decode a strict JSON response; markdown fences and commentary are rejected."""
    decoded = response
    if isinstance(response, str):
        if len(response.encode("utf-8")) > MAX_PLAN_BYTES:
            raise PlanValidationError("planner response is too large")
        try:
            decoded = json.loads(response)
        except json.JSONDecodeError as exc:
            raise PlanValidationError("planner response must be JSON only") from exc
    return parse_plan(decoded, registry=registry)


def load_plan(
    path: str | Path,
    *,
    registry: CapabilityRegistry | None = None,
) -> RobotPlan:
    """Load a bounded UTF-8 plan file and apply the same untrusted-input boundary."""
    plan_path = Path(path)
    try:
        size = plan_path.stat().st_size
    except OSError as exc:
        raise PlanValidationError("plan file is not readable") from exc
    if size > MAX_PLAN_BYTES:
        raise PlanValidationError(f"plan file exceeds {MAX_PLAN_BYTES} bytes")
    try:
        response = plan_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise PlanValidationError("plan file must be readable UTF-8") from exc
    return parse_planner_response(response, registry=registry)


def plan_to_dict(plan: RobotPlan) -> dict[str, Any]:
    return {
        "contract_version": plan.contract_version,
        "plan_id": plan.plan_id,
        "robot_id": plan.robot_id,
        "goal": plan.goal,
        "generated_by": {
            "kind": plan.generated_by.kind,
            "provider": plan.generated_by.provider,
            "model": plan.generated_by.model,
        },
        "steps": [
            {
                "step_id": step.step_id,
                "capability": step.capability,
                "arguments": step.arguments,
                "timeout_seconds": step.timeout_seconds,
                "on_failure": step.on_failure,
            }
            for step in plan.steps
        ],
    }


def planner_request(
    *,
    goal: str,
    robot_id: str,
    registry: CapabilityRegistry | None = None,
    observations: dict[str, object] | None = None,
    goal_frame: GoalFrame | dict[str, object] | None = None,
    routing_context: CapabilityRoutingContext | dict[str, object] | None = None,
    semantic_map: SemanticLocationMap | SemanticLocationStore | None = None,
    route_limit: int = 8,
) -> dict[str, Any]:
    """Build the provider-neutral request handed to an LLM or agent runtime."""
    active_registry = registry or default_capability_registry()
    bounded_goal = _bounded_text(goal, "goal", 2000)
    try:
        route = active_registry.route(
            bounded_goal,
            goal_frame=goal_frame,
            context=routing_context,
            limit=route_limit,
        )
    except CapabilityValidationError as exc:
        raise PlanValidationError(str(exc)) from exc
    active_observations = dict(observations or {})
    if semantic_map is not None:
        if "semantic_map" in active_observations:
            raise PlanValidationError(
                "observations.semantic_map is reserved for the trusted semantic map"
            )
        try:
            active_observations["semantic_map"] = semantic_map.planner_view()
        except SemanticMapValidationError as exc:
            raise PlanValidationError(str(exc)) from exc
    return {
        "planner_contract": "flyto.robotics.planner-request.v1",
        "instructions": (
            "Return one JSON object only using flyto.robotics.plan.v1. Select and order "
            "only the shortlisted capabilities below and emit each capability's "
            "runtime_name in plan steps. Never emit wheel speeds, PWM, shell commands, "
            "ROS topics, source code, canonical IDs, or unregistered tools. Every motion "
            "plan must end with safe_stop. For navigate_to_location, select only one "
            "location_id from observations.semantic_map and never emit x, y, or yaw. "
            "Use ask_human followed by matching resume when "
            "capability_route.needs_clarification is true or the goal requires approval. "
            "Use request_replan when runtime observations may require a new semantic plan."
        ),
        "goal": bounded_goal,
        "goal_frame": (
            route.goal_frame.to_dict() if route.goal_frame is not None else None
        ),
        "robot_id": _identifier(robot_id, "robot_id"),
        "capability_route": route.to_dict(),
        "capabilities": active_registry.catalog_for(route.names),
        "observations": active_observations,
    }


def request_ai_plan(
    transport: PlannerTransport,
    *,
    goal: str,
    robot_id: str,
    registry: CapabilityRegistry | None = None,
    observations: dict[str, object] | None = None,
    goal_frame: GoalFrame | dict[str, object] | None = None,
    routing_context: CapabilityRoutingContext | dict[str, object] | None = None,
    semantic_map: SemanticLocationMap | SemanticLocationStore | None = None,
    route_limit: int = 8,
) -> RobotPlan:
    """Ask a model to compose atoms, then validate its response before execution."""
    active_registry = registry or default_capability_registry()
    request = planner_request(
        goal=goal,
        robot_id=robot_id,
        registry=active_registry,
        observations=observations,
        goal_frame=goal_frame,
        routing_context=routing_context,
        semantic_map=semantic_map,
        route_limit=route_limit,
    )
    plan = parse_planner_response(
        transport.complete(request),
        registry=active_registry,
    )
    if plan.robot_id != robot_id:
        raise PlanValidationError("AI plan robot_id does not match the requested robot")
    route = _object(request["capability_route"], "capability_route")
    raw_candidates = route.get("candidates", [])
    allowed = tuple(
        str(candidate["runtime_name"])
        for candidate in raw_candidates
        if isinstance(candidate, dict) and "runtime_name" in candidate
    )
    outside_shortlist = sorted(
        {step.capability for step in plan.steps} - set(allowed)
    )
    if outside_shortlist:
        raise PlanValidationError(
            "AI plan selected capabilities outside the routed shortlist: "
            + ", ".join(outside_shortlist)
        )
    needs_clarification = route.get("needs_clarification") is True
    if needs_clarification and not any(
        step.capability == "ask_human" for step in plan.steps
    ):
        raise PlanValidationError(
            "capability route is ambiguous; the plan must ask_human before action"
        )
    return replace(
        plan,
        registry_snapshot=str(route.get("registry_snapshot", "")),
        allowed_capabilities=allowed,
        routing_confidence=float(route.get("confidence", 0.0)),
    )


def compile_workflow(
    plan: RobotPlan,
    *,
    semantic_map: SemanticLocationMap | SemanticLocationStore | None = None,
) -> WorkflowPlan:
    """Compile calls while resolving semantic IDs through trusted runtime state."""
    steps: list[WorkflowStep] = []
    for call in plan.steps:
        arguments = tuple(sorted(call.arguments.items()))
        if call.capability == "navigate":
            station = StationPose(
                station_id=str(call.arguments["station_id"]),
                x=float(call.arguments["x"]),
                y=float(call.arguments["y"]),
                yaw=float(call.arguments["yaw"]),
            )
            kind = PrimitiveKind.NAVIGATE
            state = MissionState.NAVIGATING
            dwell_seconds = 0.0
        elif call.capability == "move_relative":
            station = None
            kind = PrimitiveKind.MOVE_RELATIVE
            state = MissionState.MOVING_RELATIVE
            dwell_seconds = 0.0
        elif call.capability == "navigate_to_location":
            if semantic_map is None:
                raise PlanValidationError(
                    "navigate_to_location requires a trusted semantic map"
                )
            try:
                location = semantic_map.resolve(
                    str(call.arguments["location_id"])
                )
            except SemanticMapValidationError as exc:
                raise PlanValidationError(str(exc)) from exc
            station = location.pose
            kind = PrimitiveKind.NAVIGATE_TO_LOCATION
            state = MissionState.NAVIGATING_TO_LOCATION
            dwell_seconds = 0.0
        elif call.capability == "save_current_location":
            if not isinstance(semantic_map, SemanticLocationStore):
                raise PlanValidationError(
                    "save_current_location requires a writable semantic map store"
                )
            station = None
            kind = PrimitiveKind.SAVE_CURRENT_LOCATION
            state = MissionState.SAVING_LOCATION
            dwell_seconds = 0.0
        elif call.capability == "follow_line":
            station = None
            kind = PrimitiveKind.FOLLOW_LINE
            state = MissionState.FOLLOWING_LINE
            dwell_seconds = 0.0
        elif call.capability == "dwell":
            station = None
            kind = PrimitiveKind.DWELL
            state = MissionState.WAITING
            dwell_seconds = float(call.arguments["seconds"])
        elif call.capability == "wait_until_clear":
            station = None
            kind = PrimitiveKind.WAIT_UNTIL_CLEAR
            state = MissionState.WAITING_FOR_CLEAR
            dwell_seconds = 0.0
        elif call.capability == "ask_human":
            station = None
            kind = PrimitiveKind.ASK_HUMAN
            state = MissionState.WAITING_FOR_HUMAN
            dwell_seconds = 0.0
        elif call.capability == "resume":
            station = None
            kind = PrimitiveKind.RESUME
            state = MissionState.RESUMING
            dwell_seconds = 0.0
        elif call.capability == "safe_stop":
            station = None
            kind = PrimitiveKind.SAFE_STOP
            state = MissionState.STOPPED
            dwell_seconds = float(call.arguments["seconds"])
        else:
            raise PlanValidationError(f"no executor is installed for {call.capability}")
        steps.append(
            WorkflowStep(
                step_id=call.step_id,
                kind=kind,
                active_state=state,
                station=station,
                dwell_seconds=dwell_seconds,
                arguments=arguments,
                timeout_seconds=call.timeout_seconds,
                on_failure=call.on_failure,
            )
        )
    return WorkflowPlan(
        workflow_id=plan.plan_id,
        steps=tuple(steps),
        goal=plan.goal,
        source_kind=plan.generated_by.kind,
    )
