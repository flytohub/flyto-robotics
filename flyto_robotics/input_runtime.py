"""Transport-neutral shortcut input gate for validated robot workflows."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import Enum

from .ai_planner import compile_workflow, parse_planner_response
from .capabilities import SAFE_TEXT, CapabilityRegistry
from .contracts import DeliveryJob
from .mission import Command, MissionController, Pose2D
from .workflow import MissionState, PrimitiveKind, WorkflowPlan

INPUT_EVENT_CONTRACT_VERSION = "flyto.robotics.input-event.v1"
MAX_AUDIT_EVENTS = 2048
MOTION_PRIMITIVES = frozenset(
    {
        PrimitiveKind.NAVIGATE,
        PrimitiveKind.NAVIGATE_TO_LOCATION,
        PrimitiveKind.MOVE_RELATIVE,
        PrimitiveKind.FOLLOW_LINE,
    }
)


class InputValidationError(ValueError):
    """Raised when an input event, binding, or workflow is unsafe."""


class InputPhase(str, Enum):
    PRESS = "press"
    HEARTBEAT = "heartbeat"
    RELEASE = "release"
    DISCONNECT = "disconnect"


@dataclass(frozen=True)
class InputEvent:
    """One untrusted input-device event; arrival time is supplied separately."""

    event_id: str
    source_id: str
    control_id: str
    session_id: str
    phase: InputPhase
    sequence: int

    def __post_init__(self) -> None:
        for field_name in ("event_id", "source_id", "control_id", "session_id"):
            if not SAFE_TEXT.fullmatch(getattr(self, field_name)):
                raise InputValidationError(f"{field_name} must be a safe identifier")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise InputValidationError("sequence must be an integer")
        if self.sequence < 1:
            raise InputValidationError("sequence must be positive")


def parse_input_event(value: object) -> InputEvent:
    """Strictly parse the versioned JSON input envelope."""
    if not isinstance(value, Mapping):
        raise InputValidationError("input event must be an object")
    allowed = {
        "contract_version",
        "event_id",
        "source_id",
        "control_id",
        "session_id",
        "phase",
        "sequence",
    }
    unknown = sorted(set(value) - allowed)
    missing = sorted(allowed - set(value))
    if unknown:
        raise InputValidationError(
            "input event contains unsupported fields: " + ", ".join(unknown)
        )
    if missing:
        raise InputValidationError("input event is missing: " + ", ".join(missing))
    if value["contract_version"] != INPUT_EVENT_CONTRACT_VERSION:
        raise InputValidationError("input event contract_version is unsupported")
    try:
        phase = InputPhase(value["phase"])
    except (TypeError, ValueError) as exc:
        raise InputValidationError("input event phase is unsupported") from exc
    return InputEvent(
        event_id=str(value["event_id"]),
        source_id=str(value["source_id"]),
        control_id=str(value["control_id"]),
        session_id=str(value["session_id"]),
        phase=phase,
        sequence=value["sequence"],  # type: ignore[arg-type]
    )


@dataclass(frozen=True)
class ShortcutBinding:
    """Map one device control to one workflow identifier, never to motor output."""

    binding_id: str
    source_id: str
    control_id: str
    workflow_id: str
    deadman_timeout_seconds: float = 0.5

    def __post_init__(self) -> None:
        for field_name in ("binding_id", "source_id", "control_id", "workflow_id"):
            if not SAFE_TEXT.fullmatch(getattr(self, field_name)):
                raise InputValidationError(f"{field_name} must be a safe identifier")
        if (
            isinstance(self.deadman_timeout_seconds, bool)
            or not isinstance(self.deadman_timeout_seconds, (int, float))
            or not math.isfinite(float(self.deadman_timeout_seconds))
            or not 0.1 <= float(self.deadman_timeout_seconds) <= 5.0
        ):
            raise InputValidationError(
                "deadman_timeout_seconds must be between 0.1 and 5.0"
            )


@dataclass(frozen=True)
class RegisteredWorkflow:
    robot_id: str
    workflow: WorkflowPlan


class ValidatedWorkflowCatalog:
    """Allowlist of workflows compiled through the normal plan validator."""

    def __init__(self, workflows: Iterable[RegisteredWorkflow]) -> None:
        registered: dict[str, RegisteredWorkflow] = {}
        for item in workflows:
            if not SAFE_TEXT.fullmatch(item.robot_id):
                raise InputValidationError("registered robot_id must be safe")
            workflow = item.workflow
            if workflow.workflow_id in registered:
                raise InputValidationError(
                    f"duplicate workflow_id: {workflow.workflow_id}"
                )
            if (
                any(step.kind in MOTION_PRIMITIVES for step in workflow.steps)
                and workflow.steps[-1].kind != PrimitiveKind.SAFE_STOP
            ):
                raise InputValidationError(
                    f"motion workflow {workflow.workflow_id} must end with safe_stop"
                )
            registered[workflow.workflow_id] = item
        if not registered:
            raise InputValidationError("workflow catalog cannot be empty")
        self._workflows = registered

    @classmethod
    def from_plan_payloads(
        cls,
        payloads: Iterable[object],
        *,
        registry: CapabilityRegistry | None = None,
    ) -> ValidatedWorkflowCatalog:
        """Validate raw plan JSON before it enters the shortcut allowlist."""
        registered: list[RegisteredWorkflow] = []
        for payload in payloads:
            plan = parse_planner_response(payload, registry=registry)
            registered.append(
                RegisteredWorkflow(
                    robot_id=plan.robot_id,
                    workflow=compile_workflow(plan),
                )
            )
        return cls(registered)

    def resolve(self, workflow_id: str, *, robot_id: str) -> WorkflowPlan:
        try:
            registered = self._workflows[workflow_id]
        except KeyError as exc:
            raise InputValidationError(
                f"workflow_id is not registered: {workflow_id}"
            ) from exc
        if registered.robot_id != robot_id:
            raise InputValidationError(
                f"workflow {workflow_id} is not registered for robot {robot_id}"
            )
        return registered.workflow


@dataclass(frozen=True)
class ShortcutAction:
    """Lifecycle instruction emitted by the input gate; contains no motor values."""

    kind: str
    reason: str
    binding_id: str | None = None
    workflow: WorkflowPlan | None = None


@dataclass
class _ActiveBinding:
    binding: ShortcutBinding
    session_id: str
    last_sequence: int
    last_seen_at: float


class ShortcutDispatcher:
    """Resolve input lifecycle events into bounded workflow lifecycle actions."""

    def __init__(
        self,
        bindings: Iterable[ShortcutBinding],
        *,
        catalog: ValidatedWorkflowCatalog,
        robot_id: str,
    ) -> None:
        self.catalog = catalog
        self.robot_id = robot_id
        self._bindings: dict[tuple[str, str], ShortcutBinding] = {}
        for binding in bindings:
            key = (binding.source_id, binding.control_id)
            if key in self._bindings:
                raise InputValidationError(
                    f"duplicate shortcut binding for {binding.source_id}/{binding.control_id}"
                )
            catalog.resolve(binding.workflow_id, robot_id=robot_id)
            self._bindings[key] = binding
        if not self._bindings:
            raise InputValidationError("at least one shortcut binding is required")
        self._active: _ActiveBinding | None = None
        self._seen_event_ids: set[str] = set()

    def _remember_event(self, event_id: str) -> bool:
        if event_id in self._seen_event_ids:
            return False
        if len(self._seen_event_ids) >= 4096:
            self._seen_event_ids.clear()
        self._seen_event_ids.add(event_id)
        return True

    @staticmethod
    def _validate_now(now: float) -> None:
        if (
            isinstance(now, bool)
            or not isinstance(now, (int, float))
            or not math.isfinite(float(now))
            or now < 0.0
        ):
            raise InputValidationError("arrival time must be a non-negative number")

    def handle(self, event: InputEvent, *, now: float) -> ShortcutAction:
        self._validate_now(now)
        if not self._remember_event(event.event_id):
            return ShortcutAction("ignored", "event_replay")

        active = self._active
        if event.phase == InputPhase.DISCONNECT:
            if active is not None and active.binding.source_id == event.source_id:
                self._active = None
                return ShortcutAction(
                    "safe_stop",
                    "input_disconnected",
                    binding_id=active.binding.binding_id,
                )
            return ShortcutAction("ignored", "inactive_disconnect")

        binding = self._bindings.get((event.source_id, event.control_id))
        if binding is None:
            return ShortcutAction("ignored", "binding_not_found")

        if event.phase == InputPhase.PRESS:
            if active is not None:
                return ShortcutAction(
                    "ignored",
                    "another_binding_is_active",
                    binding_id=active.binding.binding_id,
                )
            workflow = self.catalog.resolve(binding.workflow_id, robot_id=self.robot_id)
            self._active = _ActiveBinding(
                binding=binding,
                session_id=event.session_id,
                last_sequence=event.sequence,
                last_seen_at=now,
            )
            return ShortcutAction(
                "start_workflow",
                "input_pressed",
                binding_id=binding.binding_id,
                workflow=workflow,
            )

        if (
            active is None
            or active.binding != binding
            or active.session_id != event.session_id
        ):
            return ShortcutAction("ignored", "input_session_not_active")
        if event.sequence <= active.last_sequence:
            return ShortcutAction(
                "ignored",
                "sequence_not_increasing",
                binding_id=binding.binding_id,
            )

        active.last_sequence = event.sequence
        active.last_seen_at = now
        if event.phase == InputPhase.HEARTBEAT:
            return ShortcutAction(
                "keepalive",
                "input_heartbeat",
                binding_id=binding.binding_id,
            )
        if event.phase == InputPhase.RELEASE:
            self._active = None
            return ShortcutAction(
                "safe_stop",
                "input_released",
                binding_id=binding.binding_id,
            )
        return ShortcutAction("ignored", "input_phase_not_actionable")

    def poll(self, *, now: float) -> ShortcutAction | None:
        self._validate_now(now)
        active = self._active
        if active is None:
            return None
        if now - active.last_seen_at <= active.binding.deadman_timeout_seconds:
            return None
        self._active = None
        return ShortcutAction(
            "safe_stop",
            "input_timeout",
            binding_id=active.binding.binding_id,
        )


@dataclass(frozen=True)
class InputRuntimeEvent:
    sequence: int
    at_seconds: float
    kind: str
    reason: str
    binding_id: str | None
    workflow_id: str | None


class ShortcutRuntime:
    """Connect the input gate to MissionController without exposing motor commands."""

    def __init__(
        self,
        job: DeliveryJob,
        *,
        catalog: ValidatedWorkflowCatalog,
        bindings: Iterable[ShortcutBinding],
    ) -> None:
        self.job = job
        self.dispatcher = ShortcutDispatcher(
            bindings,
            catalog=catalog,
            robot_id=job.robot_id,
        )
        self.controller: MissionController | None = None
        self.events: list[InputRuntimeEvent] = []

    def _record(self, action: ShortcutAction, now: float) -> None:
        if len(self.events) >= MAX_AUDIT_EVENTS:
            return
        workflow_id = (
            action.workflow.workflow_id
            if action.workflow is not None
            else (
                self.controller.workflow.workflow_id
                if self.controller is not None
                else None
            )
        )
        self.events.append(
            InputRuntimeEvent(
                sequence=len(self.events) + 1,
                at_seconds=round(now, 3),
                kind=action.kind,
                reason=action.reason,
                binding_id=action.binding_id,
                workflow_id=workflow_id,
            )
        )

    def _apply(self, action: ShortcutAction, *, now: float) -> None:
        self._record(action, now)
        if action.kind == "start_workflow":
            if action.workflow is None:
                raise RuntimeError("start_workflow action is missing its workflow")
            self.controller = MissionController(
                self.job,
                workflow=action.workflow,
                started_at=now,
            )
        elif action.kind == "safe_stop" and self.controller is not None:
            self.controller.cancel_for_safety(now, reason=action.reason)

    def handle_event(self, event: InputEvent, *, now: float) -> ShortcutAction:
        action = self.dispatcher.handle(event, now=now)
        self._apply(action, now=now)
        return action

    def tick(
        self,
        pose: Pose2D,
        *,
        minimum_range: float,
        now: float,
    ) -> Command:
        deadman_action = self.dispatcher.poll(now=now)
        if deadman_action is not None:
            self._apply(deadman_action, now=now)
        if self.controller is None:
            return Command(0.0, 0.0, MissionState.STOPPED, "shortcut_idle")
        return self.controller.tick(
            pose,
            minimum_range=minimum_range,
            now=now,
        )
