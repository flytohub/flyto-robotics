"""Atomic, composable mission primitives and workflow compilation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .contracts import DeliveryJob, StationPose


class MissionState(str, Enum):
    ACCEPTED = "accepted"
    NAVIGATING = "navigating"
    MOVING_RELATIVE = "moving_relative"
    TURNING_RELATIVE = "turning_relative"
    NAVIGATING_TO_PICKUP = "navigating_to_pickup"
    WAITING_FOR_PICKUP = "waiting_for_pickup"
    NAVIGATING_TO_DROPOFF = "navigating_to_dropoff"
    WAITING_FOR_DROPOFF = "waiting_for_dropoff"
    FOLLOWING_LINE = "following_line"
    WAITING = "waiting"
    WAITING_FOR_CLEAR = "waiting_for_clear"
    WAITING_FOR_HUMAN = "waiting_for_human"
    RESUMING = "resuming"
    SAVING_LOCATION = "saving_location"
    NAVIGATING_TO_LOCATION = "navigating_to_location"
    STOPPED = "stopped"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class PrimitiveKind(str, Enum):
    NAVIGATE = "navigate"
    MOVE_RELATIVE = "move_relative"
    TURN_RELATIVE = "turn_relative"
    DWELL = "dwell"
    FOLLOW_LINE = "follow_line"
    WAIT_UNTIL_CLEAR = "wait_until_clear"
    ASK_HUMAN = "ask_human"
    RESUME = "resume"
    SAVE_CURRENT_LOCATION = "save_current_location"
    NAVIGATE_TO_LOCATION = "navigate_to_location"
    SAFE_STOP = "safe_stop"


@dataclass(frozen=True)
class WorkflowStep:
    """One independently testable capability in a mission workflow."""

    step_id: str
    kind: PrimitiveKind
    active_state: MissionState
    station: StationPose | None = None
    dwell_seconds: float = 0.0
    arguments: tuple[tuple[str, object], ...] = ()
    timeout_seconds: float = 300.0
    on_failure: str = "abort"

    def __post_init__(self) -> None:
        if not self.step_id:
            raise ValueError("workflow step_id is required")
        if self.timeout_seconds <= 0.0:
            raise ValueError("timeout_seconds must be positive")
        if self.on_failure not in {"abort", "request_replan"}:
            raise ValueError("unsupported on_failure policy")
        if self.kind == PrimitiveKind.DWELL and self.dwell_seconds < 0.0:
            raise ValueError("dwell_seconds cannot be negative")
        navigate_kinds = {
            PrimitiveKind.NAVIGATE,
            PrimitiveKind.NAVIGATE_TO_LOCATION,
        }
        if self.kind in navigate_kinds and self.dwell_seconds != 0.0:
            raise ValueError("navigate primitives cannot define dwell_seconds")
        if self.kind in navigate_kinds and self.station is None:
            raise ValueError("navigate primitives require a station")
        if self.kind == PrimitiveKind.MOVE_RELATIVE:
            distance = self.argument("distance_m")
            if (
                isinstance(distance, bool)
                or not isinstance(distance, (int, float))
                or not 0.01 <= abs(float(distance)) <= 2.0
            ):
                raise ValueError(
                    "move_relative primitives require distance_m between -2.0 and 2.0"
                )
        if self.kind == PrimitiveKind.TURN_RELATIVE:
            delta = self.argument("yaw_delta_rad")
            if (
                isinstance(delta, bool)
                or not isinstance(delta, (int, float))
                or not 0.05 <= abs(float(delta)) <= 3.0
            ):
                raise ValueError(
                    "turn_relative primitives require yaw_delta_rad between -3.0 and 3.0"
                )
        if self.kind == PrimitiveKind.FOLLOW_LINE and self.argument("color") is None:
            raise ValueError("follow_line primitives require a color")
        if self.kind in {PrimitiveKind.ASK_HUMAN, PrimitiveKind.RESUME} and self.argument(
            "approval_id"
        ) is None:
            raise ValueError(f"{self.kind.value} primitives require an approval_id")
        if self.kind == PrimitiveKind.SAVE_CURRENT_LOCATION and (
            self.argument("location_id") is None or self.argument("label") is None
        ):
            raise ValueError(
                "save_current_location primitives require location_id and label"
            )
        if len(dict(self.arguments)) != len(self.arguments):
            raise ValueError("workflow argument names must be unique")

    def argument(self, name: str, default: object | None = None) -> object | None:
        """Read an immutable capability argument."""
        return dict(self.arguments).get(name, default)


@dataclass(frozen=True)
class WorkflowPlan:
    """An immutable composition of ordered robot capability primitives."""

    workflow_id: str
    steps: tuple[WorkflowStep, ...]
    goal: str = ""
    source_kind: str = "deterministic"

    def __post_init__(self) -> None:
        if not self.workflow_id:
            raise ValueError("workflow_id is required")
        if not self.steps:
            raise ValueError("a workflow requires at least one primitive")
        identifiers = [step.step_id for step in self.steps]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("workflow step_id values must be unique")


def hospital_delivery_workflow(job: DeliveryJob) -> WorkflowPlan:
    """Compile a delivery contract into reusable navigate and dwell atoms."""
    return WorkflowPlan(
        workflow_id="hospital_delivery.v1",
        steps=(
            WorkflowStep(
                step_id="navigate.pickup",
                kind=PrimitiveKind.NAVIGATE,
                active_state=MissionState.NAVIGATING_TO_PICKUP,
                station=job.pickup,
                timeout_seconds=job.safety.mission_timeout_seconds,
            ),
            WorkflowStep(
                step_id="dwell.pickup",
                kind=PrimitiveKind.DWELL,
                active_state=MissionState.WAITING_FOR_PICKUP,
                station=job.pickup,
                dwell_seconds=job.safety.pickup_dwell_seconds,
                timeout_seconds=max(1.0, job.safety.pickup_dwell_seconds + 1.0),
            ),
            WorkflowStep(
                step_id="navigate.dropoff",
                kind=PrimitiveKind.NAVIGATE,
                active_state=MissionState.NAVIGATING_TO_DROPOFF,
                station=job.dropoff,
                timeout_seconds=job.safety.mission_timeout_seconds,
            ),
            WorkflowStep(
                step_id="dwell.dropoff",
                kind=PrimitiveKind.DWELL,
                active_state=MissionState.WAITING_FOR_DROPOFF,
                station=job.dropoff,
                dwell_seconds=job.safety.dropoff_dwell_seconds,
                timeout_seconds=max(1.0, job.safety.dropoff_dwell_seconds + 1.0),
            ),
        ),
    )
