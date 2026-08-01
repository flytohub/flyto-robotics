"""Deterministic hospital-delivery mission state machine."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal

from .capabilities import SAFE_TEXT
from .contracts import RESULT_CONTRACT_VERSION, DeliveryJob, StationPose
from .line_perception import LineScene
from .semantic_map import SemanticLocationStore, SemanticMapValidationError
from .workflow import (
    MissionState,
    PrimitiveKind,
    WorkflowPlan,
    WorkflowStep,
    hospital_delivery_workflow,
)

TERMINAL_STATES = frozenset(
    {MissionState.COMPLETED, MissionState.FAILED, MissionState.CANCELLED}
)

SensorGateDecision = Literal["wait", "ready", "fail_not_ready", "fail_stale"]


def evaluate_sensor_gate(
    *,
    samples_present: bool,
    oldest_sample_age: float,
    ready_duration: float,
    startup_elapsed: float,
    startup_grace: float,
    freshness_timeout: float,
    stabilization_seconds: float,
    control_started: bool,
) -> SensorGateDecision:
    """Classify sensor readiness without trusting bootstrap samples.

    A ROS graph can briefly expose samples from a previous Gazebo generation
    while a new world is still starting.  Before the first control command,
    require all sensors to remain fresh for a bounded stabilization window.
    Once control has started, any required sensor loss fails closed.
    """
    fresh = samples_present and oldest_sample_age <= freshness_timeout
    if control_started:
        return "ready" if fresh else "fail_stale"
    if fresh and ready_duration >= stabilization_seconds:
        return "ready"
    if startup_elapsed > startup_grace:
        return "fail_not_ready"
    return "wait"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Command:
    linear_x: float
    angular_z: float
    state: MissionState
    reason: str


@dataclass(frozen=True)
class MissionEvent:
    at_seconds: float
    kind: str
    state: MissionState
    detail: str
    sequence: int
    step_id: str | None = None
    capability: str | None = None
    actor_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "at_seconds": round(self.at_seconds, 3),
            "kind": self.kind,
            "state": self.state.value,
            "detail": self.detail,
            "step_id": self.step_id,
            "capability": self.capability,
            "actor_id": self.actor_id,
        }


@dataclass(frozen=True)
class HumanDecision:
    approval_id: str
    approved: bool
    actor_id: str
    at_seconds: float


def normalize_angle(angle: float) -> float:
    """Normalize radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class MissionController:
    """Pure controller shared by deterministic dry-run and the ROS adapter."""

    def __init__(
        self,
        job: DeliveryJob,
        *,
        workflow: WorkflowPlan | None = None,
        semantic_map_store: SemanticLocationStore | None = None,
        started_at: float = 0.0,
    ) -> None:
        self.job = job
        self.workflow = workflow or hospital_delivery_workflow(job)
        self.semantic_map_store = semantic_map_store
        self.started_at = started_at
        self.state = MissionState.ACCEPTED
        self.state_entered_at = started_at
        self.step_index = -1
        self.failure_reason: str | None = None
        self.obstacle_active = False
        self.safety_stop_count = 0
        self.line_acquired_at: float | None = None
        self.line_last_seen_at: float | None = None
        self.relative_origin: Pose2D | None = None
        self.clear_since: float | None = None
        self.clearance_blocked = False
        self.approval_requests: set[str] = set()
        self.human_decisions: dict[str, HumanDecision] = {}
        self.human_decision_rejection_count = 0
        self.events: list[MissionEvent] = [
            MissionEvent(
                0.0,
                "mission_accepted",
                self.state,
                f"job contract validated; workflow={self.workflow.workflow_id}",
                1,
            )
        ]

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL_STATES

    def _elapsed(self, now: float) -> float:
        return max(0.0, now - self.started_at)

    def _record_event(
        self,
        now: float,
        kind: str,
        detail: str,
        *,
        state: MissionState | None = None,
        actor_id: str | None = None,
        step: WorkflowStep | None = None,
    ) -> None:
        active_step = step
        if active_step is None and 0 <= self.step_index < len(self.workflow.steps):
            active_step = self.workflow.steps[self.step_index]
        self.events.append(
            MissionEvent(
                at_seconds=self._elapsed(now),
                kind=kind,
                state=state or self.state,
                detail=detail,
                sequence=len(self.events) + 1,
                step_id=active_step.step_id if active_step is not None else None,
                capability=active_step.kind.value if active_step is not None else None,
                actor_id=actor_id,
            )
        )

    def _transition(
        self,
        state: MissionState,
        now: float,
        *,
        detail: str,
        kind: str = "state_transition",
    ) -> None:
        self.state = state
        self.state_entered_at = now
        self._record_event(now, kind, detail, state=state)

    def fail(self, reason: str, now: float) -> Command:
        if not self.terminal:
            self.failure_reason = reason
            self._transition(MissionState.FAILED, now, detail=reason, kind="mission_failed")
        return Command(0.0, 0.0, self.state, reason)

    def cancel(self, now: float) -> Command:
        return self.cancel_for_safety(now, reason="cancel_requested")

    def cancel_for_safety(self, now: float, *, reason: str) -> Command:
        """Cancel with a bounded external safety reason while holding zero velocity."""
        if not SAFE_TEXT.fullmatch(reason):
            raise ValueError("safety cancellation reason must be a safe identifier")
        if not self.terminal:
            self._transition(
                MissionState.CANCELLED,
                now,
                detail=reason,
                kind="mission_cancelled",
            )
        return Command(0.0, 0.0, self.state, reason)

    def submit_human_decision(
        self,
        *,
        approval_id: str,
        approved: bool,
        actor_id: str,
        now: float,
    ) -> None:
        """Accept one explicit, correlated human decision for the active gate."""
        if self.terminal:
            raise ValueError("cannot approve a terminal mission")
        if not SAFE_TEXT.fullmatch(approval_id):
            raise ValueError("approval_id must be a safe identifier")
        if not SAFE_TEXT.fullmatch(actor_id):
            raise ValueError("actor_id must be a safe identifier")
        if not isinstance(approved, bool):
            raise ValueError("approved must be a boolean")
        if not 0 <= self.step_index < len(self.workflow.steps):
            raise ValueError("no workflow step is active")
        step = self._current_step()
        expected = step.argument("approval_id")
        if step.kind != PrimitiveKind.ASK_HUMAN or expected != approval_id:
            raise ValueError("human decision does not match the active ask_human gate")
        if approval_id in self.human_decisions:
            raise ValueError("human decision has already been recorded")
        decision = HumanDecision(
            approval_id=approval_id,
            approved=approved,
            actor_id=actor_id,
            at_seconds=self._elapsed(now),
        )
        self.human_decisions[approval_id] = decision
        self._record_event(
            now,
            "human_approved" if approved else "human_denied",
            f"decision recorded for {approval_id}",
            actor_id=actor_id,
            step=step,
        )

    def record_human_decision_rejection(self, *, reason: str, now: float) -> None:
        """Record bounded security evidence without allowing result-event flooding."""
        if self.terminal or self.human_decision_rejection_count >= 20:
            return
        self.human_decision_rejection_count += 1
        self._record_event(
            now,
            "human_decision_rejected",
            reason[:128],
        )

    def _current_step(self) -> WorkflowStep:
        if not 0 <= self.step_index < len(self.workflow.steps):
            raise RuntimeError("workflow step is not active")
        return self.workflow.steps[self.step_index]

    def _start_next_step(self, now: float) -> bool:
        """Start the next primitive, or complete the workflow."""
        self.step_index += 1
        if self.step_index >= len(self.workflow.steps):
            self._transition(
                MissionState.COMPLETED,
                now,
                detail=f"workflow {self.workflow.workflow_id} completed",
                kind="mission_completed",
            )
            return False
        step = self._current_step()
        self.line_acquired_at = None
        self.line_last_seen_at = None
        self.relative_origin = None
        self.clear_since = None
        self.clearance_blocked = False
        target_detail = (
            f" at {step.station.station_id}" if step.station is not None else ""
        )
        self._transition(
            step.active_state,
            now,
            detail=f"start primitive {step.step_id}{target_detail}",
            kind="primitive_started",
        )
        return True

    def _complete_step(self, now: float, detail: str) -> Command:
        self._record_event(now, "primitive_completed", detail)
        self._start_next_step(now)
        return Command(0.0, 0.0, self.state, "primitive_completed")

    def _step_failure(self, reason: str, now: float) -> Command:
        step = self._current_step()
        if step.on_failure == "request_replan":
            self._record_event(
                now,
                "replan_requested",
                f"{step.step_id}: {reason}",
            )
            return self.fail(f"replan_required:{reason}", now)
        return self.fail(reason, now)

    def _obstacle_guard(self, minimum_range: float, now: float) -> Command | None:
        limits = self.job.safety
        if math.isfinite(minimum_range) and minimum_range < limits.obstacle_stop_distance:
            if not self.obstacle_active:
                self.obstacle_active = True
                self.safety_stop_count += 1
                self._record_event(
                    now,
                    "obstacle_stop",
                    "range below configured stop distance",
                )
            return Command(0.0, 0.0, self.state, "obstacle_stop")

        if self.obstacle_active:
            self.obstacle_active = False
            self._record_event(
                now,
                "path_clear",
                "range recovered above stop distance",
            )
        return None

    def _navigate(self, pose: Pose2D, minimum_range: float, now: float) -> Command:
        limits = self.job.safety
        guarded = self._obstacle_guard(minimum_range, now)
        if guarded is not None:
            return guarded

        step = self._current_step()
        target = step.station
        if target is None:
            return self._step_failure("navigate_target_missing", now)
        delta_x = target.x - pose.x
        delta_y = target.y - pose.y
        distance = math.hypot(delta_x, delta_y)
        if distance <= limits.pose_tolerance:
            return self._complete_step(
                now,
                f"{step.step_id} reached {target.station_id}",
            )

        desired_heading = math.atan2(delta_y, delta_x)
        heading_error = normalize_angle(desired_heading - pose.yaw)
        angular = _clamp(
            1.8 * heading_error,
            -limits.max_angular_speed,
            limits.max_angular_speed,
        )
        if abs(heading_error) > 0.45:
            return Command(0.0, angular, self.state, "turning_to_target")

        linear = min(limits.max_linear_speed, max(0.04, 0.65 * distance))
        linear *= max(0.15, math.cos(heading_error))
        return Command(linear, angular, self.state, "advancing_to_target")

    def _move_relative(
        self,
        pose: Pose2D,
        minimum_range: float,
        now: float,
    ) -> Command:
        """Move a bounded distance from a controller-captured odometry origin."""
        guarded = self._obstacle_guard(minimum_range, now)
        if guarded is not None:
            return guarded

        step = self._current_step()
        target_distance = float(step.argument("distance_m"))
        if self.relative_origin is None:
            self.relative_origin = pose
            self._record_event(
                now,
                "relative_origin_captured",
                f"{step.step_id} captured trusted odometry origin",
            )

        origin = self.relative_origin
        delta_x = pose.x - origin.x
        delta_y = pose.y - origin.y
        progress = delta_x * math.cos(origin.yaw) + delta_y * math.sin(origin.yaw)
        tolerance = max(0.015, min(0.03, self.job.safety.pose_tolerance))
        reached = (
            progress >= target_distance - tolerance
            if target_distance > 0.0
            else progress <= target_distance + tolerance
        )
        if reached:
            return self._complete_step(
                now,
                f"{step.step_id} moved {progress:.3f}m toward {target_distance:.3f}m",
            )

        remaining = target_distance - progress
        speed_limit = min(
            self.job.safety.max_linear_speed,
            float(step.argument("speed", 0.12)),
        )
        linear = math.copysign(
            min(speed_limit, max(0.02, 0.8 * abs(remaining))),
            remaining,
        )
        heading_error = normalize_angle(origin.yaw - pose.yaw)
        angular = _clamp(
            1.8 * heading_error,
            -self.job.safety.max_angular_speed,
            self.job.safety.max_angular_speed,
        )
        return Command(linear, angular, self.state, "moving_relative")

    def _follow_line(
        self,
        line_scene: LineScene | None,
        minimum_range: float,
        now: float,
    ) -> Command:
        guarded = self._obstacle_guard(minimum_range, now)
        if guarded is not None:
            return guarded
        step = self._current_step()
        color = str(step.argument("color"))
        minimum_follow = float(step.argument("minimum_follow_seconds", 0.5))
        target = line_scene.get(color) if line_scene is not None else None
        followed_for = (
            now - self.line_acquired_at if self.line_acquired_at is not None else 0.0
        )

        completion = str(step.argument("completion", "line_end"))
        next_color = step.argument("next_color")
        if (
            completion == "next_color"
            and isinstance(next_color, str)
            and line_scene is not None
            and self.line_acquired_at is not None
            and followed_for >= minimum_follow
        ):
            next_line = line_scene.get(next_color)
            if next_line is not None and next_line.visible and next_line.confidence >= 0.08:
                return self._complete_step(
                    now,
                    f"{step.step_id} observed transition to {next_color}",
                )

        if target is not None and target.visible:
            if self.line_acquired_at is None:
                self.line_acquired_at = now
                self._record_event(
                    now,
                    "line_acquired",
                    f"{step.step_id} acquired {color}",
                )
            self.line_last_seen_at = now
            speed = min(
                self.job.safety.max_linear_speed,
                float(step.argument("speed", 0.16)),
            )
            steering_gain = float(step.argument("steering_gain", 1.2))
            angular = _clamp(
                -steering_gain * target.lateral_error,
                -self.job.safety.max_angular_speed,
                self.job.safety.max_angular_speed,
            )
            linear = speed * max(0.25, 1.0 - abs(target.lateral_error) * 0.7)
            return Command(linear, angular, self.state, f"following_{color}")

        if self.line_acquired_at is None:
            if now - self.state_entered_at > 2.0:
                return self._step_failure(f"line_not_found:{color}", now)
            return Command(0.0, 0.0, self.state, f"waiting_for_{color}")

        lost_for = now - (self.line_last_seen_at or now)
        transition_search_seconds = float(
            step.argument("transition_search_seconds", 1.5)
        )
        if completion == "next_color" and lost_for < transition_search_seconds:
            transition_speed = min(
                0.10,
                self.job.safety.max_linear_speed,
                float(step.argument("speed", 0.16)) * 0.4,
            )
            return Command(
                transition_speed,
                0.0,
                self.state,
                f"searching_next_color:{next_color}",
            )
        if lost_for < 0.45:
            return Command(0.0, 0.0, self.state, f"line_temporarily_lost:{color}")
        if completion == "line_end" and followed_for >= minimum_follow:
            return self._complete_step(now, f"{step.step_id} reached end of {color}")
        return self._step_failure(f"line_lost:{color}", now)

    def _wait_until_clear(self, minimum_range: float, now: float) -> Command:
        step = self._current_step()
        threshold = self.job.safety.obstacle_stop_distance
        blocked = math.isfinite(minimum_range) and minimum_range < threshold
        if blocked:
            self.clear_since = None
            if not self.clearance_blocked:
                self.clearance_blocked = True
                self.safety_stop_count += 1
                self._record_event(
                    now,
                    "clearance_blocked",
                    "wait gate observed range below configured stop distance",
                )
            return Command(0.0, 0.0, self.state, "waiting_for_clearance")

        if self.clear_since is None:
            self.clear_since = now
            self._record_event(
                now,
                "clearance_window_started",
                "continuous safe-clearance verification started",
            )
        required = float(step.argument("clear_seconds", 0.5))
        if now - self.clear_since >= required:
            return self._complete_step(
                now,
                f"{step.step_id} observed clear path for {required:.3f}s",
            )
        return Command(0.0, 0.0, self.state, "verifying_clearance")

    def _ask_human(self, now: float) -> Command:
        step = self._current_step()
        approval_id = str(step.argument("approval_id"))
        if approval_id not in self.approval_requests:
            self.approval_requests.add(approval_id)
            self._record_event(
                now,
                "human_approval_requested",
                f"approval requested for {approval_id}; prompt={step.argument('prompt_key')}",
            )
        decision = self.human_decisions.get(approval_id)
        if decision is None:
            return Command(0.0, 0.0, self.state, "waiting_for_human")
        if not decision.approved:
            return self._step_failure(f"human_denied:{approval_id}", now)
        return self._complete_step(
            now,
            f"{step.step_id} accepted approval from {decision.actor_id}",
        )

    def _resume(self, now: float) -> Command:
        step = self._current_step()
        approval_id = str(step.argument("approval_id"))
        decision = self.human_decisions.get(approval_id)
        if decision is None or not decision.approved:
            return self._step_failure(f"resume_without_approval:{approval_id}", now)
        self._record_event(
            now,
            "resume_authorized",
            f"matching approval verified for {approval_id}",
            actor_id=decision.actor_id,
        )
        return self._complete_step(
            now,
            f"{step.step_id} resumed after {approval_id}",
        )

    def _save_current_location(self, pose: Pose2D, now: float) -> Command:
        step = self._current_step()
        if self.semantic_map_store is None:
            return self._step_failure("semantic_map_store_missing", now)
        location_id = str(step.argument("location_id"))
        label = str(step.argument("label"))
        try:
            snapshot = self.semantic_map_store.remember(
                location_id=location_id,
                label=label,
                pose=StationPose(
                    station_id=location_id,
                    x=pose.x,
                    y=pose.y,
                    yaw=pose.yaw,
                ),
            )
        except (OSError, SemanticMapValidationError) as exc:
            return self._step_failure(
                f"semantic_map_write_failed:{str(exc)[:96]}",
                now,
            )
        self._record_event(
            now,
            "semantic_location_saved",
            f"{location_id} saved in map {snapshot.map_id} revision {snapshot.revision}",
            step=step,
        )
        return self._complete_step(
            now,
            f"{step.step_id} saved {location_id}",
        )

    def tick(
        self,
        pose: Pose2D,
        *,
        minimum_range: float,
        now: float,
        line_scene: LineScene | None = None,
    ) -> Command:
        """Advance one closed-loop control step."""
        if self.terminal:
            return Command(0.0, 0.0, self.state, "terminal")
        if self._elapsed(now) > self.job.safety.mission_timeout_seconds:
            return self.fail("mission_timeout", now)
        if self.state == MissionState.ACCEPTED:
            self._start_next_step(now)

        step = self._current_step()
        if now - self.state_entered_at > step.timeout_seconds:
            return self._step_failure(f"primitive_timeout:{step.step_id}", now)
        if step.kind in {
            PrimitiveKind.NAVIGATE,
            PrimitiveKind.NAVIGATE_TO_LOCATION,
        }:
            return self._navigate(pose, minimum_range, now)

        if step.kind == PrimitiveKind.MOVE_RELATIVE:
            return self._move_relative(pose, minimum_range, now)

        if step.kind == PrimitiveKind.FOLLOW_LINE:
            return self._follow_line(line_scene, minimum_range, now)

        if step.kind == PrimitiveKind.WAIT_UNTIL_CLEAR:
            return self._wait_until_clear(minimum_range, now)

        if step.kind == PrimitiveKind.ASK_HUMAN:
            return self._ask_human(now)

        if step.kind == PrimitiveKind.RESUME:
            return self._resume(now)

        if step.kind == PrimitiveKind.SAVE_CURRENT_LOCATION:
            return self._save_current_location(pose, now)

        if step.kind in {PrimitiveKind.DWELL, PrimitiveKind.SAFE_STOP}:
            if now - self.state_entered_at >= step.dwell_seconds:
                return self._complete_step(
                    now,
                    f"{step.step_id} held stop for {step.dwell_seconds:.3f}s",
                )
            return Command(0.0, 0.0, self.state, step.step_id)

        return self.fail("unsupported_primitive_kind", now)

    def result(self, *, generated_at: str, now: float, pose: Pose2D | None) -> dict[str, Any]:
        """Build a versioned, upload-ready mission result."""
        status = "succeeded" if self.state == MissionState.COMPLETED else self.state.value
        return {
            "contract_version": RESULT_CONTRACT_VERSION,
            "job_id": self.job.job_id,
            "robot_id": self.job.robot_id,
            "status": status,
            "reason": self.failure_reason,
            "generated_at": generated_at,
            "elapsed_seconds": round(self._elapsed(now), 3),
            "final_state": self.state.value,
            "final_pose": (
                {"x": round(pose.x, 4), "y": round(pose.y, 4), "yaw": round(pose.yaw, 4)}
                if pose
                else None
            ),
            "safety_stop_count": self.safety_stop_count,
            "events": [event.to_dict() for event in self.events],
        }
