"""Deterministic hospital-delivery mission state machine."""

from __future__ import annotations

import math
from collections.abc import Sequence
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

TERMINAL_STATES = frozenset({MissionState.COMPLETED, MissionState.FAILED, MissionState.CANCELLED})

SensorGateDecision = Literal["wait", "ready", "fail_not_ready", "fail_stale"]

# The two sensor windows are deliberately asymmetric, and conflating them is
# how a robot ends up both twitchy and unsafe.
#
# STARTUP GRACE is a deadline, not a delay. Nothing has moved yet, every tick
# publishes a stop, and a healthy robot becomes ready the instant its sensors
# arrive — so raising this does not slow a good start by a millisecond. It only
# extends how long we are willing to wait before declaring the sensor absent.
# Waiting longer therefore costs nothing except how quickly a genuinely broken
# robot is reported, and the failure now names which sensor was missing.
#
# It cannot simply be made enormous, though, because it is spent out of the
# mission's own budget. While the gate says "wait", MissionController.tick is
# never called — but `started_at` was stamped at construction, so every second
# of discovery is already on the clock when the first tick finally lands. A
# grace equal to the mission timeout would leave nothing to drive with, and a
# grace larger than it could never fire at all.
#
# Measured on the lab TurtleBot3 over 30 samples: first /odom at median 1.84s,
# p95 3.58s, max 3.60s, in a bimodal distribution (either the first DDS
# announcement is caught immediately, or the next round is waited for). A
# degraded network the same day produced 9.1s.
#
# 15s covers that 9.1s worst case with room, is four times the measured p95,
# and still leaves half of a 30s mission — the tightest job here — for the
# motion itself, which a 90 degree turn or a 40 cm step does in about five.
DEFAULT_SENSOR_STARTUP_GRACE_SECONDS = 15.0

# FRESHNESS TIMEOUT is the safety-relevant one: how stale a sample may be while
# the robot is under power and moving. It must stay tight, and raising it to
# match the grace above would mean driving on second-old obstacle data.
DEFAULT_SENSOR_FRESHNESS_TIMEOUT_SECONDS = 1.0

# STABILIZATION is how long every sensor must stay fresh before the first
# command. It fits *inside* the startup grace, so the real discovery budget is
# the grace minus this — which is what made a 10s grace mean 9s of discovery.
DEFAULT_SENSOR_STABILIZATION_SECONDS = 1.0


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


def unready_sensors(
    *,
    last_seen: dict[str, float | None],
    now: float,
    freshness_timeout: float,
) -> list[str]:
    """Name the sensors holding the gate shut, in operator language.

    :func:`evaluate_sensor_gate` answers whether to drive. It deliberately does
    not say why, because the control loop does not need to know. An operator
    does: a mission that prints only ``failed`` leaves them guessing at which
    of odometry, lidar and camera never showed up, and the guess is usually
    wrong. Discovery latency here has been measured between 7ms and 9.1s
    against a 9s effective budget, so "which one was late" is the whole
    question when a run fails.

    :param last_seen: sensor name to the monotonic time of its last sample, or
        ``None`` if none has ever arrived.
    :param now: current monotonic time.
    :param freshness_timeout: how old a sample may be and still count.
    :returns: one line per sensor that is missing or stale, in the order given.
        Empty when every sensor is present and fresh — which is a real answer
        too: it means the gate is waiting on the stabilization window, not on
        a sensor.
    """
    report: list[str] = []
    for name, sample_time in last_seen.items():
        if sample_time is None:
            report.append(f"{name}: never arrived")
            continue
        age = now - sample_time
        if age > freshness_timeout:
            report.append(f"{name}: last sample {age:.1f}s ago")
    return report


def closest_range(range_field: Any, fallback: float) -> float | None:
    """The nearest return the lidar reported, or None when it saw nothing.

    Infinity is how "nothing measured there" is spelled inside the controller,
    and it must not travel outward as a number: a session claiming a clearance
    of ``inf`` reads as a wide open corridor when it actually means the sensor
    had nothing to say. None is the honest answer, and the delivery session
    payload already treats it that way.

    Lives here rather than in the ROS backend that calls it, for the same
    reason :func:`evaluate_sensor_gate` does: it is a decision about what a
    reading means, and it must be testable without a robot.
    """
    closest = getattr(range_field, "closest", None)
    if not isinstance(closest, (int, float)) or not math.isfinite(closest):
        closest = fallback
    if not isinstance(closest, (int, float)) or not math.isfinite(closest):
        return None
    return float(closest)


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


# What the robot is about to do. The guard needs this because a sector is only
# dangerous in the direction of travel: the wall a robot drives alongside is not
# the wall it drives into, and a robot reversing is not protected at all by
# whatever is in front of it.
# How close a relative move must land to count as arrived. Deliberately NOT
# job.safety.pose_tolerance: that knob is validated at a minimum of 0.05 m
# (contracts.py) because it governs navigation to a station, where a 5 cm
# arrival box is right. A bounded 40 cm jog with a 5 cm box would overshoot by
# an eighth of its own distance.
#
# This used to be written as max(0.015, min(0.03, job.safety.pose_tolerance)),
# which reads as "the operator's knob, clamped" and is not: the validated
# minimum is above the clamp's maximum, so both arms were unreachable and the
# value was always 0.03. A constant that pretends to be configurable is worse
# than one that says what it is.
RELATIVE_MOVE_TOLERANCE_M = 0.03

INTENT_FORWARD = "forward"
INTENT_REVERSE = "reverse"
INTENT_ROTATE = "rotate"


# Sector half-widths in degrees, measured from straight ahead.
#
# The forward cone covers the width the robot actually sweeps, and no more.
# At 30 degrees it spans +/-0.26 m at a quarter of a metre out — wider than a
# 28 cm aisle — so driving one it reads the far corners of the intersection and
# stops for them, with nothing in its path. Measured in simulation: 0.452 m
# forward where the aisle ahead was clear for 0.92 m, which is exactly the
# distance to the corner diagonally beyond it.
#
# 15 degrees covers +/-0.067 m at the 0.25 m stop distance, which is a Burger's
# half-width. Narrower would let something graze the corner of the chassis
# unseen; wider reads walls the robot will pass.
FORWARD_HALF_ANGLE_DEG = 15.0
# The sides are wide bands because a wall is long: any part of it entering the
# band is the same wall, and the nearest point is what matters.
SIDE_HALF_ANGLE_DEG = 30.0


def sector_field(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.0,
    range_max: float = math.inf,
) -> RangeField:
    """Turn one sweep into the nearest return per sector.

    Pure, and taking plain numbers rather than a LaserScan, so the arithmetic
    that decides whether a robot moves can be asserted without ROS present.

    A sector with no valid return stays at infinity, and infinity alone cannot
    say why. Three different sweeps used to produce exactly the same all-inf
    field as an open corridor: a stalled rotor publishing zeros, a covered
    sensor publishing sub-``range_min`` returns, and a wall closer than
    ``range_min``. Every threshold downstream then read that as room.

    So each beam is classified rather than merely filtered. A beam past
    ``range_max`` (or ``+inf``, which is how most drivers spell "no echo") is a
    definite answer: nothing is out there within reach. A beam that is NaN,
    non-positive, or below ``range_min`` is not an answer at all. A sector in
    which *no* beam gave a definite answer is reported in
    :attr:`RangeField.blind`, and a caller that would have driven on infinity
    can refuse instead.
    """
    forward = left = right = rear = closest = math.inf
    # Per sector: did any beam yield a definite answer, and were there beams at
    # all? A sector the sweep never covered is left alone — that is geometry,
    # not sensor failure, and calling it blind would strand a limited-field-of-
    # view robot that was never guarded there in the first place.
    definite = {"forward": False, "left": False, "right": False, "rear": False}
    covered = {"forward": False, "left": False, "right": False, "rear": False}

    for index, value in enumerate(ranges):
        bearing = math.degrees(angle_min + angle_increment * index) % 360.0
        if bearing <= FORWARD_HALF_ANGLE_DEG or bearing >= 360.0 - FORWARD_HALF_ANGLE_DEG:
            sector = "forward"
        elif abs(bearing - 90.0) <= SIDE_HALF_ANGLE_DEG:
            sector = "left"
        elif abs(bearing - 270.0) <= SIDE_HALF_ANGLE_DEG:
            sector = "right"
        elif abs(bearing - 180.0) <= FORWARD_HALF_ANGLE_DEG:
            sector = "rear"
        else:
            sector = None

        if sector is not None:
            covered[sector] = True

        if math.isnan(value):
            continue
        if value > range_max:
            # Includes +inf. Nothing within reach, which is an answer.
            if sector is not None:
                definite[sector] = True
            continue
        if value < range_min or value <= 0.0:
            # Under the sensor's floor. It cannot tell an obstacle from a fault.
            continue

        if sector is not None:
            definite[sector] = True
        closest = min(closest, value)
        if sector == "forward":
            forward = min(forward, value)
        elif sector == "left":
            left = min(left, value)
        elif sector == "right":
            right = min(right, value)
        elif sector == "rear":
            rear = min(rear, value)

    return RangeField(
        forward=forward,
        left=left,
        right=right,
        rear=rear,
        closest=closest,
        directional=True,
        blind=frozenset(
            name for name in definite if covered[name] and not definite[name]
        ),
    )


@dataclass(frozen=True)
class RangeField:
    """The nearest return in each direction, as the sensor saw it.

    Every field defaults to infinity — absent means "nothing measured there",
    never "nothing is there". A caller that supplies only ``closest`` gets the
    old omnidirectional behaviour, which is what every caller written before
    sectors existed meant.
    """

    forward: float = math.inf
    left: float = math.inf
    right: float = math.inf
    rear: float = math.inf
    closest: float = math.inf
    directional: bool = False

    #: Sectors the sweep covered but could not measure at all — every beam NaN,
    #: non-positive, or under the sensor's floor. Distinct from infinity, which
    #: means measured and nothing found. Empty by default so a caller that
    #: supplies readings by hand keeps its previous behaviour.
    blind: frozenset[str] = frozenset()

    @classmethod
    def omnidirectional(cls, minimum_range: float) -> RangeField:
        """One reading in every direction, the shape callers used to pass."""
        return cls(closest=minimum_range, directional=False)

    def blocking(self, intent: str) -> tuple[float, str]:
        """The range that matters for this motion, and what to call it."""
        if intent == INTENT_REVERSE:
            return self.rear, "behind"
        if intent == INTENT_ROTATE:
            # Rotating in place sweeps the whole footprint, so every side counts
            # — a robot that turns into a wall it was safely parallel to has
            # been let down by a guard that only watched where it was going.
            return min(self.forward, self.left, self.right, self.rear), "in the turn"
        return self.forward, "ahead"

    def blind_for(self, intent: str) -> str | None:
        """A sector this motion depends on that could not be measured.

        Mirrors :meth:`blocking` exactly: whatever ranges that method would
        consult, this reports the first of them the sensor could not read. A
        motion guarded by a range that was never measured is not guarded.
        """
        if intent == INTENT_REVERSE:
            consulted = ("rear",)
        elif intent == INTENT_ROTATE:
            consulted = ("forward", "left", "right", "rear")
        else:
            consulted = ("forward",)
        for sector in consulted:
            if sector in self.blind:
                return sector
        return None

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
        self.turn_origin_yaw: float | None = None
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
        self.turn_origin_yaw = None
        self.clear_since = None
        self.clearance_blocked = False
        target_detail = f" at {step.station.station_id}" if step.station is not None else ""
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

    def _obstacle_guard(
        self,
        minimum_range: float | RangeField,
        now: float,
        *,
        intent: str = INTENT_FORWARD,
    ) -> Command | None:
        """Two layers: what blocks this motion, and what is simply too close.

        The directional layer asks only about the direction of travel, because
        an omnidirectional gate cannot enter a corridor narrower than twice its
        threshold — the side walls trip it forever. The emergency layer keeps a
        floor under that: whatever the robot is doing, nothing may be nearer
        than a few centimetres in any direction, so relaxing the sides for a
        narrow aisle never becomes permission to graze one.
        """
        limits = self.job.safety
        field = (
            minimum_range
            if isinstance(minimum_range, RangeField)
            else RangeField.omnidirectional(minimum_range)
        )

        # Before any threshold: is there a reading to compare at all? Every
        # test below is guarded by math.isfinite, so an unmeasurable sector
        # skips all of them and the motion proceeds — the exact shape of "the
        # gate cannot evaluate its condition, so it allows the action". A
        # covered lidar, a stalled rotor and a wall inside the sensor's minimum
        # range all arrive here as infinity, indistinguishable from clear.
        unmeasured = field.blind_for(intent)
        if unmeasured is not None:
            return self._raise_obstacle(
                now, f"cannot measure {unmeasured}; refusing to move on an unread sector"
            )

        emergency = limits.emergency_stop_distance
        if math.isfinite(field.closest) and field.closest < emergency:
            return self._raise_obstacle(now, "closer than the emergency distance")

        if not field.directional:
            # No sectors were measured, so the only honest reading is the
            # nearest return anywhere. This is what every pre-sector caller and
            # every pre-sector job still gets.
            if math.isfinite(field.closest) and field.closest < limits.obstacle_stop_distance:
                return self._raise_obstacle(now, "range below configured stop distance")
            return self._clear_obstacle(now)

        blocking, where = field.blocking(intent)
        if math.isfinite(blocking) and blocking < limits.obstacle_stop_distance:
            return self._raise_obstacle(now, f"obstacle {where}")

        lateral_limit = (
            limits.lateral_stop_distance
            if limits.lateral_stop_distance is not None
            else limits.obstacle_stop_distance
        )
        beside = min(field.left, field.right)
        if math.isfinite(beside) and beside < lateral_limit:
            return self._raise_obstacle(now, "obstacle alongside")

        return self._clear_obstacle(now)

    def _raise_obstacle(self, now: float, detail: str) -> Command:
        if not self.obstacle_active:
            self.obstacle_active = True
            self.safety_stop_count += 1
            self._record_event(now, "obstacle_stop", detail)
        return Command(0.0, 0.0, self.state, "obstacle_stop")

    def _clear_obstacle(self, now: float) -> None:
        if self.obstacle_active:
            self.obstacle_active = False
            self._record_event(now, "path_clear", "range recovered above stop distance")
        return None

    def _navigate(self, pose: Pose2D, minimum_range: float, now: float) -> Command:
        limits = self.job.safety
        guarded = self._obstacle_guard(minimum_range, now, intent=INTENT_FORWARD)
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
        guarded = self._obstacle_guard(
            minimum_range,
            now,
            intent=INTENT_REVERSE
            if float(self._current_step().argument("distance_m") or 0.0) < 0
            else INTENT_FORWARD,
        )
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
        tolerance = RELATIVE_MOVE_TOLERANCE_M
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

    def _turn_relative(
        self,
        pose: Pose2D,
        minimum_range: float,
        now: float,
    ) -> Command:
        """Rotate a bounded angle from a controller-captured odometry origin."""
        guarded = self._obstacle_guard(minimum_range, now, intent=INTENT_ROTATE)
        if guarded is not None:
            return guarded

        step = self._current_step()
        target_delta = float(step.argument("yaw_delta_rad"))
        if self.turn_origin_yaw is None:
            self.turn_origin_yaw = pose.yaw
            self._record_event(
                now,
                "relative_origin_captured",
                f"{step.step_id} captured trusted odometry yaw",
            )

        origin_yaw = self.turn_origin_yaw
        # Accumulate so a turn larger than pi cannot alias to the short way
        # round: track the signed delta since the previous tick.
        turned = normalize_angle(pose.yaw - origin_yaw)
        if target_delta > 0.0 and turned < -math.pi / 2:
            turned += 2.0 * math.pi
        elif target_delta < 0.0 and turned > math.pi / 2:
            turned -= 2.0 * math.pi

        tolerance = 0.035
        reached = (
            turned >= target_delta - tolerance
            if target_delta > 0.0
            else turned <= target_delta + tolerance
        )
        if reached:
            return self._complete_step(
                now,
                f"{step.step_id} turned {turned:.3f}rad toward {target_delta:.3f}rad",
            )

        remaining = target_delta - turned
        speed_limit = min(
            self.job.safety.max_angular_speed,
            float(step.argument("angular_speed", 0.6)),
        )
        angular = math.copysign(
            min(speed_limit, max(0.08, 1.8 * abs(remaining))),
            remaining,
        )
        # A turn is rotation only: never translate while rotating in place.
        return Command(0.0, angular, self.state, "turning_relative")

    def _follow_line(
        self,
        line_scene: LineScene | None,
        minimum_range: float,
        now: float,
    ) -> Command:
        guarded = self._obstacle_guard(minimum_range, now, intent=INTENT_FORWARD)
        if guarded is not None:
            return guarded
        step = self._current_step()
        color = str(step.argument("color"))
        minimum_follow = float(step.argument("minimum_follow_seconds", 0.5))
        target = line_scene.get(color) if line_scene is not None else None
        followed_for = now - self.line_acquired_at if self.line_acquired_at is not None else 0.0

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
        transition_search_seconds = float(step.argument("transition_search_seconds", 1.5))
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
        minimum_range: float | RangeField,
        now: float,
        line_scene: LineScene | None = None,
    ) -> Command:
        """Advance one closed-loop control step.

        ``minimum_range`` is either the nearest return anywhere, as callers
        written before sectors existed pass it, or a :class:`RangeField` giving
        each direction separately. The first keeps the omnidirectional rule; the
        second unlocks the directional one.
        """
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

        if step.kind == PrimitiveKind.TURN_RELATIVE:
            return self._turn_relative(pose, minimum_range, now)

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
