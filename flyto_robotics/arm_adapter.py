"""A joint-space arm adapter over any :class:`~flyto_robotics.arm_bus.ArmBus`.

Milestone M0 for a serial-bus arm: read its state, refuse any target outside
the declared joint limits, and stop it with a stop that is *measured*. M1 adds
a gripper: grasp and release, each confirmed by read-back. Every capability
has a ``flyto.capability-contract.v1`` row in
:mod:`flyto_robotics.arm_contracts`:

``manipulation.state``
    read every declared joint's position and torque state. Nothing is written.
``manipulation.move_joints``
    move every joint to an absolute target, bounded by the profile's limits
    and speed. A target outside a limit is refused before the bus is touched;
    it is never clamped.
``motion.halt``
    the safe stop. In ``hold`` mode the adapter writes the measured pose as the
    goal, reads back that torque is on and that no joint drifts beyond the
    declared tolerance over the confirmation window. In ``torque_off`` mode it
    disables torque and reads back that every joint reports torque off. The
    stop is ``completed`` only when that read-back confirms it, and ``failed``
    otherwise.
``manipulation.grasp``
    close the profile's gripper joint toward its closed limit. ``completed``
    only when read-back shows the jaws stopped short of the closed limit by at
    least ``object_min_gap_rad`` and held there within ``hold.max_drift_rad``
    over the hold window: closed on something. Jaws that close fully are a
    ``failed`` grasp with ``object_held: false``, never a completed one.
``manipulation.release``
    open the gripper to ``open_to_rad``, which must lie between the profile's
    release position and the gripper's open limit. ``completed`` only when
    read-back shows the jaws there and staying there.

Everything the adapter decides comes from an :class:`ArmProfile`: joint names,
limits, tolerances, speed, stall window and stop mode. Nothing here names a
vendor, a servo family or a register. A profile whose limits have not been
accepted on the physical arm (``limits_accepted: false``) can read state and
stop, but every move is refused.

Fail closed:

* a bus error before a write refuses or fails the call with nothing written;
* a bus error during a move ends it as ``failed`` with
  ``state_known: false``. The adapter does not retry the move, does not
  re-send it under the same call id, and refuses further moves until a state
  read succeeds;
* a stop that cannot be confirmed latches: moves stay refused until a later
  stop is confirmed.

Grasp and release are moves: they follow every rule above.

Results reuse :mod:`flyto_robotics.adapter_contract` (``CallRequest``,
``CallResult``, the outcome constants), so a host invokes this adapter the way
it invokes the Generic ROS 2 and Open-RMF adapters.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
    CallResult,
)
from .arm_bus import ArmBus, BusError

PROFILE_SCHEMA = "flyto.arm-profile.v1"

# The shared capability vocabulary's ids (``adapter_contract.KNOWN_CAPABILITY_IDS``,
# Cloud's ``capabilities.CAPABILITIES``): a Space approves only those. The
# state read and joint moves are the ``manipulation`` family; the stop is the
# one shipped safe stop, whatever machine it stops.
CAPABILITY_STATE = "manipulation.state"
CAPABILITY_MOVE = "manipulation.move_joints"
CAPABILITY_HALT = "motion.halt"
CAPABILITY_GRASP = "manipulation.grasp"
CAPABILITY_RELEASE = "manipulation.release"

STOP_HOLD = "hold"
STOP_TORQUE_OFF = "torque_off"
STOP_MODES = (STOP_HOLD, STOP_TORQUE_OFF)

# The observation the contracts' evidence reads, and the frame it is in.
OBSERVATION = "joint_positions"
FRAME = "joints"
# Every target argument is the joint name plus this suffix.
ARGUMENT_SUFFIX = "_rad"

# The gripper's observations: the jaw position (frame ``joints``, keyed by the
# gripper joint's name) and whether the jaws hold an object (``held``: 1 or 0,
# next to the jaw position it was decided at).
GRIPPER_OBSERVATION = "gripper_position"
HELD_OBSERVATION = "object_held"
HELD_FIELD = "held"
# The release call's one argument.
RELEASE_ARGUMENT = "open_to_rad"
# Which end of the gripper joint's travel is closed.
CLOSES_TOWARD = ("min", "max")

# Why a call ended, machine readable (``evidence.arm_outcome.reason``).
REASON_COMPLETED = "completed"
REASON_STALLED = "stalled"
REASON_TIMEOUT = "timeout"
REASON_CONNECTION_LOST = "connection_lost"
REASON_CANCELLED = "cancelled"
REASON_STOP_CONFIRMED = "stop_confirmed"
REASON_STOP_UNCONFIRMED = "stop_unconfirmed"
REASON_STATE_INCOMPLETE = "state_incomplete"
REASON_OBJECT_HELD = "object_held"
REASON_NO_OBJECT = "no_object"
REASON_SLIPPING = "slipping"
REASON_RELEASED = "released"
REASON_NOT_OPEN = "not_open"

_IDENTIFIER_CHARS = set("abcdefghijklmnopqrstuvwxyz0123456789_")


class ProfileInvalid(ValueError):
    """An arm profile the adapter will not run with."""


@dataclass(frozen=True)
class JointLimit:
    name: str
    min_rad: float
    max_rad: float


@dataclass(frozen=True)
class GripperProfile:
    """A gripper as one of the arm's joints, with its travel read as jaws."""

    joint: str
    closed_rad: float
    open_rad: float
    release_rad: float
    max_speed_rad_s: float
    position_tolerance_rad: float
    object_min_gap_rad: float
    hold_samples: int
    hold_max_drift_rad: float

    @property
    def opening(self) -> float:
        """The sign of the opening direction."""
        return 1.0 if self.open_rad > self.closed_rad else -1.0

    def gap(self, position: float) -> float:
        """How far ``position`` is open from the closed limit."""
        return (position - self.closed_rad) * self.opening

    @property
    def release_range(self) -> tuple[float, float]:
        """``(min, max)`` of a release target: release position to open limit."""
        return tuple(sorted((self.release_rad, self.open_rad)))  # type: ignore[return-value]


@dataclass(frozen=True)
class ArmProfile:
    """What an arm is, as data. Every decision the adapter makes reads it."""

    resource_id: str
    joints: tuple[JointLimit, ...]
    limits_accepted: bool
    position_tolerance_rad: float
    max_joint_speed_rad_s: float
    control_period_s: float
    move_deadline_s: float
    stall_window_s: float
    stall_min_progress_rad: float
    settle_samples: int
    settle_max_drift_rad: float
    stop_mode: str
    stop_confirm_samples: int
    stop_confirm_period_s: float
    stop_max_drift_rad: float
    gripper: GripperProfile | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    def limit(self, name: str) -> JointLimit:
        return next(joint for joint in self.joints if joint.name == name)


def _finite(value: Any, where: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileInvalid(f"{where} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ProfileInvalid(f"{where} must be finite")
    if positive and number <= 0:
        raise ProfileInvalid(f"{where} must be greater than zero")
    return number


def _count(value: Any, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ProfileInvalid(f"{where} must be a positive integer")
    return value


# The profile's key sets are closed: a misspelt key is an error, never ignored.
PROFILE_KEYS = frozenset({
    "schema", "resource_id", "note", "limits_accepted", "position_unit", "joints",
    "position_tolerance_rad", "max_joint_speed_rad_s", "control_period_s",
    "move_deadline_s", "stall", "settle", "stop", "gripper",
})
_NESTED_KEYS = {
    "stall": frozenset({"window_s", "min_progress_rad"}),
    "settle": frozenset({"samples", "max_drift_rad"}),
    "stop": frozenset({"mode", "confirm_samples", "confirm_period_s", "max_drift_rad"}),
}
_JOINT_KEYS = frozenset({"name", "min_rad", "max_rad"})
_GRIPPER_KEYS = frozenset({
    "joint", "closes_toward", "release_rad", "max_speed_rad_s",
    "position_tolerance_rad", "object_min_gap_rad", "hold",
})
_GRIPPER_HOLD_KEYS = frozenset({"samples", "max_drift_rad"})


def _closed(value: Any, keys: frozenset[str], where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileInvalid(f"{where} must be an object")
    unknown = sorted(set(value) - keys)
    if unknown:
        raise ProfileInvalid(f"{where} has unknown keys: {unknown}")
    return value


def _parse_gripper(raw: Any, joints: Mapping[str, JointLimit]) -> GripperProfile:
    """The gripper section, every key required; its travel is a declared joint's limits."""
    _closed(raw, _GRIPPER_KEYS, "gripper")
    joint = raw.get("joint")
    if joint not in joints:
        raise ProfileInvalid("gripper.joint must name one of the declared joints")
    limit = joints[joint]
    toward = raw.get("closes_toward")
    if toward not in CLOSES_TOWARD:
        raise ProfileInvalid(f"gripper.closes_toward must be one of {CLOSES_TOWARD}")
    closed, opened = (
        (limit.min_rad, limit.max_rad) if toward == "min" else (limit.max_rad, limit.min_rad)
    )
    release = _finite(raw.get("release_rad"), "gripper.release_rad")
    tolerance = _finite(
        raw.get("position_tolerance_rad"), "gripper.position_tolerance_rad", positive=True
    )
    min_gap = _finite(raw.get("object_min_gap_rad"), "gripper.object_min_gap_rad", positive=True)
    hold = _closed(raw.get("hold"), _GRIPPER_HOLD_KEYS, "gripper.hold")
    gripper = GripperProfile(
        joint=joint,
        closed_rad=closed,
        open_rad=opened,
        release_rad=release,
        max_speed_rad_s=_finite(
            raw.get("max_speed_rad_s"), "gripper.max_speed_rad_s", positive=True
        ),
        position_tolerance_rad=tolerance,
        object_min_gap_rad=min_gap,
        hold_samples=_count(hold.get("samples"), "gripper.hold.samples"),
        hold_max_drift_rad=_finite(
            hold.get("max_drift_rad"), "gripper.hold.max_drift_rad", positive=True
        ),
    )
    if not limit.min_rad <= release <= limit.max_rad:
        # Never clamped into range: a release position off the joint is an error.
        raise ProfileInvalid("gripper.release_rad must lie within the gripper joint's limits")
    if not min_gap > tolerance:
        # Otherwise jaws that closed fully, within tolerance, could read as holding.
        raise ProfileInvalid(
            "gripper.object_min_gap_rad must be greater than gripper.position_tolerance_rad"
        )
    if not gripper.gap(release) > min_gap + tolerance:
        # A released gripper must never read as closed on an object.
        raise ProfileInvalid(
            "gripper.release_rad must be open from the closed limit by more than "
            "object_min_gap_rad plus position_tolerance_rad"
        )
    return gripper


def parse_profile(raw: Mapping[str, Any]) -> ArmProfile:
    """A validated profile, or :class:`ProfileInvalid`. Nothing is defaulted."""
    if not isinstance(raw, Mapping) or raw.get("schema") != PROFILE_SCHEMA:
        raise ProfileInvalid(f"schema must be {PROFILE_SCHEMA}")
    _closed(raw, PROFILE_KEYS, "profile")
    if raw.get("position_unit") != "rad":
        raise ProfileInvalid("position_unit must be rad")
    resource_id = raw.get("resource_id")
    if not isinstance(resource_id, str) or not resource_id:
        raise ProfileInvalid("resource_id is required")
    accepted = raw.get("limits_accepted")
    if type(accepted) is not bool:
        raise ProfileInvalid("limits_accepted must be true or false")
    joints_raw = raw.get("joints")
    if not isinstance(joints_raw, list) or not 1 <= len(joints_raw) <= 8:
        # Eight: a contract carries at most eight evidence specs, one per joint.
        raise ProfileInvalid("joints must list 1..8 joints")
    joints: list[JointLimit] = []
    for index, item in enumerate(joints_raw):
        where = f"joints[{index}]"
        _closed(item, _JOINT_KEYS, where)
        name = item.get("name")
        if (
            not isinstance(name, str)
            or not name
            or not name[0].isalpha()
            or not set(name) <= _IDENTIFIER_CHARS
            or "__" in name
            or name.endswith("_")
            or len(name) > 32
        ):
            raise ProfileInvalid(f"{where}.name must be a lower-case identifier")
        low = _finite(item.get("min_rad"), f"{where}.min_rad")
        high = _finite(item.get("max_rad"), f"{where}.max_rad")
        if not low < high:
            raise ProfileInvalid(f"{where}: min_rad must be below max_rad")
        joints.append(JointLimit(name, low, high))
    if len({joint.name for joint in joints}) != len(joints):
        raise ProfileInvalid("joint names must be unique")
    stall, settle, stop = (
        _closed(raw.get(key), keys, key) for key, keys in _NESTED_KEYS.items()
    )
    mode = stop.get("mode")
    if mode not in STOP_MODES:
        raise ProfileInvalid(f"stop.mode must be one of {STOP_MODES}")
    # A gripper is optional as a whole (an arm may have none); if declared,
    # every key of it is required.
    gripper = (
        _parse_gripper(raw["gripper"], {joint.name: joint for joint in joints})
        if "gripper" in raw
        else None
    )
    return ArmProfile(
        resource_id=resource_id,
        joints=tuple(joints),
        limits_accepted=accepted,
        position_tolerance_rad=_finite(
            raw.get("position_tolerance_rad"), "position_tolerance_rad", positive=True
        ),
        max_joint_speed_rad_s=_finite(
            raw.get("max_joint_speed_rad_s"), "max_joint_speed_rad_s", positive=True
        ),
        control_period_s=_finite(raw.get("control_period_s"), "control_period_s", positive=True),
        move_deadline_s=_finite(raw.get("move_deadline_s"), "move_deadline_s", positive=True),
        stall_window_s=_finite(stall.get("window_s"), "stall.window_s", positive=True),
        stall_min_progress_rad=_finite(
            stall.get("min_progress_rad"), "stall.min_progress_rad", positive=True
        ),
        settle_samples=_count(settle.get("samples"), "settle.samples"),
        settle_max_drift_rad=_finite(
            settle.get("max_drift_rad"), "settle.max_drift_rad", positive=True
        ),
        stop_mode=mode,
        stop_confirm_samples=_count(stop.get("confirm_samples"), "stop.confirm_samples"),
        stop_confirm_period_s=_finite(
            stop.get("confirm_period_s"), "stop.confirm_period_s", positive=True
        ),
        stop_max_drift_rad=_finite(stop.get("max_drift_rad"), "stop.max_drift_rad", positive=True),
        gripper=gripper,
    )


def load_profile(path: str | Path) -> ArmProfile:
    return parse_profile(json.loads(Path(path).read_text(encoding="utf-8")))


class _Unreadable(Exception):
    """A read that answered, but not with every declared joint as a number."""


def _held(held: bool | None) -> dict[str, Any]:
    # ``None``: whether anything is held is not known (lost connection, a
    # stop mid-grasp); a host treats it as not held.
    return {HELD_OBSERVATION: held}


def _outcome(reason: str, *, state_known: bool = True, **extra: Any) -> dict[str, Any]:
    # ``retry: never``: an ended move is never re-sent by this adapter; a host
    # that wants another try issues a new call after reading the state.
    return {"reason": reason, "state_known": state_known, "retry": "never", **extra}


class SerialArmAdapter:
    """M0 over one bus: state, bounded joint moves, measured stop."""

    def __init__(
        self,
        bus: ArmBus,
        profile: ArmProfile,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self._bus = bus
        self.profile = profile
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._results: dict[str, CallResult] = {}
        # Latched faults; see the module docstring.
        self._state_unknown = False
        self._stop_unconfirmed = False

    @property
    def resource_id(self) -> str:
        return self.profile.resource_id

    # -- reading -------------------------------------------------------------

    def _read_positions(self) -> dict[str, float]:
        with self._lock:
            raw = self._bus.read_positions()
        positions: dict[str, float] = {}
        for name in self.profile.joint_names:
            value = raw.get(name) if isinstance(raw, Mapping) else None
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise _Unreadable(f"joint {name} reported no position")
            if not math.isfinite(float(value)):
                raise _Unreadable(f"joint {name} reported a non-finite position")
            positions[name] = float(value)
        return positions

    def _read_torque(self) -> dict[str, bool]:
        with self._lock:
            raw = self._bus.read_torque_enabled()
        torque: dict[str, bool] = {}
        for name in self.profile.joint_names:
            value = raw.get(name) if isinstance(raw, Mapping) else None
            if type(value) is not bool:
                raise _Unreadable(f"joint {name} reported no torque state")
            torque[name] = value
        return torque

    @staticmethod
    def _phase(positions: Mapping[str, float]) -> dict[str, Any]:
        return {"frame": FRAME, **positions}

    def read_state(self, call_id: str = "arm-state") -> CallResult:
        try:
            positions = self._read_positions()
            torque = self._read_torque()
        except BusError as error:
            self._state_unknown = True
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={"arm_outcome": _outcome(REASON_CONNECTION_LOST, state_known=False)},
                detail=f"arm state unreadable: {error}",
            )
        except _Unreadable as error:
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={"arm_outcome": _outcome(REASON_STATE_INCOMPLETE, state_known=False)},
                detail=str(error),
            )
        # A complete read makes the arm's state known again.
        self._state_unknown = False
        outside = sorted(
            name
            for name, value in positions.items()
            if not self.profile.limit(name).min_rad <= value <= self.profile.limit(name).max_rad
        )
        return CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "arm_state": {
                    "resource_id": self.resource_id,
                    OBSERVATION: self._phase(positions),
                    "torque_enabled": torque,
                    "outside_limits": outside,
                    "limits_accepted": self.profile.limits_accepted,
                    "stop_unconfirmed": self._stop_unconfirmed,
                },
                "arm_outcome": _outcome(REASON_COMPLETED),
            },
        )

    # -- dispatch ------------------------------------------------------------

    def invoke(self, request: CallRequest) -> CallResult:
        capability_id = str(request.capability_id)
        if capability_id == CAPABILITY_HALT:
            return self.safe_stop(request.call_id)
        kept = self._results.get(request.call_id)
        if kept is not None:
            # The same call again gets its first result, never a second effect.
            return kept
        if capability_id == CAPABILITY_STATE:
            if request.arguments:
                result = CallResult(
                    request.call_id,
                    OUTCOME_REFUSED,
                    detail=f"{CAPABILITY_STATE} takes no arguments",
                )
            else:
                result = self.read_state(request.call_id)
        elif capability_id == CAPABILITY_MOVE:
            result = self._move(request)
        elif capability_id in (CAPABILITY_GRASP, CAPABILITY_RELEASE):
            result = self._grip(request)
        else:
            result = CallResult(
                request.call_id, OUTCOME_REFUSED, detail=f"unsupported capability {capability_id}"
            )
        if result.outcome != OUTCOME_REFUSED:
            self._results[request.call_id] = result
        return result

    def cancel(self, call_id: str) -> CallResult:
        """A cancel is a stop: the arm is brought to rest the measured way."""
        return self.safe_stop(call_id)

    # -- move ----------------------------------------------------------------

    def targets(self, arguments: Mapping[str, Any]) -> dict[str, float]:
        """The call's targets by joint, or ``ValueError``. Never clamped."""
        expected = {name + ARGUMENT_SUFFIX: name for name in self.profile.joint_names}
        unknown = sorted(set(arguments) - set(expected))
        if unknown:
            raise ValueError(f"unsupported arguments: {unknown}")
        targets: dict[str, float] = {}
        for argument, name in expected.items():
            if argument not in arguments:
                raise ValueError(f"{argument} is required")
            raw = arguments[argument]
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                raise ValueError(f"{argument} must be a number")
            value = float(raw)
            if not math.isfinite(value):
                raise ValueError(f"{argument} must be finite")
            limit = self.profile.limit(name)
            if value < limit.min_rad:
                raise ValueError(f"{argument} {value} is below the joint limit {limit.min_rad}")
            if value > limit.max_rad:
                raise ValueError(f"{argument} {value} is above the joint limit {limit.max_rad}")
            targets[name] = value
        return targets

    def _refuse(self, call_id: str, detail: str) -> CallResult:
        return CallResult(call_id, OUTCOME_REFUSED, detail=detail)

    def _motion_refusal(self, call_id: str) -> CallResult | None:
        """Why no motion may start now, or ``None``. Reads and writes nothing."""
        if not self.profile.limits_accepted:
            return self._refuse(
                call_id,
                "joint limits are not accepted on this arm yet (limits_accepted is false); "
                "state and stop only",
            )
        if self._stop_unconfirmed:
            return self._refuse(call_id, "the last stop was not confirmed; stop again first")
        if self._state_unknown:
            return self._refuse(
                call_id, "the arm's state is unknown after a lost connection; read state first"
            )
        return None

    def _run_guarded(
        self, call_id: str, what: str, run: Callable[[], CallResult], **unknown: Any
    ) -> CallResult:
        """Run a motion; a bus fault in it is ``failed`` with the state unknown."""
        self._stop_requested.clear()
        try:
            return run()
        except BusError as error:
            self._state_unknown = True
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={
                    "arm_outcome": _outcome(REASON_CONNECTION_LOST, state_known=False, **unknown)
                },
                detail=(
                    f"connection lost during the {what} ({error}); where the arm stopped is "
                    f"unknown and the {what} will not be retried"
                ),
            )
        except _Unreadable as error:
            self._state_unknown = True
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={
                    "arm_outcome": _outcome(REASON_STATE_INCOMPLETE, state_known=False, **unknown)
                },
                detail=f"{error}; the {what} will not be retried",
            )

    def _move(self, request: CallRequest) -> CallResult:
        call_id = request.call_id
        try:
            targets = self.targets(request.arguments)
        except ValueError as error:
            return self._refuse(call_id, str(error))
        refusal = self._motion_refusal(call_id)
        if refusal is not None:
            return refusal
        return self._run_guarded(call_id, "move", lambda: self._run_move(call_id, targets))

    def _start_pose(self, call_id: str) -> tuple[dict[str, float] | None, CallResult | None]:
        """The measured pose with torque on and holding it, or why not."""
        profile = self.profile
        before = self._read_positions()
        outside = [
            name
            for name, value in before.items()
            if not profile.limit(name).min_rad <= value <= profile.limit(name).max_rad
        ]
        if outside:
            return None, self._refuse(
                call_id, f"the arm is outside its declared limits before moving: {sorted(outside)}"
            )
        return before, None

    def _enable_torque(self, call_id: str, before: Mapping[str, float]) -> CallResult | None:
        torque = self._read_torque()
        if not all(torque.values()):
            # Hold the measured pose first, so enabling torque moves nothing.
            with self._lock:
                self._bus.write_goal_positions(dict(before))
                self._bus.set_torque(True)
            if not all(self._read_torque().values()):
                return CallResult(
                    call_id,
                    OUTCOME_FAILED,
                    evidence={"arm_outcome": _outcome("torque_not_enabled")},
                    detail="torque did not read back as enabled; nothing was moved",
                )
        return None

    def _run_move(self, call_id: str, targets: Mapping[str, float]) -> CallResult:
        profile = self.profile
        before, refused = self._start_pose(call_id)
        if refused is not None:
            return refused
        assert before is not None
        failed = self._enable_torque(call_id, before)
        if failed is not None:
            return failed
        step = profile.max_joint_speed_rad_s * profile.control_period_s
        setpoint = dict(before)
        deadline = self._clock() + profile.move_deadline_s
        reference = dict(before)
        progress_at = {name: self._clock() for name in setpoint}
        positions = before
        while True:
            if self._stop_requested.is_set():
                return CallResult(
                    call_id,
                    OUTCOME_CANCELLED,
                    evidence=self._move_evidence(before, positions, None, REASON_CANCELLED),
                    detail="stopped by a safe stop",
                )
            now = self._clock()
            if now > deadline:
                return self._end_with_stop(
                    call_id, OUTCOME_TIMEOUT, REASON_TIMEOUT, before, positions,
                    f"the move did not finish within {profile.move_deadline_s} s",
                )
            for name, target in targets.items():
                gap = target - setpoint[name]
                setpoint[name] += math.copysign(min(abs(gap), step), gap)
            with self._lock:
                if self._stop_requested.is_set():
                    continue
                self._bus.write_goal_positions(dict(setpoint))
            self._sleep(profile.control_period_s)
            positions = self._read_positions()
            now = self._clock()
            for name, value in positions.items():
                if abs(value - reference[name]) >= profile.stall_min_progress_rad:
                    reference[name] = value
                    progress_at[name] = now
            stalled = sorted(
                name
                for name, value in positions.items()
                if abs(setpoint[name] - value) > profile.position_tolerance_rad
                and now - progress_at[name] >= profile.stall_window_s
            )
            if stalled:
                return self._end_with_stop(
                    call_id, OUTCOME_FAILED, REASON_STALLED, before, positions,
                    f"joints made no progress for {profile.stall_window_s} s: {stalled}",
                    stalled=stalled,
                )
            arrived = setpoint == dict(targets) and all(
                abs(positions[name] - targets[name]) <= profile.position_tolerance_rad
                for name in targets
            )
            if arrived:
                return self._settle(call_id, before, positions)

    def _settle(
        self, call_id: str, before: Mapping[str, float], after: Mapping[str, float]
    ) -> CallResult:
        profile = self.profile
        settled = after
        drift = 0.0
        for _ in range(profile.settle_samples):
            self._sleep(profile.control_period_s)
            settled = self._read_positions()
            drift = max(drift, max(abs(settled[n] - after[n]) for n in after))
        evidence = self._move_evidence(before, after, settled, REASON_COMPLETED)
        evidence["arm_outcome"]["settle_drift_rad"] = drift
        if drift > profile.settle_max_drift_rad:
            evidence["arm_outcome"]["reason"] = "not_settled"
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence=evidence,
                detail=f"joints drifted {drift:.4f} rad after arriving",
            )
        return CallResult(call_id, OUTCOME_COMPLETED, evidence=evidence)

    def _end_with_stop(
        self,
        call_id: str,
        outcome: str,
        reason: str,
        before: Mapping[str, float],
        positions: Mapping[str, float],
        detail: str,
        **extra: Any,
    ) -> CallResult:
        stop = self._stop()
        evidence = self._move_evidence(before, positions, None, reason, **extra)
        evidence["stop"] = stop.evidence.get("stop_confirmation")
        if stop.outcome != OUTCOME_COMPLETED:
            detail = f"{detail}; the stop afterwards was NOT confirmed: {stop.detail}"
        return CallResult(call_id, outcome, evidence=evidence, detail=detail)

    def _move_evidence(
        self,
        before: Mapping[str, float],
        after: Mapping[str, float],
        settled: Mapping[str, float] | None,
        reason: str,
        **extra: Any,
    ) -> dict[str, Any]:
        phases = {"before": self._phase(before), "after": self._phase(after)}
        if settled is not None:
            phases["settled"] = self._phase(settled)
        return {
            "observations": {OBSERVATION: phases},
            "arm_outcome": _outcome(reason, **extra),
        }

    # -- gripper -------------------------------------------------------------

    def release_target(self, arguments: Mapping[str, Any]) -> float:
        """The release call's target, or ``ValueError``. Never clamped."""
        gripper = self.profile.gripper
        assert gripper is not None
        unknown = sorted(set(arguments) - {RELEASE_ARGUMENT})
        if unknown:
            raise ValueError(f"unsupported arguments: {unknown}")
        if RELEASE_ARGUMENT not in arguments:
            raise ValueError(f"{RELEASE_ARGUMENT} is required")
        raw = arguments[RELEASE_ARGUMENT]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{RELEASE_ARGUMENT} must be a number")
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"{RELEASE_ARGUMENT} must be finite")
        low, high = gripper.release_range
        if not low <= value <= high:
            raise ValueError(
                f"{RELEASE_ARGUMENT} {value} is outside the gripper's release range "
                f"[{low}, {high}] (release position to open limit)"
            )
        return value

    def _grip(self, request: CallRequest) -> CallResult:
        call_id = request.call_id
        grasp = request.capability_id == CAPABILITY_GRASP
        gripper = self.profile.gripper
        if gripper is None:
            return self._refuse(call_id, "this arm's profile declares no gripper")
        try:
            if grasp:
                if request.arguments:
                    raise ValueError(
                        f"{CAPABILITY_GRASP} takes no arguments; it closes toward the "
                        "gripper's closed limit"
                    )
                target = gripper.closed_rad
            else:
                target = self.release_target(request.arguments)
        except ValueError as error:
            return self._refuse(call_id, str(error))
        refusal = self._motion_refusal(call_id)
        if refusal is not None:
            return refusal
        what = "grasp" if grasp else "release"
        return self._run_guarded(
            call_id, what, lambda: self._run_grip(call_id, grasp, target), **_held(None)
        )

    def _grip_evidence(
        self,
        reason: str,
        phases: Mapping[str, float],
        held: bool | None,
        held_phases: Mapping[str, Mapping[str, float]] | None = None,
        **extra: Any,
    ) -> dict[str, Any]:
        joint = self.profile.gripper.joint  # type: ignore[union-attr]
        observations: dict[str, Any] = {
            GRIPPER_OBSERVATION: {
                phase: {"frame": FRAME, joint: value} for phase, value in phases.items()
            }
        }
        if held_phases is not None:
            observations[HELD_OBSERVATION] = {
                phase: dict(values) for phase, values in held_phases.items()
            }
        return {
            "observations": observations,
            "arm_outcome": _outcome(reason, **_held(held), **extra),
        }

    def _run_grip(self, call_id: str, grasp: bool, target: float) -> CallResult:
        profile = self.profile
        gripper = profile.gripper
        assert gripper is not None
        joint = gripper.joint
        before, refused = self._start_pose(call_id)
        if refused is not None:
            return refused
        assert before is not None
        start = before[joint]
        if grasp and gripper.gap(start) < gripper.gap(gripper.release_rad) - (
            gripper.position_tolerance_rad
        ):
            return self._refuse(
                call_id,
                f"the gripper is not open (at {start:.4f} rad, release position "
                f"{gripper.release_rad}); release it first",
            )
        failed = self._enable_torque(call_id, before)
        if failed is not None:
            return failed
        step = gripper.max_speed_rad_s * profile.control_period_s
        tolerance = gripper.position_tolerance_rad
        setpoint = start
        deadline = self._clock() + profile.move_deadline_s
        reference = start
        progress_at = self._clock()
        position = start

        def ended(outcome: str, reason: str, detail: str, *, stop: bool, **extra: Any):
            evidence = self._grip_evidence(
                reason, {"before": start, "after": position}, False if grasp else None, **extra
            )
            if stop:
                halt = self._stop()
                evidence["stop"] = halt.evidence.get("stop_confirmation")
                if halt.outcome != OUTCOME_COMPLETED:
                    detail = f"{detail}; the stop afterwards was NOT confirmed: {halt.detail}"
            return CallResult(call_id, outcome, evidence=evidence, detail=detail)

        while True:
            if self._stop_requested.is_set():
                return CallResult(
                    call_id,
                    OUTCOME_CANCELLED,
                    evidence=self._grip_evidence(
                        REASON_CANCELLED, {"before": start, "after": position}, None
                    ),
                    detail="stopped by a safe stop",
                )
            now = self._clock()
            if now > deadline:
                return ended(
                    OUTCOME_TIMEOUT, REASON_TIMEOUT,
                    f"the gripper did not finish within {profile.move_deadline_s} s", stop=True,
                )
            gap = target - setpoint
            setpoint += math.copysign(min(abs(gap), step), gap)
            with self._lock:
                if self._stop_requested.is_set():
                    continue
                self._bus.write_goal_positions({joint: setpoint})
            self._sleep(profile.control_period_s)
            position = self._read_positions()[joint]
            now = self._clock()
            if abs(position - reference) >= profile.stall_min_progress_rad:
                reference, progress_at = position, now
            if setpoint == target and abs(position - target) <= tolerance:
                if grasp:
                    # Closed all the way: there was nothing between the jaws.
                    return ended(
                        OUTCOME_FAILED, REASON_NO_OBJECT,
                        "the gripper closed fully; it is holding nothing", stop=False,
                    )
                return self._confirm_open(call_id, start, position, target)
            stopped = (
                abs(setpoint - position) > tolerance
                and now - progress_at >= profile.stall_window_s
            )
            if not stopped:
                continue
            if not grasp or abs(position - start) < profile.stall_min_progress_rad:
                # Opening is never blocked by an object; and jaws that never
                # moved from open have closed on nothing: both are a stall.
                return ended(
                    OUTCOME_FAILED, REASON_STALLED,
                    f"the gripper made no progress for {profile.stall_window_s} s", stop=True,
                    stalled=[joint],
                )
            if gripper.gap(position) < gripper.object_min_gap_rad:
                return ended(
                    OUTCOME_FAILED, REASON_NO_OBJECT,
                    f"the gripper stopped {gripper.gap(position):.4f} rad from closed, under "
                    f"the {gripper.object_min_gap_rad} rad an object needs", stop=True,
                )
            return self._confirm_hold(call_id, start, position)

    def _confirm_hold(self, call_id: str, start: float, contact: float) -> CallResult:
        """Closed on something: held only if the jaws stay put, still short of closed."""
        gripper = self.profile.gripper
        assert gripper is not None
        joint = gripper.joint
        position = contact
        drift = 0.0
        for _ in range(gripper.hold_samples):
            self._sleep(self.profile.control_period_s)
            position = self._read_positions()[joint]
            drift = max(drift, abs(position - contact))
        held = (
            drift <= gripper.hold_max_drift_rad
            and gripper.gap(position) >= gripper.object_min_gap_rad
        )
        evidence = self._grip_evidence(
            REASON_OBJECT_HELD if held else REASON_SLIPPING,
            {"before": start, "after": contact, "settled": position},
            held,
            # What the hold was decided on: from contact to the end of the
            # hold window, with whether it held.
            {
                "before": {HELD_FIELD: 0, joint: contact},
                "after": {HELD_FIELD: 1 if held else 0, joint: position},
            },
            hold_drift_rad=drift,
        )
        if held:
            return CallResult(call_id, OUTCOME_COMPLETED, evidence=evidence)
        halt = self._stop()
        evidence["stop"] = halt.evidence.get("stop_confirmation")
        detail = (
            f"the object slipped: the jaws moved {drift:.4f} rad while holding, over "
            f"{gripper.hold_max_drift_rad} rad"
        )
        if halt.outcome != OUTCOME_COMPLETED:
            detail = f"{detail}; the stop afterwards was NOT confirmed: {halt.detail}"
        return CallResult(call_id, OUTCOME_FAILED, evidence=evidence, detail=detail)

    def _confirm_open(
        self, call_id: str, start: float, arrived: float, target: float
    ) -> CallResult:
        """Opened to the target: released only if the jaws stay there."""
        gripper = self.profile.gripper
        assert gripper is not None
        position = arrived
        drift = 0.0
        for _ in range(gripper.hold_samples):
            self._sleep(self.profile.control_period_s)
            position = self._read_positions()[gripper.joint]
            drift = max(drift, abs(position - arrived))
        opened = (
            drift <= gripper.hold_max_drift_rad
            and abs(position - target) <= gripper.position_tolerance_rad
        )
        evidence = self._grip_evidence(
            REASON_RELEASED if opened else REASON_NOT_OPEN,
            {"before": start, "after": arrived, "settled": position},
            False if opened else None,
            settle_drift_rad=drift,
        )
        if opened:
            return CallResult(call_id, OUTCOME_COMPLETED, evidence=evidence)
        return CallResult(
            call_id,
            OUTCOME_FAILED,
            evidence=evidence,
            detail=f"the gripper did not stay open at {target} rad (drifted {drift:.4f} rad)",
        )

    # -- stop ----------------------------------------------------------------

    def safe_stop(self, call_id: str = "safe-stop") -> CallResult:
        """Stop now, and say ``completed`` only when the read-back proves it."""
        self._stop_requested.set()
        return self._stop(call_id)

    def _stop(self, call_id: str = "safe-stop") -> CallResult:
        profile = self.profile
        try:
            with self._lock:
                if profile.stop_mode == STOP_TORQUE_OFF:
                    confirmation = self._torque_off()
                else:
                    confirmation = self._hold()
        except (BusError, _Unreadable) as error:
            self._stop_unconfirmed = True
            self._state_unknown = True
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={
                    "stop_confirmation": {"mode": profile.stop_mode, "confirmed": False},
                    "arm_outcome": _outcome(REASON_STOP_UNCONFIRMED, state_known=False),
                },
                detail=f"stop not confirmed: {error}",
            )
        confirmed = confirmation["confirmed"]
        self._stop_unconfirmed = not confirmed
        observations = confirmation.pop("observations", None)
        return CallResult(
            call_id,
            OUTCOME_COMPLETED if confirmed else OUTCOME_FAILED,
            evidence={
                **({"observations": observations} if observations else {}),
                "stop_confirmation": confirmation,
                "arm_outcome": _outcome(
                    REASON_STOP_CONFIRMED if confirmed else REASON_STOP_UNCONFIRMED
                ),
            },
            detail="" if confirmed else f"stop not confirmed: {confirmation['why']}",
        )

    def _torque_off(self) -> dict[str, Any]:
        self._bus.set_torque(False)
        torque = self._read_torque()
        still_on = sorted(name for name, on in torque.items() if on)
        return {
            "mode": STOP_TORQUE_OFF,
            "confirmed": not still_on,
            "torque_enabled": torque,
            **({"why": f"torque still reads enabled on {still_on}"} if still_on else {}),
        }

    def _hold(self) -> dict[str, Any]:
        profile = self.profile
        held = self._read_positions()
        self._bus.write_goal_positions(held)
        self._bus.set_torque(True)
        torque = self._read_torque()
        not_driven = sorted(name for name, on in torque.items() if not on)
        drift = 0.0
        worst = ""
        positions = held
        for _ in range(profile.stop_confirm_samples):
            self._sleep(profile.stop_confirm_period_s)
            positions = self._read_positions()
            for name, value in positions.items():
                if abs(value - held[name]) > drift:
                    drift, worst = abs(value - held[name]), name
        why = []
        if not_driven:
            why.append(f"torque reads disabled on {not_driven}")
        if drift > profile.stop_max_drift_rad:
            why.append(
                f"joint {worst} drifted {drift:.4f} rad, over {profile.stop_max_drift_rad} rad"
            )
        return {
            "mode": STOP_HOLD,
            "confirmed": not why,
            "held_at": self._phase(held),
            # The arm_halt contract's evidence judges these two phases.
            "observations": {
                OBSERVATION: {"before": self._phase(held), "after": self._phase(positions)}
            },
            "torque_enabled": torque,
            "max_drift_rad": drift,
            "samples": profile.stop_confirm_samples,
            **({"why": "; ".join(why)} if why else {}),
        }
