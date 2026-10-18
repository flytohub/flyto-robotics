"""A joint-space arm adapter over any :class:`~flyto_robotics.arm_bus.ArmBus`.

Milestone M0 for a serial-bus arm: read its state, refuse any target outside
the declared joint limits, and stop it with a stop that is *measured*. It has
three capabilities, each with a ``flyto.capability-contract.v1`` row in
:mod:`flyto_robotics.arm_contracts`:

``arm.state``
    read every declared joint's position and torque state. Nothing is written.
``arm.move_joints``
    move every joint to an absolute target, bounded by the profile's limits
    and speed. A target outside a limit is refused before the bus is touched;
    it is never clamped.
``arm.halt``
    the safe stop. In ``hold`` mode the adapter writes the measured pose as the
    goal, reads back that torque is on and that no joint drifts beyond the
    declared tolerance over the confirmation window. In ``torque_off`` mode it
    disables torque and reads back that every joint reports torque off. The
    stop is ``completed`` only when that read-back confirms it, and ``failed``
    otherwise.

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

CAPABILITY_STATE = "arm.state"
CAPABILITY_MOVE = "arm.move_joints"
CAPABILITY_HALT = "arm.halt"

STOP_HOLD = "hold"
STOP_TORQUE_OFF = "torque_off"
STOP_MODES = (STOP_HOLD, STOP_TORQUE_OFF)

# The observation the contracts' evidence reads, and the frame it is in.
OBSERVATION = "joint_positions"
FRAME = "joints"
# Every target argument is the joint name plus this suffix.
ARGUMENT_SUFFIX = "_rad"

# Why a call ended, machine readable (``evidence.arm_outcome.reason``).
REASON_COMPLETED = "completed"
REASON_STALLED = "stalled"
REASON_TIMEOUT = "timeout"
REASON_CONNECTION_LOST = "connection_lost"
REASON_CANCELLED = "cancelled"
REASON_STOP_CONFIRMED = "stop_confirmed"
REASON_STOP_UNCONFIRMED = "stop_unconfirmed"
REASON_STATE_INCOMPLETE = "state_incomplete"

_IDENTIFIER_CHARS = set("abcdefghijklmnopqrstuvwxyz0123456789_")


class ProfileInvalid(ValueError):
    """An arm profile the adapter will not run with."""


@dataclass(frozen=True)
class JointLimit:
    name: str
    min_rad: float
    max_rad: float


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
    "move_deadline_s", "stall", "settle", "stop",
})
_NESTED_KEYS = {
    "stall": frozenset({"window_s", "min_progress_rad"}),
    "settle": frozenset({"samples", "max_drift_rad"}),
    "stop": frozenset({"mode", "confirm_samples", "confirm_period_s", "max_drift_rad"}),
}
_JOINT_KEYS = frozenset({"name", "min_rad", "max_rad"})


def _closed(value: Any, keys: frozenset[str], where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileInvalid(f"{where} must be an object")
    unknown = sorted(set(value) - keys)
    if unknown:
        raise ProfileInvalid(f"{where} has unknown keys: {unknown}")
    return value


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
    )


def load_profile(path: str | Path) -> ArmProfile:
    return parse_profile(json.loads(Path(path).read_text(encoding="utf-8")))


class _Unreadable(Exception):
    """A read that answered, but not with every declared joint as a number."""


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
                    request.call_id, OUTCOME_REFUSED, detail="arm.state takes no arguments"
                )
            else:
                result = self.read_state(request.call_id)
        elif capability_id == CAPABILITY_MOVE:
            result = self._move(request)
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

    def _move(self, request: CallRequest) -> CallResult:
        call_id = request.call_id
        try:
            targets = self.targets(request.arguments)
        except ValueError as error:
            return self._refuse(call_id, str(error))
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
        self._stop_requested.clear()
        try:
            return self._run_move(call_id, targets)
        except BusError as error:
            self._state_unknown = True
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={"arm_outcome": _outcome(REASON_CONNECTION_LOST, state_known=False)},
                detail=(
                    f"connection lost during the move ({error}); where the arm stopped is "
                    "unknown and the move will not be retried"
                ),
            )
        except _Unreadable as error:
            self._state_unknown = True
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                evidence={"arm_outcome": _outcome(REASON_STATE_INCOMPLETE, state_known=False)},
                detail=f"{error}; the move will not be retried",
            )

    def _run_move(self, call_id: str, targets: Mapping[str, float]) -> CallResult:
        profile = self.profile
        before = self._read_positions()
        outside = [
            name
            for name, value in before.items()
            if not profile.limit(name).min_rad <= value <= profile.limit(name).max_rad
        ]
        if outside:
            return self._refuse(
                call_id, f"the arm is outside its declared limits before moving: {sorted(outside)}"
            )
        torque = self._read_torque()
        if not all(torque.values()):
            # Hold the measured pose first, so enabling torque moves nothing.
            with self._lock:
                self._bus.write_goal_positions(before)
                self._bus.set_torque(True)
            if not all(self._read_torque().values()):
                return CallResult(
                    call_id,
                    OUTCOME_FAILED,
                    evidence={"arm_outcome": _outcome("torque_not_enabled")},
                    detail="torque did not read back as enabled; nothing was moved",
                )
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
