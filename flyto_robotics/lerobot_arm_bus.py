"""An :class:`~flyto_robotics.arm_bus.ArmBus` over a LeRobot-style motors bus.

UNVERIFIED UNTIL HARDWARE ACCEPTANCE. Nothing in this file has run against a
real arm or a real LeRobot install. It records what the bus side is expected
to look like so arrival day starts from a short list of things to confirm,
not from a blank file. The adapter's decisions never depend on it.

Assumed of the injected ``motors_bus`` (a LeRobot ``MotorsBus`` subclass such
as its serial-servo bus, already constructed, calibrated and connected by the
caller; this module does not import LeRobot and never opens a port):

* ``sync_read("Present_Position") -> {joint: value}``
* ``sync_write("Goal_Position", {joint: value})``
* ``read("Torque_Enable", joint) -> 0 | 1``
* ``enable_torque()`` / ``disable_torque()``

and that every joint named in ``degree_joints`` reports and accepts degrees
(LeRobot's degrees normalisation). A joint calibrated to another scale (for
example a 0..100 gripper range) is not converted by a guess: the bus refuses
to start unless every joint is declared in degrees. Each of these is an item
on the arrival-day checklist in ``docs/ARM_M0.md``.

Any exception from ``motors_bus`` is re-raised as :class:`BusError`, which the
adapter treats as "state unknown" and fails closed on.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from typing import Any

from .arm_bus import BusError

PRESENT_POSITION = "Present_Position"  # unverified register name
GOAL_POSITION = "Goal_Position"  # unverified register name
TORQUE_ENABLE = "Torque_Enable"  # unverified register name


class LeRobotArmBus:
    """Radians in, degrees out, every failure a :class:`BusError`."""

    def __init__(self, motors_bus: Any, *, joints: Iterable[str], degree_joints: Iterable[str]):
        self._bus = motors_bus
        self._joints = tuple(joints)
        undeclared = sorted(set(self._joints) - set(degree_joints))
        if undeclared:
            raise ValueError(
                f"joints without a declared degrees scale: {undeclared}; "
                "no unit conversion is guessed"
            )

    def _call(self, what: str, fn, *args):
        try:
            return fn(*args)
        except Exception as error:  # any bus failure means the state is unknown
            raise BusError(f"{what} failed: {type(error).__name__}: {error}") from error

    def read_positions(self) -> Mapping[str, float]:
        raw = self._call("position read", self._bus.sync_read, PRESENT_POSITION)
        positions: dict[str, float] = {}
        for name in self._joints:
            value = raw.get(name) if isinstance(raw, Mapping) else None
            # A value that is not a number is left out, and the adapter fails
            # closed on the missing joint rather than this bus guessing.
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                positions[name] = math.radians(float(value))
        return positions

    def read_torque_enabled(self) -> Mapping[str, bool]:
        result: dict[str, bool] = {}
        for name in self._joints:
            value = self._call("torque read", self._bus.read, TORQUE_ENABLE, name)
            # Only 0 and 1 are a torque state; anything else is left out, and
            # the adapter fails closed on the missing joint.
            if value in (0, 1) and not isinstance(value, bool):
                result[name] = value == 1
        return result

    def write_goal_positions(self, goals: Mapping[str, float]) -> None:
        degrees = {name: math.degrees(float(value)) for name, value in goals.items()}
        self._call("goal write", self._bus.sync_write, GOAL_POSITION, degrees)

    def set_torque(self, enabled: bool) -> None:
        if enabled:
            self._call("torque enable", self._bus.enable_torque)
        else:
            self._call("torque disable", self._bus.disable_torque)
