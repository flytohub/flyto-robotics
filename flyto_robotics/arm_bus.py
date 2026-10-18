"""The bus an arm adapter talks through, and a simulated one.

:class:`ArmBus` is the whole surface :mod:`flyto_robotics.arm_adapter` uses:
read joint positions, read whether each joint's torque is enabled, write goal
positions, enable or disable torque. Positions cross it in radians keyed by the
joint names the arm profile declares. How a real bus turns that into servo
registers, ticks or calibration offsets is the bus implementation's business
and never the adapter's; the adapter decides only from the profile's declared
limits and from what it reads back.

Every method raises :class:`BusError` when the bus cannot answer (a closed
port, a timeout, a garbled reply). The adapter treats that as "the arm's state
is unknown" and fails closed; it never retries a write on its own.

:class:`SimulatedArmBus` is the bus the tests and dry runs use. It moves each
joint toward its goal by a bounded step per read and can be told to misbehave
the ways a real serial arm does: a joint that stalls, a connection that drops
after a number of writes, torque that does not turn off when asked, and a held
joint that sags. A gripper is one of its joints; :meth:`SimulatedArmBus.place_object`
puts a simulated object between its jaws, which blocks it from closing past
the object and can be told to slip. Nothing in it describes a particular
servo, gripper or vendor.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Protocol, runtime_checkable


class BusError(RuntimeError):
    """The bus did not answer; the arm's state is unknown."""


@runtime_checkable
class ArmBus(Protocol):
    """What an arm adapter needs from a bus, in radians by joint name."""

    def read_positions(self) -> Mapping[str, float]:
        """The measured position of every joint the bus can see."""

    def read_torque_enabled(self) -> Mapping[str, bool]:
        """Whether each joint is actively driven (holding or moving)."""

    def write_goal_positions(self, goals: Mapping[str, float]) -> None:
        """Set the goal position of the named joints."""

    def set_torque(self, enabled: bool) -> None:
        """Enable or disable torque on every joint."""


@dataclass
class SimulatedObject:
    """Something between a gripper's jaws.

    The jaws cannot close past ``at_rad``. ``closing`` is the sign of the
    closing direction (-1 closes toward lower positions). Once the jaws touch
    it, after ``slip_after_reads`` position reads it gives way by
    ``slip_rad_per_read`` per read toward closed: an object slipping out.
    Opening the jaws away from it after contact lets go of it.
    """

    joint: str
    at_rad: float
    closing: int = -1
    slip_rad_per_read: float = 0.0
    slip_after_reads: int = 0
    reads_in_contact: int = 0
    touched: bool = False


class SimulatedArmBus:
    """An arm in memory, with the faults M0 must survive.

    ``servo_step_rad`` is how far a driven joint moves toward its goal per
    position read; an undriven joint stays where it is unless ``sag_rad`` is
    set, in which case every joint sags by that much per read whatever its
    torque and goal (a hold that does not hold).
    """

    def __init__(
        self,
        joints: Iterable[str],
        *,
        positions: Mapping[str, float] | None = None,
        torque_enabled: bool = False,
        servo_step_rad: float = 0.5,
    ):
        names = tuple(joints)
        self.positions: dict[str, float] = {name: 0.0 for name in names}
        if positions:
            self.positions.update({k: float(v) for k, v in positions.items()})
        self.goals: dict[str, float] = dict(self.positions)
        self.torque: dict[str, bool] = {name: bool(torque_enabled) for name in names}
        self.servo_step_rad = float(servo_step_rad)
        # Fault knobs.
        self.stalled: set[str] = set()
        self.drop_after_writes: int | None = None
        self.torque_off_ignored = False
        self.sag_rad = 0.0
        self.missing_joints: set[str] = set()
        self.connected = True
        # Objects between a gripper's jaws, and those it has let go of.
        self.objects: list[SimulatedObject] = []
        self.released: list[SimulatedObject] = []
        # What was asked of the bus, for the tests.
        self.writes: list[dict[str, float]] = []
        self.torque_commands: list[bool] = []

    def place_object(
        self,
        joint: str,
        at_rad: float,
        *,
        closing: int = -1,
        slip_rad_per_read: float = 0.0,
        slip_after_reads: int = 0,
    ) -> SimulatedObject:
        """Put an object between ``joint``'s jaws that stops it at ``at_rad``."""
        item = SimulatedObject(
            joint, float(at_rad), -1 if closing < 0 else 1,
            float(slip_rad_per_read), int(slip_after_reads),
        )
        self.objects.append(item)
        return item

    def _apply_objects(self) -> None:
        for item in list(self.objects):
            position = self.positions[item.joint]
            # How far the jaws are past the object in the closing direction.
            past = (position - item.at_rad) * item.closing
            if past >= 0:
                if item.touched and item.slip_rad_per_read:
                    item.reads_in_contact += 1
                    if item.reads_in_contact > item.slip_after_reads:
                        item.at_rad += item.slip_rad_per_read * item.closing
                item.touched = True
                self.positions[item.joint] = item.at_rad
            elif item.touched and past < -1e-9:
                # The jaws opened away from it: it is let go of.
                self.objects.remove(item)
                self.released.append(item)

    def _check(self) -> None:
        if not self.connected:
            raise BusError("simulated bus: connection lost")

    def read_positions(self) -> Mapping[str, float]:
        self._check()
        for name, goal in self.goals.items():
            if self.sag_rad:
                # The load overpowers the joint: it sags whatever its goal.
                self.positions[name] -= self.sag_rad
            elif self.torque.get(name) and name not in self.stalled:
                gap = goal - self.positions[name]
                step = math.copysign(min(abs(gap), self.servo_step_rad), gap)
                self.positions[name] += step
        self._apply_objects()
        return {k: v for k, v in self.positions.items() if k not in self.missing_joints}

    def read_torque_enabled(self) -> Mapping[str, bool]:
        self._check()
        return dict(self.torque)

    def write_goal_positions(self, goals: Mapping[str, float]) -> None:
        self._check()
        if self.drop_after_writes is not None and len(self.writes) >= self.drop_after_writes:
            self.connected = False
            raise BusError("simulated bus: connection lost during write")
        self.writes.append(dict(goals))
        for name, value in goals.items():
            if name in self.goals:
                self.goals[name] = float(value)

    def set_torque(self, enabled: bool) -> None:
        self._check()
        self.torque_commands.append(bool(enabled))
        if not enabled and self.torque_off_ignored:
            return
        for name in self.torque:
            self.torque[name] = bool(enabled)
