"""How much room a moving robot needs to stop at the clearance floor.

The adapter refuses to start a motion when the nearest LiDAR return is inside
the clearance floor (0.35 m by default). That check is static: it compares a
distance with a distance. A robot that is already moving cannot stop where a
static check would like it to. Between the scan that shows an obstacle and the
wheels at rest there are two physical terms:

* latency, ``t``: the scan has to be taken and delivered, the stop has to be
  decided and travel back to the robot, its controller has to run, and the
  motors have to react. The robot keeps its speed for all of it, covering
  ``v * t``.
* braking, ``a``: from there the base slows at its achievable deceleration and
  covers ``v**2 / (2 * a)``.

So the clearance a guard has to enforce *while moving* is::

    floor + v * t + v**2 / (2 * a)

measured along the way the robot is going. Stopping when the obstacle reaches
the floor itself guarantees the robot comes to rest inside it, by exactly the
two terms above. The inverse gives the fastest speed that can still stop at the
floor from a given distance, which is what scales the commanded speed down as
an obstacle comes closer.

Everything here is arithmetic on numbers the caller passes in; nothing imports
ROS, so the physics is testable on its own.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import Any

#: End-to-end stop latency assumed when nothing better is known, in seconds.
#: Conservative for a LiDAR at 5-10 Hz read over a network bridge: one scan
#: period, a network hop each way, a 10 Hz behavior loop and motor response.
DEFAULT_LATENCY_S = 0.5

#: Deceleration assumed when nothing better is known, in m/s^2. Far below what
#: a small differential base's velocity limits allow (a TurtleBot3 Burger's
#: velocity smoother declares 2.5 m/s^2), so wheel slip, battery sag and a
#: loaded base still stop inside the prediction.
DEFAULT_DECEL_MPS2 = 0.5

#: Below this a commanded speed is not motion; a straight drive that cannot be
#: given at least this much and still stop at the floor is refused.
MIN_SPEED_MPS = 0.02

#: Bounds any configured or measured value is clamped to, so a typo or a bad
#: clock never turns into "the robot stops instantly".
LATENCY_BOUNDS_S = (0.05, 3.0)
DECEL_BOUNDS_MPS2 = (0.05, 5.0)

#: Transport delays above this are clock skew between the robot and this
#: computer, not latency; they are ignored rather than trusted.
MAX_PLAUSIBLE_TRANSPORT_S = 1.0

#: Scans whose arrival intervals and delays the latency estimate keeps.
LATENCY_WINDOW = 50


def _clamp(value: float, bounds: tuple[float, float]) -> float:
    low, high = bounds
    return min(high, max(low, float(value)))


@dataclass(frozen=True)
class LatencyBudget:
    """What one stop has to wait for, in seconds, term by term."""

    scan_period_s: float
    scan_transport_s: float
    command_transport_s: float
    control_period_s: float
    actuation_s: float

    @property
    def total_s(self) -> float:
        return (
            self.scan_period_s
            + self.scan_transport_s
            + self.command_transport_s
            + self.control_period_s
            + self.actuation_s
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "scan_period_s": round(self.scan_period_s, 4),
            "scan_transport_s": round(self.scan_transport_s, 4),
            "command_transport_s": round(self.command_transport_s, 4),
            "control_period_s": round(self.control_period_s, 4),
            "actuation_s": round(self.actuation_s, 4),
            "total_s": round(self.total_s, 4),
        }


@dataclass
class LatencyEstimator:
    """Scan arrival intervals and delivery delays, measured as scans arrive.

    The interval between two scans is how stale the newest return can be when
    it is read. The delivery delay is the receive time minus the scan's own
    header stamp; it needs the robot's and this computer's clocks to agree, so
    a delay that is negative or implausibly long is dropped as skew and the
    budget falls back on the configured default instead.
    """

    window: int = LATENCY_WINDOW
    _last_arrival: float | None = None
    _intervals: deque = field(default_factory=deque)
    _delays: deque = field(default_factory=deque)

    def observe(self, arrival_monotonic: float, delay_s: float | None = None) -> None:
        if self._last_arrival is not None:
            interval = arrival_monotonic - self._last_arrival
            if 0.0 < interval < 5.0:
                self._push(self._intervals, interval)
        self._last_arrival = arrival_monotonic
        if delay_s is not None and 0.0 <= delay_s <= MAX_PLAUSIBLE_TRANSPORT_S:
            self._push(self._delays, delay_s)

    def _push(self, store: deque, value: float) -> None:
        store.append(float(value))
        while len(store) > self.window:
            store.popleft()

    def reset(self) -> None:
        self._last_arrival = None
        self._intervals.clear()
        self._delays.clear()

    @property
    def scan_period_s(self) -> float | None:
        # The worst recent interval, not the mean: the guard has to hold for
        # the slow scan, not the typical one.
        return max(self._intervals) if len(self._intervals) >= 3 else None

    @property
    def scan_transport_s(self) -> float | None:
        return max(self._delays) if len(self._delays) >= 3 else None

    def budget(self, *, control_period_s: float, actuation_s: float) -> LatencyBudget | None:
        """The measured budget, or None until enough scans have arrived.

        A cancel travels the same link back as a scan travelled in, so the
        command leg is taken equal to the measured scan delivery.
        """
        period = self.scan_period_s
        if period is None:
            return None
        transport = self.scan_transport_s or 0.0
        return LatencyBudget(
            scan_period_s=period,
            scan_transport_s=transport,
            command_transport_s=transport,
            control_period_s=max(0.0, control_period_s),
            actuation_s=max(0.0, actuation_s),
        )


@dataclass(frozen=True)
class BrakingProfile:
    """The floor and the two physical terms a moving stop has to add to it."""

    floor_m: float
    latency_s: float
    decel_mps2: float
    sources: Mapping[str, Any] = field(default_factory=dict)

    def stopping_distance(self, speed_mps: float) -> float:
        """How far the robot travels from the scan that triggers a stop to rest."""
        v = abs(float(speed_mps))
        return v * self.latency_s + v * v / (2.0 * self.decel_mps2)

    def required_clearance(self, speed_mps: float) -> float:
        """The clearance at which a robot moving at ``speed_mps`` must stop."""
        return self.floor_m + self.stopping_distance(speed_mps)

    def max_speed(self, room_m: float) -> float:
        """The fastest speed that still comes to rest within ``room_m``.

        ``room_m`` is the distance the robot can still travel before the
        floor is reached. Solves ``v*t + v**2/(2a) = room`` for ``v``.
        """
        if not math.isfinite(room_m):
            return math.inf
        if room_m <= 0.0:
            return 0.0
        t = self.latency_s
        a = self.decel_mps2
        return a * (-t + math.sqrt(t * t + 2.0 * room_m / a))

    def to_dict(self) -> dict[str, Any]:
        return {
            "floor_m": round(self.floor_m, 4),
            "latency_s": round(self.latency_s, 4),
            "decel_mps2": round(self.decel_mps2, 4),
            "sources": dict(self.sources),
        }


def resolve_profile(
    *,
    floor_m: float,
    configured_latency_s: float = DEFAULT_LATENCY_S,
    configured_decel_mps2: float = DEFAULT_DECEL_MPS2,
    measured_latency: LatencyBudget | None = None,
    declared_decel_mps2: float | None = None,
) -> BrakingProfile:
    """Combine what is configured, measured and declared, always conservatively.

    Latency is the *larger* of the configured value and the measured budget: a
    measurement can show the link is slower than assumed, never let the guard
    assume it is faster than configured. Deceleration is the *smaller* of the
    configured value and what the robot declares: a velocity limit is the most
    the base will be asked to do, not a promise it can.
    """
    latency = _clamp(configured_latency_s, LATENCY_BOUNDS_S)
    sources: dict[str, Any] = {"latency": "configured", "decel": "configured"}
    if measured_latency is not None:
        sources["measured_latency"] = measured_latency.to_dict()
        if measured_latency.total_s > latency:
            latency = _clamp(measured_latency.total_s, LATENCY_BOUNDS_S)
            sources["latency"] = "measured"
    decel = _clamp(configured_decel_mps2, DECEL_BOUNDS_MPS2)
    if declared_decel_mps2 is not None and math.isfinite(declared_decel_mps2):
        declared = abs(float(declared_decel_mps2))
        sources["declared_decel_mps2"] = round(declared, 4)
        if 0.0 < declared < decel:
            decel = _clamp(declared, DECEL_BOUNDS_MPS2)
            sources["decel"] = "declared"
    return BrakingProfile(
        floor_m=float(floor_m), latency_s=latency, decel_mps2=decel, sources=sources
    )


@dataclass(frozen=True)
class Room:
    """How far a straight drive can go before a return is at the floor."""

    room_m: float
    #: The return that limits it: its range and bearing (radians from the
    #: direction of travel, counter-clockwise). None when nothing limits it.
    range_m: float | None
    bearing_rad: float | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "room_m": round(self.room_m, 4) if math.isfinite(self.room_m) else None,
            "limiting_range_m": round(self.range_m, 4) if self.range_m is not None else None,
            "limiting_bearing_rad": (
                round(self.bearing_rad, 4) if self.bearing_rad is not None else None
            ),
        }


def _beams(sweep: Mapping[str, Any] | None):
    if not isinstance(sweep, Mapping):
        return None
    try:
        start = float(sweep["angle_min_rad"])
        step = float(sweep["angle_increment_rad"])
        ranges = list(sweep["ranges_m"])
    except (KeyError, TypeError, ValueError):
        return None
    beams = []
    for index, value in enumerate(ranges):
        if value is None:
            continue
        try:
            beam = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(beam) and beam > 0.0:
            beams.append((start + step * index, beam))
    return beams


def room_to_floor(
    sweep: Mapping[str, Any] | None, direction_rad: float, floor_m: float
) -> Room | None:
    """Distance a straight drive in ``direction_rad`` has before the floor.

    A return at ``(x, y)`` in the frame of travel (``x`` along it) comes
    within the floor once the robot has gone ``x - sqrt(floor**2 - y**2)``;
    a return wider than the floor to either side, or behind, never does. The
    room is the smallest of those, so an obstacle straight ahead at distance
    ``d`` leaves ``d - floor``, and a wall alongside that the robot would
    graze closer than the floor limits the drive too.

    None when the sweep cannot be read: not knowing is not room. A readable
    sweep with nothing in the way has infinite room.
    """
    beams = _beams(sweep)
    if beams is None or not beams:
        return None
    floor = float(floor_m)
    best = Room(math.inf, None, None)
    for angle, beam in beams:
        bearing = math.remainder(angle - direction_rad, math.tau)
        x = beam * math.cos(bearing)
        y = beam * math.sin(bearing)
        if x <= 0.0 or abs(y) >= floor:
            continue
        room = x - math.sqrt(floor * floor - y * y)
        if room < best.room_m:
            best = Room(room, beam, bearing)
    return best


def nearest_return(
    sweep: Mapping[str, Any] | None, reference_rad: float = 0.0
) -> tuple[float, float] | None:
    """The nearest return in the sweep and its bearing from ``reference_rad``."""
    beams = _beams(sweep)
    if not beams:
        return None
    angle, beam = min(beams, key=lambda item: item[1])
    return beam, math.remainder(angle - reference_rad, math.tau)


def direction_label(bearing_rad: float) -> str:
    """Ahead, left, right or behind, for a bearing from the direction of travel."""
    degrees = math.degrees(math.remainder(bearing_rad, math.tau))
    if abs(degrees) <= 45.0:
        return "ahead"
    if abs(degrees) >= 135.0:
        return "behind"
    return "left" if degrees > 0 else "right"


# -- how a running drive's speed can be governed ----------------------------------
#
# A guard that sees the room shrink can slow a drive only if the server running
# it accepts a new goal in place of the running one (preemption) and carries on
# from the robot's current speed. Nav2's straight-drive behaviors do not:
# ``TimedBehavior::execute`` answers a preemption with "Received a preemption
# request for drive_on_heading, however feature is currently not implemented.
# Aborting and stopping", stops the robot and starts the new goal from rest
# (read from the installed nav2_behaviors 1.3.13 header and from
# navigation2 main, 2026-10-06; logged by the twin 4 times on 2026-10-06). A
# "slowdown" sent there is a stop and a restart, and the restarted drive is
# tripped by the guard within a scan: the advance ends early.
#
# So the speed is governed one of two ways, chosen per drive by
# ``decide_governance`` from facts about the server running it:
#
# * ``GOVERNANCE_RESEND``: the server takes a preempting goal; the guard
#   re-sends the rest of the drive at the lower speed its room allows.
# * ``GOVERNANCE_PLANNED``: no mid-drive goal is ever sent. The speed and the
#   distance are both fixed before sending from the room measured then, so
#   the drive ends by itself where it can still come to rest at or beyond the
#   floor (``plan_drive``). The guard stays a hard safety stop only.

GOVERNANCE_RESEND = "preemptive_resend"
GOVERNANCE_PLANNED = "planned_before_send"

#: Whether the code behind a straight drive replaces a running goal with a new
#: one without stopping. Keyed by what the running server says implements the
#: drive: the behavior plugin class it loaded (Nav2's ``<behavior>.plugin``
#: parameter, the most specific fact), else the action's interface type. An
#: implementation not listed is treated as not preemptible, since planning
#: before sending is safe on any server.
PREEMPTION_BY_IMPLEMENTATION: Mapping[str, bool] = {
    # nav2_behaviors TimedBehavior plugins (see above).
    "nav2_behaviors::DriveOnHeading": False,
    "nav2_behaviors::BackUp": False,
    # The action contracts those plugins serve, when the plugin is unreadable.
    "nav2_msgs/action/DriveOnHeading": False,
    "nav2_msgs/action/BackUp": False,
}

#: Why a drive got the mode it did.
GOVERNANCE_SOURCE_PLUGIN = "server_plugin"
GOVERNANCE_SOURCE_TABLE = "action_type"
GOVERNANCE_SOURCE_UNKNOWN_TYPE = "implementation_not_declared"
GOVERNANCE_SOURCE_TRANSPORT = "transport_cannot_resend"


@dataclass(frozen=True)
class SpeedGovernance:
    """How one straight drive's speed is governed, and the fact that decided it."""

    mode: str
    action_type: str | None
    preemption_supported: bool | None
    source: str
    server_plugin: str | None = None

    @property
    def resends(self) -> bool:
        return self.mode == GOVERNANCE_RESEND

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "action_type": self.action_type,
            "preemption_supported": self.preemption_supported,
            "source": self.source,
            "server_plugin": self.server_plugin,
        }


def decide_governance(
    action_type: str | None,
    *,
    transport_resends: bool,
    server_plugin: str | None = None,
    table: Mapping[str, bool] | None = None,
) -> SpeedGovernance:
    """The one place a straight drive's speed governance is decided.

    Both inputs are read from the running server: ``server_plugin`` is the
    behavior plugin class it loaded for the drive (None when it does not say)
    and ``action_type`` the interface it serves the action with. Nav2 has no
    parameter or service that says whether a behavior takes a preempting goal,
    and its answer to one cannot be told from a supported preemption by status
    alone (both abort the old goal), so what implements the drive decides,
    through ``PREEMPTION_BY_IMPLEMENTATION``. A transport that cannot send a
    second goal plans before sending. An implementation nobody declared is
    planned, the mode that is safe on every server.
    """
    known = PREEMPTION_BY_IMPLEMENTATION if table is None else table
    declared: bool | None = None
    source = GOVERNANCE_SOURCE_UNKNOWN_TYPE
    if server_plugin and server_plugin in known:
        declared, source = known[server_plugin], GOVERNANCE_SOURCE_PLUGIN
    elif action_type and action_type in known:
        declared, source = known[action_type], GOVERNANCE_SOURCE_TABLE
    if not transport_resends:
        source = GOVERNANCE_SOURCE_TRANSPORT
    mode = GOVERNANCE_RESEND if transport_resends and declared else GOVERNANCE_PLANNED
    return SpeedGovernance(mode, action_type, declared, source, server_plugin)


#: Shortest straight drive worth sending: below this the base's odometry
#: cannot tell it from standing still.
MIN_DISTANCE_M = 0.01


@dataclass(frozen=True)
class DrivePlan:
    """The speed and distance a straight drive is sent with, and why."""

    speed_mps: float
    distance_m: float
    requested_speed_mps: float
    requested_distance_m: float
    room_m: float
    mode: str
    #: The drive was given less distance than asked so it ends before the floor.
    shortened: bool
    #: Why it cannot be sent at all; None when it can.
    refusal: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "requested_speed_mps": round(self.requested_speed_mps, 4),
            "requested_distance_m": round(self.requested_distance_m, 4),
            "commanded_speed_mps": round(self.speed_mps, 4),
            "commanded_distance_m": round(self.distance_m, 4),
            "room_to_floor_at_send_m": (
                round(self.room_m, 4) if math.isfinite(self.room_m) else None
            ),
            "shortened": self.shortened,
        }


def plan_drive(
    profile: BrakingProfile,
    *,
    room_m: float,
    requested_speed_mps: float,
    requested_distance_m: float,
    mode: str,
    speed_fraction: float,
) -> DrivePlan:
    """The speed and distance to send a straight drive with, from ``room_m``.

    The speed is the requested one, capped at ``speed_fraction`` of the fastest
    speed that can still stop within the room (``v = a(-t + sqrt(t^2 +
    2 room / a))``), so a guard watching the drive is not tripped by the next
    scan of an approach it can stop from.

    Under ``GOVERNANCE_PLANNED`` the distance is also capped, at ``room -
    stopping_distance(v)``: a drive that ends there by itself and then needs
    the whole stopping distance (latency at ``v``, then braking) still comes
    to rest at the floor, never inside it. Under ``GOVERNANCE_RESEND`` the
    distance is sent as asked and the guard lowers the speed on the way.
    """
    requested_speed = abs(float(requested_speed_mps))
    requested_distance = abs(float(requested_distance_m))
    speed = min(requested_speed, profile.max_speed(room_m) * speed_fraction)
    distance = requested_distance
    if mode == GOVERNANCE_PLANNED and math.isfinite(room_m):
        distance = min(requested_distance, room_m - profile.stopping_distance(speed))
    plan = DrivePlan(
        speed_mps=speed,
        distance_m=max(0.0, distance),
        requested_speed_mps=requested_speed,
        requested_distance_m=requested_distance,
        room_m=room_m,
        mode=mode,
        shortened=distance < requested_distance,
    )
    if speed < MIN_SPEED_MPS:
        return replace(plan, refusal=f"it could not stop there from even {MIN_SPEED_MPS:.2f} m/s")
    if requested_distance >= MIN_DISTANCE_M and distance < MIN_DISTANCE_M:
        return replace(
            plan,
            refusal=f"at {speed:.3f} m/s it needs {profile.stopping_distance(speed):.3f} m "
            f"to stop, which leaves less than {MIN_DISTANCE_M:.2f} m to drive",
        )
    return plan

