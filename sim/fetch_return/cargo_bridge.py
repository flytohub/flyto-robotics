"""SIMULATION cargo bridge: the twin's tray state on ``/flyto/cargo_present``.

The Gazebo TurtleBot3 twin has no cargo sensor. This process holds the
simulated tray in :class:`sim.fetch_return.mobile_cargo.MobileCargoSimulator`
(the same tray the offline mission uses) and publishes it as
``std_msgs/msg/Bool`` so the robot's adapter reads ``sensing.cargo_present``
exactly as it would read a tray sensor. The value is simulation state, not a
measurement, and it can only reach a simulation: the one publisher is the
adapter backend's ``publish_simulated_state``, which refuses unless the
configured deployment is ``simulation`` and the graph carries the simulator
marker (``/clock``), and this process refuses to start otherwise.

The adapter's subscription is volatile (``flyto_robotics/cargo_sensor.py``
states the publish-rate contract), so the value is re-published at
``cargo_sensor.MIN_PUBLISH_HZ`` or faster and never latched.

The tray loads only through ``MobileCargoSimulator.accept_station_handoff``,
and only when every input holds for the same place:

* the arm reported a completed place into the tray proxy, seen inside by its
  camera, released and not held (``arm_reports.placement_observed``);
* its own readings of the tray proxy after that place show the block settled
  inside and not held (``arm_reports.settled_run``);
* the arm reported it retracted to ``ready``;
* the robot's odometry put it within the station tolerance, and still, from
  before the place started (the report's ``started_at``) until the moment the
  handoff is decided, with no gap in the readings. Each pose sample is an
  odometry message, stamped when this host received it: a message read twice
  is one sample, and odometry silence is a gap however often this process
  ticks. The decision waits for a message received after the decision time.

A missing or unreadable input never loads it, nor does a place stamped
later than this process's clock (``MAX_REPORT_SKEW_S``). An ``unload``,
``loss`` or ``mission_end`` report retires every place reported before it,
decided or not. It empties on an ``unload``
report (delivered at home), a ``mission_end`` report, a ``loss`` report, an
arm reading that shows the block out of the tray or held again before the
robot left the station, after ``--max-carry-s`` of carrying, and when this
process stops (after any tick in flight, it publishes ``false`` last).
After departure the arm's tray proxy no longer stands for the robot's tray,
so arm readings stamped after departure are ignored.

The reports are opt-in (``--reports FILE``) and pushed by the arm's and
mission's host (``run.py --reports``); this process never reads, steps or
locks anyone's simulation. Without them the tray stays empty.

Robot odometry and the publish share ONE rosbridge connection: the adapter's
own :class:`RosbridgeROS2Backend` in presence mode (odometry only, never
motion).
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Protocol

if __package__ in ("", None):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from flyto_robotics import cargo_sensor
from flyto_robotics.generic_ros2_adapter import (
    RosbridgeROS2Backend,
    SimulationOnlyError,
    simulation_publish_refusal,
)
from sim.fetch_return.arm_reports import (
    KIND_LOSS,
    KIND_MISSION_END,
    KIND_PLACE,
    KIND_READING,
    KIND_RETRACT,
    KIND_UNLOAD,
    ArmReport,
    ArmReportTail,
    placement_observed,
    reading_in_tray,
    retracted,
    settled_run,
)
from sim.fetch_return.contract import MissionFailure
from sim.fetch_return.mobile_cargo import MobileCargoSimulator

#: The robot counts as parked when every odometry sample through the handoff
#: is within the station tolerance and within this distance of the first one.
PARKED_SPREAD_M = 0.01
#: Two odometry messages received further apart than this leave the robot
#: unaccounted for.
MAX_SAMPLE_GAP_S = 1.5
#: How long after the retract the bridge waits for a settled reading before
#: it decides the handoff with what it has.
DECIDE_AFTER_S = 5.0
#: The longest the tray may stay loaded without an unload or mission end.
MAX_CARRY_S = 600.0
#: How far ahead of this process's clock a report may be stamped (the arm's
#: host and this one share a wall clock). A place stamped later than that, or
#: one that says it started later, is retired: it cannot describe a robot this
#: process saw parked.
MAX_REPORT_SKEW_S = 0.5
HISTORY = 1200
MAX_REPORTS = 2000
MAX_EVENTS = 50
#: Above the consumer's minimum (cargo_sensor.MIN_PUBLISH_HZ); a cap, not a goal.
MAX_RATE_HZ = 10.0


@dataclass(frozen=True)
class Station:
    """Where the robot parks for the handoff, in the odometry frame (metres)."""

    x: float
    y: float
    tolerance_m: float = 0.10

    def holds(self, xy: tuple[float, float]) -> bool:
        return math.hypot(xy[0] - self.x, xy[1] - self.y) <= self.tolerance_m

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "tolerance_m": self.tolerance_m}


def _xy(pose: Any) -> tuple[float, float] | None:
    if not isinstance(pose, dict):
        return None
    try:
        x, y = float(pose["x"]), float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


class PoseHistory:
    """Odometry samples in wall-clock time order; an unknown pose is kept, never skipped."""

    def __init__(self, maxlen: int = HISTORY):
        self._maxlen = int(maxlen)
        self._samples: list[tuple[float, tuple[float, float] | None]] = []

    def add(self, at: float, pose: Any) -> None:
        # A message's receive time can precede a sample recorded earlier (an
        # unknown read at the bridge's own clock), so keep the history ordered.
        times = [t for t, _ in self._samples]
        self._samples.insert(bisect.bisect_right(times, float(at)), (float(at), _xy(pose)))
        del self._samples[: max(0, len(self._samples) - self._maxlen)]

    def parked(self, start: float, end: float, station: Station) -> bool | None:
        """Whether the robot stayed parked at the station from ``start`` to ``end``.

        None while no sample at or after ``end`` exists yet (undecided). False
        when ``start`` is after ``end``, the history does not reach back to
        ``start``, a sample is unknown or off the station, the robot moved, or
        two samples are too far apart: the samples must cover all of it.
        """
        if start > end:
            return False
        samples = list(self._samples)
        before = [i for i, (t, _) in enumerate(samples) if t <= start]
        after = [i for i, (t, _) in enumerate(samples) if t >= end]
        if not after:
            return None
        if not before:
            return False
        window = samples[before[-1] : after[0] + 1]
        origin = window[0][1]
        if origin is None:
            return False
        previous_t = window[0][0]
        for t, xy in window:
            if xy is None or not station.holds(xy) or math.dist(origin, xy) > PARKED_SPREAD_M:
                return False
            if t - previous_t > MAX_SAMPLE_GAP_S:
                return False
            previous_t = t
        return True

    def left_station_after(self, start: float, station: Station) -> float | None:
        """When a known pose after ``start`` was first off the station, else None."""
        for t, xy in self._samples:
            if t > start and xy is not None and not station.holds(xy):
                return t
        return None


def bridge_topic() -> str:
    """The topic the adapter reads on a simulation (``cargo_sensor.simulated_tray_topic``)."""
    return cargo_sensor.simulated_tray_topic()


class CargoHandoff:
    """Decides the tray from reports and odometry; no I/O, wall-clock time passed in."""

    def __init__(
        self,
        station: Station,
        *,
        tray: MobileCargoSimulator | None = None,
        max_carry_s: float = MAX_CARRY_S,
    ):
        self.station = station
        self.tray = tray or MobileCargoSimulator()
        self.max_carry_s = float(max_carry_s)
        self.poses = PoseHistory()
        self._reports: list[ArmReport] = []
        self._consumed = -1  # reports at or before this index never load the tray again
        self._loaded_through: float | None = None
        self._loaded_at: float | None = None
        self._watch_from = 0  # first report index the loss watch has not read
        self._lifecycle_from = 0  # first report index not yet checked for unload/loss/end
        self._departed_at: float | None = None
        # (place index, decision time): fixed when a place is first ready to
        # be decided, so the decision waits for odometry received after it.
        self._decision: tuple[int, float] | None = None
        self.events: list[dict[str, Any]] = []

    @property
    def cargo_present(self) -> bool:
        return self.tray.carrying

    def observe_pose(self, at: float, pose: Any) -> None:
        self.poses.add(at, pose)

    def observe_reports(self, reports: list[ArmReport]) -> None:
        self._reports.extend(reports)
        overflow = len(self._reports) - MAX_REPORTS
        if overflow > 0:
            del self._reports[:overflow]
            self._consumed -= overflow
            self._watch_from = max(0, self._watch_from - overflow)
            self._lifecycle_from = max(0, self._lifecycle_from - overflow)

    def _record(self, event: dict[str, Any]) -> None:
        self.events.append({"simulation": True, **event})
        del self.events[:-MAX_EVENTS]

    def evaluate(self, now: float) -> None:
        self._apply_lifecycle()
        if self.tray.carrying:
            self._watch_loaded()
            if (
                self.tray.carrying
                and self._loaded_at is not None
                and now - self._loaded_at > self.max_carry_s
            ):
                self.empty("carry_expired", max_carry_s=self.max_carry_s)
        else:
            self._try_load(now)

    def empty(self, kind: str, **detail: Any) -> None:
        """Empty the tray (it only ever loads through a handoff) and record why."""
        was = self.tray.carrying
        if kind in ("lost", "lost_before_departure"):
            self.tray.report_loss()
        else:
            self.tray.unload()
        self._loaded_at = None
        self._record({"kind": kind, "was_carrying": was, **detail})

    def _apply_lifecycle(self) -> None:
        """Unload, loss and mission-end reports only ever empty the tray.

        Each also retires every place reported before it, so a place that
        was still undecided when one of them arrived never loads the tray.
        """
        new = list(enumerate(self._reports))[self._lifecycle_from :]  # (index, report)
        self._lifecycle_from = len(self._reports)
        for index, report in new:
            if report.kind in (KIND_UNLOAD, KIND_LOSS, KIND_MISSION_END):
                self._consumed = max(self._consumed, index)
            if report.kind == KIND_UNLOAD:
                self.empty("unloaded", outcome=report.fields["outcome"])
            elif report.kind == KIND_LOSS:
                self.empty("lost", cause=report.fields["cause"])
            elif report.kind == KIND_MISSION_END:
                # A place followed by a mission end never loads (_try_load).
                self.empty("mission_end", state=report.fields["state"])

    def _try_load(self, now: float) -> None:
        places = [
            i for i, r in enumerate(self._reports) if r.kind == KIND_PLACE and i > self._consumed
        ]
        if not places:
            return
        p = places[-1]
        place = self._reports[p]
        later = self._reports[p + 1 :]
        if max(place.at, place.fields["started_at"]) > now + MAX_REPORT_SKEW_S:
            self._consumed = p
            self._record(
                {
                    "kind": "handoff",
                    "accepted": False,
                    "reason": "report_from_the_future",
                    "report_at": place.at,
                    "started_at": place.fields["started_at"],
                    "decided_at": now,
                }
            )
            return
        retracts = [r for r in later if r.kind == KIND_RETRACT]
        if not retracts:
            return
        retract = retracts[0]
        readings = [r for r in later if r.kind == KIND_READING]
        settled = settled_run(readings)
        if settled is None and now - retract.at < DECIDE_AFTER_S:
            return  # the block may still be settling
        if self._decision is None or self._decision[0] != p:
            self._decision = (p, now)
        decided_at = self._decision[1]
        # Parked from before the place was commanded until the decision time.
        docked = self.poses.parked(place.fields["started_at"], decided_at, self.station)
        if docked is None:
            return  # no odometry received yet at or after the decision time
        self._consumed = p
        try:
            accepted = self.tray.accept_station_handoff(
                is_docked=docked,
                fixed_arm_retracted=retracted(retract) and retract.at >= place.at,
                observed_placement=placement_observed(place),
                physics_placement=settled is not None,
            )
        except MissionFailure as refusal:
            self._record({"kind": "handoff", "accepted": False, "reason": refusal.reason})
            return
        self._loaded_through = now
        self._loaded_at = now
        self._departed_at = None
        last_used = settled[-1] if settled else retract
        self._watch_from = max(i for i, r in enumerate(self._reports) if r is last_used) + 1
        self._record({"kind": "handoff", "accepted": True, **accepted})

    def _watch_loaded(self) -> None:
        if self._departed_at is None and self._loaded_through is not None:
            self._departed_at = self.poses.left_station_after(self._loaded_through, self.station)
            if self._departed_at is not None:
                self._record({"kind": "departed", "at": self._departed_at})
        new = self._reports[self._watch_from :]
        self._watch_from = len(self._reports)
        for report in new:
            if report.kind != KIND_READING:
                continue
            if self._departed_at is not None and report.at >= self._departed_at:
                continue  # the arm's tray proxy says nothing about a robot that left
            if not reading_in_tray(report):
                self._consumed = len(self._reports) - 1
                self.empty(
                    "lost_before_departure",
                    inside=report.fields["inside"],
                    held=report.fields["held"],
                )
                return


class SimulatedStatePublisher(Protocol):
    def latest_odometry(self) -> dict[str, Any] | None: ...

    def publish_simulated_state(self, topic: str, value: bool) -> None: ...

    def reconnect(self) -> Any: ...


class CargoBridge:
    """Samples odometry, reads new arm reports, decides the tray, publishes it."""

    def __init__(
        self,
        *,
        link: SimulatedStatePublisher,
        handoff: CargoHandoff,
        topic: str | None = None,
        reports: ArmReportTail | None = None,
        clock: Callable[[], float] = time.time,
    ):
        self.link, self.handoff, self.reports = link, handoff, reports
        self.topic = topic or bridge_topic()
        self._clock = clock
        self.last_error: str | None = None
        self._stop = threading.Event()
        self._odometry_sequence: int | None = None
        # One tick at a time, and none after stop() published the last false.
        self._lock = threading.Lock()
        self._stopped = False

    def tick(self) -> bool:
        with self._lock:
            if self._stopped:
                return False
            return self._tick()

    def _tick(self) -> bool:
        now = self._clock()
        self._sample_odometry(now)
        if self.reports is not None:
            self.handoff.observe_reports(self.reports.read_new())
        self.handoff.evaluate(now)
        value = self.handoff.cargo_present
        self._publish(value)
        return value

    def _sample_odometry(self, now: float) -> None:
        """Record the newest odometry message once, at the time it was received.

        No message since the last tick records nothing, so silence is a gap
        in the history. No odometry at all (none yet, a dropped connection, a
        read that raised) is an unknown pose at ``now``.
        """
        try:
            message = self.link.latest_odometry()
            sequence = int(message["sequence"]) if message is not None else None
            age = float(message["age_s"]) if message is not None else None
        except Exception:  # a dropped connection is an unknown pose
            message, sequence, age = None, None, None
        if message is None or sequence is None or age is None or not math.isfinite(age):
            self._odometry_sequence = None
            self.handoff.observe_pose(now, None)
            return
        if sequence == self._odometry_sequence:
            return  # the same message again: not a new sample
        self._odometry_sequence = sequence
        self.handoff.observe_pose(now - max(0.0, age), message.get("pose"))

    def _publish(self, value: bool) -> None:
        try:
            self.link.publish_simulated_state(self.topic, value)
            self.last_error = None
        except SimulationOnlyError as refusal:
            # Not a simulation (or not confirmed one): nothing to reconnect to.
            self.last_error = f"SimulationOnlyError: {refusal}"[:200]
        except Exception as error:
            self.last_error = f"{type(error).__name__}: {error}"[:200]
            try:
                self.link.reconnect()
            except Exception as reconnect_error:
                self.last_error += f"; reconnect: {type(reconnect_error).__name__}"

    def run(self, *, rate_hz: float = cargo_sensor.MIN_PUBLISH_HZ) -> None:
        if not cargo_sensor.MIN_PUBLISH_HZ <= rate_hz <= MAX_RATE_HZ:
            raise ValueError(
                f"publish rate must be {cargo_sensor.MIN_PUBLISH_HZ:g}-{MAX_RATE_HZ:g} Hz: "
                "the reader's subscription is volatile"
            )
        period = 1.0 / rate_hz
        while not self._stop.is_set():
            self.tick()
            self._stop.wait(period)

    def start(self, *, rate_hz: float = cargo_sensor.MIN_PUBLISH_HZ) -> threading.Thread:
        """Run the loop on a worker thread."""
        worker = threading.Thread(
            target=self.run, kwargs={"rate_hz": rate_hz}, name="cargo-bridge", daemon=True
        )
        worker.start()
        return worker

    def stop(self) -> None:
        """End the loop, empty the tray, and leave ``false`` as the last value.

        Ticks and this run under one lock: a tick in flight finishes its
        publish before the final ``false``, and no tick publishes after it.
        """
        self._stop.set()
        with self._lock:
            self._stopped = True
            if self.handoff.cargo_present:
                self.handoff.empty("bridge_stopped")
            self._publish(False)

    def state(self) -> dict[str, Any]:
        return {
            "simulation": True,
            "deployment_mode": "simulation",
            "source": "mobile_cargo_simulator_not_a_sensor",
            "topic": self.topic,
            "cargo_present": self.handoff.cargo_present,
            "station": self.handoff.station.as_dict(),
            "reports": None if self.reports is None else str(self.reports.path),
            "rejected_reports": 0 if self.reports is None else self.reports.rejected,
            "publish_error": self.last_error,
            "last_event": self.handoff.events[-1] if self.handoff.events else None,
        }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--station-x", type=float, required=True, help="odometry frame, metres")
    parser.add_argument("--station-y", type=float, required=True, help="odometry frame, metres")
    parser.add_argument("--station-tolerance-m", type=float, default=0.10)
    parser.add_argument(
        "--reports",
        type=Path,
        help="report file the arm's and mission's host appends to "
        "(opt-in; without it the tray never loads)",
    )
    parser.add_argument("--rate-hz", type=float, default=cargo_sensor.MIN_PUBLISH_HZ)
    parser.add_argument("--max-carry-s", type=float, default=MAX_CARRY_S)
    args = parser.parse_args(argv)
    if not cargo_sensor.MIN_PUBLISH_HZ <= args.rate_hz <= MAX_RATE_HZ:
        parser.error(
            f"--rate-hz must be between {cargo_sensor.MIN_PUBLISH_HZ:g} and {MAX_RATE_HZ:g}: "
            "the reader's subscription is volatile"
        )
    if not 0.0 < args.station_tolerance_m <= 0.25:
        parser.error("--station-tolerance-m must be in (0, 0.25]")
    if not 0.0 < args.max_carry_s <= 3600.0:
        parser.error("--max-carry-s must be in (0, 3600]")
    refusal = simulation_publish_refusal()
    if refusal is not None:
        print(f"[cargo-bridge SIMULATION] refused to start: {refusal}", file=sys.stderr)
        return 2

    # Presence mode: odometry only, never a motion command. FLYTO_ROSBRIDGE_URL
    # names the endpoint (default ws://127.0.0.1:19090, the twin's).
    link = RosbridgeROS2Backend(presence_only=True, connect=False)
    bridge = CargoBridge(
        link=link,
        handoff=CargoHandoff(
            Station(args.station_x, args.station_y, args.station_tolerance_m),
            max_carry_s=args.max_carry_s,
        ),
        reports=None if args.reports is None else ArmReportTail(args.reports),
    )
    try:
        link.reconnect()
    except Exception as error:
        print(f"[cargo-bridge SIMULATION] rosbridge not reachable yet: {error}", flush=True)
    print(f"[cargo-bridge SIMULATION] {json.dumps(bridge.state())}", flush=True)
    worker = bridge.start(rate_hz=args.rate_hz)
    last: Any = None
    try:
        while worker.is_alive():
            state = bridge.state()
            shown = (state["cargo_present"], state["publish_error"], state["last_event"])
            if shown != last:
                print(f"[cargo-bridge SIMULATION] {json.dumps(state)}", flush=True)
                last = shown
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        link.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
