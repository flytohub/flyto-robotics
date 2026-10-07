"""A straight drive's speed is governed the way its server can take it.

Simulator runs of 2026-10-06 (19:32, 20:13, 20:20, 20:26 local) logged Nav2's
behavior server answering the braking guard's slower re-send with "Received a
preemption request for drive_on_heading, however feature is currently not
implemented. Aborting and stopping": the slowdown stopped the robot, restarted
the drive from rest, and the guard then tripped it within a scan, so a 1.2 m
advance ended after about 0.05 m. These tests pin, with fakes only:

* one decision (``braking.decide_governance``) picks the mode from the action
  type the running server serves and from what the transport can send;
* on a server that cannot be preempted, speed *and* distance are planned
  before sending, nothing is sent mid-drive, and a drive that ends at its
  planned stop point reports ``obstacle_blocked`` as a guard stop does;
* on a server that can, the smooth re-send path is unchanged;
* for any clearance at or beyond the floor, the planned speed and distance
  never bring the robot to rest inside the floor, whatever latency and
  deceleration the profile carries.
"""

from __future__ import annotations

import math
import random
import time

import pytest

from flyto_robotics import braking_envelope as braking
from flyto_robotics import motion_outcome as mo
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
    CallResult,
)
from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    SLOWDOWN_FRACTION,
    GenericROS2Adapter,
    RclpyROS2Backend,
    RosbridgeROS2Backend,
    StandardInterface,
)
from tests.test_braking_envelope import (
    FLOOR,
    PREEMPTIVE,
    ApproachRosbridge,
    connected,
    odom,
    profile,
    scan,
    sweep_with,
)

DRIVE_ON_HEADING = "nav2_msgs/action/DriveOnHeading"
BACK_UP = "nav2_msgs/action/BackUp"


# -- the one decision ----------------------------------------------------------------


@pytest.mark.parametrize("action_type", [DRIVE_ON_HEADING, BACK_UP])
def test_nav2_straight_drives_are_planned_before_sending(action_type):
    decision = braking.decide_governance(action_type, transport_resends=True)
    assert decision.mode == braking.GOVERNANCE_PLANNED
    assert decision.preemption_supported is False
    assert decision.source == braking.GOVERNANCE_SOURCE_TABLE
    assert not decision.resends


@pytest.mark.parametrize(
    "plugin", ["nav2_behaviors::DriveOnHeading", "nav2_behaviors::BackUp"]
)
def test_the_loaded_nav2_behavior_plugin_decides_first(plugin):
    decision = braking.decide_governance(
        DRIVE_ON_HEADING, transport_resends=True, server_plugin=plugin
    )
    assert decision.mode == braking.GOVERNANCE_PLANNED
    assert decision.source == braking.GOVERNANCE_SOURCE_PLUGIN
    assert decision.server_plugin == plugin


def test_a_plugin_declared_preemptible_outranks_its_action_type():
    decision = braking.decide_governance(
        DRIVE_ON_HEADING,
        transport_resends=True,
        server_plugin="vendor::SmoothDrive",
        table={"vendor::SmoothDrive": True, DRIVE_ON_HEADING: False},
    )
    assert decision.mode == braking.GOVERNANCE_RESEND
    assert decision.source == braking.GOVERNANCE_SOURCE_PLUGIN


def test_an_unknown_plugin_falls_back_to_the_action_type():
    decision = braking.decide_governance(
        DRIVE_ON_HEADING, transport_resends=True, server_plugin="vendor::Other"
    )
    assert decision.mode == braking.GOVERNANCE_PLANNED
    assert decision.source == braking.GOVERNANCE_SOURCE_TABLE


def test_a_server_declared_preemptible_keeps_the_resend_path():
    assert PREEMPTIVE.mode == braking.GOVERNANCE_RESEND
    assert PREEMPTIVE.preemption_supported is True
    assert PREEMPTIVE.resends


def test_an_undeclared_action_type_is_planned():
    decision = braking.decide_governance("vendor_msgs/action/Drive", transport_resends=True)
    assert decision.mode == braking.GOVERNANCE_PLANNED
    assert decision.preemption_supported is None
    assert decision.source == braking.GOVERNANCE_SOURCE_UNKNOWN_TYPE


def test_a_transport_that_cannot_resend_plans_even_for_a_preemptible_server():
    decision = braking.decide_governance(
        "x/action/D", transport_resends=False, table={"x/action/D": True}
    )
    assert decision.mode == braking.GOVERNANCE_PLANNED
    assert decision.source == braking.GOVERNANCE_SOURCE_TRANSPORT


def test_the_transports_say_what_they_can_send():
    assert RosbridgeROS2Backend.RESENDS_GOALS is True
    assert RclpyROS2Backend.RESENDS_GOALS is False


# -- the plan, as a property ----------------------------------------------------------


def rest_gap_after_plan(plan: braking.DrivePlan, *, latency: float, decel: float) -> float:
    """Room left to the floor once the robot is at rest, worst case.

    The robot is already at the planned speed when it starts (no
    acceleration, the longest it can go), drives the planned distance, keeps
    that speed for the whole latency after the drive is over, then brakes.
    """
    v = plan.speed_mps
    return plan.room_m - plan.distance_m - v * latency - v * v / (2.0 * decel)


def guarded_rest_gap(
    plan: braking.DrivePlan,
    guard: braking.BrakingProfile,
    *,
    scan_period: float,
    phase: float,
    dt: float = 2e-4,
) -> float:
    """The same drive stepped in time with the guard watching every scan.

    The drive ends itself at the planned distance; the guard trips when the
    scanned room is within the stopping distance. Whichever acts first, the
    stop takes effect after the rest of the latency budget, then the base
    brakes at the profile's deceleration.
    """
    room, v, t, travelled = plan.room_m, plan.speed_mps, 0.0, 0.0
    next_scan = phase * scan_period
    brake_at = None
    while v > 0.0:
        if brake_at is None:
            if travelled >= plan.distance_m:
                brake_at = t + guard.latency_s
            elif t >= next_scan:
                if room - travelled <= guard.stopping_distance(v):
                    brake_at = t + guard.latency_s - scan_period
                next_scan += scan_period
        if brake_at is not None and t >= brake_at:
            v = max(0.0, v - guard.decel_mps2 * dt)
        travelled += v * dt
        t += dt
        if t > 600.0:
            raise AssertionError("never stopped")
    return room - travelled


def random_cases(count: int, seed: int):
    rng = random.Random(seed)
    for _ in range(count):
        yield (
            rng.choice([0.0, rng.uniform(0.0, 0.05), rng.uniform(0.0, 0.5), rng.uniform(0.0, 4.0)]),
            rng.uniform(*braking.LATENCY_BOUNDS_S),
            rng.uniform(*braking.DECEL_BOUNDS_MPS2),
            rng.uniform(braking.MIN_SPEED_MPS, 1.0),
            rng.choice([rng.uniform(0.0, 0.05), rng.uniform(0.0, 4.0)]),
        )


def test_a_planned_drive_never_comes_to_rest_inside_the_floor():
    """For any clearance >= floor and any profile, the plan stops at or beyond it."""
    sent = refused = shortened = 0
    for room, latency, decel, speed, distance in random_cases(20000, seed=20261006):
        guard = braking.BrakingProfile(floor_m=FLOOR, latency_s=latency, decel_mps2=decel)
        plan = braking.plan_drive(
            guard,
            room_m=room,
            requested_speed_mps=speed,
            requested_distance_m=distance,
            mode=braking.GOVERNANCE_PLANNED,
            speed_fraction=SLOWDOWN_FRACTION,
        )
        if plan.refusal is not None:
            refused += 1
            assert (
                plan.speed_mps < braking.MIN_SPEED_MPS
                or plan.distance_m < braking.MIN_DISTANCE_M
            )
            continue
        sent += 1
        shortened += plan.shortened
        assert plan.speed_mps <= speed + 1e-12
        assert plan.distance_m <= distance + 1e-12
        assert plan.speed_mps >= braking.MIN_SPEED_MPS
        assert rest_gap_after_plan(plan, latency=latency, decel=decel) >= -1e-9, (
            room, latency, decel, speed, distance, plan,
        )
    # The sample covered all three outcomes, not just the easy one.
    assert sent > 1000 and refused > 100 and shortened > 1000


def test_a_planned_drive_rests_beyond_the_floor_with_the_guard_stepping_in_time():
    rng = random.Random(7)
    for _ in range(150):
        room = rng.uniform(0.02, 1.5)
        latency = rng.uniform(0.2, 1.0)
        decel = rng.uniform(0.2, 2.5)
        guard = braking.BrakingProfile(floor_m=FLOOR, latency_s=latency, decel_mps2=decel)
        plan = braking.plan_drive(
            guard,
            room_m=room,
            requested_speed_mps=rng.uniform(0.05, 0.5),
            requested_distance_m=rng.uniform(0.05, 2.0),
            mode=braking.GOVERNANCE_PLANNED,
            speed_fraction=SLOWDOWN_FRACTION,
        )
        if plan.refusal is not None:
            continue
        gap = guarded_rest_gap(
            plan, guard, scan_period=min(0.1, latency), phase=rng.random()
        )
        assert gap >= -2e-3, (room, latency, decel, plan)


def test_a_drive_with_room_to_spare_is_sent_as_asked():
    plan = braking.plan_drive(
        profile(0.5, 0.5), room_m=3.0, requested_speed_mps=0.12,
        requested_distance_m=1.0, mode=braking.GOVERNANCE_PLANNED,
        speed_fraction=SLOWDOWN_FRACTION,
    )
    assert (plan.speed_mps, plan.distance_m, plan.shortened) == (0.12, 1.0, False)


def test_an_open_sweep_is_not_shortened():
    plan = braking.plan_drive(
        profile(0.5, 0.5), room_m=math.inf, requested_speed_mps=0.2,
        requested_distance_m=5.0, mode=braking.GOVERNANCE_PLANNED,
        speed_fraction=SLOWDOWN_FRACTION,
    )
    assert (plan.speed_mps, plan.distance_m, plan.shortened) == (0.2, 5.0, False)


def test_the_resend_mode_sends_the_whole_distance():
    plan = braking.plan_drive(
        profile(0.5, 0.5), room_m=0.5, requested_speed_mps=0.2,
        requested_distance_m=2.0, mode=braking.GOVERNANCE_RESEND,
        speed_fraction=SLOWDOWN_FRACTION,
    )
    assert plan.distance_m == 2.0 and not plan.shortened


# -- through the adapter ---------------------------------------------------------------


class GraphBackend:
    """A backend whose graph serves the advance with Nav2's DriveOnHeading."""

    RESENDS_GOALS = True

    def __init__(self, sweep, plugin="nav2_behaviors::DriveOnHeading"):
        kind, name, action_type = DEFAULT_INTERFACES["motion.advance"]
        self.interfaces = (StandardInterface(kind, name, action_type),)
        self.plugin = plugin
        self.parameter_reads: list[tuple[str, str]] = []
        self.sweep = sweep
        self.armed: list[dict] = []
        self.calls: list[dict] = []

    def discover(self):
        return self.interfaces

    def get_parameter(self, node, name):
        self.parameter_reads.append((node, name))
        if self.plugin is None:
            raise RuntimeError("no behavior server")
        return {"type": 4, "string_value": self.plugin}

    def observation(self, required=None):
        return {
            "pose": {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
            "range": {"minimum_range_m": 1.0, "sample_count": 400, "sweep": self.sweep},
            "map_tf_available": True,
        }

    def braking_profile(self):
        return profile(latency=0.5, decel=0.5)

    def arm_braking_guard(self, call_id, capability_id, **kwargs):
        self.armed.append(kwargs)

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        self.calls.append(dict(arguments))
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={})

    def execution_count(self, call_id):
        return len(self.calls)


def advance(backend, **arguments):
    device = GenericROS2Adapter(backend=backend, resource_id="t")
    device.describe()
    return device.invoke(CallRequest("adv", "motion.advance", arguments=arguments))


def test_an_unpreemptible_server_gets_a_distance_that_ends_before_the_floor():
    backend = GraphBackend(sweep_with({0: 1.35}))  # room 1.0
    advance(backend, distance_m=2.0, speed_mps=0.2)
    sent = backend.calls[0]
    guard = profile(0.5, 0.5)
    assert sent["speed_mps"] == pytest.approx(0.2)
    assert sent["distance_m"] == pytest.approx(1.0 - guard.stopping_distance(0.2))
    armed = backend.armed[0]
    assert armed["governance"].mode == braking.GOVERNANCE_PLANNED
    assert armed["governance"].action_type == DRIVE_ON_HEADING
    # Decided from the plugin the running behavior server says it loaded.
    assert backend.parameter_reads == [("/behavior_server", "drive_on_heading.plugin")]
    assert armed["governance"].server_plugin == "nav2_behaviors::DriveOnHeading"
    assert armed["governance"].source == braking.GOVERNANCE_SOURCE_PLUGIN
    assert armed["plan"].shortened is True
    assert armed["plan"].requested_distance_m == 2.0


def test_an_unreadable_plugin_is_decided_by_the_action_type_and_read_once():
    backend = GraphBackend(sweep_with({0: 1.35}), plugin=None)
    device = GenericROS2Adapter(backend=backend, resource_id="t")
    device.describe()
    for call in ("a1", "a2"):
        device.invoke(CallRequest(call, "motion.advance", arguments={"distance_m": 2.0}))
    assert len(backend.parameter_reads) == 1  # no second wait before the next drive
    governance = backend.armed[0]["governance"]
    assert governance.mode == braking.GOVERNANCE_PLANNED
    assert governance.source == braking.GOVERNANCE_SOURCE_TABLE


def test_a_preemptible_server_gets_the_whole_distance(monkeypatch):
    # As if a behavior server loaded a plugin declared preemptible.
    monkeypatch.setitem(braking.PREEMPTION_BY_IMPLEMENTATION, "vendor::SmoothDrive", True)
    backend = GraphBackend(sweep_with({0: 1.35}), plugin="vendor::SmoothDrive")
    advance(backend, distance_m=2.0, speed_mps=0.2)
    assert backend.calls[0]["distance_m"] == 2.0
    assert backend.armed[0]["governance"].mode == braking.GOVERNANCE_RESEND


def test_too_little_room_to_drive_and_stop_is_refused_with_the_plan():
    # 2 cm of room: some speed fits, but no distance is left after stopping.
    backend = GraphBackend(sweep_with({0: FLOOR + 0.02}))
    result = advance(backend, distance_m=1.0, speed_mps=0.2)
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == mo.REASON_PATH_BLOCKED
    assert result.evidence["braking"]["governance"]["mode"] == braking.GOVERNANCE_PLANNED
    assert "clearance floor" in result.detail
    assert backend.calls == [] and backend.armed == []


# -- through the rosbridge transport ------------------------------------------------------


def approach_then_finish(goal):
    """A drive at a box 1.35 m ahead (room 1.0) that ends where it was sent to.

    At 0.85 m in, the room is 0.15 m: a resend-mode guard asks for a slower
    goal there (0.8 * v_max(0.15) = 0.169 < 0.9 * 0.2), and the stopping
    distance at 0.2 m/s (0.14 m) is not yet reached, so nothing trips.
    """
    if goal["id"] == "adv":
        yield odom(0.6, 0.2)
        yield scan(0.75)
        yield odom(0.85, 0.2)
        yield scan(0.50)
    if goal["id"] == "adv" and goal["args"]["target"]["x"] < 1.0:
        yield odom(0.86, 0.0)
        yield {"op": "action_result", "id": "adv", "status": 4, "result": True,
               "values": {"error_code": 0}}
    elif goal["id"] != "adv":
        yield {"op": "action_result", "id": "adv", "status": 6, "result": False}
        yield odom(1.0, 0.0)
        yield {"op": "action_result", "id": goal["id"], "status": 4, "result": True}


def planned_drive(backend, governance):
    guard = profile(0.5, 0.5)
    plan = braking.plan_drive(
        guard, room_m=1.0, requested_speed_mps=0.2, requested_distance_m=2.0,
        mode=governance.mode, speed_fraction=SLOWDOWN_FRACTION,
    )
    backend.arm_braking_guard(
        "adv", "motion.advance", profile=guard, speed_mps=plan.speed_mps,
        requested_speed_mps=0.2, distance_m=plan.distance_m, start_room_m=1.0,
        governance=governance, plan=plan,
    )
    with backend._condition:
        backend._guards["adv"].last_slowdown_at = time.monotonic() - 10
    return backend.invoke(
        call_id="adv", capability_id="motion.advance",
        arguments={"distance_m": plan.distance_m, "speed_mps": plan.speed_mps},
        deadline_seconds=5.0,
    )


def test_an_unpreemptible_drive_is_never_resent_and_reports_the_obstacle(monkeypatch):
    socket = ApproachRosbridge(lambda goal: list(approach_then_finish(goal)))
    backend = connected(socket, monkeypatch)
    governance = braking.decide_governance(DRIVE_ON_HEADING, transport_resends=True)
    result = planned_drive(backend, governance)
    backend.disconnect()

    goals = [m for m in socket.sent if m["op"] == "send_action_goal"]
    assert [g["id"] for g in goals] == ["adv"]  # no preempting goal, ever
    assert goals[0]["args"]["target"]["x"] == pytest.approx(0.86)
    assert not any(m["op"] == "cancel_action_goal" for m in socket.sent)

    # Nav2 said "succeeded" at 0.86 m; the call asked for 2.0 m and the box is why.
    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["motion_outcome"]
    assert outcome["reason"] == mo.REASON_OBSTACLE_BLOCKED
    assert outcome["action_status"] == 4
    assert outcome["requested_distance_m"] == 2.0
    assert outcome["commanded_distance_m"] == pytest.approx(0.86)
    record = outcome["braking"]
    assert record["ended"] == mo.GUARD_END_PLANNED_STOP
    assert record["tripped"] is None
    assert record["governance"] == {
        "mode": braking.GOVERNANCE_PLANNED,
        "action_type": DRIVE_ON_HEADING,
        "preemption_supported": False,
        "source": braking.GOVERNANCE_SOURCE_TABLE,
        "server_plugin": None,
    }
    assert record["plan"]["room_to_floor_at_send_m"] == 1.0
    assert record["speed_changes"] == [{"at_s": 0.0, "speed_mps": 0.2, "room_m": 1.0}]
    # The operator line Cloud shows says why and how it was governed.
    assert result.detail.startswith("ROS 2 action status 4 (succeeded): obstacle_blocked")
    assert "travelled 0.860 of 2.000 m" in result.detail
    assert "ended at the stop point planned from 1.000 m" in result.detail
    assert "speed governance planned_before_send (action_type)" in result.detail


def test_a_preemptible_drive_is_still_slowed_by_a_resent_goal(monkeypatch):
    socket = ApproachRosbridge(lambda goal: list(approach_then_finish(goal)))
    backend = connected(socket, monkeypatch)
    result = planned_drive(backend, PREEMPTIVE)
    backend.disconnect()

    goals = [m for m in socket.sent if m["op"] == "send_action_goal"]
    assert [g["id"] for g in goals] == ["adv", "adv#slow1"]
    assert goals[0]["args"]["target"]["x"] == 2.0
    assert goals[1]["args"]["speed"] < 0.2
    assert result.outcome == OUTCOME_COMPLETED
    record = result.evidence["motion_outcome"]["braking"]
    assert record["governance"]["mode"] == braking.GOVERNANCE_RESEND
    assert "ended" not in record


def test_a_planned_drive_is_still_stopped_by_the_guard_when_the_room_runs_out(monkeypatch):
    # Something moved into the path after the plan: the guard is the hard stop.
    def approach(goal):
        yield odom(0.3, 0.2)
        yield scan(0.45)  # room 0.10 < stopping distance 0.14 at 0.2 m/s

    socket = ApproachRosbridge(lambda goal: list(approach(goal)))
    backend = connected(socket, monkeypatch)
    governance = braking.decide_governance(DRIVE_ON_HEADING, transport_resends=True)
    result = planned_drive(backend, governance)
    backend.disconnect()

    assert any(m["op"] == "cancel_action_goal" for m in socket.sent)
    assert len([m for m in socket.sent if m["op"] == "send_action_goal"]) == 1
    outcome = result.evidence["motion_outcome"]
    assert result.outcome == OUTCOME_FAILED
    assert outcome["reason"] == mo.REASON_OBSTACLE_BLOCKED
    assert outcome["braking"]["tripped"] == mo.GUARD_TRIP_CLEARANCE
    assert "ended" not in outcome["braking"]
    assert "speed governance planned_before_send" in result.detail
