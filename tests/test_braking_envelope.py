"""The clearance floor has to hold where the robot comes to rest, not where it
was told to stop.

The live run of 2026-10-05 (motion.advance 2.0 m on the physical TurtleBot3)
ended ``obstacle_blocked; error 723; travelled 1.208 of 2.000 m; nearest LiDAR
return at stop 0.181 m (floor 0.350 m)``. These tests pin, without ROS:

* a deterministic kinematic model (scan sampling, latency, constant
  deceleration) in which the braking envelope stops at or beyond the floor for
  every speed up to the base's declared limit, and the old "stop when the
  obstacle reaches the floor" rule overshoots it;
* the geometry of the room left before the floor, including a return beside
  the path that the robot would pass closer than the floor;
* how configured, measured and declared values combine (always conservatively);
* the live record, re-read with the direction of its nearest return;
* the guard acting through the rosbridge transport: a stop, a slowdown, a
  LiDAR gone quiet, and the starting speed cap.
"""

from __future__ import annotations

import json
import math
import queue
import time

import pytest

from flyto_robotics import braking_envelope as braking
from flyto_robotics import motion_outcome as mo
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
)
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RosbridgeROS2Backend

FLOOR = 0.35

# Read from the physical robot (read-only) on 2026-10-05:
#   /scan: 10.0 Hz, header-stamp-to-receipt 1-23 ms on the robot itself
#   Nav2 behavior_server cycle_frequency 10.0 Hz, simulate_ahead_time 2.0 s
#   velocity_smoother max_decel [-2.5, 0, -3.2], max_velocity [0.5, 0, 2.5]
#   local costmap robot_radius 0.10 m, update 5 Hz; base_scan 0.032 m behind base_link
ROBOT_SCAN_PERIOD_S = 0.1
ROBOT_CONTROL_PERIOD_S = 0.1
ROBOT_DECLARED_DECEL = 2.5
ROBOT_MAX_SPEED = 0.5


def profile(latency=braking.DEFAULT_LATENCY_S, decel=braking.DEFAULT_DECEL_MPS2):
    return braking.BrakingProfile(floor_m=FLOOR, latency_s=latency, decel_mps2=decel)


# -- the kinematic model -----------------------------------------------------------


def drive_at_wall(
    *,
    speed: float,
    start_gap: float,
    trigger,
    scan_period: float,
    delay_after_scan: float,
    decel: float,
    phase: float = 0.0,
    dt: float = 1e-4,
) -> float:
    """Gap between LiDAR and a wall straight ahead once the robot is at rest.

    The robot holds ``speed``. The wall is sampled at scan times (offset by
    ``phase`` within a period); ``trigger(gap)`` decides on each scan. A
    decision takes effect ``delay_after_scan`` after the scan it was made on,
    and from then the base slows at ``decel`` to rest.
    """
    t = 0.0
    gap = start_gap
    v = speed
    next_scan = phase * scan_period
    brake_at = None
    while v > 0.0:
        if brake_at is None and t >= next_scan:
            if trigger(gap):
                brake_at = t + delay_after_scan
            next_scan += scan_period
        if brake_at is not None and t >= brake_at:
            v = max(0.0, v - decel * dt)
        gap -= v * dt
        t += dt
        if t > 120.0:
            raise AssertionError("never stopped")
    return gap


def robot_budget() -> braking.LatencyBudget:
    return braking.LatencyBudget(
        scan_period_s=ROBOT_SCAN_PERIOD_S,
        scan_transport_s=0.05,
        command_transport_s=0.05,
        control_period_s=ROBOT_CONTROL_PERIOD_S,
        actuation_s=0.1,
    )


SPEEDS = [0.02, 0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.22, 0.25, 0.30, 0.40, ROBOT_MAX_SPEED]


@pytest.mark.parametrize("speed", SPEEDS)
@pytest.mark.parametrize("phase", [0.0, 0.37, 0.99])
def test_envelope_comes_to_rest_at_or_beyond_the_floor(speed, phase):
    budget = robot_budget()
    # The model's latency is the budget's, term for term: worst case, the
    # deciding scan comes a whole period after the exact crossing.
    guard = profile(latency=budget.total_s, decel=braking.DEFAULT_DECEL_MPS2)
    rest = drive_at_wall(
        speed=speed,
        start_gap=1.2,
        trigger=lambda gap: gap - FLOOR <= guard.stopping_distance(speed),
        scan_period=budget.scan_period_s,
        delay_after_scan=budget.total_s - budget.scan_period_s,
        decel=guard.decel_mps2,
        phase=phase,
    )
    assert rest >= FLOOR - 1e-3


@pytest.mark.parametrize("speed", [s for s in SPEEDS if s >= 0.05])
def test_the_old_rule_overshoots_in_the_same_model(speed):
    budget = robot_budget()
    rest = drive_at_wall(
        speed=speed,
        start_gap=1.2,
        trigger=lambda gap: gap <= FLOOR,  # stop when the wall reaches the floor
        scan_period=budget.scan_period_s,
        delay_after_scan=budget.total_s - budget.scan_period_s,
        decel=braking.DEFAULT_DECEL_MPS2,
    )
    assert rest < FLOOR
    # By about the two physical terms the floor-only rule leaves out.
    overshoot = FLOOR - rest
    assert overshoot == pytest.approx(
        profile(latency=budget.total_s - budget.scan_period_s).stopping_distance(speed),
        abs=speed * budget.scan_period_s + 2e-3,
    )


@pytest.mark.parametrize("speed", SPEEDS)
def test_a_faster_real_base_only_adds_margin(speed):
    """The guard assumes 0.5 m/s^2; the base declares 2.5. Stopping harder is safe."""
    budget = robot_budget()
    guard = profile(latency=budget.total_s)
    rest = drive_at_wall(
        speed=speed,
        start_gap=1.2,
        trigger=lambda gap: gap - FLOOR <= guard.stopping_distance(speed),
        scan_period=budget.scan_period_s,
        delay_after_scan=budget.total_s - budget.scan_period_s,
        decel=ROBOT_DECLARED_DECEL,
    )
    assert rest >= FLOOR


def test_speed_scaled_by_room_approaches_the_floor_without_crossing_it():
    """Commanding v = 0.8 * v_max(room) on every scan slows smoothly to the floor."""
    budget = robot_budget()
    guard = profile(latency=budget.total_s)
    gap, v, t = 1.5, 0.25, 0.0
    commanded = v
    dt = 1e-3
    next_scan = 0.0
    pending: list[tuple[float, float]] = []
    speeds = []
    while t < 60.0:
        if t >= next_scan:
            room = gap - FLOOR
            allowed = guard.max_speed(room) * 0.8
            if allowed < braking.MIN_SPEED_MPS:
                pending.append((t + budget.total_s - budget.scan_period_s, 0.0))
            elif allowed < commanded * 0.9:
                commanded = allowed
                pending.append((t + budget.total_s - budget.scan_period_s, allowed))
            next_scan += budget.scan_period_s
        while pending and t >= pending[0][0]:
            target = pending.pop(0)[1]
            commanded = min(commanded, target) if target > 0 else 0.0
        if v > commanded:
            v = max(commanded, v - braking.DEFAULT_DECEL_MPS2 * dt)
        gap -= v * dt
        speeds.append(v)
        t += dt
        if v == 0.0 and commanded == 0.0:
            break
    assert gap >= FLOOR
    # It slowed in steps before the final stop rather than at the last moment.
    assert len({round(s, 3) for s in speeds}) > 10


def test_max_speed_is_the_inverse_of_stopping_distance():
    guard = profile(latency=0.4, decel=0.7)
    for room in (0.001, 0.01, 0.05, 0.1, 0.3, 1.0, 3.0):
        assert guard.stopping_distance(guard.max_speed(room)) == pytest.approx(room)
    assert guard.max_speed(0.0) == 0.0
    assert guard.max_speed(-0.1) == 0.0
    assert guard.max_speed(math.inf) == math.inf


def test_required_clearance_is_floor_plus_latency_plus_braking():
    guard = profile(latency=0.5, decel=0.5)
    v = 0.12
    assert guard.required_clearance(v) == pytest.approx(0.35 + 0.12 * 0.5 + 0.12**2 / 1.0)


# -- geometry ------------------------------------------------------------------------


def sweep_with(points: dict[float, float], bins: int = 180) -> dict:
    """A sweep from straight ahead, counter-clockwise, with returns at degrees."""
    step = math.tau / bins
    ranges: list[float | None] = [None] * bins
    for degrees, distance in points.items():
        ranges[round(math.radians(degrees) / step) % bins] = distance
    return {"angle_min_rad": 0.0, "angle_increment_rad": step, "ranges_m": ranges}


def test_room_straight_ahead_is_distance_minus_floor():
    room = braking.room_to_floor(sweep_with({0: 1.0}), 0.0, FLOOR)
    assert room.room_m == pytest.approx(0.65)
    assert room.bearing_rad == pytest.approx(0.0)


def test_a_wall_wider_than_the_floor_does_not_limit_a_straight_drive():
    room = braking.room_to_floor(sweep_with({90: 0.40, -90: 0.40, 180: 0.2}), 0.0, FLOOR)
    assert room.room_m == math.inf


def test_a_return_the_robot_would_pass_closer_than_the_floor_limits_it():
    # 0.181 m beside the path, 0.5 m ahead: passing it would break the floor.
    x, y = 0.5, 0.181
    room = braking.room_to_floor(
        sweep_with({math.degrees(math.atan2(y, x)): math.hypot(x, y)}, bins=3600), 0.0, FLOOR
    )
    assert room.room_m == pytest.approx(x - math.sqrt(FLOOR**2 - y**2), abs=2e-3)


def test_retreat_reads_behind_and_ignores_ahead():
    sweep = sweep_with({0: 0.2, 180: 0.9})
    assert braking.room_to_floor(sweep, math.pi, FLOOR).room_m == pytest.approx(0.55)
    assert braking.room_to_floor(sweep, 0.0, FLOOR).room_m < 0


def test_an_unreadable_sweep_is_not_room():
    assert braking.room_to_floor(None, 0.0, FLOOR) is None
    assert braking.room_to_floor(sweep_with({}), 0.0, FLOOR) is None


def test_direction_labels():
    assert braking.direction_label(0.1) == "ahead"
    assert braking.direction_label(math.radians(95)) == "left"
    assert braking.direction_label(math.radians(-95)) == "right"
    assert braking.direction_label(math.radians(170)) == "behind"


# -- where the constants come from ---------------------------------------------------


def test_measured_latency_can_raise_but_never_lower_the_configured_one():
    slow = braking.LatencyBudget(0.2, 0.3, 0.3, 0.1, 0.1)  # 1.0 s
    fast = braking.LatencyBudget(0.1, 0.0, 0.0, 0.1, 0.1)  # 0.3 s
    assert braking.resolve_profile(floor_m=FLOOR, measured_latency=slow).latency_s == 1.0
    raised = braking.resolve_profile(floor_m=FLOOR, measured_latency=slow)
    assert raised.sources["latency"] == "measured"
    kept = braking.resolve_profile(floor_m=FLOOR, measured_latency=fast)
    assert kept.latency_s == braking.DEFAULT_LATENCY_S
    assert kept.sources["latency"] == "configured"


def test_declared_deceleration_can_lower_but_never_raise_the_configured_one():
    robot = braking.resolve_profile(floor_m=FLOOR, declared_decel_mps2=-ROBOT_DECLARED_DECEL)
    assert robot.decel_mps2 == braking.DEFAULT_DECEL_MPS2
    assert robot.sources["declared_decel_mps2"] == ROBOT_DECLARED_DECEL
    weak = braking.resolve_profile(floor_m=FLOOR, declared_decel_mps2=0.3)
    assert weak.decel_mps2 == 0.3 and weak.sources["decel"] == "declared"


def test_configured_values_are_clamped_and_the_floor_is_passed_through():
    wild = braking.resolve_profile(
        floor_m=FLOOR, configured_latency_s=0.0, configured_decel_mps2=100.0
    )
    assert wild.latency_s == braking.LATENCY_BOUNDS_S[0]
    assert wild.decel_mps2 == braking.DECEL_BOUNDS_MPS2[1]
    assert wild.floor_m == FLOOR


def test_latency_estimator_measures_period_and_drops_clock_skew():
    estimator = braking.LatencyEstimator()
    assert estimator.budget(control_period_s=0.1, actuation_s=0.1) is None
    for index in range(10):
        estimator.observe(index * 0.1, 0.02 if index % 2 else 5.0)  # 5 s = skew
    budget = estimator.budget(control_period_s=0.1, actuation_s=0.1)
    assert budget.scan_period_s == pytest.approx(0.1)
    assert budget.scan_transport_s == pytest.approx(0.02)
    assert budget.total_s == pytest.approx(0.1 + 0.02 + 0.02 + 0.1 + 0.1)


# -- the live record ------------------------------------------------------------------


def test_live_overshoot_cannot_be_straight_ahead_coasting_at_the_live_speed():
    """1.208 m in 11.50 s (behavior_server log 22:02:15.643 -> 22:02:27.145).

    Commanded 0.12 m/s (the default; the job named no speed). Read as a stop
    0.169 m past the floor straight ahead, it would need a latency the robot
    does not have: its own stack measures about a quarter of that.
    """
    commanded = 0.12
    average = 1.208 / 11.50
    assert average < commanded
    overshoot = 0.350 - 0.181
    implied = (overshoot - commanded**2 / (2 * ROBOT_DECLARED_DECEL)) / commanded
    on_robot = ROBOT_SCAN_PERIOD_S + 1 / 5.0 + ROBOT_CONTROL_PERIOD_S  # scan, costmap, behavior
    assert implied > 1.3
    assert implied > 3 * on_robot


def test_live_record_reads_with_the_direction_of_its_nearest_return():
    # The run's figures, with the nearest return beside the path: the robot
    # passed it, so the floor was broken without anything being in the way.
    track = mo.MotionTrack(
        capability_id="motion.advance",
        arguments={"distance_m": 2.0},
        start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        started_at=0.0,
    )
    sweep = sweep_with({100: 0.181, 0: 1.97})
    summary = mo.summarize(
        track,
        status=6,
        result_values={"error_code": 723, "error_msg": ""},
        end_pose={"x": 1.208, "y": 0.0, "yaw": 0.0},
        range_observation={"minimum_range_m": 0.181, "sweep": sweep},
        clearance_floor_m=FLOOR,
        ended_at=11.5,
    )
    assert summary["reason"] == mo.REASON_OBSTACLE_BLOCKED
    clearance = summary["stop_clearance"]
    assert clearance["nearest_direction"] == "left"
    assert clearance["floor_held"] is False
    assert clearance["travel_floor_held"] is True  # nothing ahead inside the floor
    line = mo.describe(summary)
    assert "travelled 1.208 of 2.000 m" in line
    assert "nearest LiDAR return at stop 0.181 m left" in line
    assert "floor NOT held" in line


# -- through the rosbridge transport --------------------------------------------------


class ApproachRosbridge:
    """A rosbridge socket whose robot drives at a wall until told otherwise."""

    def __init__(self, scans, *, finish=None):
        self.inbox: queue.Queue = queue.Queue()
        self.sent: list[dict] = []
        self.scans = scans
        self.finish = finish

    def send(self, raw):
        message = json.loads(raw)
        self.sent.append(message)
        op = message.get("op")
        if op == "send_action_goal":
            for reply in self.scans(message):
                self.inbox.put(json.dumps(reply))
            if self.finish is not None:
                for reply in self.finish(message):
                    self.inbox.put(json.dumps(reply))
        elif op == "cancel_action_goal":
            self.inbox.put(json.dumps(
                {"op": "action_result", "id": message["id"], "status": 5, "result": False}
            ))

    def recv(self, timeout=None):
        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError from None

    def pong(self, _data=b""):
        pass

    def close(self):
        pass


def odom(x, v=0.0):
    return {
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "pose": {"pose": {"position": {"x": x, "y": 0.0},
                              "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
            "twist": {"twist": {"linear": {"x": v}, "angular": {"z": 0.0}}},
        },
    }


def scan(front):
    ranges = [front] * 3 + [3.0] * 94 + [front] * 3
    return {
        "op": "publish",
        "topic": "/scan",
        "msg": {"ranges": ranges, "range_min": 0.1, "range_max": 8.0,
                "angle_min": 0.0, "angle_increment": math.tau / 100},
    }


def connected(socket, monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.25")
    monkeypatch.setenv("FLYTO_ROS2_DECEL_PARAMETER", "")
    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=lambda _u: socket)
    backend._topic_types = {"/cmd_vel": "geometry_msgs/msg/TwistStamped"}
    for message in (odom(0.0), scan(3.0)):
        backend._handle_message(message)
    return backend


def arm(backend, call_id, *, speed=0.12, distance=2.0, room=2.65):
    backend.arm_braking_guard(
        call_id,
        "motion.advance",
        profile=profile(latency=0.5, decel=0.5),
        speed_mps=speed,
        requested_speed_mps=speed,
        distance_m=distance,
        start_room_m=room,
    )


def test_guard_stops_a_drive_while_room_remains_and_says_so(monkeypatch):
    # 0.12 m/s needs 0.0744 m beyond the floor: 0.40 m ahead leaves 0.05.
    def approach(goal):
        yield odom(0.5, 0.12)
        yield scan(0.80)
        yield odom(0.9, 0.12)
        yield scan(0.40)

    socket = ApproachRosbridge(lambda goal: list(approach(goal)))
    backend = connected(socket, monkeypatch)
    arm(backend, "adv")
    result = backend.invoke(
        call_id="adv", capability_id="motion.advance",
        arguments={"distance_m": 2.0, "speed_mps": 0.12}, deadline_seconds=5.0,
    )
    backend.disconnect()

    ops = [message["op"] for message in socket.sent]
    assert "cancel_action_goal" in ops
    cancel_at = ops.index("cancel_action_goal")
    assert "publish" in ops[cancel_at - 1:cancel_at] and "publish" in ops[cancel_at + 1:]
    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["motion_outcome"]
    assert outcome["reason"] == mo.REASON_OBSTACLE_BLOCKED
    trip = outcome["braking"]["trip"]
    assert outcome["braking"]["tripped"] == mo.GUARD_TRIP_CLEARANCE
    assert trip["room_m"] == pytest.approx(0.05, abs=1e-3)
    assert trip["stopping_distance_m"] == pytest.approx(0.0744, abs=1e-4)
    assert trip["required_clearance_m"] == pytest.approx(0.4244, abs=1e-4)
    assert outcome["stop_clearance"]["floor_held"] is True
    assert "braking guard stopped at 0.120 m/s" in result.detail


def test_guard_slows_the_drive_as_the_room_shrinks(monkeypatch):
    def approach(goal):
        if goal["id"] == "adv":
            yield odom(0.3, 0.2)
            yield scan(0.80)  # room 0.45: 0.8 * v_max = 0.229, not below 0.9 * 0.2
            yield odom(0.6, 0.2)
            yield scan(0.50)  # room 0.15: 0.8 * v_max = 0.169 -> slow down, no stop
        else:
            yield {"op": "action_result", "id": "adv", "status": 6, "result": False,
                   "values": {"error_code": 0}}  # the preempted goal
            yield odom(0.7, 0.0)
            yield {"op": "action_result", "id": goal["id"], "status": 4, "result": True}

    socket = ApproachRosbridge(lambda goal: list(approach(goal)))
    backend = connected(socket, monkeypatch)
    arm(backend, "adv", speed=0.2, distance=0.8, room=2.65)
    with backend._condition:
        backend._guards["adv"].last_slowdown_at = time.monotonic() - 10
    result = backend.invoke(
        call_id="adv", capability_id="motion.advance",
        arguments={"distance_m": 0.8, "speed_mps": 0.2}, deadline_seconds=5.0,
    )
    backend.disconnect()

    goals = [m for m in socket.sent if m["op"] == "send_action_goal"]
    assert [g["id"] for g in goals] == ["adv", "adv#slow1"]
    slower = goals[1]["args"]
    assert slower["speed"] == pytest.approx(0.8 * profile(0.5, 0.5).max_speed(0.15), abs=1e-3)
    assert slower["target"]["x"] == pytest.approx(0.2, abs=1e-6)  # 0.8 asked, 0.6 done
    # The preempted goal's abort did not end the call; the slower goal did.
    assert result.outcome == OUTCOME_COMPLETED
    changes = result.evidence["motion_outcome"]["braking"]["speed_changes"]
    assert [c["speed_mps"] for c in changes][0] == 0.2
    assert changes[1]["speed_mps"] == pytest.approx(slower["speed"], abs=1e-4)


def test_guard_stops_a_drive_whose_lidar_goes_quiet(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_STOP_SCAN_TIMEOUT_S", "0.2")
    socket = ApproachRosbridge(lambda goal: [odom(0.1, 0.12)])
    backend = connected(socket, monkeypatch)
    arm(backend, "adv")
    result = backend.invoke(
        call_id="adv", capability_id="motion.advance",
        arguments={"distance_m": 2.0, "speed_mps": 0.12}, deadline_seconds=5.0,
    )
    backend.disconnect()
    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["motion_outcome"]
    assert outcome["braking"]["tripped"] == mo.GUARD_TRIP_STALE
    assert outcome["reason"] == mo.REASON_SENSOR_STALE


def test_an_unguarded_cancel_is_still_a_cancel(monkeypatch):
    socket = ApproachRosbridge(lambda goal: [])
    backend = connected(socket, monkeypatch)
    backend._handle_message({"op": "action_result", "id": "x", "status": 5})
    assert backend._action_results["x"]["status"] == 5
    backend.disconnect()


# -- the starting speed cap ------------------------------------------------------------


class CappingBackend:
    """Just enough backend for the adapter to cap and arm a straight drive."""

    def __init__(self, sweep):
        from flyto_robotics.generic_ros2_adapter import DEFAULT_INTERFACES, StandardInterface

        kind, name, interface_type = DEFAULT_INTERFACES["motion.advance"]
        self.interfaces = (StandardInterface(kind, name, interface_type),)
        self.sweep = sweep
        self.armed = []
        self.calls = []

    def discover(self):
        return self.interfaces

    def observation(self, required=None):
        return {
            "pose": {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
            "range": {"minimum_range_m": 0.36, "sample_count": 400, "sweep": self.sweep},
            "map_tf_available": True,
        }

    def braking_profile(self):
        return profile(latency=0.5, decel=0.5)

    def arm_braking_guard(self, call_id, capability_id, **kwargs):
        self.armed.append((call_id, capability_id, kwargs))

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        from flyto_robotics.adapter_contract import CallResult

        self.calls.append(dict(arguments))
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={})

    def execution_count(self, call_id):
        return len(self.calls)


def test_a_drive_starts_no_faster_than_its_room_allows():
    backend = CappingBackend(sweep_with({0: 0.50}))  # room 0.15
    device = GenericROS2Adapter(backend=backend, resource_id="t")
    device.describe()
    result = device.invoke(
        CallRequest("adv", "motion.advance", arguments={"distance_m": 0.1, "speed_mps": 0.25})
    )
    assert result.outcome == OUTCOME_COMPLETED
    sent = backend.calls[0]["speed_mps"]
    assert sent == pytest.approx(profile(0.5, 0.5).max_speed(0.15) * 0.8)
    assert sent < 0.25
    assert backend.armed[0][2]["requested_speed_mps"] == 0.25


def test_a_drive_with_no_room_to_stop_is_refused_before_it_moves():
    backend = CappingBackend(sweep_with({0: 0.355}))  # 5 mm beyond the floor
    device = GenericROS2Adapter(backend=backend, resource_id="t")
    device.describe()
    result = device.invoke(CallRequest("adv", "motion.advance", arguments={"distance_m": 0.1}))
    assert result.outcome == OUTCOME_REFUSED
    assert "clearance floor" in result.detail
    assert backend.calls == [] and backend.armed == []
