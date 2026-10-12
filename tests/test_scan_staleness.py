"""How long a guarded straight drive may go without a LiDAR scan.

Live, physical TurtleBot3, 2026-10-12 23:29 (0.8.1, run 88ccbec1): a 2.0 m
``motion.advance`` stopped at 0.612 m with ``sensor_stale``. The adapter log
reads ``braking guard stopping ...: stale {'scan_age_s': 0.965, 'limit_s':
0.96}``: the limit was ``max(0.5, 3 x worst recent scan interval)`` and the
worst interval was 0.32 s. Read-only through the same tunnel afterwards, the
sensor's header stamps step 0.100 s (one 0.199 s) while receipt intervals reach
0.36 s, and every receipt gap over 0.2 s stalls odometry with it: the scans
were late in transport (an overloaded rosbridge on the robot), not missing.

The limit is now derived, not guessed (``braking_envelope.scan_stale_limit``):

* a rate cap: ``SCAN_STALENESS`` missed nominal (median) scan periods, never
  below the configured minimum or 3 x the worst recent interval, never above
  an absolute ceiling;
* a floor bound: the receipt age at which a stop, decided then, still brings
  the robot to rest at or beyond the floor from the room the last scan showed.

These tests pin the numbers, prove the floor bound with the stopping-distance
model, and check the adapter applies it.
"""

from __future__ import annotations

import math
import random
import time

import pytest

from flyto_robotics import braking_envelope as braking
from flyto_robotics import motion_outcome as mo
from flyto_robotics.generic_ros2_adapter import (
    GUARD_KIND_LEG,
    GUARD_KIND_STRAIGHT,
    GUARD_POLL_S,
    SCAN_STALENESS,
)
from tests.test_braking_envelope import ApproachRosbridge, connected, scan

FLOOR = 0.35

#: The live link, 2026-10-12: sensor at 10 Hz, worst receipt interval 0.32 s.
LIVE_NOMINAL_PERIOD_S = 0.1
LIVE_WORST_PERIOD_S = 0.32
LIVE_STALL_S = 0.965


def measured_profile(
    *, period=LIVE_WORST_PERIOD_S, transport=0.05, control=0.1, actuation=0.1,
    configured_latency=braking.DEFAULT_LATENCY_S, decel=braking.DEFAULT_DECEL_MPS2,
):
    budget = braking.LatencyBudget(
        scan_period_s=period,
        scan_transport_s=transport,
        command_transport_s=transport,
        control_period_s=control,
        actuation_s=actuation,
    )
    return braking.resolve_profile(
        floor_m=FLOOR,
        configured_latency_s=configured_latency,
        configured_decel_mps2=decel,
        measured_latency=budget,
    )


def straight_limit(profile, *, room_m, speed_mps, nominal=LIVE_NOMINAL_PERIOD_S,
                   worst=LIVE_WORST_PERIOD_S, configured=0.5):
    return braking.scan_stale_limit(
        SCAN_STALENESS[GUARD_KIND_STRAIGHT],
        configured_s=configured,
        nominal_period_s=nominal,
        worst_period_s=worst,
        profile=profile,
        room_m=room_m,
        speed_mps=speed_mps,
        poll_s=GUARD_POLL_S,
    )


# -- the rule's numbers ------------------------------------------------------------


def test_the_rate_cap_is_missed_periods_of_the_measured_rate_under_a_ceiling():
    policy = SCAN_STALENESS[GUARD_KIND_STRAIGHT]
    limit = straight_limit(measured_profile(), room_m=math.inf, speed_mps=0.12)
    # 15 x 0.1 s = 1.5 s beats 3 x 0.32 = 0.96 s and the configured 0.5 s.
    assert limit.rate_limit_s == pytest.approx(policy.missed_periods * LIVE_NOMINAL_PERIOD_S)
    assert limit.rate_limit_s == pytest.approx(1.5)
    assert limit.limit_s == pytest.approx(1.5)
    assert limit.basis == braking.STALE_BASIS_RATE
    # A slow sensor never stretches it past the absolute ceiling.
    slow = straight_limit(measured_profile(), room_m=math.inf, speed_mps=0.12,
                          nominal=0.5, worst=0.9)
    assert slow.rate_limit_s == pytest.approx(policy.ceiling_s) == pytest.approx(2.0)


def test_without_a_measured_rate_the_configured_minimum_holds():
    limit = straight_limit(measured_profile(), room_m=math.inf, speed_mps=0.12,
                           nominal=None, worst=None, configured=0.2)
    assert limit.limit_s == pytest.approx(0.2)


def test_the_live_stall_with_room_ahead_is_tolerated():
    # Mid-drive, 1 m to the floor at 0.12 m/s: the stall is 0.965 s.
    limit = straight_limit(measured_profile(), room_m=1.0, speed_mps=0.12)
    assert limit.limit_s > LIVE_STALL_S
    assert limit.basis == braking.STALE_BASIS_RATE


def test_the_floor_bound_at_0_1_mps_is_the_user_worked_example():
    # At 0.1 m/s the robot covers 0.035 m in 0.35 s. With 0.035 m of room
    # beyond its stopping distance it may be blind for the scan period the
    # latency already allows (credit) plus 0.35 s, less one guard poll.
    profile = measured_profile()
    v = 0.1
    room = profile.stopping_distance(v) + 0.035
    limit = straight_limit(profile, room_m=room, speed_mps=v)
    credit = braking.period_credit_s(profile)
    assert credit == pytest.approx(LIVE_WORST_PERIOD_S)
    assert limit.floor_limit_s == pytest.approx(credit + 0.35 - GUARD_POLL_S)
    assert limit.basis == braking.STALE_BASIS_FLOOR
    # And the same stall the live robot had is refused there: it would not
    # be able to stop at the floor.
    assert limit.limit_s < LIVE_STALL_S


def test_a_navigation_leg_keeps_its_previous_rule():
    policy = SCAN_STALENESS[GUARD_KIND_LEG]
    limit = braking.scan_stale_limit(
        policy, configured_s=0.5, nominal_period_s=0.1, worst_period_s=0.32,
        profile=measured_profile(), room_m=None, speed_mps=0.0, poll_s=GUARD_POLL_S,
    )
    assert limit.limit_s == pytest.approx(max(0.5, 3 * 0.32))
    assert limit.floor_limit_s is None


def test_the_credit_never_exceeds_what_the_latency_holds():
    # A clamped latency (3.0 s ceiling) holds less than the measured total:
    # the credit shrinks so latency - credit still covers every other term.
    profile = measured_profile(period=0.3, transport=1.0, control=1.0, actuation=1.0)
    assert profile.latency_s == pytest.approx(braking.LATENCY_BOUNDS_S[1])
    rest = 1.0 + 1.0 + 1.0 + 1.0
    assert braking.period_credit_s(profile) == pytest.approx(max(0.0, 3.0 - rest))
    configured_only = braking.resolve_profile(floor_m=FLOOR)
    assert braking.period_credit_s(configured_only) == 0.0


# -- the proof: the floor holds whatever the stall --------------------------------------


def rest_travel(*, v, a, transport_in, decided_age, transport_out, control, actuation):
    """Distance from where the last scan was taken to where the robot rests.

    The robot keeps ``v`` from the scan's capture until the stop takes
    effect (scan delivery, the receipt age the stop is decided at, the stop's
    way back, the robot's control cycle and motor response), then brakes at
    ``a``.
    """
    held = transport_in + decided_age + transport_out + control + actuation
    return v * held + v * v / (2.0 * a)


def test_a_stop_at_the_stale_limit_rests_at_or_beyond_the_floor():
    rng = random.Random(20261012)
    checked = 0
    for _ in range(20000):
        period = rng.uniform(0.05, 0.6)
        transport = rng.uniform(0.0, 0.3)
        control = rng.uniform(0.0, 0.2)
        actuation = rng.uniform(0.0, 0.2)
        configured = rng.uniform(0.05, 1.5)
        decel = rng.uniform(0.1, 2.5)
        profile = measured_profile(
            period=period, transport=transport, control=control, actuation=actuation,
            configured_latency=configured, decel=decel,
        )
        v = rng.uniform(0.02, 0.5)
        stopping = profile.stopping_distance(v)
        # A room inside the stopping distance trips on clearance at receipt.
        room = stopping + rng.uniform(0.0, 1.0)
        limit = straight_limit(
            profile, room_m=room, speed_mps=v,
            nominal=rng.uniform(0.02, period), worst=period,
            configured=rng.uniform(0.1, 1.0),
        )
        # The guard decides at its first poll past the limit (or at once).
        decided = max(0.0, limit.limit_s) + GUARD_POLL_S
        travel = rest_travel(
            v=v, a=profile.decel_mps2, transport_in=transport, decided_age=decided,
            transport_out=transport, control=control, actuation=actuation,
        )
        if limit.limit_s >= 0.0:
            assert travel <= room + 1e-9, (period, transport, v, room, limit)
            checked += 1
    assert checked > 15000


@pytest.mark.parametrize("speed", [0.05, 0.10, 0.12, 0.15, 0.22])
def test_the_live_link_never_lets_the_floor_be_breached(speed):
    profile = measured_profile()
    for room in (0.0, 0.01, 0.05, 0.1, 0.2, 0.5, 1.0, 3.0):
        room = profile.stopping_distance(speed) + room
        limit = straight_limit(profile, room_m=room, speed_mps=speed)
        travel = rest_travel(
            v=speed, a=profile.decel_mps2, transport_in=0.05,
            decided_age=limit.limit_s + GUARD_POLL_S, transport_out=0.05,
            control=0.1, actuation=0.1,
        )
        assert travel <= room + 1e-9


# -- through the adapter ------------------------------------------------------------------


def stalled_backend(monkeypatch, *, front_m):
    """A rosbridge backend whose last scan showed ``front_m`` ahead, with the
    live link's interval history (49 scans at 0.1 s and one at 0.32 s)."""
    backend = connected(ApproachRosbridge(lambda goal: []), monkeypatch)
    backend._handle_message(scan(front_m))
    with backend._condition:
        backend._latency.reset()
        t = 100.0
        backend._latency.observe(t)
        for index in range(50):
            t += LIVE_WORST_PERIOD_S if index == 25 else LIVE_NOMINAL_PERIOD_S
            backend._latency.observe(t, 0.05)
    profile = backend.braking_profile()
    backend.arm_braking_guard(
        "adv", "motion.advance", profile=profile, speed_mps=0.12,
        requested_speed_mps=0.12, distance_m=2.0, start_room_m=front_m - FLOOR,
    )
    with backend._condition:
        backend._check_guards()
    return backend


def stall(backend, seconds):
    with backend._condition:
        backend._range_seen_at = time.monotonic() - seconds
    backend._check_scan_fresh("adv")
    return backend._guard("adv")


def test_the_live_stall_with_room_ahead_does_not_stop_the_drive(monkeypatch):
    backend = stalled_backend(monkeypatch, front_m=FLOOR + 1.0)
    guard = stall(backend, LIVE_STALL_S)
    backend.disconnect()
    assert guard.tripped is None


def test_a_stall_close_to_the_floor_still_stops_the_drive(monkeypatch):
    backend = stalled_backend(monkeypatch, front_m=FLOOR + 0.12)
    guard = stall(backend, LIVE_STALL_S)
    backend.disconnect()
    assert guard.tripped == mo.GUARD_TRIP_STALE
    trip = guard.trip
    assert trip["basis"] == braking.STALE_BASIS_FLOOR
    assert trip["scan_age_s"] >= trip["limit_s"]
    assert trip["room_m"] == pytest.approx(0.12, abs=1e-3)
    assert trip["speed_mps"] == pytest.approx(0.12)
    for key in ("rate_limit_s", "floor_limit_s", "nominal_period_s", "worst_period_s"):
        assert key in trip


def test_a_lidar_that_never_comes_back_stops_the_drive_at_the_ceiling(monkeypatch):
    backend = stalled_backend(monkeypatch, front_m=8.0)
    guard = stall(backend, SCAN_STALENESS[GUARD_KIND_STRAIGHT].ceiling_s + 0.01)
    backend.disconnect()
    assert guard.tripped == mo.GUARD_TRIP_STALE
    assert guard.trip["basis"] == braking.STALE_BASIS_RATE


def test_no_scan_at_all_stops_the_drive(monkeypatch):
    backend = stalled_backend(monkeypatch, front_m=8.0)
    with backend._condition:
        backend._range_seen_at = None
    backend._check_scan_fresh("adv")
    guard = backend._guard("adv")
    backend.disconnect()
    assert guard.tripped == mo.GUARD_TRIP_STALE
    assert guard.trip["scan_age_s"] is None
