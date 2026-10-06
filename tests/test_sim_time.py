"""How fast a simulator ran during a motion: /clock against the wall.

Twin 2026-10-06 22:01 local: every phase of a detour took 1.8-2.4x its usual
wall time (BackUp 9.1 s against ~5 s for the same 0.20 m) with no stall and no
recovery. These records name that case in the result instead of leaving it to
be reconstructed from Nav2's logs.
"""

from __future__ import annotations

import time

from test_inflation_escape import OBSERVED, EscapeBackend, make, navigate
from test_presence_on_state import RecordingSocket, no_sleep  # noqa: F401

from flyto_robotics import motion_outcome, sim_time
from flyto_robotics.adapter_contract import CallResult
from flyto_robotics.generic_ros2_adapter import CLOCK_TYPE, RosbridgeROS2Backend


def _clock(sim_s: float) -> dict:
    whole = int(sim_s)
    return {"clock": {"sec": whole, "nanosec": int(round((sim_s - whole) * 1e9))}}


def _record(rates: list[tuple[float, float]], step: float = 0.1) -> sim_time.ClockRecord:
    """Samples every ``step`` wall seconds; ``rates`` is [(wall seconds, factor)]."""
    record = sim_time.ClockRecord("/clock")
    wall = sim = 0.0
    record.add(sim, wall)
    for span, factor in rates:
        for _ in range(round(span / step)):
            wall += step
            sim += step * factor
            record.add(sim, wall)
    return record


def test_a_steady_twin_is_measured_and_not_slow():
    summary = _record([(10.0, 0.7)]).summary(threshold=0.5)
    assert summary["measured"] and summary["applicable"]
    assert summary["rtf_mean"] == 0.7 and summary["rtf_min"] == 0.7
    assert summary["sim_slow"] is False
    assert summary["wall_s"] == 10.0 and summary["sim_s"] == 7.0
    assert summary["sim_slow_threshold"] == 0.5 and summary["basis"]


def test_a_slow_spell_sets_sim_slow_on_the_minimum_window():
    # Mostly 0.7, three seconds at 0.35 (the 22:01 run's ~2x stretch).
    summary = _record([(5.0, 0.7), (3.0, 0.35), (5.0, 0.7)]).summary(threshold=0.5)
    assert summary["rtf_min"] == 0.35
    assert 0.35 < summary["rtf_mean"] < 0.7
    assert summary["sim_slow"] is True


def test_too_few_samples_and_a_clock_reset_are_said():
    record = sim_time.ClockRecord("/clock")
    record.add(5.0, 1.0)
    assert record.summary()["measured"] is False
    record.add(6.0, 2.0)
    record.add(0.5, 3.0)  # the simulator was reset
    record.add(1.0, 4.0)
    summary = record.summary(threshold=0.5)
    assert summary["clock_reset_seen"] is True
    assert summary["sim_s"] == 0.5 and summary["wall_s"] == 1.0
    record.add(2.0, 3.5)  # wall time going back is ignored
    assert record.samples[-1] == (1.0, 4.0)


def test_clock_messages_and_the_threshold_are_read_strictly(monkeypatch):
    assert sim_time.clock_seconds(_clock(12.25)) == 12.25
    assert sim_time.clock_seconds({"clock": "x"}) is None
    assert sim_time.clock_seconds(None) is None
    assert sim_time.slow_threshold() == sim_time.SIM_SLOW_RTF
    monkeypatch.setenv("FLYTO_ROS2_SIM_SLOW_RTF", "0.3")
    assert sim_time.slow_threshold() == 0.3
    for bad in ("0", "1.5", "nan", "fast"):
        monkeypatch.setenv("FLYTO_ROS2_SIM_SLOW_RTF", bad)
        assert sim_time.slow_threshold() == sim_time.SIM_SLOW_RTF


def test_legs_combine_into_one_record_for_the_call():
    fast = _record([(5.0, 0.7)]).summary(threshold=0.5)
    slow = _record([(10.0, 0.35)]).summary(threshold=0.5)
    combined = sim_time.combine([fast, slow], threshold=0.5)
    assert combined["legs_measured"] == 2
    assert combined["rtf_min"] == 0.35 and combined["sim_slow"] is True
    assert combined["rtf_mean"] == round((3.5 + 3.5) / 15.0, 3)
    real = sim_time.combine([sim_time.not_applicable()])
    assert real == {"applicable": False, "reason": sim_time.NOT_APPLICABLE_REAL}


def _summary(track):
    return motion_outcome.summarize(
        track, status=4, result_values=None, end_pose=None, range_observation=None,
        clearance_floor_m=0.35, ended_at=track.started_at + 1.0,
    )


def _track(**fields):
    return motion_outcome.MotionTrack(
        capability_id="motion.navigate", arguments={}, start_pose=None, started_at=0.0,
        **fields,
    )


def test_a_real_motion_says_not_applicable_and_nothing_else():
    assert _summary(_track())["sim_time"] == {
        "applicable": False,
        "reason": "real deployment: no simulator clock",
    }


def test_a_simulated_motion_on_a_transport_without_the_clock_says_so():
    note = {"applicable": True, "measured": False, "reason": "no clock here"}
    assert _summary(_track(sim_time_note=note))["sim_time"] == note


def _backend(sockets):
    def connect(_url):
        sockets.append(RecordingSocket({}))
        return sockets[-1]

    return RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=connect)


def test_a_simulation_reads_clock_into_each_running_motion(monkeypatch, no_sleep):  # noqa: F811
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "simulation")
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "/clock")
    sockets: list[RecordingSocket] = []
    backend = _backend(sockets)
    try:
        clock = [item for item in sockets[-1].subscribed if item["topic"] == "/clock"]
        assert len(clock) == 1
        assert clock[0]["type"] == CLOCK_TYPE
        assert clock[0]["throttle_rate"] == sim_time.CLOCK_THROTTLE_MS
        backend._begin_track("nav-1", "motion.navigate", {"x": 1.0, "y": 0.0})
        track = backend._motion_tracks["nav-1"]
        assert track.clock is not None and track.clock.topic == "/clock"
        for sim in (100.0, 100.07, 100.14):
            backend._handle_message({"op": "publish", "topic": "/clock", "msg": _clock(sim)})
            time.sleep(0.01)
        assert len(track.clock.samples) == 3
        assert track.sim_time_summary()["measured"] is True
    finally:
        backend.disconnect()


def test_real_hardware_neither_reads_clock_nor_measures(monkeypatch, no_sleep):  # noqa: F811
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    sockets: list[RecordingSocket] = []
    backend = _backend(sockets)
    try:
        assert all(item["topic"] != "/clock" for item in sockets[-1].subscribed)
        backend._begin_track("nav-1", "motion.navigate", {"x": 1.0, "y": 0.0})
        track = backend._motion_tracks["nav-1"]
        backend._handle_message({"op": "publish", "topic": "/clock", "msg": _clock(1.0)})
        assert track.clock is None
        assert track.sim_time_summary()["applicable"] is False
    finally:
        backend.disconnect()


class _TimedEscapeBackend(EscapeBackend):
    """Each leg reports the simulator clock its motion saw."""

    def __init__(self, segments, factors):
        super().__init__(segments)
        self.factors = dict(factors)

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        result = super().invoke(
            call_id=call_id, capability_id=capability_id,
            arguments=arguments, deadline_seconds=deadline_seconds,
        )
        leg = call_id.rsplit(":", 1)[-1] if ":" in call_id else "goal"
        summary = _record([(4.0, self.factors[leg])]).summary(threshold=0.5)
        return CallResult(
            result.call_id, result.outcome,
            evidence={**dict(result.evidence or {}), "motion_outcome": {"sim_time": summary}},
            detail=result.detail,
        )


def test_an_escape_reports_the_clock_per_leg_and_for_the_call(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    monkeypatch.delenv("FLYTO_ROS2_INFLATION_ESCAPE", raising=False)
    backend = _TimedEscapeBackend(
        OBSERVED, {"escape-backoff": 0.7, "escape-waypoint": 0.35, "goal": 0.7}
    )
    escape = navigate(make(backend)).evidence["navigation_escape"]
    by_leg = {leg["leg"]: leg["sim_time"] for leg in escape["legs"]}
    assert by_leg["waypoint"]["sim_slow"] is True
    assert by_leg["goal"]["sim_slow"] is False
    assert escape["sim_time"]["legs_measured"] == 3
    assert escape["sim_time"]["rtf_min"] == 0.35
    assert escape["sim_time"]["sim_slow"] is True
