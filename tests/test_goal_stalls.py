"""A goal that stands still says what was commanded meanwhile.

Twin 2026-10-06 (t-f044dad913d21612): the goal leg stood still about two
minutes while Nav2's controller ran and its progress checker failed eight
times; no collision-monitor action and no gap between legs were seen. The
logs could not say whether the controller commanded nothing or commanded
motion that never reached the base. These records make the next one say.
"""

from __future__ import annotations

import json
import time

from test_presence_on_state import RecordingSocket, no_sleep  # noqa: F401

import flyto_robotics.generic_ros2_adapter as adapter_module
from flyto_robotics import motion_outcome
from flyto_robotics.generic_ros2_adapter import RosbridgeROS2Backend


def _track(started_at: float = 100.0) -> motion_outcome.MotionTrack:
    return motion_outcome.MotionTrack(
        capability_id="motion.navigate",
        arguments={"x": 1.2, "y": 0.0},
        start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        started_at=started_at,
    )


def _summary(track, ended_at):
    return motion_outcome.summarize(
        track,
        status=4,
        result_values=None,
        end_pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        range_observation=None,
        clearance_floor_m=0.35,
        ended_at=ended_at,
    )


def test_a_still_spell_with_near_zero_commands_is_recorded_as_a_stall():
    track = _track()
    track.saw_base((0.2, 0.1), 101.0)
    track.saw_base((0.0, 0.026), 102.0, {"x": -0.128, "y": 0.388, "yaw": 0.48})
    for step in range(20):
        track.saw_command("/cmd_vel_nav", 0.0, 0.02 if step % 2 else -0.02)
        track.saw_command("/cmd_vel", 0.0, 0.0)
    track.saw_base((0.0, 0.01), 110.0)
    track.saw_base((0.25, 0.0), 112.5)
    stalls = _summary(track, 120.0)["stalls"]
    assert stalls == [
        {
            "at_s": 2.0,
            "duration_s": 10.5,
            "commanded": {
                "/cmd_vel": {"samples": 20, "max_linear_mps": 0.0, "max_angular_radps": 0.0},
                "/cmd_vel_nav": {
                    "samples": 20,
                    "max_linear_mps": 0.0,
                    "max_angular_radps": 0.02,
                },
            },
            "pose": {"x": -0.128, "y": 0.388, "yaw": 0.48},
        }
    ]


def test_commands_while_moving_and_short_pauses_are_not_stalls():
    track = _track()
    track.saw_command("/cmd_vel_nav", 0.3, 0.1)  # moving: not kept
    track.saw_base((0.0, 0.0), 101.0)
    track.saw_base((0.2, 0.0), 101.0 + motion_outcome.STALL_MIN_S / 2)
    assert "stalls" not in _summary(track, 105.0)


def test_a_stall_still_open_when_the_goal_ends_is_closed_by_the_summary():
    track = _track()
    track.saw_base((0.0, 0.0), 101.0)
    track.saw_command("/cmd_vel_nav", 0.3, 0.0)
    summary = _summary(track, 105.0)
    assert summary["stalls"][0]["duration_s"] == 4.0
    assert summary["stalls"][0]["commanded"]["/cmd_vel_nav"]["max_linear_mps"] == 0.3


def test_recoveries_are_timed_when_their_count_rises():
    track = _track()
    track.saw_feedback({"number_of_recoveries": 0.0}, 101.0)
    track.saw_feedback({"number_of_recoveries": 1.0}, 111.6)
    track.saw_feedback({"number_of_recoveries": 1.0}, 112.0)
    track.saw_feedback({"number_of_recoveries": 2.0}, 123.3)
    assert _summary(track, 130.0)["recoveries_at_s"] == [11.6, 23.3]


def test_twist_and_stamped_twist_are_both_read():
    twist = {"linear": {"x": 0.3, "y": 0.0, "z": 0.0}, "angular": {"x": 0, "y": 0, "z": -0.5}}
    assert adapter_module._twist_command(twist) == (0.3, -0.5)
    assert adapter_module._twist_command({"header": {}, "twist": twist}) == (0.3, -0.5)
    assert adapter_module._twist_command({"twist": {}}) is None
    assert adapter_module._twist_command(None) is None


def test_watched_topics_come_from_the_environment(monkeypatch):
    assert adapter_module._command_watch_topics() == ("/cmd_vel_nav", "/cmd_vel")
    monkeypatch.setenv("FLYTO_ROS2_COMMAND_WATCH_TOPICS", "")
    assert adapter_module._command_watch_topics() == ()


def test_the_rosbridge_backend_reads_the_watched_topics_into_a_running_goal(no_sleep):  # noqa: F811
    sockets: list[RecordingSocket] = []

    def connect(_url):
        sockets.append(RecordingSocket({}))
        return sockets[-1]

    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=connect)
    try:
        watched = {
            item["topic"]: item
            for item in sockets[-1].subscribed
            if item["topic"] in ("/cmd_vel_nav", "/cmd_vel")
        }
        assert set(watched) == {"/cmd_vel_nav", "/cmd_vel"}
        for item in watched.values():
            # Sampled, and typed by the graph (stamped or not), never written.
            assert item["throttle_rate"] == adapter_module.COMMAND_WATCH_THROTTLE_MS
            assert "type" not in item
        track = _track(started_at=time.monotonic())
        backend._motion_tracks["nav-1"] = track
        backend._store_odometry({"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
                                (0.0, 0.0), time.monotonic())
        backend._handle_message(
            json.loads(json.dumps({
                "op": "publish",
                "topic": "/cmd_vel_nav",
                "msg": {"header": {}, "twist": {"linear": {"x": 0.0}, "angular": {"z": 0.03}}},
            }))
        )
        assert track._still_commands["/cmd_vel_nav"].samples == 1
    finally:
        backend.disconnect()


def test_escape_legs_carry_their_timing(monkeypatch):
    from test_inflation_escape import OBSERVED, EscapeBackend, make, navigate

    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    monkeypatch.delenv("FLYTO_ROS2_INFLATION_ESCAPE", raising=False)
    leg_keys = ("leg", "outcome", "started_at_s", "elapsed_s")
    result = navigate(make(EscapeBackend(OBSERVED)))
    legs = result.evidence["navigation_escape"]["legs"]
    assert [leg["leg"] for leg in legs] == ["backoff", "waypoint", "goal"]
    for leg in legs:
        assert tuple(leg) == leg_keys
        assert leg["elapsed_s"] >= 0.0
    starts = [leg["started_at_s"] for leg in legs]
    assert starts == sorted(starts)
