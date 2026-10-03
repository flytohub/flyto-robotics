"""A motion result says why the motion ended, not only that it did."""

from __future__ import annotations

import json
import math
import queue

import pytest

from flyto_robotics import motion_outcome as mo
from flyto_robotics.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_TIMEOUT,
)
from flyto_robotics.generic_ros2_adapter import RosbridgeROS2Backend

FLOOR = 0.35


def sweep(front: float, rear: float = 2.0, sides: float = 2.0) -> dict:
    """Eight bins from straight ahead, counter-clockwise."""
    ranges = [front, sides, sides, sides, rear, sides, sides, sides]
    return {"angle_min_rad": 0.0, "angle_increment_rad": math.tau / 8, "ranges_m": ranges}


def track(capability_id="motion.advance", **arguments):
    return mo.MotionTrack(
        capability_id=capability_id,
        arguments=arguments or {"distance_m": 0.30},
        start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        started_at=0.0,
    )


def summarize(tracked, *, status, values=None, end=(0.12, 0.0, 0.0), reading=None):
    x, y, yaw = end
    return mo.summarize(
        tracked,
        status=status,
        result_values=values,
        end_pose={"x": x, "y": y, "yaw": yaw},
        range_observation=reading,
        clearance_floor_m=FLOOR,
        ended_at=2.5,
    )


def test_collision_ahead_is_obstacle_blocked_with_distance_and_range():
    summary = summarize(
        track(),
        status=6,
        values={"error_code": 723, "error_msg": "Collision Ahead"},
        reading={"minimum_range_m": 0.24, "sweep": sweep(front=0.24)},
    )
    assert summary["reason"] == mo.REASON_OBSTACLE_BLOCKED
    assert summary["distance_travelled_m"] == pytest.approx(0.12)
    assert summary["requested_distance_m"] == 0.30
    assert summary["minimum_range_at_stop_m"] == 0.24
    assert summary["travel_direction_range_m"] == 0.24
    assert summary["final_pose"] == {"x": 0.12, "y": 0.0, "yaw": 0.0}
    assert summary["action_status_name"] == "aborted"
    line = mo.describe(summary)
    assert "obstacle_blocked" in line and "travelled 0.120 of 0.300 m" in line
    assert "0.240 m" in line and "floor 0.350 m" in line


def test_a_return_ahead_inside_the_floor_is_an_obstacle_even_without_a_code():
    summary = summarize(
        track(), status=6, reading={"minimum_range_m": 0.30, "sweep": sweep(front=0.30)}
    )
    assert summary["reason"] == mo.REASON_OBSTACLE_BLOCKED


def test_a_wall_behind_an_advance_is_not_in_its_way():
    summary = summarize(
        track(), status=6, reading={"minimum_range_m": 0.20, "sweep": sweep(front=1.5, rear=0.20)}
    )
    assert summary["travel_direction_range_m"] == 1.5
    assert summary["reason"] == mo.REASON_ABORTED_BY_SERVER


def test_a_wall_behind_blocks_a_retreat():
    summary = summarize(
        track("motion.retreat", distance_m=0.2),
        status=6,
        reading={"minimum_range_m": 0.20, "sweep": sweep(front=1.5, rear=0.20)},
    )
    assert summary["reason"] == mo.REASON_OBSTACLE_BLOCKED


def test_collision_monitor_polygon_stop_is_an_obstacle_but_invalid_source_is_stale():
    blocked = track()
    blocked.saw_collision_state(mo.COLLISION_STOP, "PolygonStop")
    assert summarize(blocked, status=6)["reason"] == mo.REASON_OBSTACLE_BLOCKED

    blind = track()
    blind.saw_collision_state(mo.COLLISION_STOP, mo.INVALID_SOURCE)
    blind.saw_collision_state(0, "")
    summary = summarize(blind, status=6, values={"error_code": 721})
    assert summary["reason"] == mo.REASON_SENSOR_STALE
    assert summary["collision_monitor"] == [{"action_type": 1, "polygon": "invalid source"}]


@pytest.mark.parametrize(
    ("code", "reason"),
    [
        (721, mo.REASON_TIMEOUT),
        (701, mo.REASON_TIMEOUT),
        (107, mo.REASON_TIMEOUT),
        (722, mo.REASON_LOCALIZATION_ERROR),
        (208, mo.REASON_NO_PATH),
        (206, mo.REASON_NO_PATH),
        (105, mo.REASON_NO_PROGRESS),
        (0, mo.REASON_ABORTED_BY_SERVER),
        (999, mo.REASON_ABORTED_BY_SERVER),
    ],
)
def test_nav2_error_codes_map_to_reasons(code, reason):
    summary = summarize(track(), status=6, values={"error_code": code})
    assert summary["reason"] == reason


def test_success_cancel_and_adapter_deadline():
    assert summarize(track(), status=4)["reason"] == mo.REASON_COMPLETED
    assert summarize(track(), status=5)["reason"] == mo.REASON_CANCELLED
    running = summarize(track(), status=None)
    assert running["reason"] == mo.REASON_TIMEOUT
    assert running["action_status_name"] == "still_running"
    assert mo.describe(running).startswith("ROS 2 action still running")
    assert summarize(track(), status=2)["reason"] == mo.REASON_UNKNOWN


def test_rotate_reports_yaw_and_navigate_reports_its_goal():
    turned = summarize(
        track("motion.rotate", yaw_radians=-1.0), status=6, end=(0.0, 0.0, -0.4)
    )
    assert turned["yaw_turned_rad"] == pytest.approx(0.4)
    assert turned["requested_yaw_rad"] == 1.0
    assert "distance_travelled_m" not in turned
    goal = summarize(track("motion.navigate", x=1.0, y=2.0), status=6)
    assert goal["requested_goal"] == {"x": 1.0, "y": 2.0}


def test_unknown_pose_is_reported_as_unknown_not_zero():
    tracked = mo.MotionTrack("motion.advance", {"distance_m": 0.3}, None, 0.0)
    summary = mo.summarize(
        tracked, status=6, result_values=None, end_pose=None, range_observation=None,
        clearance_floor_m=FLOOR, ended_at=1.0,
    )
    assert summary["start_pose"] is None and "distance_travelled_m" not in summary
    assert summary["minimum_range_at_stop_m"] is None


class ScriptedRosbridge:
    """A rosbridge socket that answers a motion goal with a scripted run."""

    def __init__(self, run):
        self.inbox: queue.Queue = queue.Queue()
        self.sent: list[dict] = []
        self.run = run

    def send(self, raw):
        message = json.loads(raw)
        self.sent.append(message)
        if message.get("op") == "send_action_goal":
            for reply in self.run(message["id"]):
                self.inbox.put(json.dumps(reply))
        elif message.get("op") == "cancel_action_goal":
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


def odom(x):
    return {
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "pose": {"pose": {"position": {"x": x, "y": 0.0},
                              "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
            "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
        },
    }


def scan(front):
    ranges = [front] * 10 + [2.0] * 80 + [2.0] * 10
    return {
        "op": "publish",
        "topic": "/scan",
        "msg": {"ranges": ranges, "range_min": 0.1, "range_max": 8.0,
                "angle_min": 0.0, "angle_increment": math.tau / 100},
    }


def backend_with(run, monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.25")
    socket = ScriptedRosbridge(run)
    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=lambda _u: socket)
    for message in (odom(0.0), scan(1.0)):
        backend._handle_message(message)
    return backend, socket


def test_rosbridge_abort_carries_the_reason(monkeypatch):
    def blocked(call_id):
        return [
            {"op": "action_feedback", "id": call_id, "values": {"distance_traveled": 0.11}},
            {"op": "publish", "topic": "/collision_monitor_state",
             "msg": {"action_type": 1, "polygon_name": "PolygonStop"}},
            odom(0.115),
            scan(0.22),
            {"op": "action_result", "id": call_id, "status": 6, "result": False,
             "values": {"error_code": 723,
                        "error_msg": "Collision Ahead - Exiting DriveOnHeading"}},
        ]

    backend, socket = backend_with(blocked, monkeypatch)
    result = backend.invoke(
        call_id="adv-1", capability_id="motion.advance",
        arguments={"distance_m": 0.3}, deadline_seconds=3.0,
    )
    backend.disconnect()
    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["motion_outcome"]
    assert outcome["reason"] == "obstacle_blocked"
    assert outcome["error_code"] == 723
    assert outcome["distance_travelled_m"] == pytest.approx(0.115)
    assert outcome["requested_distance_m"] == 0.3
    assert outcome["minimum_range_at_stop_m"] == pytest.approx(0.22)
    assert outcome["feedback"] == {"distance_traveled": pytest.approx(0.11)}
    assert outcome["collision_monitor"] == [{"action_type": 1, "polygon": "PolygonStop"}]
    assert result.detail.startswith("ROS 2 action status 6 (aborted): obstacle_blocked")
    subscribed = {m["topic"] for m in socket.sent if m.get("op") == "subscribe"}
    assert "/collision_monitor_state" in subscribed
    # The motion is over: nothing is left tracked for it.
    assert backend._motion_tracks == {}


def test_rosbridge_success_and_cancel_carry_an_outcome(monkeypatch):
    def finished(call_id):
        return [odom(0.3), {"op": "action_result", "id": call_id, "status": 4, "result": True,
                            "values": {"error_code": 0}}]

    backend, _ = backend_with(finished, monkeypatch)
    result = backend.invoke(call_id="adv-2", capability_id="motion.advance",
                            arguments={"distance_m": 0.3}, deadline_seconds=3.0)
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence["motion_outcome"]["reason"] == "completed"
    assert result.evidence["motion_outcome"]["distance_travelled_m"] == pytest.approx(0.3)
    assert result.detail == ""
    backend.disconnect()

    def cancelled(call_id):
        return [{"op": "action_result", "id": call_id, "status": 5, "result": False}]

    backend, _ = backend_with(cancelled, monkeypatch)
    result = backend.invoke(call_id="adv-3", capability_id="motion.advance",
                            arguments={"distance_m": 0.3}, deadline_seconds=3.0)
    assert result.outcome == OUTCOME_CANCELLED and result.detail == "cancelled"
    assert result.evidence["motion_outcome"]["reason"] == "cancelled"
    backend.disconnect()


def test_rosbridge_deadline_reports_timeout_and_keeps_tracking(monkeypatch):
    backend, _ = backend_with(lambda _id: [odom(0.05)], monkeypatch)
    result = backend.invoke(call_id="adv-4", capability_id="motion.advance",
                            arguments={"distance_m": 0.3}, deadline_seconds=0.3)
    assert result.outcome == OUTCOME_TIMEOUT
    assert result.evidence["motion_outcome"]["reason"] == "timeout"
    assert result.detail.startswith("ROS 2 action still running")
    assert "adv-4" in backend._motion_tracks
    # An emergency stop forgets the goal, and with it the track.
    backend._topic_types["/cmd_vel"] = "geometry_msgs/msg/Twist"
    assert backend.safe_stop("stop-1").outcome == OUTCOME_COMPLETED
    assert backend._motion_tracks == {}
    backend.disconnect()
