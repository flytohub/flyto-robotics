"""Travelling around obstacles, and reading one photo or map, through standard ROS 2."""

from __future__ import annotations

import base64
import math

import pytest

from flyto_robotics.adapter_contract import OUTCOME_COMPLETED, OUTCOME_REFUSED, CallRequest
from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    RosbridgeROS2Backend,
    map_capture,
    photo_capture,
)
from tests.test_generic_ros2_adapter import adapter

JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 16 + b"\xff\xd9"


def test_travel_sends_a_planned_goal_the_asked_distance_ahead_in_odom():
    device = adapter({"motion.travel"})
    device.backend.observation_payload["pose"] = {
        "frame": "odom", "x": 1.0, "y": 2.0, "yaw": math.pi / 2,
    }
    device.describe()
    result = device.invoke(CallRequest("travel", "motion.travel", arguments={"distance_m": 1.2}))
    assert result.outcome == OUTCOME_COMPLETED
    [(_, capability_id, arguments)] = device.backend.calls
    assert capability_id == "motion.travel"
    assert arguments["x"] == pytest.approx(1.0)
    assert arguments["y"] == pytest.approx(3.2)
    assert arguments["yaw_radians"] == pytest.approx(math.pi / 2)
    assert arguments["in_map"] == 0.0


def test_travel_needs_the_map_like_navigation():
    device = adapter({"motion.travel"})
    device.backend.observation_payload["map_tf_available"] = False
    device.describe()
    result = device.invoke(CallRequest("travel", "motion.travel", arguments={"distance_m": 1.0}))
    assert result.outcome == OUTCOME_REFUSED
    assert device.backend.calls == []


def test_travel_is_refused_when_only_a_person_keeps_it_safe(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "operator_present")
    device = adapter({"motion.travel"})
    device.describe()
    result = device.invoke(CallRequest("travel", "motion.travel", arguments={"distance_m": 0.2}))
    assert result.outcome == OUTCOME_REFUSED
    assert "LiDAR" in result.detail


def test_the_rosbridge_travel_goal_is_in_the_odometry_frame():
    travel = {"x": 1.0, "y": 0.0, "yaw_radians": 0.0}
    goal = RosbridgeROS2Backend._action_goal(None, "motion.travel", travel)
    assert goal["pose"]["header"]["frame_id"] == "odom"
    navigate = RosbridgeROS2Backend._action_goal(None, "motion.navigate", {"x": 1.0, "y": 0.0})
    assert navigate["pose"]["header"]["frame_id"] == "map"


def test_a_photo_must_be_a_jpeg():
    frame = {"format": "rgb8; jpeg compressed bgr8", "data": base64.b64encode(JPEG).decode()}
    payload = photo_capture(frame)
    assert base64.b64decode(payload["data_base64"]) == JPEG
    with pytest.raises(ValueError):
        photo_capture({"data": base64.b64encode(b"\x89PNG").decode()})


def test_a_map_is_its_cells_with_size_and_origin():
    message = {
        "info": {"width": 2, "height": 2, "resolution": 0.05,
                 "origin": {"position": {"x": -1.0, "y": -2.0}}},
        "data": [0, 100, -1, 50],
    }
    payload = map_capture(message)
    assert (payload["width"], payload["height"], payload["resolution_m"]) == (2, 2, 0.05)
    assert payload["origin"] == {"x": -1.0, "y": -2.0}
    assert base64.b64decode(payload["cells_base64"]) == bytes([0, 100, 255, 50])
    with pytest.raises(ValueError):
        map_capture({**message, "data": [0]})


def test_capture_topics_are_declared_when_present():
    device = adapter({"vision.observe", "sensing.map"})
    declared = {item.capability_id for item in device.describe()}
    assert {"vision.observe", "sensing.map"} <= declared
    assert DEFAULT_INTERFACES["sensing.map"][2] == "nav_msgs/msg/OccupancyGrid"


def test_capture_without_rosbridge_is_refused_not_faked():
    device = adapter({"vision.observe"})
    device.describe()
    result = device.invoke(CallRequest("photo", "vision.observe", arguments={}))
    assert result.outcome == OUTCOME_REFUSED
    assert "rosbridge" in result.detail


class PongRecorder:
    def __init__(self):
        self.pongs = 0
        self.closed = False

    def send(self, _payload):
        pass

    def pong(self, _data=b""):
        self.pongs += 1

    def recv(self, timeout=None):
        import time

        time.sleep(timeout or 0.05)
        raise TimeoutError

    def close(self):
        self.closed = True


def test_the_rosbridge_socket_sends_its_own_heartbeat(monkeypatch):
    import time

    import flyto_robotics.generic_ros2_adapter as adapter_module

    monkeypatch.setattr(adapter_module, "KEEPALIVE_SECONDS", 0.05)
    socket = PongRecorder()
    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=lambda _url: socket)
    backend.reconnect()
    time.sleep(0.3)
    backend.disconnect()
    assert socket.pongs >= 3


def test_travel_aims_on_the_map_when_the_map_is_live():
    device = adapter({"motion.travel"})
    live = {"frame": "map", "x": 5.0, "y": -1.0, "yaw": 0.0}
    device.backend.map_pose = lambda: live
    device.describe()
    device.invoke(CallRequest("travel", "motion.travel", arguments={"distance_m": 1.2}))
    [(_, _, arguments)] = device.backend.calls
    assert arguments["x"] == pytest.approx(6.2)
    assert arguments["y"] == pytest.approx(-1.0)
    assert arguments["in_map"] == 1.0
    goal = RosbridgeROS2Backend._action_goal(None, "motion.travel", arguments)
    assert goal["pose"]["header"]["frame_id"] == "map"


def test_the_map_pose_is_the_map_transform_applied_to_odometry():
    from flyto_robotics.generic_ros2_adapter import _compose_map_pose, _transform_tuple

    quarter = {"x": 0.0, "y": 0.0, "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}
    turn = _transform_tuple({"x": 1.0, "y": 2.0}, quarter)
    pose = _compose_map_pose(turn, {"x": 1.0, "y": 0.0, "yaw": 0.0})
    assert pose["x"] == pytest.approx(1.0)
    assert pose["y"] == pytest.approx(3.0)
    assert pose["yaw"] == pytest.approx(math.pi / 2)
    assert _compose_map_pose(None, {"x": 1.0, "y": 0.0, "yaw": 0.0}) is None


def test_rosbridge_keeps_the_map_transform_it_sees():
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: PongRecorder()
    )
    identity = {"x": 0, "y": 0, "z": 0, "w": 1}
    message = {"transforms": [{
        "header": {"frame_id": "map"}, "child_frame_id": "odom",
        "transform": {"translation": {"x": 0.5, "y": 0.0}, "rotation": identity},
    }]}
    with backend._condition:
        backend._update_tf(message, 0.0)
    assert backend._map_odom == (0.5, 0.0, 0.0)
