"""Reading one photo or map through standard ROS 2, and the rosbridge heartbeat."""

from __future__ import annotations

import base64

import pytest

from flyto_robotics.adapter_contract import OUTCOME_REFUSED, CallRequest
from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    RosbridgeROS2Backend,
    map_capture,
    photo_capture,
)
from tests.test_generic_ros2_adapter import adapter

JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 16 + b"\xff\xd9"


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
