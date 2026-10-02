"""A freshly connected adapter waits for its first LiDAR before judging."""

from __future__ import annotations

import threading
import time

from flyto_robotics.generic_ros2_adapter import RosbridgeROS2Backend

ODOM = {"pose": {"pose": {"position": {"x": 0.0, "y": 0.0}, "orientation": {"w": 1.0}}}}
SCAN = {"ranges": [0.8, 0.9, 1.0], "range_min": 0.1, "range_max": 100.0}


class QuietSocket:
    """A rosbridge connection nothing on the graph ever answers on."""

    def __init__(self):
        self._closed = threading.Event()

    def send(self, _payload):
        pass

    def recv(self, timeout=None):
        self._closed.wait(timeout if timeout is not None else 0.2)
        raise TimeoutError

    def close(self):
        self._closed.set()


def connected_backend():
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda url: QuietSocket()
    )
    backend.reconnect()
    return backend


def deliver(backend, delay, update, message):
    def run():
        time.sleep(delay)
        with backend._condition:
            update(message, time.monotonic())

    threading.Thread(target=run, daemon=True).start()


def test_first_observation_waits_for_lidar_that_follows_odometry(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "3")
    backend = connected_backend()
    deliver(backend, 0.0, backend._update_odometry, ODOM)
    deliver(backend, 0.8, backend._update_scan, SCAN)

    observation = backend.observation()

    assert observation["range"] == {"minimum_range_m": 0.8, "sample_count": 3}


def test_missing_lidar_is_still_reported_once_the_wait_ends(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.3")
    backend = connected_backend()
    deliver(backend, 0.0, backend._update_odometry, ODOM)

    started = time.monotonic()
    observation = backend.observation()

    assert observation["range"] is None
    assert time.monotonic() - started < 1.0
