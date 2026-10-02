"""A reduced LiDAR sweep travels with the observation so a person can see it."""

from __future__ import annotations

import math
import time

import pytest

from flyto_robotics.ros2_observation_bundle import (
    Ros2ObservationError,
    build_ros2_observation_bundle,
)
from flyto_robotics.scan_clearance import sweep
from tests.test_rosbridge_observation_wait import ODOM, connected_backend

FULL = 2 * math.pi


def test_reducing_a_sweep_keeps_the_nearest_obstacle_in_each_bin():
    ranges = [2.0] * 400
    ranges[7] = 0.31
    reduced = sweep(ranges, angle_min=0.0, angle_increment=FULL / 400, beams=100)
    assert len(reduced["ranges_m"]) == 100
    assert reduced["ranges_m"][1] == 0.31
    assert reduced["angle_increment_rad"] == pytest.approx(4 * FULL / 400, abs=1e-6)


def test_a_bin_with_no_valid_return_is_unseen_not_clear():
    ranges = [1.0, float("nan"), float("inf"), 0.0]
    reduced = sweep(ranges, angle_min=0.0, angle_increment=0.1, beams=4)
    assert reduced["ranges_m"] == [1.0, None, None, None]


@pytest.mark.parametrize(
    "ranges, increment",
    [([], 0.1), ([float("nan")] * 3, 0.1), ([1.0, 1.0], 0.0)],
)
def test_nothing_to_draw_is_no_sweep(ranges, increment):
    assert sweep(ranges, angle_min=0.0, angle_increment=increment) is None


def _bundle(range_observation):
    return build_ros2_observation_bundle(
        resource_id="turtlebot3-lab",
        runtime_snapshot="a" * 64,
        deployment_mode="simulation",
        provider="gazebo",
        phase="after",
        execution_id="exec-sweep-001",
        pose={"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
        range_observation=range_observation,
        camera=None,
        map_tf_available=False,
    )


def test_the_bundle_carries_a_valid_sweep():
    reduced = sweep([0.5] * 360, angle_min=0.0, angle_increment=FULL / 360)
    bundle = _bundle({"minimum_range_m": 0.5, "sample_count": 360, "sweep": reduced})
    assert bundle["range"]["sweep"]["ranges_m"][0] == 0.5


@pytest.mark.parametrize(
    "bad",
    [
        {"angle_min_rad": 0.0, "angle_increment_rad": 0.0, "ranges_m": [1.0]},
        {"angle_min_rad": 0.0, "angle_increment_rad": 0.1, "ranges_m": []},
        {"angle_min_rad": 0.0, "angle_increment_rad": 0.1, "ranges_m": ["far"]},
        {"angle_min_rad": 0.0, "angle_increment_rad": 0.1, "ranges_m": [1.0], "extra": 1},
    ],
)
def test_the_bundle_refuses_a_malformed_sweep(bad):
    with pytest.raises(Ros2ObservationError):
        _bundle({"minimum_range_m": 0.5, "sample_count": 1, "sweep": bad})


def test_rosbridge_reports_the_sweep_with_the_range(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "1")
    backend = connected_backend()
    scan = {
        "ranges": [0.8, 0.9, 1.0, 1.1],
        "range_min": 0.1,
        "range_max": 100.0,
        "angle_min": 0.0,
        "angle_increment": FULL / 4,
    }
    with backend._condition:
        backend._update_odometry(ODOM, time.monotonic())
        backend._update_scan(scan, time.monotonic())

    observed = backend.observation()["range"]

    assert observed["minimum_range_m"] == 0.8
    assert observed["sweep"]["ranges_m"] == [0.8, 0.9, 1.0, 1.1]
