"""The robot's pose in the map frame is reported beside odometry, never instead.

Localization publishes map->odom; composed with the odometry pose it names the
robot's place in the frame a navigation goal is given in. Every odometry field
stays as it was: motion is still judged on odometry.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

import flyto_robotics.generic_ros2_adapter as adapter_module
from flyto_robotics import map_frame
from flyto_robotics import motion_outcome as mo
from flyto_robotics import provider_evidence as pe
from flyto_robotics.generic_ros2_adapter import (
    GenericROS2Adapter,
    RclpyROS2Backend,
    RosbridgeROS2Backend,
)
from flyto_robotics.ros2_observation_bundle import (
    Ros2ObservationError,
    build_ros2_observation_bundle,
    parse_ros2_observation_bundle,
)
from tests.test_generic_ros2_adapter import FakeROS2Backend
from tests.test_observation_on_state import QuietSocket, fake_rclpy  # noqa: F401 - fixture

# map->odom: odometry's origin sits at (1.0, 2.0) in the map, turned 90 degrees.
HALF_PI = math.pi / 2
TF_X, TF_Y, TF_YAW = 1.0, 2.0, HALF_PI
QUAT_Z, QUAT_W = math.sin(TF_YAW / 2), math.cos(TF_YAW / 2)


def rclpy_tf(*, with_transform: bool = True):
    item = SimpleNamespace(header=SimpleNamespace(frame_id="map"), child_frame_id="odom")
    if with_transform:
        item.transform = SimpleNamespace(
            translation=SimpleNamespace(x=TF_X, y=TF_Y, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=QUAT_Z, w=QUAT_W),
        )
    return SimpleNamespace(transforms=[item])


def rclpy_odometry(x: float, y: float = 0.0):
    vector = SimpleNamespace
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(x=x, y=y, z=0.0),
                orientation=vector(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vector(x=0.0, y=0.0, z=0.0), angular=vector(x=0.0, y=0.0, z=0.0)
            )
        ),
    )


ROSBRIDGE_TF = {
    "transforms": [
        {
            "header": {"frame_id": "map"},
            "child_frame_id": "odom",
            "transform": {
                "translation": {"x": TF_X, "y": TF_Y, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": QUAT_Z, "w": QUAT_W},
            },
        }
    ]
}
ROSBRIDGE_ODOM = {
    "pose": {"pose": {"position": {"x": 0.5, "y": 0.0}, "orientation": {"w": 1.0}}},
    "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
}


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def monotonic(self) -> float:
        return self.now


@pytest.fixture
def clock(monkeypatch):
    fake = Clock()
    monkeypatch.setattr(
        adapter_module,
        "time",
        SimpleNamespace(monotonic=fake.monotonic, monotonic_ns=lambda: 0, sleep=lambda _s: None),
    )
    return fake


# --- composition --------------------------------------------------------------


def test_compose_places_the_odometry_pose_in_the_map():
    odom = {"frame": "odom", "x": 0.5, "y": 0.0, "yaw": 0.0}
    pose = map_frame.compose((TF_X, TF_Y, TF_YAW), odom)

    assert pose["frame"] == "map"
    assert pose["x"] == pytest.approx(1.0)
    assert pose["y"] == pytest.approx(2.5)
    assert pose["yaw"] == pytest.approx(HALF_PI)


def test_compose_wraps_the_heading_and_needs_both_halves():
    pose = map_frame.compose((0.0, 0.0, 3.0), {"x": 0.0, "y": 0.0, "yaw": 1.0})
    assert -math.pi <= pose["yaw"] <= math.pi
    assert pose["yaw"] == pytest.approx(4.0 - math.tau)
    assert map_frame.compose(None, {"x": 0.0, "y": 0.0, "yaw": 0.0}) is None
    assert map_frame.compose((0.0, 0.0, 0.0), None) is None
    assert map_frame.planar_transform({"x": 1.0}, {"w": 1.0}) is None


# --- both backends report it, odometry unchanged ------------------------------


@pytest.mark.usefixtures("fake_rclpy")
def test_rclpy_snapshot_reports_the_map_pose_beside_odometry(clock):
    backend = RclpyROS2Backend()
    try:
        backend._on_odometry(rclpy_odometry(0.5))
        backend._on_tf(rclpy_tf())
        snapshot = backend.held_observation()
    finally:
        backend.disconnect()

    assert snapshot["pose"] == {"frame": "odom", "x": 0.5, "y": 0.0, "yaw": 0.0}
    assert snapshot["map_pose"]["frame"] == "map"
    assert snapshot["map_pose"]["x"] == pytest.approx(1.0)
    assert snapshot["map_pose"]["y"] == pytest.approx(2.5)
    assert snapshot["map_tf_available"] is True


def test_rosbridge_snapshot_reports_the_map_pose_beside_odometry(clock):
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: QuietSocket()
    )
    try:
        with backend._condition:
            backend._update_odometry(ROSBRIDGE_ODOM, clock.now)
            backend._update_tf(ROSBRIDGE_TF, clock.now)
        snapshot = backend.held_observation()
        assert snapshot["pose"]["frame"] == "odom" and snapshot["pose"]["x"] == 0.5
        assert snapshot["map_pose"]["x"] == pytest.approx(1.0)
        assert snapshot["map_pose"]["y"] == pytest.approx(2.5)
        assert snapshot["map_pose"]["yaw"] == pytest.approx(HALF_PI)

        evidence = backend._evidence("motion.advance", wait=False)
        assert evidence["odom"]["frame"] == "odom"
        assert evidence["map_pose"]["frame"] == "map"
    finally:
        backend.disconnect()


@pytest.mark.usefixtures("fake_rclpy")
def test_no_map_pose_without_a_fresh_transform(clock):
    backend = RclpyROS2Backend()
    backend._on_odometry(rclpy_odometry(0.5))
    assert backend.held_observation()["map_pose"] is None

    # A transform with no values (a localization that only proves it is up)
    # makes the map available but places nothing.
    backend._on_tf(rclpy_tf(with_transform=False))
    assert backend.held_observation()["map_tf_available"] is True
    assert backend.held_observation()["map_pose"] is None

    backend._on_tf(rclpy_tf())
    assert backend.held_observation()["map_pose"] is not None
    # Stale: localization stopped publishing, odometry did not.
    clock.now += 60.0
    backend._on_odometry(rclpy_odometry(0.6))
    snapshot = backend.held_observation()
    assert snapshot["pose"]["x"] == 0.6
    assert snapshot["map_pose"] is None


# --- the observation bundle carries it, additively ----------------------------

AT = datetime(2026, 10, 4, 7, 0, tzinfo=timezone.utc)


def _bundle(**extra):
    return build_ros2_observation_bundle(
        resource_id="turtlebot3-lab",
        runtime_snapshot="a" * 64,
        deployment_mode="simulation",
        provider="gazebo",
        phase="before",
        execution_id="exec-1",
        pose={"frame": "odom", "x": 0.5, "y": 0.0, "yaw": 0.0},
        range_observation={"minimum_range_m": 0.8, "sample_count": 360},
        camera=None,
        map_tf_available=True,
        observed_at=AT,
        **extra,
    )


def test_bundle_without_a_map_pose_is_unchanged():
    bundle = _bundle()
    assert "map_pose" not in bundle
    assert _bundle(map_pose=None) == bundle


def test_bundle_carries_the_map_pose_and_rejects_a_wrong_one():
    map_pose = {"frame": "map", "x": 1.0, "y": 2.5, "yaw": 1.5708}
    bundle = _bundle(map_pose=map_pose)
    assert bundle["map_pose"] == map_pose
    assert bundle["pose"]["frame"] == "odom"
    assert parse_ros2_observation_bundle(bundle) == bundle

    with pytest.raises(Ros2ObservationError):
        _bundle(map_pose={**map_pose, "frame": "odom"})
    with pytest.raises(Ros2ObservationError):
        build_ros2_observation_bundle(
            resource_id="turtlebot3-lab",
            runtime_snapshot="a" * 64,
            deployment_mode="simulation",
            provider="gazebo",
            phase="preflight",
            execution_id=None,
            pose={"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
            range_observation=None,
            camera=None,
            map_tf_available=False,
            map_pose=map_pose,
        )


def test_adapter_observation_carries_the_map_pose():
    backend = FakeROS2Backend({"motion.advance", "motion.halt"})
    backend.observation_payload["map_pose"] = {"frame": "map", "x": 1.0, "y": 2.5, "yaw": 0.3}
    device = GenericROS2Adapter(backend=backend, resource_id="standard-turtlebot3")

    bundle = device.observe(phase="before", execution_id="call-1")

    assert bundle["pose"] == {"frame": "odom", "x": 0.25, "y": 0.0, "yaw": 0.0}
    assert bundle["map_pose"] == {"frame": "map", "x": 1.0, "y": 2.5, "yaw": 0.3}


# --- the motion's own record and its abort reason -----------------------------


def test_motion_summary_and_recovery_context_carry_map_poses():
    track = mo.MotionTrack(
        capability_id="motion.advance",
        arguments={"distance_m": 1.2},
        start_pose={"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
        start_map_pose={"frame": "map", "x": 1.0, "y": 2.0, "yaw": HALF_PI},
        started_at=0.0,
    )
    summary = mo.summarize(
        track,
        status=6,
        result_values={"error_code": 0},
        end_pose={"frame": "odom", "x": 0.36, "y": 0.0, "yaw": 0.0},
        end_map_pose={"frame": "map", "x": 1.0, "y": 2.36, "yaw": HALF_PI},
        range_observation={"minimum_range_m": 0.34, "sample_count": 360},
        clearance_floor_m=0.35,
        ended_at=4.0,
    )

    assert summary["reason"] == mo.REASON_OBSTACLE_BLOCKED
    assert summary["start_pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert summary["distance_travelled_m"] == pytest.approx(0.36)
    assert summary["start_map_pose"] == {"frame": "map", "x": 1.0, "y": 2.0, "yaw": 1.5708}
    assert summary["final_map_pose"]["y"] == pytest.approx(2.36)
    assert "stopped at map x=1.000 y=2.360" in mo.describe(summary)

    context = pe.recovery_context(summary)
    assert context["start_map_pose"] == summary["start_map_pose"]
    assert context["final_map_pose"] == summary["final_map_pose"]
    assert context["travelled_m"] == pytest.approx(0.36)


def test_motion_summary_without_localization_has_no_map_keys():
    track = mo.MotionTrack(
        capability_id="motion.advance",
        arguments={"distance_m": 1.2},
        start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        started_at=0.0,
    )
    summary = mo.summarize(
        track,
        status=4,
        result_values=None,
        end_pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        end_map_pose={"frame": "odom", "x": 1.2, "y": 0.0, "yaw": 0.0},
        range_observation=None,
        clearance_floor_m=0.35,
        ended_at=4.0,
    )
    assert "start_map_pose" not in summary and "final_map_pose" not in summary
    assert "stopped at map" not in mo.describe(summary)
