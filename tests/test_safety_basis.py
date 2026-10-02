"""A robot without LiDAR declares that a person is present instead."""

from __future__ import annotations

import math

import pytest

from flyto_robotics.adapter_contract import OUTCOME_REFUSED, CallRequest
from flyto_robotics.generic_ros2_adapter import (
    OPERATOR_PRESENT_OBSERVATION,
    SUPERVISED_MAX_SPEED_MPS,
)
from tests.test_generic_ros2_adapter import adapter

MOTIONS = {"motion.advance", "motion.retreat", "motion.rotate", "motion.navigate"}


def blind(device):
    """A robot whose observation carries odometry and no LiDAR at all."""
    device.backend.observation_payload["range"] = None
    return device


def call(device, capability_id, **arguments):
    return device.invoke(CallRequest(f"call-{capability_id}", capability_id, arguments=arguments))


def test_lidar_clearance_stays_the_default(monkeypatch):
    monkeypatch.delenv("FLYTO_ROS2_SAFETY_BASIS", raising=False)
    device = blind(adapter(MOTIONS))
    device.describe()
    result = call(device, "motion.advance", distance_m=0.1)
    assert result.outcome == OUTCOME_REFUSED
    assert "LiDAR" in result.detail
    assert device.backend.calls == []


def test_operator_present_moves_without_lidar_at_a_bounded_speed(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "operator_present")
    device = blind(adapter(MOTIONS))
    device.describe()
    result = call(device, "motion.advance", distance_m=0.2, speed_mps=0.2)
    assert result.outcome != OUTCOME_REFUSED, result.detail
    [(_, capability_id, arguments)] = device.backend.calls
    assert capability_id == "motion.advance"
    assert arguments["speed_mps"] == SUPERVISED_MAX_SPEED_MPS


def test_operator_present_declares_what_motion_rests_on(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "operator_present")
    [declaration, *_] = adapter({"motion.advance"}).describe()
    assert OPERATOR_PRESENT_OBSERVATION in declaration.required_observations
    assert not any("/scan" in item for item in declaration.required_observations)


@pytest.mark.parametrize(
    "capability_id, arguments, reason",
    [
        ("motion.advance", {"distance_m": 0.31}, "at most 0.30m"),
        ("motion.retreat", {"distance_m": 0.5}, "at most 0.30m"),
        ("motion.rotate", {"yaw_radians": math.pi}, "per turn"),
        ("motion.navigate", {"x": 1.0, "y": 0.0}, "needs LiDAR"),
    ],
)
def test_operator_present_bounds_each_motion(monkeypatch, capability_id, arguments, reason):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "operator_present")
    device = blind(adapter(MOTIONS))
    device.describe()
    result = call(device, capability_id, **arguments)
    assert result.outcome == OUTCOME_REFUSED
    assert reason in result.detail
    assert device.backend.calls == []


def test_operator_present_still_needs_odometry(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "operator_present")
    device = blind(adapter(MOTIONS))
    device.backend.observation_payload["pose"] = None
    device.describe()
    result = call(device, "motion.advance", distance_m=0.1)
    assert result.outcome == OUTCOME_REFUSED
    assert "odometry" in result.detail


def test_an_unknown_safety_basis_refuses_motion(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "trust_me")
    device = adapter(MOTIONS)
    device.describe()
    result = call(device, "motion.advance", distance_m=0.1)
    assert result.outcome == OUTCOME_REFUSED
    assert "trust_me" in result.detail
