from __future__ import annotations

import pytest

from flyto_robotics.ros2_action_executor import (
    Ros2ActionExecutionError,
    _inputs_ready_for_goal,
)


def _ready(odom: object, lidar: object, *, now: object = 10.0) -> bool:
    return _inputs_ready_for_goal(
        {
            "pose": object(),
            "safety": False,
            "odometry_seen_at": odom,
            "lidar_seen_at": lidar,
        },
        sensor_timeout_seconds=0.4,
        transform_ready=lambda: True,
        clock=lambda: now,  # type: ignore[arg-type]
    )


def test_hardware_executor_enforces_freshness_boundary() -> None:
    assert _ready(9.61, 9.61) is True
    assert _ready(9.599, 9.6) is False
    assert _ready(9.6, 10.001) is False


@pytest.mark.parametrize("receipt", [True, float("nan"), float("inf")])
def test_hardware_executor_rejects_invalid_receipts(receipt: object) -> None:
    assert _ready(receipt, 10.0) is False


def test_hardware_executor_fails_closed_on_invalid_clock_or_safety() -> None:
    assert _ready(10.0, 10.0, now=float("nan")) is False
    latest = {
        "pose": object(),
        "safety": True,
        "odometry_seen_at": 1.0,
        "lidar_seen_at": 1.0,
    }
    assert (
        _inputs_ready_for_goal(
            latest,
            sensor_timeout_seconds=0.4,
            transform_ready=lambda: True,
            clock=lambda: 1.0,
        )
        is False
    )


@pytest.mark.parametrize("timeout", [True, 0.049, 5.001, float("nan")])
def test_hardware_executor_rejects_unsafe_timeout(timeout: object) -> None:
    with pytest.raises(Ros2ActionExecutionError, match="sensor_timeout_seconds"):
        _inputs_ready_for_goal(
            {},
            sensor_timeout_seconds=timeout,  # type: ignore[arg-type]
            transform_ready=lambda: True,
        )
