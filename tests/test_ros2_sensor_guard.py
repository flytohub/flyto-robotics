from __future__ import annotations

import pytest

from flyto_robotics.ros2_sensor_guard import FaultInjectionController


@pytest.mark.parametrize(
    ("scenario", "blocked_sensor"),
    [("lidar_dropout", "lidar"), ("odometry_freeze", "odometry")],
)
def test_sensor_fault_activates_only_after_real_motion_and_delay(
    scenario: str,
    blocked_sensor: str,
) -> None:
    now = [10.0]
    controller = FaultInjectionController(
        scenario,
        delay_seconds=0.35,
        clock=lambda: now[0],
    )

    controller.observe_command(0.0, 0.0)
    now[0] = 20.0
    assert controller.activation_due() is False
    controller.observe_command(0.2, 0.0)
    now[0] = 20.34
    assert controller.activation_due() is False
    now[0] = 20.35
    assert controller.activation_due() is True
    controller.activate()

    assert controller.active is True
    assert controller.should_forward(blocked_sensor) is False
    other = "odometry" if blocked_sensor == "lidar" else "lidar"
    assert controller.should_forward(other) is True


def test_lifecycle_fault_and_none_mode_do_not_suppress_sensors() -> None:
    now = [0.0]
    lifecycle = FaultInjectionController(
        "nav2_lifecycle_failure",
        delay_seconds=0.1,
        clock=lambda: now[0],
    )
    lifecycle.observe_command(0.1, 0.0)
    now[0] = 0.1
    lifecycle.activate()
    assert lifecycle.should_forward("odometry") is True
    assert lifecycle.should_forward("lidar") is True

    none = FaultInjectionController("none", delay_seconds=0.1, clock=lambda: now[0])
    none.observe_command(0.1, 0.0)
    now[0] = 10.0
    assert none.activation_due() is False


def test_fault_configuration_fails_closed() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        FaultInjectionController("unknown", delay_seconds=0.2)
    with pytest.raises(ValueError, match="safe range"):
        FaultInjectionController("lidar_dropout", delay_seconds=0.0)
