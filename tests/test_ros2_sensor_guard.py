from __future__ import annotations

import pytest

from flyto_robotics.ros2_sensor_guard import FaultInjectionController


@pytest.mark.parametrize(
    ("scenario", "blocked"), [("lidar_dropout", "lidar"), ("odometry_freeze", "odometry")]
)
def test_fault_needs_motion_delay_and_physical_displacement(scenario: str, blocked: str) -> None:
    now = [0.0]
    controller = FaultInjectionController(
        scenario, delay_seconds=0.1, minimum_displacement_m=0.1, clock=lambda: now[0]
    )
    controller.observe_odometry(1.0, 2.0)
    controller.observe_command(0.1, 0.0)
    now[0] = 0.1
    controller.observe_odometry(1.099, 2.0)
    assert controller.activation_due() is False
    controller.observe_odometry(float("nan"), 2.0)
    controller.observe_odometry(1.1, 2.0)
    assert controller.activation_due() is True
    controller.activate()
    assert controller.should_forward(blocked) is False
    other = "odometry" if blocked == "lidar" else "lidar"
    assert controller.should_forward(other) is True


def test_non_finite_commands_never_arm_fault() -> None:
    controller = FaultInjectionController("lidar_dropout", delay_seconds=0.1)
    controller.observe_command(float("inf"), 0.0)
    controller.observe_command(0.0, float("nan"))
    assert controller.motion_started_at is None


@pytest.mark.parametrize("value", [True, -0.01, float("nan"), float("inf")])
def test_fault_displacement_configuration_fails_closed(value: object) -> None:
    with pytest.raises(ValueError, match="displacement"):
        FaultInjectionController(
            "nav2_lifecycle_failure", delay_seconds=0.2, minimum_displacement_m=value
        )  # type: ignore[arg-type]


def test_fault_and_sensor_names_fail_closed() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        FaultInjectionController("unknown", delay_seconds=0.2)
    controller = FaultInjectionController("none", delay_seconds=0.2)
    with pytest.raises(ValueError, match="unsupported"):
        controller.should_forward("camera")


def test_lifecycle_fault_and_none_mode_do_not_suppress_sensors() -> None:
    now = [0.0]
    lifecycle = FaultInjectionController(
        "nav2_lifecycle_failure", delay_seconds=0.1, clock=lambda: now[0]
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


@pytest.mark.parametrize("value", [True, 0.0, float("nan"), float("inf")])
def test_fault_delay_configuration_fails_closed(value: object) -> None:
    with pytest.raises(ValueError, match="safe range"):
        FaultInjectionController("lidar_dropout", delay_seconds=value)  # type: ignore[arg-type]
