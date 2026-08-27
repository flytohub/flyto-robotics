from __future__ import annotations

import pytest

from flyto_robotics.ros2_sensor_guard import FaultInjectionController


def test_device_driver_ignores_non_finite_odometry() -> None:
    controller = FaultInjectionController(
        "odometry_freeze", delay_seconds=0.1, minimum_displacement_m=0.2
    )
    controller.observe_odometry(1.0, 1.0)
    controller.observe_odometry(float("nan"), 2.0)
    controller.observe_odometry(2.0, float("inf"))
    assert controller.initial_position == (1.0, 1.0)
    assert controller.observed_displacement_m == 0.0


def test_device_driver_measures_displacement_from_first_finite_receipt() -> None:
    controller = FaultInjectionController(
        "lidar_dropout", delay_seconds=0.1, minimum_displacement_m=0.5
    )
    controller.observe_odometry(3.0, 4.0)
    controller.observe_odometry(3.3, 4.4)
    assert controller.observed_displacement_m == pytest.approx(0.5)


def test_device_fault_boundary_requires_command_and_exact_delay() -> None:
    now = [0.0]
    controller = FaultInjectionController(
        "lidar_dropout",
        delay_seconds=0.25,
        minimum_displacement_m=0.125,
        clock=lambda: now[0],
    )
    controller.observe_odometry(0.0, 0.0)
    now[0] = 1.0
    assert controller.activation_due() is False
    controller.observe_command(0.01, 0.0)
    now[0] = 1.249
    assert controller.activation_due() is False
    now[0] = 1.25
    assert controller.activation_due() is False
    controller.observe_odometry(0.125, 0.0)
    assert controller.activation_due() is True
