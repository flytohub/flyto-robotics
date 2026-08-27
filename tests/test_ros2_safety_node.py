from __future__ import annotations

import pytest

from flyto_robotics.ros2_safety_node import SafetyWatchdog, VelocitySafetyEnvelope


def test_velocity_envelope_clamps_both_axes_and_latched_state_stops() -> None:
    envelope = VelocitySafetyEnvelope(max_abs_linear_speed_mps=0.1, max_abs_angular_speed_rps=0.5)
    assert envelope.gate(0.3, -0.8, latched=False) == (0.1, -0.5)
    assert envelope.gate(-0.3, 0.8, latched=False) == (-0.1, 0.5)
    assert envelope.gate(0.3, 0.8, latched=True) == (0.0, 0.0)


@pytest.mark.parametrize("value", [True, float("nan"), float("inf")])
def test_velocity_envelope_rejects_non_finite_commands(value: object) -> None:
    with pytest.raises(ValueError, match="speed is invalid"):
        VelocitySafetyEnvelope(
            max_abs_linear_speed_mps=0.1,
            max_abs_angular_speed_rps=0.5,
        ).gate(value, 0.0, latched=False)  # type: ignore[arg-type]


def test_watchdog_latches_each_stale_input_only_during_an_active_goal() -> None:
    now = [0.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4,
        command_timeout_seconds=0.3,
        clock=lambda: now[0],
    )
    for source in ("odometry", "lidar", "command"):
        watchdog.observe(source)

    now[0] = 2.0
    assert watchdog.evaluate() is None
    watchdog.update_goal_active(True)
    now[0] = 2.2
    for source in ("odometry", "lidar", "command"):
        watchdog.observe(source)
    now[0] = 2.51
    watchdog.observe("odometry")
    watchdog.observe("lidar")

    assert watchdog.evaluate_transition() == ("command_stale", True)
    assert watchdog.latched is True
    watchdog.observe("command")
    assert watchdog.evaluate_transition() == ("command_stale", False)


def test_watchdog_latches_stale_motion_command() -> None:
    now = [0.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4, command_timeout_seconds=0.3, clock=lambda: now[0]
    )
    watchdog.update_goal_active(True)
    watchdog.observe_command(0.1, 0.0)
    now[0] = 0.31
    watchdog.observe("odometry")
    watchdog.observe("lidar")
    assert watchdog.evaluate_transition() == ("command_stale", True)
    assert watchdog.evaluate_transition() == ("command_stale", False)


def test_zero_command_receipt_is_still_subject_to_watchdog_staleness() -> None:
    now = [0.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.6, command_timeout_seconds=0.3, clock=lambda: now[0]
    )
    watchdog.update_goal_active(True)
    watchdog.observe_command(0.0, 0.0)
    now[0] = 0.31
    watchdog.observe("odometry")
    watchdog.observe("lidar")
    assert watchdog.evaluate_transition() == ("command_stale", True)


def test_new_goal_gets_fresh_sensor_window() -> None:
    now = [10.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4, command_timeout_seconds=0.3, clock=lambda: now[0]
    )
    watchdog.update_goal_active(True)
    now[0] = 10.401
    assert watchdog.evaluate_transition() == ("odometry_stale", True)


def test_invalid_watchdog_clock_fails_closed() -> None:
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4, command_timeout_seconds=0.3, clock=lambda: float("nan")
    )
    watchdog.goal_active = True
    assert watchdog.evaluate_transition() == ("watchdog_clock_invalid", True)


def test_watchdog_reset_and_sensor_reason_are_deterministic() -> None:
    now = [0.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4,
        command_timeout_seconds=0.5,
        clock=lambda: now[0],
    )
    for source in ("odometry", "lidar", "command"):
        watchdog.observe(source)
    watchdog.update_goal_active(True)
    now[0] = 0.41
    watchdog.observe("odometry")
    watchdog.observe("command")

    assert watchdog.evaluate() == "lidar_stale"
    watchdog.reset()
    assert watchdog.latched is False
    assert watchdog.evaluate() is None


def test_command_watchdog_arms_only_after_this_goal_receives_a_command() -> None:
    now = [0.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4,
        command_timeout_seconds=0.3,
        clock=lambda: now[0],
    )
    watchdog.observe("command")
    watchdog.update_goal_active(True)

    now[0] = 0.35
    watchdog.observe("odometry")
    watchdog.observe("lidar")
    assert watchdog.evaluate() is None

    watchdog.observe("command")
    now[0] = 0.66
    watchdog.observe("odometry")
    watchdog.observe("lidar")
    assert watchdog.evaluate_transition() == ("command_stale", True)
