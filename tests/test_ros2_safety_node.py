from __future__ import annotations

from flyto_robotics.ros2_safety_node import SafetyWatchdog


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
