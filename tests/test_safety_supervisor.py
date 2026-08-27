from __future__ import annotations

import pytest

from flyto_robotics.ros2_safety_node import (
    WATCHDOG_POLL_SECONDS,
    SafetyWatchdog,
    watchdog_source_for_fault_state,
)


def test_supervisor_maps_only_canonical_active_faults() -> None:
    assert watchdog_source_for_fault_state("lidar_dropout:active") == "lidar"
    assert watchdog_source_for_fault_state("odometry_freeze:active") == "odometry"
    assert watchdog_source_for_fault_state("nav2_lifecycle_failure:active") == "command"
    assert watchdog_source_for_fault_state("lidar_dropout:cleared") is None
    assert watchdog_source_for_fault_state("unknown:active") is None


def test_supervisor_poll_stays_inside_watchdog_boundary() -> None:
    assert 0.0 < WATCHDOG_POLL_SECONDS < 0.05


def test_supervisor_preserves_first_latched_reason() -> None:
    watchdog = SafetyWatchdog(sensor_timeout_seconds=0.4, command_timeout_seconds=0.3)
    assert watchdog.latch("odometry_stale") == "odometry_stale"
    assert watchdog.latch("lidar_stale") == "odometry_stale"


@pytest.mark.parametrize("source", ["camera", "", "command_stale"])
def test_supervisor_rejects_unknown_sources(source: str) -> None:
    watchdog = SafetyWatchdog(sensor_timeout_seconds=0.4, command_timeout_seconds=0.3)
    with pytest.raises(ValueError, match="unsupported"):
        watchdog.observe(source)


def test_future_sensor_receipt_fails_closed() -> None:
    now = [1.0]
    watchdog = SafetyWatchdog(
        sensor_timeout_seconds=0.4, command_timeout_seconds=0.3, clock=lambda: now[0]
    )
    watchdog.update_goal_active(True)
    watchdog.receipts["odometry"] = 2.0
    assert watchdog.evaluate_transition() == ("odometry_stale", True)
