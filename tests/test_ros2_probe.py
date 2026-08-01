from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from flyto_robotics.ros2_pairing import (
    standard_ros2_adapter_manifest,
    verify_ros2_pairing,
)
from flyto_robotics.ros2_probe_node import (
    _split_node_path,
    collect_ros2_runtime_snapshot,
)

OBSERVED_AT = datetime(2026, 8, 1, 12, 0, tzinfo=timezone.utc)


class FakeGraphProbe:
    def __init__(
        self,
        *,
        interface_available: bool = True,
        lifecycle_state: str = "active",
        emergency_stop_ready: bool = True,
    ) -> None:
        self.available = interface_available
        self.state = lifecycle_state
        self.emergency_ready = emergency_stop_ready
        self.interface_calls: list[tuple[str, str, str]] = []

    def interface_available(
        self,
        *,
        kind: str,
        name: str,
        interface_type: str,
        timeout_seconds: float,
    ) -> bool:
        assert timeout_seconds == 2.0
        self.interface_calls.append((kind, name, interface_type))
        return self.available

    def lifecycle_state(
        self,
        managed_nodes: tuple[str, ...] | list[str],
        *,
        timeout_seconds: float,
    ) -> str:
        assert managed_nodes
        assert timeout_seconds == 2.0
        return self.state

    def external_emergency_stop_ready(
        self,
        *,
        owner_node: str,
        service_name: str,
        timeout_seconds: float,
    ) -> bool:
        assert owner_node == "/safety/emergency_supervisor"
        assert service_name == "/safety/emergency_stop"
        assert timeout_seconds == 2.0
        return self.emergency_ready


def test_live_probe_builds_redacted_ready_evidence() -> None:
    manifest = standard_ros2_adapter_manifest("robot-001")
    probe = FakeGraphProbe()

    runtime = collect_ros2_runtime_snapshot(
        manifest,
        probe,
        deployment_mode="simulation",
        emergency_stop_node="/safety/emergency_supervisor",
        emergency_stop_service="/safety/emergency_stop",
        timeout_seconds=2.0,
        observed_at=OBSERVED_AT,
    )
    report = verify_ros2_pairing(manifest, runtime, observed_at=OBSERVED_AT)

    assert probe.interface_calls == [
        ("action", "/navigate_to_pose", "nav2_msgs/action/NavigateToPose")
    ]
    assert report["passed"] is True
    assert runtime["emergency_stop_ready"] is True
    assert runtime["adapters"][0]["observation_sequence"].startswith("ros-graph:")
    encoded = json.dumps(runtime, sort_keys=True)
    assert "/navigate_to_pose" not in encoded
    assert "nav2_msgs" not in encoded
    assert "/safety/emergency_stop" not in encoded


@pytest.mark.parametrize(
    ("available", "state", "emergency_ready", "failed_code"),
    (
        (False, "active", True, "interface_available"),
        (True, "inactive", True, "lifecycle_active"),
        (True, "active", False, "emergency_stop_ready"),
    ),
)
def test_live_probe_fails_closed_on_each_runtime_dependency(
    available: bool,
    state: str,
    emergency_ready: bool,
    failed_code: str,
) -> None:
    manifest = standard_ros2_adapter_manifest("robot-001")
    runtime = collect_ros2_runtime_snapshot(
        manifest,
        FakeGraphProbe(
            interface_available=available,
            lifecycle_state=state,
            emergency_stop_ready=emergency_ready,
        ),
        deployment_mode="hardware",
        emergency_stop_node="/safety/emergency_supervisor",
        emergency_stop_service="/safety/emergency_stop",
        timeout_seconds=2.0,
        observed_at=OBSERVED_AT,
    )

    report = verify_ros2_pairing(manifest, runtime, observed_at=OBSERVED_AT)

    assert report["passed"] is False
    assert report["ready_adapter_ids"] == []
    assert any(
        failed_code in check["code"] and check["passed"] is False
        for check in report["checks"]
    )


def test_emergency_stop_owner_must_be_an_absolute_external_node() -> None:
    assert _split_node_path("/safety/emergency_supervisor") == (
        "emergency_supervisor",
        "/safety",
    )
    with pytest.raises(ValueError, match="absolute ROS node name"):
        _split_node_path("emergency_supervisor")
