from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest

from flyto_robotics.ros2_pairing import (
    Ros2PairingError,
    load_ros2_adapter_manifest,
    load_ros2_runtime_snapshot,
    parse_ros2_adapter_manifest,
    ros2_profile_summary,
    standard_ros2_adapter_manifest,
    verify_ros2_pairing,
)

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "examples/ros2-adapters/flyto2-standard.json"
RUNTIME = ROOT / "examples/ros2-runtime/ready-sim.json"
REPLAY_TIME = datetime(2026, 8, 1, 10, 0, tzinfo=timezone.utc)


def _resign(value: dict[str, object]) -> dict[str, object]:
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    value["snapshot"] = hashlib.sha256(
        json.dumps(
            unsigned,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()
    return value


def test_standard_profile_is_portable_registered_and_ai_safe() -> None:
    manifest = standard_ros2_adapter_manifest("robot-001")

    assert manifest["direct_actuation"] is False
    assert manifest["adapters"][0]["supported_modes"] == [
        "hardware",
        "simulation",
    ]
    summary = ros2_profile_summary(manifest)
    encoded = json.dumps(summary, sort_keys=True)
    assert summary["portable_modes"] == ["hardware", "simulation"]
    assert summary["adapters"][0]["capability_ids"] == [
        "robotics.motion.navigate@1",
        "robotics.motion.navigate_to_location@1",
    ]
    assert "nav2_msgs" not in encoded
    assert "/navigate_to_pose" not in encoded
    assert "interface" not in encoded


def test_ready_simulation_evidence_passes_every_pairing_check() -> None:
    manifest = load_ros2_adapter_manifest(MANIFEST)
    runtime = load_ros2_runtime_snapshot(RUNTIME)

    report = verify_ros2_pairing(manifest, runtime, observed_at=REPLAY_TIME)

    assert report["passed"] is True
    assert report["ready_adapter_ids"] == ["ros2.nav2.navigate_to_pose.v1"]
    assert report["ready_capability_ids"] == [
        "robotics.motion.navigate@1",
        "robotics.motion.navigate_to_location@1",
    ]
    assert all(check["passed"] for check in report["checks"])
    encoded = json.dumps(report, sort_keys=True)
    assert "nav2_msgs" not in encoded
    assert "/navigate_to_pose" not in encoded


def test_stale_or_inactive_runtime_fails_closed_without_partial_authority() -> None:
    manifest = load_ros2_adapter_manifest(MANIFEST)
    runtime = load_ros2_runtime_snapshot(RUNTIME)
    degraded = copy.deepcopy(runtime)
    degraded["adapters"][0]["lifecycle_state"] = "inactive"
    _resign(degraded)

    report = verify_ros2_pairing(
        manifest,
        degraded,
        observed_at=datetime(2026, 8, 1, 10, 2, tzinfo=timezone.utc),
    )

    assert report["passed"] is False
    assert report["ready_adapter_ids"] == []
    assert report["ready_capability_ids"] == []
    failed = {check["code"] for check in report["checks"] if not check["passed"]}
    assert "runtime_fresh" in failed
    assert "adapter.ros2.nav2.navigate_to_pose.v1.lifecycle_active" in failed


def test_manifest_rejects_raw_actuation_and_unregistered_capabilities() -> None:
    manifest = load_ros2_adapter_manifest(MANIFEST)
    raw = copy.deepcopy(manifest)
    raw["adapters"][0]["interface"]["name"] = "/cmd_vel"
    _resign(raw)
    with pytest.raises(Ros2PairingError, match="raw actuator"):
        parse_ros2_adapter_manifest(raw)

    invented = copy.deepcopy(manifest)
    invented["adapters"][0]["capability_ids"] = [
        "robotics.manipulation.unimplemented@1"
    ]
    _resign(invented)
    with pytest.raises(Ros2PairingError, match="unregistered capability"):
        parse_ros2_adapter_manifest(invented)


def test_manifest_rejects_profiles_that_split_simulation_and_hardware() -> None:
    manifest = load_ros2_adapter_manifest(MANIFEST)
    simulation_only = copy.deepcopy(manifest)
    simulation_only["adapters"][0]["supported_modes"] = ["simulation"]
    _resign(simulation_only)

    with pytest.raises(Ros2PairingError, match="2 to 2 text values"):
        parse_ros2_adapter_manifest(simulation_only)


def test_cli_replays_content_addressed_pairing_evidence() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "flyto_robotics.cli",
            "verify-ros2-pairing",
            "--manifest",
            str(MANIFEST),
            "--runtime",
            str(RUNTIME),
            "--at",
            "2026-08-01T10:00:00Z",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )

    assert completed.returncode == 0, completed.stderr
    report = json.loads(completed.stdout)
    assert report["passed"] is True
