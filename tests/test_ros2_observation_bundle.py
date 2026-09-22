from __future__ import annotations

import copy
import hashlib
import json
from datetime import datetime, timezone

import pytest

from flyto_robotics.ros2_observation_bundle import (
    Ros2ObservationError,
    build_ros2_observation_bundle,
    evaluate_observation_bundle,
    parse_ros2_observation_bundle,
)

AT = datetime(2026, 9, 21, 7, 0, tzinfo=timezone.utc)
RUNTIME = "a" * 64
FRAME = "b" * 64
CALIBRATION = "c" * 64


def _bundle(mode: str, *, calibrated: bool = False) -> dict:
    return build_ros2_observation_bundle(
        resource_id="turtlebot3-lab",
        runtime_snapshot=RUNTIME,
        deployment_mode=mode,
        provider="gazebo" if mode == "simulation" else "ros2",
        phase="after",
        execution_id="exec-parity-001",
        pose={"frame": "map", "x": 0.3, "y": 0.0, "yaw": 0.1},
        range_observation={"minimum_range_m": 0.82, "sample_count": 400},
        camera={
            "encoding": "rgb8" if mode == "simulation" else "yuv422_yuy2",
            "width": 640,
            "height": 480,
            "calibrated": calibrated,
            "calibration_snapshot": CALIBRATION if calibrated else None,
            "frame_snapshot": FRAME,
        },
        map_tf_available=True,
        observed_at=AT,
    )


def test_simulation_and_hardware_share_one_contract_and_verdict_shape() -> None:
    simulation = _bundle("simulation")
    hardware = _bundle("hardware")

    assert set(simulation) == set(hardware)
    assert simulation["contract_version"] == hardware["contract_version"]
    assert simulation["phase"] == hardware["phase"]
    assert simulation["resource_id"] == hardware["resource_id"]
    assert simulation["execution_id"] == hardware["execution_id"]

    simulation_verdict = evaluate_observation_bundle(simulation)
    hardware_verdict = evaluate_observation_bundle(hardware)
    assert [row["code"] for row in simulation_verdict["checks"]] == [
        row["code"] for row in hardware_verdict["checks"]
    ]
    assert simulation_verdict["passed"] is True
    assert hardware_verdict["passed"] is True


def test_uncalibrated_camera_is_valid_raw_evidence_but_not_metric_vision() -> None:
    verdict = evaluate_observation_bundle(_bundle("hardware", calibrated=False))
    checks = {item["code"]: item["passed"] for item in verdict["checks"]}

    assert checks["raw_vision_observed"] is True
    assert checks["metric_vision_ready"] is False


def test_calibrated_camera_requires_bound_calibration_snapshot() -> None:
    bundle = _bundle("hardware", calibrated=True)
    verdict = evaluate_observation_bundle(bundle)
    checks = {item["code"]: item["passed"] for item in verdict["checks"]}
    assert checks["metric_vision_ready"] is True

    bad = copy.deepcopy(bundle)
    bad["camera"]["calibration_snapshot"] = None
    bad["snapshot"] = _resign(bad)
    with pytest.raises(Ros2ObservationError, match="calibration_snapshot"):
        parse_ros2_observation_bundle(bad)


def test_execution_phases_require_execution_binding() -> None:
    bundle = _bundle("simulation")
    bundle["execution_id"] = None
    bundle["snapshot"] = _resign(bundle)

    with pytest.raises(Ros2ObservationError, match="execution_id"):
        parse_ros2_observation_bundle(bundle)


def test_tampering_fails_closed() -> None:
    bundle = _bundle("hardware")
    bundle["range"]["minimum_range_m"] = 0.01

    with pytest.raises(Ros2ObservationError, match="snapshot"):
        parse_ros2_observation_bundle(bundle)


def test_empty_preflight_observation_is_refused() -> None:
    with pytest.raises(Ros2ObservationError, match="no observations"):
        build_ros2_observation_bundle(
            resource_id="turtlebot3-lab",
            runtime_snapshot=RUNTIME,
            deployment_mode="hardware",
            provider="ros2",
            phase="preflight",
            execution_id=None,
            pose=None,
            range_observation=None,
            camera=None,
            map_tf_available=False,
            observed_at=AT,
        )


def _resign(value: dict) -> str:
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    return hashlib.sha256(
        json.dumps(
            unsigned,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("ascii")
    ).hexdigest()
