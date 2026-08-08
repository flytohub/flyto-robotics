"""Mission Stations transport, capability, calibration, and evidence gates."""

from __future__ import annotations

import copy
import hashlib
import json
import urllib.error
import urllib.request

import pytest

from flyto_robotics.capabilities import default_capability_registry
from flyto_robotics.mission_gateway import (
    MissionGateway,
    MissionGatewayError,
    build_action_evidence,
    parse_mission_dispatch,
)

TOKEN = "synthetic-mission-gateway-token"
NOW = "2026-08-08T12:00:00Z"


def _hash(value: object) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _calibration() -> dict[str, object]:
    value: dict[str, object] = {
        "calibration_id": "arena-a",
        "revision": 3,
        "coordinate_frame": "mission-space",
        "status": "READY",
        "markers": [
            {
                "marker_id": marker_id,
                "x": float(index),
                "y": float(index % 2),
                "yaw": 0.0,
                "confidence": 1.0,
                "source": "manual",
                "observed_at": NOW,
            }
            for index, marker_id in enumerate(("Z1", "Z2", "Z3", "Z4", "START"))
        ],
        "created_at": NOW,
        "created_by": "operator-1",
    }
    value["contract_hash"] = _hash(value)
    return {"contract_version": "flyto.space-calibration.v1", **value}


def _dispatch() -> dict[str, object]:
    registry = default_capability_registry()
    catalog = registry.execution_catalog()
    return {
        "contract_version": "flyto.robotics.mission-dispatch.v1",
        "task_id": "mission-1",
        "space_id": "demo-space",
        "objective_id": "passage-check",
        "zone_id": "zone-04",
        "zone_marker_id": "Z4",
        "card_source": "judge_draw",
        "resource_id": "turtlebot3-1",
        "evidence_requirements": ["passage.clearance"],
        "calibration": _calibration(),
        "plan_revision": 2,
        "plan_hash": "a" * 64,
        "assignment_revision": 4,
        "capability_registry_hash": catalog["contract_hash"],
        "steps": [
            {
                "step_id": "measure-clearance",
                "capability_id": "robotics.safety.wait_until_clear@1",
                "executor_kind": "flyto-robotics",
                "arguments": {"clear_seconds": 0.5},
                "evidence_outputs": ["passage.clearance"],
            },
            {
                "step_id": "safe-stop",
                "capability_id": "robotics.safety.safe_stop@1",
                "executor_kind": "flyto-robotics",
                "arguments": {"seconds": 0.0},
                "evidence_outputs": [],
            },
        ],
    }


def _request(
    url: str,
    *,
    payload: dict[str, object] | None = None,
    token: str = TOKEN,
) -> tuple[int, dict[str, object]]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Authorization": f"Bearer {token}"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as exc:
        return exc.code, json.load(exc)


def test_execution_catalog_is_approved_versioned_and_content_addressed():
    catalog = default_capability_registry().execution_catalog()

    assert catalog["contract_version"] == "flyto.robotics.capability-catalog.v1"
    assert len(catalog["contract_hash"]) == 64
    assert all(item["approval_status"] == "APPROVED" for item in catalog["capabilities"])
    assert all(item["executor_kind"] == "flyto-robotics" for item in catalog["capabilities"])
    assert next(
        item
        for item in catalog["capabilities"]
        if item["runtime_name"] == "move_relative"
    )["requires_safe_stop"] is True


def test_valid_dispatch_binds_judge_cards_revisions_calibration_and_registry():
    grant = parse_mission_dispatch(_dispatch(), default_capability_registry())

    assert grant.task_id == "mission-1"
    assert grant.plan_revision == 2
    assert grant.assignment_revision == 4
    assert grant.calibration.revision == 3
    assert grant.steps[0].runtime_name == "wait_until_clear"


def test_system_selected_cards_are_rejected():
    payload = _dispatch()
    payload["card_source"] = "system_random"

    with pytest.raises(MissionGatewayError, match="physical judge draw"):
        parse_mission_dispatch(payload, default_capability_registry())


def test_calibration_requires_all_four_stations_and_start():
    payload = _dispatch()
    calibration = payload["calibration"]
    assert isinstance(calibration, dict)
    calibration["markers"] = calibration["markers"][:-1]
    hash_payload = {
        key: value
        for key, value in calibration.items()
        if key not in {"contract_version", "contract_hash"}
    }
    calibration["contract_hash"] = _hash(hash_payload)

    with pytest.raises(MissionGatewayError, match="START"):
        parse_mission_dispatch(payload, default_capability_registry())


def test_stale_registry_and_unapproved_capability_fail_closed():
    stale = _dispatch()
    stale["capability_registry_hash"] = "0" * 64
    with pytest.raises(MissionGatewayError, match="registry snapshot"):
        parse_mission_dispatch(stale, default_capability_registry())

    unknown = _dispatch()
    unknown["steps"][0]["capability_id"] = "robotics.motor.raw@1"
    with pytest.raises(MissionGatewayError, match="not approved"):
        parse_mission_dispatch(unknown, default_capability_registry())


def test_raw_actuator_fields_and_missing_safe_stop_fail_before_controller():
    raw = _dispatch()
    raw["steps"][0]["velocity"] = 1.0
    with pytest.raises(MissionGatewayError, match="raw actuator"):
        parse_mission_dispatch(raw, default_capability_registry())

    movement = _dispatch()
    movement["steps"] = [
        {
            "step_id": "move",
            "capability_id": "robotics.motion.move_relative@1",
            "executor_kind": "flyto-robotics",
            "arguments": {"distance_m": 0.3, "speed": 0.12},
            "evidence_outputs": [],
        }
    ]
    with pytest.raises(MissionGatewayError, match="end with safe_stop"):
        parse_mission_dispatch(movement, default_capability_registry())


def test_action_success_receipt_cannot_complete_card_evidence():
    grant = parse_mission_dispatch(_dispatch(), default_capability_registry())
    evidence = build_action_evidence(
        grant,
        step_id="measure-clearance",
        status="VALID",
        outcome="SUCCEEDED",
        observed_at=NOW,
    )

    observation = evidence["evidence"]
    assert observation["kind"] == "action.execution"
    assert observation["value"]["task_completion_eligible"] is False
    assert observation["kind"] not in grant.evidence_requirements
    unsigned = {key: value for key, value in observation.items() if key != "digest"}
    assert observation["digest"] == _hash(unsigned)
    envelope = {key: value for key, value in evidence.items() if key != "contract_hash"}
    assert evidence["contract_hash"] == _hash(envelope)


def test_http_gateway_exposes_discovery_and_validation_but_no_motor_endpoint():
    with MissionGateway(default_capability_registry(), TOKEN) as gateway:
        host, port = gateway.address
        base = f"http://{host}:{port}"
        status, catalog = _request(f"{base}/v1/capabilities")
        assert status == 200
        assert catalog["contract_version"] == "flyto.robotics.capability-catalog.v1"

        status, body = _request(
            f"{base}/v1/missions/validate",
            payload=_dispatch(),
        )
        assert status == 200
        assert body["grant"]["task_id"] == "mission-1"

        status, body = _request(
            f"{base}/v1/cmd_vel",
            payload={"velocity": 1.0},
        )
        assert status == 404
        assert body["error"] == "not_found"

        status, body = _request(
            f"{base}/v1/missions/validate",
            payload=copy.deepcopy(_dispatch()),
            token="wrong",
        )
        assert status == 401
        assert body["error"] == "unauthorized"
