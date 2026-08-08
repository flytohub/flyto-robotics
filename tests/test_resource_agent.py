from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path

import pytest

from flyto_robotics.resource_agent import (
    FlytoCloudResourcePublisher,
    ResourceContractError,
    parse_resource_manifest,
    parse_resource_telemetry,
)


def manifest_payload(*, deployment_mode: str = "simulation") -> dict:
    return {
        "contract": "flyto.resource-manifest.v1",
        "resource_id": "facility-resource-1",
        "resource_type": "mobile-platform",
        "display_name": "Facility resource",
        "revision": 1,
        "adapter": {
            "adapter_id": "adapter.reference",
            "version": "1.0.0",
            "provider": "reference",
        },
        "deployment_mode": deployment_mode,
        "capability_ids": ["mobility.navigate", "sensing.range"],
        "settings": [
            {
                "setting_id": "safety.stop_distance",
                "display_name": "Stop distance",
                "value_kind": "number",
                "value": 0.4,
                "configured": True,
                "secret": False,
                "mutable": True,
                "unit": "m",
                "options": [],
                "description": "Minimum clearance before the local stop policy latches.",
            },
            {
                "setting_id": "cloud.api_token",
                "display_name": "Cloud credential",
                "value_kind": "string",
                "value": None,
                "configured": True,
                "secret": True,
                "mutable": False,
                "unit": "",
                "options": [],
                "description": "Only configured state leaves the installation.",
            },
        ],
        "telemetry_channels": [
            {
                "channel_id": "power.level",
                "display_name": "Power level",
                "payload_schema": {
                    "type": "object",
                    "properties": {"value": {"type": "number"}},
                },
                "presentation": {"kind": "metric", "unit": "%", "precision": 1},
                "max_age_seconds": 30.0,
            }
        ],
        "observed_at": "2026-08-08T10:00:00Z",
    }


def telemetry_payload() -> dict:
    return {
        "contract": "flyto.resource-telemetry.v1",
        "resource_id": "facility-resource-1",
        "channel_id": "power.level",
        "sequence": 7,
        "observed_at": "2026-08-08T10:00:01Z",
        "quality": "good",
        "payload": {"value": 82.5},
    }


def test_real_and_simulation_share_one_manifest_shape():
    simulated = parse_resource_manifest(manifest_payload())
    real = parse_resource_manifest(manifest_payload(deployment_mode="real"))

    assert set(simulated) == set(real)
    assert simulated["deployment_mode"] == "simulation"
    assert real["deployment_mode"] == "real"
    assert simulated["capability_ids"] == real["capability_ids"]
    assert len(simulated["contract_hash"]) == 64


def test_future_presentation_kinds_are_namespaced_and_forward_compatible():
    future = manifest_payload()
    future["telemetry_channels"][0]["presentation"]["kind"] = (
        "partner.timeline.v2"
    )
    assert parse_resource_manifest(future)["telemetry_channels"][0][
        "presentation"
    ]["kind"] == "partner.timeline.v2"

    unsafe = manifest_payload()
    unsafe["telemetry_channels"][0]["presentation"]["kind"] = "<script>"
    with pytest.raises(ResourceContractError, match="safe identifier"):
        parse_resource_manifest(unsafe)


def test_secret_values_and_command_fields_fail_closed():
    leaked = manifest_payload()
    leaked["settings"][1]["value"] = "do-not-upload"
    with pytest.raises(ResourceContractError, match="must never enter Cloud"):
        parse_resource_manifest(leaked)

    command_surface = manifest_payload()
    command_surface["commands"] = [{"name": "direct-actuation"}]
    with pytest.raises(ResourceContractError, match="unknown resource fields"):
        parse_resource_manifest(command_surface)

    camel_case_secret = manifest_payload()
    camel_case_secret["settings"][1]["setting_id"] = "cloud.apiToken"
    camel_case_secret["settings"][1]["secret"] = False
    with pytest.raises(ResourceContractError, match="declared secret"):
        parse_resource_manifest(camel_case_secret)

    non_secret_token_limit = manifest_payload()
    non_secret_token_limit["settings"][1] = {
        "setting_id": "model.token_limit",
        "display_name": "Token limit",
        "value_kind": "integer",
        "value": 2048,
        "configured": True,
    }
    assert parse_resource_manifest(non_secret_token_limit)["settings"][1]["value"] == 2048


def test_telemetry_is_bounded_and_content_addressed():
    telemetry = parse_resource_telemetry(telemetry_payload())
    assert telemetry["payload"] == {"value": 82.5}
    assert len(telemetry["payload_hash"]) == 64

    invalid = telemetry_payload()
    invalid["payload"] = {"samples": list(range(513))}
    with pytest.raises(ResourceContractError, match="list is too large"):
        parse_resource_telemetry(invalid)


class _Response:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    @staticmethod
    def read(_limit):
        return b'{"ok":true}'


class _Opener:
    def __init__(self):
        self.requests = []

    def open(self, request, timeout):
        self.requests.append((request, timeout))
        return _Response()


def test_publisher_uses_exact_paired_device_boundary_without_body_secret():
    opener = _Opener()
    secret = "s" * 48
    publisher = FlytoCloudResourcePublisher(
        cloud_url="https://api.example.test",
        device_id="device-1",
        credential_loader=lambda: secret,
        opener=opener,
    )

    assert publisher.publish_manifest(manifest_payload()) == {"ok": True}
    assert publisher.publish_telemetry(telemetry_payload()) == {"ok": True}

    manifest_request = opener.requests[0][0]
    assert manifest_request.full_url.endswith(
        "/api/devices/device-1/resources/facility-resource-1"
    )
    assert manifest_request.get_method() == "PUT"
    assert secret not in manifest_request.data.decode("utf-8")
    assert manifest_request.get_header("Authorization") == (
        f"Bearer device:device-1.{secret}"
    )
    assert opener.requests[1][0].get_method() == "POST"


def test_schema_files_publish_the_same_contract_ids():
    root = Path(__file__).resolve().parent.parent
    manifest_schema = json.loads(
        (root / "contracts/resource-manifest-v1.schema.json").read_text()
    )
    telemetry_schema = json.loads(
        (root / "contracts/resource-telemetry-v1.schema.json").read_text()
    )
    assert manifest_schema["properties"]["contract"]["const"] == (
        "flyto.resource-manifest.v1"
    )
    assert telemetry_schema["properties"]["contract"]["const"] == (
        "flyto.resource-telemetry.v1"
    )
    presentation_kind = manifest_schema["$defs"]["channel"]["properties"][
        "presentation"
    ]["properties"]["kind"]
    assert presentation_kind == {"$ref": "#/$defs/identifier"}


def test_manifest_hash_rejects_tampering():
    signed = parse_resource_manifest(manifest_payload())
    tampered = deepcopy(signed)
    tampered["display_name"] = "Changed after signing"
    with pytest.raises(ResourceContractError, match="hash does not match"):
        parse_resource_manifest(tampered)
