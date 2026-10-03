"""A discovered resource names the module pack that drives it, on request.

flyto-modules-robotics declares this library's capabilities to Flyto2 through
``@register_module``; its ``flyto.modules`` entry point is ``robotics``. A host
joins a discovered resource to that pack by the manifest's ``module_pack``.

Released hosts validate the manifest with ``extra="forbid"``, so the field is
emitted only when the host asks for it. Without the request the manifest keeps
exactly the shape those hosts were released against.
"""

from __future__ import annotations

import io
import json
from contextlib import redirect_stdout
from types import SimpleNamespace

import pytest

import flyto_robotics.adapter_provider as provider
from flyto_robotics import adapter_contract as contract

LEGACY_FIELDS = {
    "contract",
    "resource_id",
    "resource_type",
    "display_name",
    "revision",
    "adapter",
    "deployment_mode",
    "capability_ids",
    "capability_contracts",
    "settings",
    "telemetry_channels",
    "observed_at",
    "contract_hash",
}


class FakeAdapter:
    resource_id = "robot-1"

    def __init__(self) -> None:
        self.disconnected = False

    def describe(self):
        return (
            contract.declare(
                capability_id="motion.halt",
                resource_id=self.resource_id,
                executor_kind=contract.EXECUTOR_EXTERNAL_API,
                source=contract.SOURCE_DEVICE,
            ),
        )

    def disconnect(self) -> None:
        self.disconnected = True


@pytest.fixture
def discoverable(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_TRANSPORT", "rosbridge")
    monkeypatch.setenv("FLYTO_ROSBRIDGE_URL", "ws://127.0.0.1:1")
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    monkeypatch.delenv("FLYTO_ROS2_AUTODISCOVER", raising=False)
    monkeypatch.setattr(provider, "_watched_adapter", lambda _rid: None)
    monkeypatch.setattr(provider, "build", lambda _rid: FakeAdapter())


def test_the_pack_name_is_the_flyto_modules_entry_point_of_the_robotics_pack():
    assert provider.MODULE_PACK == "robotics"
    assert provider.MANIFEST_EXTENSIONS == ("module_pack",)


def test_a_host_that_asks_nothing_gets_the_released_manifest_shape(discoverable):
    (manifest,) = provider.discover_resource_manifests(execution_host_id="host")
    assert set(manifest) == LEGACY_FIELDS
    assert "module_pack" not in manifest


def test_a_host_that_asks_for_module_pack_gets_it_beside_every_existing_field(
    discoverable,
):
    (legacy,) = provider.discover_resource_manifests(execution_host_id="host")
    (joined,) = provider.discover_resource_manifests(
        execution_host_id="host", manifest_extensions=["module_pack"]
    )
    assert joined["module_pack"] == "robotics"
    assert set(joined) == LEGACY_FIELDS | {"module_pack"}
    for key in LEGACY_FIELDS - {"observed_at"}:
        assert joined[key] == legacy[key]


@pytest.mark.parametrize("requested", [None, (), ["unknown"], "module_pack", 7])
def test_unknown_or_malformed_requests_add_nothing(discoverable, requested):
    (manifest,) = provider.discover_resource_manifests(
        execution_host_id="host", manifest_extensions=requested
    )
    assert set(manifest) == LEGACY_FIELDS


def test_the_discoverer_advertises_the_pack_and_its_extensions():
    discover = provider.discover_resource_manifests
    assert discover.module_pack == "robotics"
    assert discover.manifest_extensions == ("module_pack",)
    assert discover.watch is provider.watch_resources


def test_the_describe_operation_honours_the_same_request():
    adapter = FakeAdapter()
    plain = provider._manifest(adapter, resource_name="Robot")
    joined = provider._manifest(
        adapter, resource_name="Robot", extensions=("module_pack",)
    )
    assert set(plain) == LEGACY_FIELDS
    assert joined["module_pack"] == "robotics"


def test_the_cli_discover_flag_passes_the_request(discoverable):
    out = io.StringIO()
    with redirect_stdout(out):
        assert provider.main(
            ["--discover", "--execution-host-id", "h", "--manifest-extension", "module_pack"]
        ) == 0
    payload = json.loads(out.getvalue())
    assert payload["resources"][0]["module_pack"] == "robotics"

    out = io.StringIO()
    with redirect_stdout(out):
        assert provider.main(["--discover", "--execution-host-id", "h"]) == 0
    assert "module_pack" not in json.loads(out.getvalue())["resources"][0]


def test_the_manifest_without_the_request_passes_a_strict_validator():
    """Mirror of a released host's extra="forbid" check."""
    manifest = provider._manifest(FakeAdapter(), resource_name="Robot")
    unknown = set(manifest) - LEGACY_FIELDS
    assert not unknown, SimpleNamespace(unknown=unknown)
