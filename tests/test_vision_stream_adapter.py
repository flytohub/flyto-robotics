"""The camera end of the reference design, against a fake gateway.

What is tested is the translation and its refusals — which catalog rows become
declarations, which are recorded as unusable, and that the reference goes
through ``streams.mint`` rather than being assembled by the adapter, so a
gateway offering something unsafe is refused by the same code that refuses it
everywhere else.
"""

from __future__ import annotations

import pytest

from flyto_robotics import adapter_contract as decl
from flyto_robotics import stream_contract as st
from flyto_robotics.vision_stream_adapter import (
    CATALOG_CONTRACT,
    CATALOG_PATH_ENV,
    DEFAULT_CATALOG_PATH,
    GATEWAY_TO_PROTOCOL,
    VisionStreamAdapter,
    catalog_path,
)


def row(**overrides):
    payload = {
        "resource_id": "cam-z3",
        "zone_id": "zone-03",
        "protocol": "whep",
        "url": "https://media.local:8889/zone3/whep",
        "label": "Zone 3 overhead",
        "ttl_seconds": 120,
        "expires_at": "2026-08-18T07:02:00Z",
    }
    payload.update(overrides)
    return payload


def adapter(*rows, contract=CATALOG_CONTRACT, seen=None):
    def fetch(path, **_):
        if seen is not None:
            seen.append(path)
        return {"contract_version": contract, "streams": list(rows)}

    return VisionStreamAdapter(fetch=fetch)


# -- the contract -------------------------------------------------------------


def test_a_catalog_of_the_wrong_contract_is_refused():
    with pytest.raises(ValueError):
        adapter(row(), contract="flyto.vision.stream-catalog.v99").describe()


def test_every_gateway_protocol_lands_on_one_the_vocabulary_admits():
    assert set(GATEWAY_TO_PROTOCOL.values()) <= st.PROTOCOLS


# -- which gateway ------------------------------------------------------------


def test_it_asks_the_gateway_that_exists_today(monkeypatch):
    """flyto-cloud's own local backend, on the host the camera is plugged into.
    Defaulting to the robot's future path would fetch a 404 and report an empty
    catalog, which reads as "this Space has no cameras"."""
    monkeypatch.delenv(CATALOG_PATH_ENV, raising=False)
    seen = []
    adapter(row(), seen=seen).describe()
    assert seen == [DEFAULT_CATALOG_PATH] == ["/api/spaces/zone-camera/streams"]


def test_the_robot_s_own_gateway_is_a_variable_not_a_rewrite(monkeypatch):
    monkeypatch.setenv(CATALOG_PATH_ENV, "/v1/streams")
    seen = []
    adapter(row(), seen=seen).describe()
    assert seen == ["/v1/streams"]


def test_a_path_configured_without_its_leading_slash_still_works(monkeypatch):
    monkeypatch.setenv(CATALOG_PATH_ENV, "v1/streams")
    assert catalog_path() == "/v1/streams"


# -- what becomes a declaration ----------------------------------------------


def test_a_camera_becomes_a_declaration_nobody_approved():
    """Plugging a camera in is not a decision about who may watch a room."""
    device = adapter(row())
    declared = device.describe()
    assert len(declared) == 1
    assert declared[0].capability_id == st.STREAM_CAPABILITY
    assert declared[0].approval_status == decl.STATUS_DISCOVERED
    assert declared[0].usable is False


def test_the_executor_is_the_media_server_not_this_platform():
    device = adapter(row())
    assert device.describe()[0].executor_kind == decl.EXECUTOR_EXTERNAL_API


def test_watching_is_read_only_and_needs_no_permission_or_stop():
    device = adapter(row())
    item = device.describe()[0]
    assert item.safety_class == "read_only"
    assert item.requires_safe_stop is False
    assert item.required_permissions == ()


def test_a_row_with_no_resource_id_is_recorded_rather_than_guessed():
    device = adapter(row(resource_id=""))
    assert device.describe() == []
    assert device.unmapped == [("", "a stream with no resource id")]


def test_a_protocol_nobody_recognises_is_recorded_rather_than_passed_through():
    """A gateway inventing a protocol name must reach a refusal here, not a
    black rectangle in the room."""
    device = adapter(row(protocol="magicstream"))
    assert device.describe() == []
    assert device.unmapped[0][1] == "protocol 'magicstream' unknown"


def test_several_cameras_each_declare_separately():
    device = adapter(
        row(resource_id="cam-z1", zone_id="zone-01"),
        row(resource_id="cam-z2", zone_id="zone-02"),
    )
    assert [item.resource_id for item in device.describe()] == ["cam-z1", "cam-z2"]


# -- the reference goes through mint -----------------------------------------


def test_the_address_is_handed_back_unchanged():
    device = adapter(row())
    device.describe()
    reference = device.stream_reference("cam-z3")
    assert reference.url == "https://media.local:8889/zone3/whep"
    assert reference.protocol == st.PROTOCOL_WHEP
    assert reference.zone_id == "zone-03"


def test_a_plaintext_address_off_the_loopback_is_refused_by_mint():
    """The adapter cannot skip the check by building its own reference."""
    device = adapter(row(url="http://192.168.1.50:8889/whep"))
    device.describe()
    with pytest.raises(st.StreamRefused) as excinfo:
        device.stream_reference("cam-z3")
    assert st.REFUSAL_PLAINTEXT in str(excinfo.value)


def test_a_loopback_address_is_allowed():
    device = adapter(row(url="http://127.0.0.1:8889/whep", protocol="mjpeg"))
    device.describe()
    assert device.stream_reference("cam-z3").protocol == st.PROTOCOL_MJPEG


def test_a_gateway_lifetime_beyond_the_ceiling_is_refused():
    device = adapter(row(ttl_seconds=st.MAX_TTL_SECONDS + 1))
    device.describe()
    with pytest.raises(st.StreamRefused) as excinfo:
        device.stream_reference("cam-z3")
    assert st.REFUSAL_TTL in str(excinfo.value)


def test_a_measured_latency_from_the_gateway_is_carried():
    device = adapter(row(measured_latency_ms=210))
    device.describe()
    reference = device.stream_reference("cam-z3")
    assert reference.latency_ms == 210
    assert reference.to_dict()["latency_measured"] is True


def test_segmented_playback_is_still_reported_as_behind():
    device = adapter(row(protocol="hls", url="https://media.local:8888/z3.m3u8"))
    device.describe()
    assert device.stream_reference("cam-z3").delayed is True


def test_a_resource_the_gateway_does_not_serve_answers_nothing():
    device = adapter(row())
    device.describe()
    assert device.stream_reference("cam-z9") is None


# -- the three methods that do not apply to a camera --------------------------


def test_a_stream_is_watched_not_dispatched():
    from flyto_robotics.adapter_contract import CallRequest

    device = adapter(row())
    device.describe()
    assert device.cancel("x").outcome == "refused"
    assert device.invoke(CallRequest("x", "motion.navigate")).outcome == "refused"
    stream = device.invoke(CallRequest("stream", "vision.stream"))
    assert stream.outcome == "refused"
    assert "stream_reference" in stream.detail


def test_a_camera_has_nothing_to_bring_to_rest():
    """Answering 'stopped' about a device that never moves would be a stop that
    proved nothing."""
    assert adapter(row()).safe_stop().outcome == "refused"


