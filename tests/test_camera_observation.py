import json

import pytest

from flyto_robotics.camera_gateway import _settings
from flyto_robotics.camera_observation import (
    CATALOG_CONTRACT,
    ROUTE,
    STREAM_ROUTE,
    CameraConfigurationError,
    CameraObservation,
    CameraStreamCatalog,
    validate_bind,
)


def test_offline_gateway_settings_match_the_canonical_consumer_default(monkeypatch):
    for name in (
        "FLYTO_CAMERA_BIND",
        "FLYTO_CAMERA_PORT",
        "FLYTO_CAMERA_TOPIC",
        "FLYTO_CAMERA_ZONE",
        "FLYTO_CAMERA_FRESHNESS_SECONDS",
        "FLYTO_CAMERA_PROVIDER",
        "FLYTO_CAMERA_SOURCE_ID",
        "FLYTO_CAMERA_ROTATION",
        "FLYTO_CAMERA_FLIP",
        "FLYTO_CAMERA_DEVICE",
    ):
        monkeypatch.delenv(name, raising=False)
    assert _settings() == ("127.0.0.1", 9000, "/camera/image_raw", "camera-zone", 2.0)


def test_camera_port_remains_bounded_operator_configuration(monkeypatch):
    monkeypatch.setenv("FLYTO_CAMERA_PORT", "9100")
    assert _settings()[1] == 9100
    monkeypatch.setenv("FLYTO_CAMERA_PORT", "0")
    with pytest.raises(ValueError, match="camera_port_invalid"):
        _settings()


@pytest.mark.parametrize("value", ["0.0.0.0", "localhost", "::1", "127.0.0.1:8",
                                         "http://127.0.0.1", "127.0.0.1/path", "user@127.0.0.1"])
def test_bind_rejects_everything_except_literal_ipv4_loopback(value):
    with pytest.raises(CameraConfigurationError):
        validate_bind(value)


def test_fixed_route_method_and_pre_frame_contract():
    state = CameraObservation("ward-a", 2, clock=lambda: 1)
    assert json.loads(state.handle("GET", ROUTE).body) == []
    assert state.handle("GET", "/other").status == 404
    response = state.handle("POST", ROUTE)
    assert (response.status, response.allow) == (405, "GET")
    assert state.handle("GET", ROUTE, request_size=1025).status == 413


def test_valid_frame_is_explicitly_usable_then_stale_without_pixels():
    now = [10.0]
    state = CameraObservation("ward-a", 2, provider="ros_image", source_id="overhead-1",
                              clock=lambda: now[0])
    pixels = b"patient-content" * 3
    assert state.accept_image(encoding="rgb8", width=15, height=1, step=45, data=pixels)
    item = json.loads(state.handle("GET", ROUTE).body)[0]
    assert item == {"kind": "zone.overview", "zone": "ward-a", "usable": True,
                    "detail": "camera_frame_fresh",
                    "source": {"provider": "ros_image", "source_id": "overhead-1"}}
    encoded = json.dumps(item)
    assert "patient" not in encoded and "rgb8" not in encoded and "/camera" not in encoded
    now[0] = 13.0
    stale = state.payload()[0]
    assert stale == {"kind": "zone.overview", "zone": "ward-a", "usable": False,
                     "detail": "camera_frame_stale",
                     "source": {"provider": "ros_image", "source_id": "overhead-1"}}


def test_provider_frame_proof_keeps_only_bounded_metadata():
    state = CameraObservation("ward-a", 2, provider="avfoundation", source_id="usb-1",
                              clock=lambda: 1)
    assert state.accept_frame()
    assert state.payload()[0]["source"] == {
        "provider": "avfoundation", "source_id": "usb-1",
    }
    assert "width" not in state.payload()[0]


@pytest.mark.parametrize("kwargs", [
    {"encoding": "mono8", "width": 1, "height": 1, "step": 1, "data": b"x"},
    {"encoding": "bgr8", "width": 1, "height": 1, "step": 4, "data": b"xxxx"},
    {"encoding": "bgr8", "width": 1, "height": 1, "step": 3, "data": b"xx"},
])
def test_malformed_frames_fail_closed(kwargs):
    state = CameraObservation("ward-a", 2, clock=lambda: 1)
    assert not state.accept_image(**kwargs)
    assert state.payload() == []


def test_zone_and_freshness_are_bounded():
    with pytest.raises(CameraConfigurationError):
        CameraObservation("x" * 65, 2)
    with pytest.raises(CameraConfigurationError):
        CameraObservation("ward/a", 2)
    with pytest.raises(CameraConfigurationError):
        CameraObservation("ward-a", 301)


def test_unconfigured_catalog_says_so_rather_than_inventing_an_address():
    """A robot with no media server must be distinguishable from a broken one.

    An empty stream list on its own reads as "this room has no cameras" and
    sends an operator looking for a hardware fault that is not there.
    """
    catalog = CameraStreamCatalog("robot-front")
    body = catalog.payload()
    assert body["contract_version"] == CATALOG_CONTRACT
    assert body["configured"] is False
    assert body["streams"] == []
    assert "FLYTO_CAMERA_STREAM_URL" in body["unconfigured_reason"]


def test_configured_catalog_serves_the_contract_the_adapter_validates():
    catalog = CameraStreamCatalog(
        "robot-front",
        url="http://127.0.0.1:8080/stream?topic=/camera/image_raw",
        protocol="mjpeg",
        label="TurtleBot3 front camera",
    )
    body = catalog.payload()
    assert body["contract_version"] == CATALOG_CONTRACT
    assert body["configured"] is True
    assert body["unconfigured_reason"] == ""
    (row,) = body["streams"]
    assert row["resource_id"] == "robot-front"
    assert row["zone_id"] == "robot-front"
    assert row["protocol"] == "mjpeg"
    assert row["url"] == "http://127.0.0.1:8080/stream?topic=/camera/image_raw"
    assert row["label"] == "TurtleBot3 front camera"
    assert row["ttl_seconds"] == 120


def test_a_camera_answers_both_questions_under_one_id():
    """`resource_id` must equal the zone the observation route reports.

    One camera answering "what is there" and "where to watch it" under two
    different ids is two cameras to whoever approves them, and approving the
    stream would not be approving the thing that produced the evidence.
    """
    state = CameraObservation(
        "robot-front", 2, clock=lambda: 1,
        streams=CameraStreamCatalog("robot-front", url="http://127.0.0.1:8080/s"),
    )
    state.accept_frame()
    (observed,) = state.payload()
    (offered,) = state.streams.payload()["streams"]
    assert observed["zone"] == offered["zone_id"] == offered["resource_id"]


def test_streams_route_is_absent_rather_than_empty_when_no_catalog_is_built():
    """404, not an empty catalog: "this build does not serve that contract" and
    "this robot has no media server" are different answers."""
    state = CameraObservation("robot-front", 2, clock=lambda: 1)
    assert state.handle("GET", STREAM_ROUTE).status == 404


def test_streams_route_refuses_anything_but_get():
    state = CameraObservation(
        "robot-front", 2, clock=lambda: 1, streams=CameraStreamCatalog("robot-front"),
    )
    result = state.handle("POST", STREAM_ROUTE)
    assert result.status == 405
    assert result.allow == "GET"


def test_streams_route_serves_ascii_json_the_adapter_can_parse():
    state = CameraObservation(
        "robot-front", 2, clock=lambda: 1,
        streams=CameraStreamCatalog(
            "robot-front", url="http://127.0.0.1:8080/s", label="前視鏡頭",
        ),
    )
    result = state.handle("GET", STREAM_ROUTE)
    assert result.status == 200
    body = json.loads(result.body)
    assert body["contract_version"] == CATALOG_CONTRACT
    # Non-ASCII labels survive as escapes rather than breaking the ascii encode.
    assert body["streams"][0]["label"] == "前視鏡頭"


def test_observation_route_is_unaffected_by_a_configured_stream():
    """Evidence must never depend on anything being watchable."""
    state = CameraObservation(
        "robot-front", 2, clock=lambda: 1,
        streams=CameraStreamCatalog("robot-front", url="http://127.0.0.1:8080/s"),
    )
    state.accept_frame()
    result = state.handle("GET", ROUTE)
    assert result.status == 200
    assert json.loads(result.body)[0]["usable"] is True


@pytest.mark.parametrize(
    "url",
    [
        "not-a-url",
        "ftp://host/stream",          # not a scheme a browser opens
        "http://",                    # no host
        "http://host/\nstream",       # control character
        "http://host/" + "x" * 3000,  # unbounded
        " http://host/stream",        # unstripped
    ],
)
def test_stream_url_is_refused_when_it_is_not_one_a_browser_could_open(url):
    with pytest.raises(CameraConfigurationError):
        CameraStreamCatalog("robot-front", url=url)


def test_stream_protocol_and_label_are_bounded():
    with pytest.raises(CameraConfigurationError):
        CameraStreamCatalog("robot-front", protocol="carrier-pigeon")
    with pytest.raises(CameraConfigurationError):
        CameraStreamCatalog("robot-front", label="x" * 129)


def test_stream_ttl_is_clamped_rather_than_refused():
    """A host misconfigured with a week-long lifetime should serve a short one,
    not stop serving. The refusal that matters happens where it is minted."""
    assert CameraStreamCatalog("robot-front", ttl_seconds=100_000).ttl_seconds == 900
    assert CameraStreamCatalog("robot-front", ttl_seconds=0).ttl_seconds == 1
    with pytest.raises(CameraConfigurationError):
        CameraStreamCatalog("robot-front", ttl_seconds=True)
