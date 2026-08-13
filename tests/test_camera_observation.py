import json

import pytest

from flyto_robotics.camera_gateway import _settings
from flyto_robotics.camera_observation import (
    ROUTE,
    CameraConfigurationError,
    CameraObservation,
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
