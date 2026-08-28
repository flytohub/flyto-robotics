"""Bounded, transport-neutral state for the robot-local camera observation API."""

from __future__ import annotations

import ipaddress
import json
import re
import time
from dataclasses import dataclass
from urllib.parse import urlsplit

ROUTE = "/api/spaces/zone-camera/observation"
STREAM_ROUTE = "/api/spaces/zone-camera/streams"
#: The catalog contract flyto-cloud's vision_stream_adapter validates before it
#: reads a single row. Answering with anything else is refused there by version
#: rather than misread, which is why it is stated and never inferred.
CATALOG_CONTRACT = "flyto.vision.stream-catalog.v1"
#: A reference is minted with an expiry. The platform clamps whatever arrives
#: into its own bounds, so this only has to be a sane declaration, not a policy.
MIN_STREAM_TTL = 1
MAX_STREAM_TTL = 900
DEFAULT_STREAM_TTL = 120
MAX_STREAM_URL = 2048
MAX_STREAM_LABEL = 128
#: What a browser can be handed. Kept as a set rather than a single value
#: because the media transport is a property of what is serving, not of this.
STREAM_PROTOCOLS = frozenset({"mjpeg", "whep", "webrtc", "hls"})
MAX_ZONE = 64
MAX_SOURCE_ID = 64
MAX_FRAME_BYTES = 32 * 1024 * 1024
DETAIL_USABLE = "camera_frame_fresh"
DETAIL_STALE = "camera_frame_stale"
_ZONE = re.compile(r"[A-Za-z0-9](?:[A-Za-z0-9._-]{0,63})\Z")


class CameraConfigurationError(ValueError):
    """A local gateway setting would widen or corrupt its trust boundary."""


def validate_bind(value: str) -> str:
    """Accept only a literal IPv4 loopback address, never a URL or hostname."""

    if not isinstance(value, str) or not value or value != value.strip():
        raise CameraConfigurationError("camera_bind_invalid")
    try:
        address = ipaddress.ip_address(value)
    except ValueError as exc:
        raise CameraConfigurationError("camera_bind_invalid") from exc
    if address.version != 4 or not address.is_loopback:
        raise CameraConfigurationError("camera_bind_invalid")
    return str(address)


def validate_stream_url(value: str) -> str:
    """Accept a well-formed absolute URL a browser could open. Nothing more.

    Deliberately *not* where plaintext-off-loopback is refused. flyto-cloud's
    ``services/space_tasks/streams.refuse_insecure_address`` owns that rule and
    says why in its own words: the gateway is the party that would be exposed,
    and a party cannot be relied on to refuse its own misconfiguration. A second
    copy of a security rule here would eventually disagree with that one, and
    the dangerous outcome is whichever copy is more permissive. So this checks
    that the value is a URL, and leaves whether it may be served to the side
    that is not the one being served.
    """

    if not isinstance(value, str) or not value or value != value.strip():
        raise CameraConfigurationError("camera_stream_url_invalid")
    if len(value) > MAX_STREAM_URL or not value.isascii():
        raise CameraConfigurationError("camera_stream_url_invalid")
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise CameraConfigurationError("camera_stream_url_invalid")
    parts = urlsplit(value)
    if parts.scheme not in {"http", "https", "ws", "wss"} or not parts.hostname:
        raise CameraConfigurationError("camera_stream_url_invalid")
    return value


def validate_zone(value: str) -> str:
    if not isinstance(value, str) or len(value) > MAX_ZONE or not _ZONE.fullmatch(value):
        raise CameraConfigurationError("camera_zone_invalid")
    return value


def validate_source_id(value: str) -> str:
    """Validate the public, operator-assigned source label."""

    if not isinstance(value, str) or len(value) > MAX_SOURCE_ID or not _ZONE.fullmatch(value):
        raise CameraConfigurationError("camera_source_id_invalid")
    return value


@dataclass(frozen=True)
class HttpResponse:
    status: int
    body: bytes
    allow: str | None = None


class CameraStreamCatalog:
    """Where this robot's camera can be watched, as an address and never a frame.

    The other half of the vision-gateway contract. ``CameraObservation`` answers
    *what can be seen*; this answers *where it can be watched*, and the two are
    separate questions with separate consequences — a venue whose media path is
    down must still be able to prove a zone was visible, and evidence must never
    depend on anything being watchable.

    **This holds no pixels and opens no device.** It maps one configured zone
    onto one address. The frame path is camera -> ROS topic -> web_video_server
    -> browser, and this process is on none of it.

    **An unconfigured catalog says so rather than inventing a URL.** A robot
    with a working camera and no media server in front of it has nothing to hand
    a browser, and a made-up address produces a viewer spinning forever against
    a port nobody is listening on. ``configured`` is the field that tells an
    operator which of the two they are looking at.
    """

    def __init__(self, zone: str, *, url: str = "", protocol: str = "mjpeg",
                 label: str = "", ttl_seconds: int = DEFAULT_STREAM_TTL):
        self.zone = validate_zone(zone)
        self.url = validate_stream_url(url) if url else ""
        if protocol not in STREAM_PROTOCOLS:
            raise CameraConfigurationError("camera_stream_protocol_invalid")
        self.protocol = protocol
        if not isinstance(label, str) or len(label) > MAX_STREAM_LABEL:
            raise CameraConfigurationError("camera_stream_label_invalid")
        if any(ord(char) < 32 for char in label):
            raise CameraConfigurationError("camera_stream_label_invalid")
        self.label = label or self.zone
        if isinstance(ttl_seconds, bool) or not isinstance(ttl_seconds, int):
            raise CameraConfigurationError("camera_stream_ttl_invalid")
        # Clamped rather than refused, matching what the platform does with the
        # same field: a host configured with a week-long lifetime should serve a
        # short one, not stop serving. The refusal that matters is minting.
        self.ttl_seconds = min(max(ttl_seconds, MIN_STREAM_TTL), MAX_STREAM_TTL)

    def payload(self) -> dict:
        if not self.url:
            return {
                "contract_version": CATALOG_CONTRACT,
                "configured": False,
                "unconfigured_reason": (
                    "FLYTO_CAMERA_STREAM_URL is unset, so this robot has no media "
                    "server to point a browser at. Watching needs one; observing "
                    "does not, and the observation route is unaffected."
                ),
                "streams": [],
            }
        return {
            "contract_version": CATALOG_CONTRACT,
            "configured": True,
            "unconfigured_reason": "",
            "streams": [{
                # The resource is the zone, named identically to the observation
                # route's. One camera answering "what is there" and "where to
                # watch it" under two different ids would be two cameras to
                # anyone approving them.
                "resource_id": self.zone,
                "zone_id": self.zone,
                "protocol": self.protocol,
                "url": self.url,
                "label": self.label,
                "ttl_seconds": self.ttl_seconds,
            }],
        }


class CameraObservation:
    """Keep only acceptance time; pixel buffers are never retained or serialized."""

    def __init__(
        self, zone: str, freshness_seconds: float, *, provider: str = "ros_image",
        source_id: str = "camera-0", clock=time.monotonic,
        streams: CameraStreamCatalog | None = None,
    ):
        self.zone = validate_zone(zone)
        if provider not in {"ros_image", "avfoundation"}:
            raise CameraConfigurationError("camera_provider_invalid")
        self.provider = provider
        self.source_id = validate_source_id(source_id)
        if isinstance(freshness_seconds, bool) or not isinstance(freshness_seconds, (int, float)):
            raise CameraConfigurationError("camera_freshness_invalid")
        if not 0.1 <= freshness_seconds <= 300:
            raise CameraConfigurationError("camera_freshness_invalid")
        self.freshness_seconds = float(freshness_seconds)
        self._clock = clock
        self._accepted_at: float | None = None
        # Optional, and absent means the streams route answers 404 rather than
        # an empty catalog. "This build does not serve that contract" and "this
        # robot has no media server" are different answers, and a caller that
        # cannot tell them apart goes looking for the wrong fault.
        self.streams = streams

    def accept_frame(self) -> bool:
        """Accept proof of one decoded frame, retaining no frame dimensions or content."""

        self._accepted_at = self._clock()
        return True

    def clear(self) -> None:
        self._accepted_at = None

    def accept_image(self, *, encoding: str, width: int, height: int, step: int, data) -> bool:
        """Validate an RGB/BGR frame without copying or retaining its content."""

        valid_shape = (
            encoding in {"rgb8", "bgr8"}
            and type(width) is int
            and type(height) is int
            and type(step) is int
            and 1 <= width <= 8192
            and 1 <= height <= 8192
            and step == width * 3
            and step * height <= MAX_FRAME_BYTES
        )
        try:
            size = len(data)
        except (TypeError, OverflowError):
            size = -1
        if not valid_shape or size != step * height:
            self.clear()
            return False
        return self.accept_frame()

    def payload(self) -> list[dict]:
        if self._accepted_at is None:
            return []
        usable = 0 <= self._clock() - self._accepted_at <= self.freshness_seconds
        return [{
            "kind": "zone.overview",
            "zone": self.zone,
            "source": {"provider": self.provider, "source_id": self.source_id},
            "usable": usable,
            "detail": DETAIL_USABLE if usable else DETAIL_STALE,
        }]

    def handle(self, method: str, path: str, *, request_size: int = 0) -> HttpResponse:
        if request_size < 0 or request_size > 1024:
            return HttpResponse(413, b'{"error":"request_too_large"}')
        if path == STREAM_ROUTE and self.streams is not None:
            if method != "GET":
                return HttpResponse(405, b'{"error":"method_not_allowed"}', "GET")
            body = json.dumps(
                self.streams.payload(), separators=(",", ":"), sort_keys=True,
            ).encode("ascii")
            return HttpResponse(200, body)
        if path != ROUTE:
            return HttpResponse(404, b'{"error":"not_found"}')
        if method != "GET":
            return HttpResponse(405, b'{"error":"method_not_allowed"}', "GET")
        body = json.dumps(self.payload(), separators=(",", ":")).encode("ascii")
        return HttpResponse(200, body)
