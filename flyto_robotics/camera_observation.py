"""Bounded, transport-neutral state for the robot-local camera observation API."""

from __future__ import annotations

import ipaddress
import json
import re
import time
from dataclasses import dataclass

ROUTE = "/api/spaces/zone-camera/observation"
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


class CameraObservation:
    """Keep only acceptance time; pixel buffers are never retained or serialized."""

    def __init__(
        self, zone: str, freshness_seconds: float, *, provider: str = "ros_image",
        source_id: str = "camera-0", clock=time.monotonic,
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
        if path != ROUTE:
            return HttpResponse(404, b'{"error":"not_found"}')
        if method != "GET":
            return HttpResponse(405, b'{"error":"method_not_allowed"}', "GET")
        body = json.dumps(self.payload(), separators=(",", ":")).encode("ascii")
        return HttpResponse(200, body)
