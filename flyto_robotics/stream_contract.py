"""External live-stream reference contract for adapter providers."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any
from urllib.parse import urlsplit

from .adapter_contract import CapabilityDeclaration

STREAM_CAPABILITY = "vision.stream"
STREAM_SCHEMA = "flyto.space.stream-reference.v1"

PROTOCOL_WEBRTC = "webrtc"
PROTOCOL_WHEP = "whep"
PROTOCOL_MJPEG = "mjpeg"
PROTOCOL_HLS = "hls"
PROTOCOLS = frozenset({
    PROTOCOL_WEBRTC,
    PROTOCOL_WHEP,
    PROTOCOL_MJPEG,
    PROTOCOL_HLS,
})
PROTOCOL_TYPICAL_LATENCY_MS: Mapping[str, int] = {
    PROTOCOL_WEBRTC: 200,
    PROTOCOL_WHEP: 250,
    PROTOCOL_MJPEG: 500,
    PROTOCOL_HLS: 6000,
}
DELAYED_ABOVE_MS = 1000
DEFAULT_TTL_SECONDS = 120
MAX_TTL_SECONDS = 900
MAX_URL_LENGTH = 2048

LOOPBACK_HOSTS = frozenset({"127.0.0.1", "::1", "localhost"})
SECURE_SCHEMES = frozenset({"https", "wss"})
PLAINTEXT_SCHEMES = frozenset({"http", "ws"})

REFUSAL_NOT_APPROVED = "stream_not_approved"
REFUSAL_UNKNOWN_PROTOCOL = "stream_protocol_unknown"
REFUSAL_PLAINTEXT = "stream_plaintext_off_loopback"
REFUSAL_NO_URL = "stream_url_missing"
REFUSAL_URL_TOO_LONG = "stream_url_too_long"
REFUSAL_BAD_SCHEME = "stream_scheme_unusable"
REFUSAL_WRONG_CAPABILITY = "stream_capability_mismatch"
REFUSAL_TTL = "stream_ttl_out_of_range"
REFUSAL_WRONG_RESOURCE = "stream_resource_mismatch"
REFUSAL_NO_RESOURCE = "stream_resource_missing"


class StreamRefused(ValueError):
    def __init__(self, code: str, detail: str):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class StreamReference:
    resource_id: str
    zone_id: str
    protocol: str
    url: str
    expires_at: str
    label: str = ""
    audio: bool = False
    measured_latency_ms: int | None = None

    @property
    def latency_ms(self) -> int:
        return (
            int(self.measured_latency_ms)
            if self.measured_latency_ms is not None
            else PROTOCOL_TYPICAL_LATENCY_MS[self.protocol]
        )

    @property
    def delayed(self) -> bool:
        return self.latency_ms >= DELAYED_ABOVE_MS

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": STREAM_SCHEMA,
            "resource_id": self.resource_id,
            "zone_id": self.zone_id,
            "protocol": self.protocol,
            "url": self.url,
            "expires_at": self.expires_at,
            "label": self.label,
            "audio": self.audio,
            "latency_ms": self.latency_ms,
            "latency_measured": self.measured_latency_ms is not None,
            "delayed": self.delayed,
        }


def refuse_insecure_address(url: str) -> None:
    parsed = urlsplit(url)
    scheme = (parsed.scheme or "").lower()
    if scheme not in SECURE_SCHEMES | PLAINTEXT_SCHEMES:
        raise StreamRefused(
            REFUSAL_BAD_SCHEME,
            f"{scheme or 'no scheme'} is not one a browser opens",
        )
    if scheme in SECURE_SCHEMES:
        return
    host = (parsed.hostname or "").lower()
    if host not in LOOPBACK_HOSTS:
        raise StreamRefused(
            REFUSAL_PLAINTEXT,
            f"{scheme}://{host} would serve a camera in the clear",
        )


def mint(
    declaration: CapabilityDeclaration,
    *,
    protocol: str,
    url: str,
    expires_at: str,
    resource_id: str = "",
    zone_id: str = "",
    label: str = "",
    audio: bool = False,
    measured_latency_ms: int | None = None,
    ttl_seconds: int = DEFAULT_TTL_SECONDS,
) -> StreamReference:
    if declaration.capability_id != STREAM_CAPABILITY:
        raise StreamRefused(
            REFUSAL_WRONG_CAPABILITY,
            f"{declaration.capability_id} is not {STREAM_CAPABILITY}",
        )
    if not declaration.usable:
        raise StreamRefused(
            REFUSAL_NOT_APPROVED,
            f"{declaration.resource_id} is not approved for {STREAM_CAPABILITY}",
        )
    resource_id = str(resource_id or "").strip() or declaration.resource_id
    if declaration.resource_id and declaration.resource_id != resource_id:
        raise StreamRefused(
            REFUSAL_WRONG_RESOURCE,
            f"approval is for {declaration.resource_id}, not {resource_id}",
        )
    if not resource_id:
        raise StreamRefused(REFUSAL_NO_RESOURCE, "stream reference has no resource")
    if protocol not in PROTOCOLS:
        raise StreamRefused(
            REFUSAL_UNKNOWN_PROTOCOL,
            f"{protocol!r} is not one of {sorted(PROTOCOLS)}",
        )
    url = str(url or "").strip()
    if not url:
        raise StreamRefused(REFUSAL_NO_URL, "stream reference has no URL")
    if len(url) > MAX_URL_LENGTH:
        raise StreamRefused(REFUSAL_URL_TOO_LONG, "stream URL exceeds size ceiling")
    if not 0 < int(ttl_seconds) <= MAX_TTL_SECONDS:
        raise StreamRefused(
            REFUSAL_TTL,
            f"{ttl_seconds}s is outside 1..{MAX_TTL_SECONDS}s",
        )
    refuse_insecure_address(url)
    return StreamReference(
        resource_id=resource_id,
        zone_id=str(zone_id or ""),
        protocol=protocol,
        url=url,
        expires_at=str(expires_at or ""),
        label=str(label or ""),
        audio=bool(audio),
        measured_latency_ms=(
            None if measured_latency_ms is None else int(measured_latency_ms)
        ),
    )
