"""Generic installed-resource contracts and outbound Flyto Cloud publisher.

The Cloud describes and observes resources through these versioned envelopes;
it never imports ROS, simulator, or robot-model implementation details.  This
module intentionally contains no command or actuator endpoint.
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import math
import os
import re
import stat
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlparse
from urllib.request import HTTPRedirectHandler, Request, build_opener

RESOURCE_MANIFEST_CONTRACT = "flyto.resource-manifest.v1"
RESOURCE_TELEMETRY_CONTRACT = "flyto.resource-telemetry.v1"
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$")
_VALUE_KINDS = frozenset({"string", "integer", "number", "boolean", "enum"})
_QUALITY_VALUES = frozenset({"good", "degraded", "stale", "error"})


class ResourceContractError(ValueError):
    """Raised when a resource envelope is unsafe or incompatible."""


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, new_url):
        return None


def parse_resource_manifest(value: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one generic manifest and bind its deterministic hash."""
    manifest = _mapping(value, "resource manifest")
    _exact_fields(
        manifest,
        required={
            "contract",
            "resource_id",
            "resource_type",
            "display_name",
            "revision",
            "adapter",
            "deployment_mode",
            "capability_ids",
            "settings",
            "telemetry_channels",
            "observed_at",
        },
        optional={"contract_hash"},
    )
    if manifest["contract"] != RESOURCE_MANIFEST_CONTRACT:
        raise ResourceContractError("unsupported resource manifest contract")
    normalized = {
        "contract": RESOURCE_MANIFEST_CONTRACT,
        "resource_id": _identifier(manifest["resource_id"], "resource ID"),
        "resource_type": _identifier(manifest["resource_type"], "resource type"),
        "display_name": _text(manifest["display_name"], "display name", 200),
        "revision": _integer(manifest["revision"], "manifest revision", minimum=1),
        "adapter": _adapter(manifest["adapter"]),
        "deployment_mode": _choice(
            manifest["deployment_mode"],
            "deployment mode",
            {"real", "simulation", "hybrid"},
        ),
        "capability_ids": _identifiers(
            manifest["capability_ids"], "capability IDs", limit=128
        ),
        "settings": _settings(manifest["settings"]),
        "telemetry_channels": _channels(manifest["telemetry_channels"]),
        "observed_at": _text(manifest["observed_at"], "observed_at", 64),
    }
    expected = _content_hash(normalized)
    supplied = manifest.get("contract_hash")
    if supplied not in (None, "") and supplied != expected:
        raise ResourceContractError("resource manifest hash does not match contract")
    normalized["contract_hash"] = expected
    return normalized


def parse_resource_telemetry(value: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one bounded latest-sample envelope and bind its payload hash."""
    telemetry = _mapping(value, "resource telemetry")
    _exact_fields(
        telemetry,
        required={
            "contract",
            "resource_id",
            "channel_id",
            "sequence",
            "observed_at",
            "quality",
            "payload",
        },
        optional={"payload_hash"},
    )
    if telemetry["contract"] != RESOURCE_TELEMETRY_CONTRACT:
        raise ResourceContractError("unsupported resource telemetry contract")
    payload = _mapping(telemetry["payload"], "telemetry payload")
    _bounded_json(payload)
    if len(_canonical_json(payload).encode("utf-8")) > 64 * 1024:
        raise ResourceContractError("telemetry payload exceeds 64 KiB")
    normalized = {
        "contract": RESOURCE_TELEMETRY_CONTRACT,
        "resource_id": _identifier(telemetry["resource_id"], "resource ID"),
        "channel_id": _identifier(telemetry["channel_id"], "channel ID"),
        "sequence": _integer(telemetry["sequence"], "sequence", minimum=0),
        "observed_at": _text(telemetry["observed_at"], "observed_at", 64),
        "quality": _choice(telemetry["quality"], "quality", _QUALITY_VALUES),
        "payload": dict(payload),
    }
    expected = _content_hash(
        {
            "resource_id": normalized["resource_id"],
            "channel_id": normalized["channel_id"],
            "sequence": normalized["sequence"],
            "observed_at": normalized["observed_at"],
            "quality": normalized["quality"],
            "payload": normalized["payload"],
        }
    )
    supplied = telemetry.get("payload_hash")
    if supplied not in (None, "") and supplied != expected:
        raise ResourceContractError("telemetry hash does not match payload")
    normalized["payload_hash"] = expected
    return normalized


class FlytoCloudResourcePublisher:
    """Outbound-only publisher authenticated as one exact paired device."""

    def __init__(
        self,
        *,
        cloud_url: str,
        device_id: str,
        credential_loader: Callable[[], str],
        timeout_seconds: float = 10.0,
        opener=None,
    ) -> None:
        self.cloud_url = _safe_cloud_url(cloud_url)
        self.device_id = _identifier(device_id, "device ID")
        self._credential_loader = credential_loader
        self.timeout_seconds = max(1.0, min(float(timeout_seconds), 60.0))
        self._opener = opener or build_opener(_NoRedirect())

    def publish_manifest(self, value: Mapping[str, Any]) -> dict[str, Any]:
        manifest = parse_resource_manifest(value)
        resource_id = quote(manifest["resource_id"], safe="")
        return self._request(
            "PUT",
            f"/api/devices/{quote(self.device_id, safe='')}/resources/{resource_id}",
            manifest,
        )

    def publish_telemetry(self, value: Mapping[str, Any]) -> dict[str, Any]:
        telemetry = parse_resource_telemetry(value)
        resource_id = quote(telemetry["resource_id"], safe="")
        return self._request(
            "POST",
            (
                f"/api/devices/{quote(self.device_id, safe='')}/resources/"
                f"{resource_id}/telemetry"
            ),
            telemetry,
        )

    def _request(self, method: str, path: str, payload: Mapping[str, Any]) -> dict[str, Any]:
        secret = self._credential_loader().strip()
        if len(secret) < 32 or len(secret) > 512 or not secret.isascii():
            raise ResourceContractError("paired-device credential is invalid")
        body = _canonical_json(payload).encode("utf-8")
        request = Request(
            f"{self.cloud_url}{path}",
            data=body,
            method=method,
            headers={
                "Authorization": f"Bearer device:{self.device_id}.{secret}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )
        try:
            with self._opener.open(request, timeout=self.timeout_seconds) as response:
                raw = response.read(256 * 1024 + 1)
        except HTTPError as exc:
            raise ResourceContractError(
                f"Cloud rejected resource payload (HTTP {exc.code})"
            ) from exc
        except URLError as exc:
            raise ResourceContractError("Cloud resource endpoint is unavailable") from exc
        if len(raw) > 256 * 1024:
            raise ResourceContractError("Cloud resource response is too large")
        try:
            parsed = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ResourceContractError("Cloud returned invalid resource JSON") from exc
        return _mapping(parsed, "Cloud resource response")


def _load_secret_file(path: Path) -> str:
    info = path.stat()
    if os.name == "posix" and stat.S_IMODE(info.st_mode) & 0o077:
        raise ResourceContractError("device secret file must be owner-only")
    if info.st_size > 1024:
        raise ResourceContractError("device secret file is too large")
    return path.read_text(encoding="utf-8").strip()


def _load_json_file(path: Path) -> dict[str, Any]:
    if path.stat().st_size > 256 * 1024:
        raise ResourceContractError("resource input file is too large")
    try:
        return _mapping(json.loads(path.read_text(encoding="utf-8")), "resource input")
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ResourceContractError("resource input is not valid UTF-8 JSON") from exc


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Publish installed resource state to Flyto Cloud")
    parser.add_argument("--cloud-url", required=True)
    parser.add_argument("--device-id", required=True)
    parser.add_argument("--device-secret-file", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path, action="append", default=[])
    parser.add_argument("--interval-seconds", type=float, default=0.0)
    args = parser.parse_args(argv)

    publisher = FlytoCloudResourcePublisher(
        cloud_url=args.cloud_url,
        device_id=args.device_id,
        credential_loader=lambda: _load_secret_file(args.device_secret_file),
    )
    interval = max(0.0, min(float(args.interval_seconds), 3600.0))
    while True:
        publisher.publish_manifest(_load_json_file(args.manifest))
        for telemetry_path in args.telemetry:
            publisher.publish_telemetry(_load_json_file(telemetry_path))
        if interval <= 0.0:
            return 0
        time.sleep(interval)


def _adapter(value: Any) -> dict[str, str]:
    adapter = _mapping(value, "adapter")
    _exact_fields(adapter, required={"adapter_id", "version"}, optional={"provider"})
    return {
        "adapter_id": _identifier(adapter["adapter_id"], "adapter ID"),
        "version": _text(adapter["version"], "adapter version", 64),
        "provider": _text(adapter.get("provider", ""), "adapter provider", 128, allow_empty=True),
    }


def _settings(value: Any) -> list[dict[str, Any]]:
    items = _sequence(value, "settings", 128)
    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in items:
        setting = _mapping(item, "setting")
        _exact_fields(
            setting,
            required={"setting_id", "display_name", "value_kind"},
            optional={"value", "configured", "secret", "mutable", "unit", "options", "description"},
        )
        setting_id = _identifier(setting["setting_id"], "setting ID")
        secret = _boolean(setting.get("secret", False), "setting secret")
        if _is_sensitive_setting_id(setting_id) and not secret:
            raise ResourceContractError("sensitive settings must be declared secret")
        if secret and setting.get("value") is not None:
            raise ResourceContractError("secret setting values must never enter Cloud")
        value_kind = _choice(setting["value_kind"], "setting value kind", _VALUE_KINDS)
        options = _texts(setting.get("options", []), "setting options", 64, 200)
        scalar = setting.get("value")
        if scalar is not None and not isinstance(scalar, (str, int, float, bool)):
            raise ResourceContractError("setting value must be scalar")
        if isinstance(scalar, float) and not math.isfinite(scalar):
            raise ResourceContractError("setting value must be finite")
        if value_kind == "enum" and scalar is not None and str(scalar) not in options:
            raise ResourceContractError("enum setting value must be one of its options")
        if setting_id in seen:
            raise ResourceContractError("resource setting IDs must be unique")
        seen.add(setting_id)
        normalized.append(
            {
                "setting_id": setting_id,
                "display_name": _text(setting["display_name"], "setting display name", 200),
                "value_kind": value_kind,
                "value": scalar,
                "configured": _boolean(setting.get("configured", False), "setting configured"),
                "secret": secret,
                "mutable": _boolean(setting.get("mutable", False), "setting mutable"),
                "unit": _text(setting.get("unit", ""), "setting unit", 32, allow_empty=True),
                "options": options,
                "description": _text(
                    setting.get("description", ""), "setting description", 500, allow_empty=True
                ),
            }
        )
    return normalized


def _channels(value: Any) -> list[dict[str, Any]]:
    items = _sequence(value, "telemetry channels", 128)
    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in items:
        channel = _mapping(item, "telemetry channel")
        _exact_fields(
            channel,
            required={"channel_id", "display_name"},
            optional={"payload_schema", "presentation", "max_age_seconds"},
        )
        channel_id = _identifier(channel["channel_id"], "channel ID")
        if channel_id in seen:
            raise ResourceContractError("telemetry channel IDs must be unique")
        seen.add(channel_id)
        schema = _mapping(channel.get("payload_schema", {}), "payload schema")
        _bounded_json(schema)
        presentation = _mapping(channel.get("presentation", {}), "presentation")
        _exact_fields(presentation, required=set(), optional={"kind", "unit", "precision"})
        precision = _integer(presentation.get("precision", 2), "presentation precision", minimum=0)
        if precision > 8:
            raise ResourceContractError("presentation precision exceeds 8")
        max_age = _finite_number(channel.get("max_age_seconds", 30.0), "max age")
        if not 0.1 <= max_age <= 86_400.0:
            raise ResourceContractError("telemetry max age is out of bounds")
        normalized.append(
            {
                "channel_id": channel_id,
                "display_name": _text(channel["display_name"], "channel display name", 200),
                "payload_schema": dict(schema),
                "presentation": {
                    "kind": _identifier(
                        presentation.get("kind", "json"),
                        "presentation kind",
                    ),
                    "unit": _text(
                        presentation.get("unit", ""),
                        "presentation unit",
                        32,
                        allow_empty=True,
                    ),
                    "precision": precision,
                },
                "max_age_seconds": max_age,
            }
        )
    return normalized


def _safe_cloud_url(value: str) -> str:
    url = value.strip().rstrip("/")
    parsed = urlparse(url)
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ResourceContractError("Cloud URL must be an origin without credentials or query")
    if parsed.scheme == "https" and parsed.hostname:
        return url
    if parsed.scheme == "http" and parsed.hostname:
        try:
            if ipaddress.ip_address(parsed.hostname).is_loopback:
                return url
        except ValueError:
            if parsed.hostname == "localhost":
                return url
    raise ResourceContractError("Cloud URL must use HTTPS or loopback HTTP")


def _is_sensitive_setting_id(value: str) -> bool:
    separated = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value)
    parts = [part for part in re.split(r"[^A-Za-z0-9]+", separated.lower()) if part]
    if any(part in {"credential", "password", "secret"} for part in parts):
        return True
    pairs = set(zip(parts, parts[1:]))
    if ("private", "key") in pairs or ("api", "key") in pairs:
        return True
    benign_token_qualifiers = {
        "budget",
        "count",
        "length",
        "limit",
        "max",
        "maximum",
        "minimum",
        "threshold",
        "window",
    }
    for index, part in enumerate(parts):
        if part != "token":
            continue
        neighbors = parts[max(0, index - 1) : index] + parts[index + 1 : index + 2]
        if not any(neighbor in benign_token_qualifiers for neighbor in neighbors):
            return True
    return False


def _bounded_json(
    value: Any, *, depth: int = 0, nodes: list[int] | None = None
) -> None:
    nodes = nodes if nodes is not None else [0]
    nodes[0] += 1
    if nodes[0] > 512 or depth > 10:
        raise ResourceContractError("resource JSON exceeds structural bounds")
    if value is None or isinstance(value, (bool, int, str)):
        if isinstance(value, str) and len(value) > 8_000:
            raise ResourceContractError("resource JSON string is too large")
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ResourceContractError("resource JSON numbers must be finite")
        return
    if isinstance(value, list):
        if len(value) > 512:
            raise ResourceContractError("resource JSON list is too large")
        for item in value:
            _bounded_json(item, depth=depth + 1, nodes=nodes)
        return
    if isinstance(value, dict):
        if len(value) > 128:
            raise ResourceContractError("resource JSON object is too large")
        for key, item in value.items():
            if not isinstance(key, str) or not key or len(key) > 128:
                raise ResourceContractError("resource JSON keys must be bounded strings")
            _bounded_json(item, depth=depth + 1, nodes=nodes)
        return
    raise ResourceContractError("resource JSON contains an unsupported value")


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ResourceContractError(f"{label} must be an object")
    return dict(value)


def _sequence(value: Any, label: str, limit: int) -> list[Any]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ResourceContractError(f"{label} must be a list")
    if len(value) > limit:
        raise ResourceContractError(f"{label} exceeds {limit} items")
    return list(value)


def _exact_fields(value: Mapping[str, Any], *, required: set[str], optional: set[str]) -> None:
    missing = required - set(value)
    extra = set(value) - required - optional
    if missing:
        raise ResourceContractError(f"missing resource fields: {', '.join(sorted(missing))}")
    if extra:
        raise ResourceContractError(f"unknown resource fields: {', '.join(sorted(extra))}")


def _identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value.strip()):
        raise ResourceContractError(f"{label} must be a safe identifier")
    return value.strip()


def _identifiers(value: Any, label: str, *, limit: int) -> list[str]:
    result = [_identifier(item, label) for item in _sequence(value, label, limit)]
    return list(dict.fromkeys(result))


def _texts(value: Any, label: str, limit: int, max_length: int) -> list[str]:
    return [
        _text(item, label, max_length)
        for item in _sequence(value, label, limit)
    ]


def _text(value: Any, label: str, max_length: int, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise ResourceContractError(f"{label} must be text")
    normalized = value.strip()
    if (not normalized and not allow_empty) or len(normalized) > max_length:
        raise ResourceContractError(f"{label} is out of bounds")
    return normalized


def _integer(value: Any, label: str, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ResourceContractError(f"{label} must be an integer >= {minimum}")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ResourceContractError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ResourceContractError(f"{label} must be finite")
    return result


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ResourceContractError(f"{label} must be boolean")
    return value


def _choice(value: Any, label: str, choices) -> str:
    if not isinstance(value, str) or value not in choices:
        raise ResourceContractError(f"{label} is unsupported")
    return value


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


def _content_hash(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
