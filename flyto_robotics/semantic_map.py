"""Language-neutral, map-scoped semantic location memory."""

from __future__ import annotations

import json
import math
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
from typing import Any, Protocol

from .capabilities import SAFE_TEXT
from .contracts import StationPose, write_json_atomic

SEMANTIC_MAP_CONTRACT_VERSION = "flyto.robotics.semantic-location-map.v1"
SEMANTIC_CATALOG_CONTRACT_VERSION = (
    "flyto.robotics.semantic-location-catalog.v1"
)
MAX_SEMANTIC_MAP_BYTES = 256 * 1024
MAX_LOCATIONS = 2048
MAX_LABELS_PER_LOCATION = 32
MAX_LABEL_CHARACTERS = 128
MAX_LABEL_BYTES = 512


class SemanticMapValidationError(ValueError):
    """Raised when semantic map state is missing, ambiguous, stale, or unsafe."""


class PoseLike(Protocol):
    """Minimal trusted-pose interface accepted from odometry adapters."""

    x: float
    y: float
    yaw: float


def _identifier(value: object, field_name: str) -> str:
    if not isinstance(value, str) or not SAFE_TEXT.fullmatch(value):
        raise SemanticMapValidationError(f"{field_name} must be a safe identifier")
    return value


def _number(
    value: object,
    field_name: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SemanticMapValidationError(f"{field_name} must be a number")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise SemanticMapValidationError(
            f"{field_name} must be between {minimum} and {maximum}"
        )
    return parsed


def _label(value: object, field_name: str) -> str:
    if not isinstance(value, str):
        raise SemanticMapValidationError(f"{field_name} must be text")
    normalized = unicodedata.normalize("NFKC", value).strip()
    if (
        not normalized
        or len(normalized) > MAX_LABEL_CHARACTERS
        or len(normalized.encode("utf-8")) > MAX_LABEL_BYTES
        or any(unicodedata.category(character) in {"Cc", "Cs"} for character in normalized)
    ):
        raise SemanticMapValidationError(
            f"{field_name} must be bounded printable Unicode text"
        )
    return normalized


@dataclass(frozen=True)
class SemanticLocation:
    """One stable location ID with multilingual display labels and a trusted pose."""

    location_id: str
    labels: tuple[str, ...]
    pose: StationPose

    def __post_init__(self) -> None:
        safe_id = _identifier(self.location_id, "semantic_location.location_id")
        if safe_id != self.location_id:
            raise SemanticMapValidationError(
                "semantic_location.location_id must already be normalized"
            )
        if not isinstance(self.labels, tuple) or not (
            1 <= len(self.labels) <= MAX_LABELS_PER_LOCATION
        ):
            raise SemanticMapValidationError(
                f"semantic_location.labels must contain 1 to "
                f"{MAX_LABELS_PER_LOCATION} items"
            )
        normalized_labels = tuple(
            dict.fromkeys(
                _label(label, f"semantic_location.labels[{index}]")
                for index, label in enumerate(self.labels)
            )
        )
        if normalized_labels != self.labels:
            raise SemanticMapValidationError(
                "semantic_location.labels must be normalized and unique"
            )
        if not isinstance(self.pose, StationPose):
            raise SemanticMapValidationError(
                "semantic_location.pose must be a StationPose"
            )
        if self.pose.station_id != self.location_id:
            raise SemanticMapValidationError(
                "semantic_location.pose.station_id must match location_id"
            )
        _number(
            self.pose.x,
            "semantic_location.pose.x",
            minimum=-1000.0,
            maximum=1000.0,
        )
        _number(
            self.pose.y,
            "semantic_location.pose.y",
            minimum=-1000.0,
            maximum=1000.0,
        )
        _number(
            self.pose.yaw,
            "semantic_location.pose.yaw",
            minimum=-math.pi,
            maximum=math.pi,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "location_id": self.location_id,
            "labels": list(self.labels),
            "pose": {
                "x": self.pose.x,
                "y": self.pose.y,
                "yaw": self.pose.yaw,
            },
        }


@dataclass(frozen=True)
class SemanticLocationMap:
    """Immutable snapshot of the semantic locations for one physical map."""

    map_id: str
    revision: int = 0
    locations: tuple[SemanticLocation, ...] = ()

    def __post_init__(self) -> None:
        _identifier(self.map_id, "semantic_map.map_id")
        if isinstance(self.revision, bool) or not isinstance(self.revision, int):
            raise SemanticMapValidationError("semantic_map.revision must be an integer")
        if self.revision < 0:
            raise SemanticMapValidationError(
                "semantic_map.revision cannot be negative"
            )
        if len(self.locations) > MAX_LOCATIONS:
            raise SemanticMapValidationError(
                f"semantic_map.locations exceeds {MAX_LOCATIONS} items"
            )
        if any(
            not isinstance(location, SemanticLocation)
            for location in self.locations
        ):
            raise SemanticMapValidationError(
                "semantic_map.locations must contain SemanticLocation values"
            )
        identifiers = [location.location_id for location in self.locations]
        if len(identifiers) != len(set(identifiers)):
            raise SemanticMapValidationError(
                "semantic_map.location_id values must be unique"
            )

    def resolve(self, location_id: str) -> SemanticLocation:
        safe_id = _identifier(location_id, "location_id")
        try:
            return next(
                location
                for location in self.locations
                if location.location_id == safe_id
            )
        except StopIteration as exc:
            raise SemanticMapValidationError(
                f"semantic location is not registered: {safe_id}"
            ) from exc

    def planner_view(self) -> dict[str, Any]:
        """Return IDs and labels only; trusted coordinates stay outside the LLM."""
        return {
            "contract_version": SEMANTIC_CATALOG_CONTRACT_VERSION,
            "map_id": self.map_id,
            "revision": self.revision,
            "locations": [
                {
                    "location_id": location.location_id,
                    "labels": list(location.labels),
                }
                for location in self.locations
            ],
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "contract_version": SEMANTIC_MAP_CONTRACT_VERSION,
            "map_id": self.map_id,
            "revision": self.revision,
            "locations": [location.to_dict() for location in self.locations],
        }


def parse_semantic_location_map(value: object) -> SemanticLocationMap:
    """Validate and normalize one decoded semantic map document."""
    if not isinstance(value, dict):
        raise SemanticMapValidationError("semantic_map must be an object")
    allowed = {"contract_version", "map_id", "revision", "locations"}
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise SemanticMapValidationError(
            "semantic_map contains unsupported fields: " + ", ".join(unknown)
        )
    if set(value) != allowed:
        missing = sorted(allowed - set(value))
        raise SemanticMapValidationError(
            "semantic_map is missing: " + ", ".join(missing)
        )
    if value["contract_version"] != SEMANTIC_MAP_CONTRACT_VERSION:
        raise SemanticMapValidationError(
            f"semantic_map.contract_version must be {SEMANTIC_MAP_CONTRACT_VERSION}"
        )
    raw_locations = value["locations"]
    if not isinstance(raw_locations, list) or len(raw_locations) > MAX_LOCATIONS:
        raise SemanticMapValidationError(
            f"semantic_map.locations must contain at most {MAX_LOCATIONS} items"
        )
    locations: list[SemanticLocation] = []
    for index, raw_location in enumerate(raw_locations):
        field_name = f"semantic_map.locations[{index}]"
        if not isinstance(raw_location, dict):
            raise SemanticMapValidationError(f"{field_name} must be an object")
        if set(raw_location) != {"location_id", "labels", "pose"}:
            raise SemanticMapValidationError(
                f"{field_name} requires only location_id, labels, and pose"
            )
        raw_labels = raw_location["labels"]
        if (
            not isinstance(raw_labels, list)
            or not 1 <= len(raw_labels) <= MAX_LABELS_PER_LOCATION
        ):
            raise SemanticMapValidationError(
                f"{field_name}.labels must contain 1 to "
                f"{MAX_LABELS_PER_LOCATION} items"
            )
        labels = tuple(
            dict.fromkeys(
                _label(item, f"{field_name}.labels[{label_index}]")
                for label_index, item in enumerate(raw_labels)
            )
        )
        if len(labels) != len(raw_labels):
            raise SemanticMapValidationError(
                f"{field_name}.labels must be normalized and unique"
            )
        raw_pose = raw_location["pose"]
        if not isinstance(raw_pose, dict) or set(raw_pose) != {"x", "y", "yaw"}:
            raise SemanticMapValidationError(
                f"{field_name}.pose requires only x, y, and yaw"
            )
        location_id = _identifier(
            raw_location["location_id"],
            f"{field_name}.location_id",
        )
        locations.append(
            SemanticLocation(
                location_id=location_id,
                labels=labels,
                pose=StationPose(
                    station_id=location_id,
                    x=_number(
                        raw_pose["x"],
                        f"{field_name}.pose.x",
                        minimum=-1000.0,
                        maximum=1000.0,
                    ),
                    y=_number(
                        raw_pose["y"],
                        f"{field_name}.pose.y",
                        minimum=-1000.0,
                        maximum=1000.0,
                    ),
                    yaw=_number(
                        raw_pose["yaw"],
                        f"{field_name}.pose.yaw",
                        minimum=-math.pi,
                        maximum=math.pi,
                    ),
                ),
            )
        )
    revision = value["revision"]
    if isinstance(revision, bool) or not isinstance(revision, int):
        raise SemanticMapValidationError("semantic_map.revision must be an integer")
    return SemanticLocationMap(
        map_id=_identifier(value["map_id"], "semantic_map.map_id"),
        revision=revision,
        locations=tuple(sorted(locations, key=lambda item: item.location_id)),
    )


class SemanticLocationStore:
    """Atomic JSON-backed store with an explicit physical-map identity."""

    def __init__(self, path: str | Path, *, map_id: str) -> None:
        self.path = Path(path)
        self.map_id = _identifier(map_id, "semantic_map.map_id")
        self._lock = RLock()

    def load(self) -> SemanticLocationMap:
        with self._lock:
            if not self.path.exists():
                return SemanticLocationMap(map_id=self.map_id)
            try:
                size = self.path.stat().st_size
                if size > MAX_SEMANTIC_MAP_BYTES:
                    raise SemanticMapValidationError(
                        f"semantic map exceeds {MAX_SEMANTIC_MAP_BYTES} bytes"
                    )
                decoded = json.loads(self.path.read_text(encoding="utf-8"))
            except SemanticMapValidationError:
                raise
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                raise SemanticMapValidationError(
                    "semantic map must contain readable UTF-8 JSON"
                ) from exc
            snapshot = parse_semantic_location_map(decoded)
            if snapshot.map_id != self.map_id:
                raise SemanticMapValidationError(
                    f"semantic map_id mismatch: expected {self.map_id}, "
                    f"received {snapshot.map_id}"
                )
            return snapshot

    def resolve(self, location_id: str) -> SemanticLocation:
        return self.load().resolve(location_id)

    def planner_view(self) -> dict[str, Any]:
        return self.load().planner_view()

    def remember(
        self,
        *,
        location_id: str,
        label: str,
        pose: PoseLike,
        expected_revision: int | None = None,
    ) -> SemanticLocationMap:
        """Atomically insert or replace a pose without deriving identity from language."""
        safe_id = _identifier(location_id, "location_id")
        safe_label = _label(label, "label")
        trusted_pose = StationPose(
            station_id=safe_id,
            x=_number(pose.x, "pose.x", minimum=-1000.0, maximum=1000.0),
            y=_number(pose.y, "pose.y", minimum=-1000.0, maximum=1000.0),
            yaw=_number(pose.yaw, "pose.yaw", minimum=-math.pi, maximum=math.pi),
        )
        with self._lock:
            snapshot = self.load()
            if expected_revision is not None:
                if isinstance(expected_revision, bool) or not isinstance(
                    expected_revision, int
                ):
                    raise SemanticMapValidationError(
                        "expected_revision must be an integer"
                    )
                if snapshot.revision != expected_revision:
                    raise SemanticMapValidationError(
                        "semantic map revision changed before location write"
                    )
            by_id = {location.location_id: location for location in snapshot.locations}
            previous = by_id.get(safe_id)
            labels = (
                tuple(dict.fromkeys((*previous.labels, safe_label)))
                if previous is not None
                else (safe_label,)
            )
            by_id[safe_id] = SemanticLocation(
                location_id=safe_id,
                labels=labels,
                pose=trusted_pose,
            )
            updated = SemanticLocationMap(
                map_id=self.map_id,
                revision=snapshot.revision + 1,
                locations=tuple(sorted(by_id.values(), key=lambda item: item.location_id)),
            )
            document = updated.to_dict()
            encoded = (
                json.dumps(
                    document,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                + "\n"
            ).encode("utf-8")
            if len(encoded) > MAX_SEMANTIC_MAP_BYTES:
                raise SemanticMapValidationError(
                    f"semantic map exceeds {MAX_SEMANTIC_MAP_BYTES} bytes"
                )
            write_json_atomic(self.path, document)
            return updated


__all__ = [
    "MAX_SEMANTIC_MAP_BYTES",
    "SEMANTIC_CATALOG_CONTRACT_VERSION",
    "SEMANTIC_MAP_CONTRACT_VERSION",
    "SemanticLocation",
    "SemanticLocationMap",
    "SemanticLocationStore",
    "SemanticMapValidationError",
    "parse_semantic_location_map",
]
