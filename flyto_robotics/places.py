"""Named places on a robot's map, kept on the execution host.

A place is a name for a pose in the map frame: ``{name, frame: "map", x, y,
yaw}``. Places are saved on the computer that runs the adapter, never on the
robot (which runs stock ROS 2 and no Flyto2 code) and never in Flyto2 Cloud.
They belong to one map: the robot's SLAM map is built and kept on the robot,
so the host names the map it has places for with ``FLYTO_ROS2_MAP_ID``
(default ``default``) and keeps one file per robot per map.

Where the file lives
    ``FLYTO_ROS2_PLACES_FILE`` names it outright. Otherwise it is
    ``<data dir>/places/<resource id>/<map id>.json``, where the data dir is
    ``FLYTO_ROBOTICS_DATA_DIR``, else ``%LOCALAPPDATA%/flyto-robotics`` on
    Windows, else ``$XDG_DATA_HOME/flyto-robotics`` (``~/.local/share``).

Guarantees
    * Names are free text (any script), trimmed and NFC-normalized, 1 to
      ``MAX_NAME_LENGTH`` characters, without control characters. They are
      unique per map, compared case-insensitively, so "Lobby" and "lobby" are
      one place.
    * Every write is atomic (``fsio.atomic_write``): a reader sees the old
      file or the new one, never half of one.
    * A file that cannot be read exactly as written -- bad JSON, another
      schema or map, a malformed or duplicated entry, too large -- raises
      ``PlacesStoreError``. Nothing is guessed, and a corrupt file is never
      overwritten, so marking a place cannot destroy the ones it could not read.

Pure: no ROS, no network.
"""

from __future__ import annotations

import contextlib
import hashlib
import json
import math
import os
import re
import threading
import unicodedata
from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import fsio

PLACES_SCHEMA = "flyto.robot-places.v1"
PLACES_FILE_ENV = "FLYTO_ROS2_PLACES_FILE"
DATA_DIR_ENV = "FLYTO_ROBOTICS_DATA_DIR"
MAP_ID_ENV = "FLYTO_ROS2_MAP_ID"
DEFAULT_MAP_ID = "default"
MAP_FRAME = "map"

MAX_NAME_LENGTH = 64
MAX_PLACES = 200
# 200 entries of 64 four-byte characters and five numbers fit well inside it.
MAX_FILE_BYTES = 256 * 1024
# The navigate bounds (generic_ros2_adapter.ARGUMENTS["motion.navigate"]).
MAX_COORDINATE_M = 1000.0

_ENTRY_KEYS = frozenset(("name", "frame", "x", "y", "yaw"))
_FILE_KEYS = frozenset(("schema", "map_id", "places"))
# Unicode categories a name may not contain: controls, and line/paragraph
# separators that would split it on a screen or in a log.
_FORBIDDEN_CATEGORIES = frozenset(("Cc", "Zl", "Zp"))
_SAFE_FRAGMENT = re.compile(r"[^A-Za-z0-9_.-]+")

# Writers in this process; ``_file_lock`` adds a lock other processes see.
_PROCESS_LOCK = threading.Lock()


class PlacesError(ValueError):
    """A request about places that cannot be honoured (a bad name, a full map)."""


class UnknownPlace(PlacesError):
    """The named place is not saved on this map."""

    def __init__(self, name: str, known: tuple[str, ...]):
        self.name = name
        self.known = known
        listed = ", ".join(known) if known else "none saved"
        super().__init__(f"unknown place {name!r}; known places: {listed}")


class PlacesStoreError(RuntimeError):
    """The places file exists but cannot be read exactly as written."""


@dataclass(frozen=True)
class Place:
    name: str
    x: float
    y: float
    yaw: float
    frame: str = MAP_FRAME

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "frame": self.frame, "x": self.x, "y": self.y, "yaw": self.yaw}


def normalize_name(raw: Any) -> str:
    """The canonical form of a place name, or ``PlacesError``."""
    if not isinstance(raw, str):
        raise PlacesError("a place name must be text")
    name = unicodedata.normalize("NFC", raw).strip()
    if any(unicodedata.category(character) in _FORBIDDEN_CATEGORIES for character in name):
        raise PlacesError("a place name must not contain control characters")
    if not 1 <= len(name) <= MAX_NAME_LENGTH:
        raise PlacesError(f"a place name must be 1 to {MAX_NAME_LENGTH} characters")
    return name


def name_key(name: str) -> str:
    """What makes two names the same place: case-insensitive, after NFC."""
    return unicodedata.normalize("NFC", name.casefold())


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _wrap(angle: float) -> float:
    """Into [-pi, pi]; an angle already there is kept exactly as it is."""
    if -math.pi <= angle <= math.pi:
        return angle
    return math.atan2(math.sin(angle), math.cos(angle))


def place_from_pose(name: str, pose: Mapping[str, Any]) -> Place:
    """A place at a map-frame pose (the adapter's ``map_pose``), validated."""
    if not isinstance(pose, Mapping) or pose.get("frame") != MAP_FRAME:
        raise PlacesError("a place is saved from a pose in the map frame")
    x, y, yaw = (_finite(pose.get(key)) for key in ("x", "y", "yaw"))
    if x is None or y is None or yaw is None:
        raise PlacesError("the map pose has no finite x, y and yaw")
    if abs(x) > MAX_COORDINATE_M or abs(y) > MAX_COORDINATE_M:
        raise PlacesError(f"the map pose is more than {MAX_COORDINATE_M:g} m from the origin")
    return Place(name=normalize_name(name), x=x, y=y, yaw=_wrap(yaw))


def _safe_fragment(value: str) -> str:
    """A path component for an id; a changed id gets a digest so two never meet."""
    text = str(value or "").strip()
    safe = _SAFE_FRAGMENT.sub("-", text).strip("-.")[:64] or "unnamed"
    if safe != text:
        safe = f"{safe}-{hashlib.sha256(text.encode('utf-8')).hexdigest()[:8]}"
    return safe


def data_dir() -> Path:
    configured = os.getenv(DATA_DIR_ENV, "").strip()
    if configured:
        return Path(configured).expanduser()
    if os.name == "nt" and os.getenv("LOCALAPPDATA"):
        return Path(os.environ["LOCALAPPDATA"]) / "flyto-robotics"
    xdg = os.getenv("XDG_DATA_HOME", "").strip()
    base = Path(xdg).expanduser() if xdg else Path.home() / ".local" / "share"
    return base / "flyto-robotics"


def map_id() -> str:
    return os.getenv(MAP_ID_ENV, "").strip() or DEFAULT_MAP_ID


def default_path(resource_id: str, map_name: str | None = None) -> Path:
    """Where this robot's places for this map are kept (see the module doc)."""
    explicit = os.getenv(PLACES_FILE_ENV, "").strip()
    if explicit:
        return Path(explicit).expanduser()
    chosen = map_name or map_id()
    return data_dir() / "places" / _safe_fragment(resource_id) / f"{_safe_fragment(chosen)}.json"


@contextlib.contextmanager
def _file_lock(path: Path) -> Iterator[None]:
    """Hold an exclusive lock other processes on this host also respect.

    POSIX ``flock`` on a sibling lock file; where there is none (Windows), the
    in-process lock still serialises writers in this process.
    """
    with _PROCESS_LOCK:
        try:
            import fcntl
        except ImportError:  # pragma: no cover - Windows
            yield
            return
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        handle = os.open(path.with_name(f".{path.name}.lock"), os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(handle, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
            os.close(handle)


class PlacesStore:
    """One robot's named places on one map, in one JSON file."""

    def __init__(self, path: Path | str, *, map_name: str = DEFAULT_MAP_ID):
        self.path = Path(path)
        self.map_id = map_name

    @classmethod
    def for_resource(cls, resource_id: str) -> PlacesStore:
        chosen = map_id()
        return cls(default_path(resource_id, chosen), map_name=chosen)

    # -- reading -------------------------------------------------------------

    def _corrupt(self, why: str) -> PlacesStoreError:
        return PlacesStoreError(f"places file {self.path.name} is unreadable: {why}")

    def _entry(self, index: int, item: Any) -> Place:
        if not isinstance(item, Mapping) or set(item) != _ENTRY_KEYS:
            raise self._corrupt(f"entry {index} is not {{name, frame, x, y, yaw}}")
        try:
            place = place_from_pose(item["name"], item)
        except PlacesError as error:
            raise self._corrupt(f"entry {index}: {error}") from None
        if place.name != item["name"] or not -math.pi <= float(item["yaw"]) <= math.pi:
            raise self._corrupt(f"entry {index} is not in canonical form")
        return place

    def _read(self) -> list[Place]:
        try:
            size = self.path.stat().st_size
        except FileNotFoundError:
            return []
        except OSError as error:
            raise self._corrupt(type(error).__name__) from None
        if size > MAX_FILE_BYTES:
            raise self._corrupt(f"larger than {MAX_FILE_BYTES} bytes")
        try:
            document = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, ValueError) as error:
            raise self._corrupt(type(error).__name__) from None
        if not isinstance(document, dict) or set(document) != _FILE_KEYS:
            raise self._corrupt("not a places document")
        if document["schema"] != PLACES_SCHEMA:
            raise self._corrupt(f"schema is not {PLACES_SCHEMA}")
        if document["map_id"] != self.map_id:
            raise self._corrupt(f"it holds map {document['map_id']!r}, not {self.map_id!r}")
        entries = document["places"]
        if not isinstance(entries, list) or len(entries) > MAX_PLACES:
            raise self._corrupt(f"places is not a list of at most {MAX_PLACES}")
        places = [self._entry(index, item) for index, item in enumerate(entries)]
        keys = [name_key(place.name) for place in places]
        if len(set(keys)) != len(keys):
            raise self._corrupt("a name appears twice")
        return places

    def list(self) -> list[Place]:
        """Every place on this map, in the order they were first saved."""
        return self._read()

    def resolve(self, name: Any) -> Place:
        """The place with this name, or ``UnknownPlace`` naming the ones there are."""
        wanted = name_key(normalize_name(name))
        places = self._read()
        for place in places:
            if name_key(place.name) == wanted:
                return place
        raise UnknownPlace(normalize_name(name), tuple(place.name for place in places))

    # -- writing -------------------------------------------------------------

    def mark(self, name: Any, pose: Mapping[str, Any]) -> tuple[Place, Place | None]:
        """Save ``pose`` under ``name``; returns the place and the one it replaced.

        A name already saved is moved to the new pose and keeps its position in
        the list; the previous pose is returned so the caller can report it.
        """
        place = place_from_pose(normalize_name(name), pose)
        with _file_lock(self.path):
            places = self._read()
            key = name_key(place.name)
            replaced = next((item for item in places if name_key(item.name) == key), None)
            if replaced is not None:
                places = [place if item is replaced else item for item in places]
            elif len(places) >= MAX_PLACES:
                raise PlacesError(f"this map already has {MAX_PLACES} places")
            else:
                places.append(place)
            self._write(places)
        return place, replaced

    def _write(self, places: list[Place]) -> None:
        document = {
            "schema": PLACES_SCHEMA,
            "map_id": self.map_id,
            "places": [place.to_dict() for place in places],
        }
        text = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if len(text.encode("utf-8")) > MAX_FILE_BYTES:  # pragma: no cover - bounded above
            raise PlacesError("the places file would be too large")
        fsio.atomic_write(self.path, text, mode=0o644)
