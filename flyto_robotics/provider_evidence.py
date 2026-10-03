"""Evidence the adapter produces itself, in the shapes its hosts already read.

A host used to turn what the adapter observed into evidence: the clearance a
motion started with, the odometry before, after and once settled, and the
picture a capture returned. Each host did it again, with the robot's own
numbers (the 0.35 m floor) copied into it. This module lets the adapter hand
those items over finished, so a host can pass them through.

Everything here is additive. A result keeps every key it had before
(``capture``, ``odom``, ``motion_outcome``); the new keys are

``evidence_items``
    a list of ``flyto.space.evidence.v1``-shaped items: ``passage.clearance``
    and ``robot.arrival``, byte for byte the shapes Flyto2 Cloud and Desktop
    have projected since 2026-10-02, so a host that adopts them judges exactly
    what it judged before.
``artifacts``
    ``[{"kind", "media_type", "data_base64"}]``, the artifact transport of
    ``flyto.capability-contract.v1`` (flyto-core 2.36.0): the photo as it came
    from the camera, and the occupancy map drawn as a picture.
``recovery_context``
    what a straight motion measured when it stopped short, for the pack's
    declared ``recovery`` (``observe: recovery_context``).

Pure: no ROS, no network. Pillow is used to encode the map as JPEG when it is
installed (it is on every Flyto2 Desktop); without it the map is a PNG built
with the standard library, and the contract declares both media types.
"""

from __future__ import annotations

import base64
import math
import struct
import zlib
from collections.abc import Mapping
from typing import Any

PASSAGE_CLEARANCE = "passage.clearance"
ROBOT_ARRIVAL = "robot.arrival"

ARTIFACT_PHOTO = "photo"
ARTIFACT_MAP = "map"
MEDIA_JPEG = "image/jpeg"
MEDIA_PNG = "image/png"

# Motions a host judges from odometry before, after and once settled.
JUDGED_MOTIONS = frozenset({"motion.advance", "motion.retreat", "motion.rotate"})
# Straight motions: the ones a detour can be planned around.
STRAIGHT_MOTIONS = frozenset({"motion.advance", "motion.retreat"})

# The narrowest map picture: a 20-cell map drawn one pixel per cell is a dot.
MAP_MIN_WIDTH_PX = 600
# The most pixels the upscale may produce. A map is at most 4,000,000 cells, but
# a thin one (1 x 4,000,000) scaled to 600 px wide would need terabytes; the
# upscale stops where this budget ends, and a map already past it is drawn
# one pixel per cell.
MAP_MAX_PIXELS = 16_000_000
# Cell shades, the way maps are read: free white, occupied black, unknown grey.
UNKNOWN_SHADE = 205
CLEARANCE_SOURCE = "FLYTO_ROS2_MIN_CLEARANCE_M"
OBSERVATION_SOURCE = "ros2.observation-bundle.v1"


# -- motion evidence ---------------------------------------------------------


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _phase_pose(bundle: Mapping[str, Any] | None) -> dict[str, Any] | None:
    pose = bundle.get("pose") if isinstance(bundle, Mapping) else None
    return dict(pose) if isinstance(pose, Mapping) else None


def _phase_sweep(bundle: Mapping[str, Any] | None) -> dict[str, Any] | None:
    reading = bundle.get("range") if isinstance(bundle, Mapping) else None
    sweep = reading.get("sweep") if isinstance(reading, Mapping) else None
    return dict(sweep) if isinstance(sweep, Mapping) else None


def clearance_item(
    before: Mapping[str, Any] | None, floor_m: float
) -> dict[str, Any] | None:
    """The clearance a motion started with, against the adapter's own floor.

    None when the robot reported no range (a robot whose safety basis is a
    present operator has no LiDAR to measure with).
    """
    reading = before.get("range") if isinstance(before, Mapping) else None
    if not isinstance(reading, Mapping):
        return None
    measured = _number(reading.get("minimum_range_m"))
    floor = float(floor_m)
    return {
        "kind": PASSAGE_CLEARANCE,
        "usable": measured is not None and measured >= floor,
        "detail": f"minimum LiDAR clearance {measured} m before moving (floor {floor} m)",
        "reason": {
            "code": "lidar_minimum_range",
            "observed": {"minimum_range_m": measured},
            "required": {"minimum_range_m": floor},
            "source": CLEARANCE_SOURCE,
        },
    }


def arrival_item(
    capability_id: str,
    *,
    before: Mapping[str, Any] | None,
    after: Mapping[str, Any] | None,
    settled: Mapping[str, Any] | None,
) -> dict[str, Any] | None:
    """Where the robot was before, right after and once settled.

    For a judged motion the item claims nothing (``usable`` False): the host
    compares the poses with the step's own arguments and sets the verdict, as
    it always has. A navigation reports the pose it ended at.
    """
    if capability_id in JUDGED_MOTIONS:
        return {
            "kind": ROBOT_ARRIVAL,
            # The host's verdict replaces this; the adapter makes no claim.
            "usable": False,
            "detail": "odometry before, after and once settled; judged by Cloud",
            "reason": {
                "code": "odom_displacement",
                "observed": {
                    "capability_id": capability_id,
                    "before": _phase_pose(before),
                    "after": _phase_pose(after),
                    "post_stop": _phase_pose(settled),
                    # What the LiDAR saw before moving and once stopped, for
                    # the operator to look at; not part of the verdict.
                    "sweeps": {
                        phase: sweep
                        for phase, sweep in (
                            ("before", _phase_sweep(before)),
                            ("post_stop", _phase_sweep(settled)),
                        )
                        if sweep is not None
                    },
                    "snapshots": [
                        bundle.get("snapshot", "")
                        for bundle in (before, after, settled)
                        if isinstance(bundle, Mapping)
                    ],
                },
                "required": {},
                "source": OBSERVATION_SOURCE,
            },
        }
    pose = after.get("pose") if isinstance(after, Mapping) else None
    if capability_id == "motion.navigate" and isinstance(pose, Mapping):
        return {
            "kind": ROBOT_ARRIVAL,
            "usable": True,
            "detail": (
                f"observed pose {pose.get('frame', '')}: "
                f"x={pose.get('x')}, y={pose.get('y')}, yaw={pose.get('yaw')}"
            )[:200],
        }
    return None


# -- recovery ----------------------------------------------------------------


def _along(start: Mapping[str, Any] | None, end: Mapping[str, Any] | None) -> float | None:
    """Signed travel along the starting heading; negative is backwards."""
    if not isinstance(start, Mapping) or not isinstance(end, Mapping):
        return None
    x0, y0, yaw = _number(start.get("x")), _number(start.get("y")), _number(start.get("yaw"))
    x1, y1 = _number(end.get("x")), _number(end.get("y"))
    if None in (x0, y0, yaw, x1, y1):
        return None
    return (x1 - x0) * math.cos(yaw) + (y1 - y0) * math.sin(yaw)


def recovery_context(
    summary: Mapping[str, Any] | None,
    *,
    sweep: Mapping[str, Any] | None = None,
) -> dict[str, Any] | None:
    """What a straight motion measured when it stopped short.

    Facts only: why it stopped (``motion_outcome``'s reason), how far it was
    asked to go and how far it went along its own heading, the nearest return
    at the stop, the floor, and the robot-frame sweep at the stop. Planning
    the way round belongs to whoever reads the pack's declared ``recovery``.
    """
    if not isinstance(summary, Mapping):
        return None
    capability_id = str(summary.get("capability_id") or "")
    if capability_id not in STRAIGHT_MOTIONS:
        return None
    requested = _number(summary.get("requested_distance_m"))
    along = _along(summary.get("start_pose"), summary.get("final_pose"))
    travelled = None
    if along is not None:
        travelled = round(max(0.0, -along if capability_id == "motion.retreat" else along), 4)
    context: dict[str, Any] = {
        "capability_id": capability_id,
        "reason": str(summary.get("reason") or ""),
        "requested_distance_m": requested,
        "travelled_m": travelled,
        "remaining_m": (
            round(max(0.0, requested - travelled), 4)
            if requested is not None and travelled is not None
            else None
        ),
        "minimum_range_at_stop_m": summary.get("minimum_range_at_stop_m"),
        "travel_direction_range_m": summary.get("travel_direction_range_m"),
        "clearance_floor_m": summary.get("clearance_floor_m"),
        "start_pose": summary.get("start_pose"),
        "final_pose": summary.get("final_pose"),
    }
    if isinstance(sweep, Mapping):
        context["sweep"] = dict(sweep)
    return context


# -- artifacts ---------------------------------------------------------------


def photo_artifact(capture: Mapping[str, Any]) -> dict[str, Any]:
    """The camera's JPEG, as the contract's artifact transport carries it."""
    if capture.get("kind") != ARTIFACT_PHOTO or capture.get("media_type") != MEDIA_JPEG:
        raise ValueError("the capture is not a JPEG photo")
    return {
        "kind": ARTIFACT_PHOTO,
        "media_type": MEDIA_JPEG,
        "data_base64": str(capture["data_base64"]),
    }


def _map_shades(capture: Mapping[str, Any]) -> tuple[bytes, int, int]:
    """Grey levels, top row first (an OccupancyGrid's first row is the bottom)."""
    width, height = int(capture["width"]), int(capture["height"])
    cells = base64.b64decode(str(capture["cells_base64"]))
    if width <= 0 or height <= 0 or len(cells) != width * height:
        raise ValueError("map cells do not match its size")
    # 0..100 is occupancy in percent; 255 (int8 -1) and anything above 100 is
    # unknown.
    table = bytes(
        UNKNOWN_SHADE if value > 100 else 255 - round(value * 2.55) for value in range(256)
    )
    shades = cells.translate(table)
    rows = [shades[row * width : (row + 1) * width] for row in range(height)]
    rows.reverse()
    return b"".join(rows), width, height


def _scaled(shades: bytes, width: int, height: int) -> tuple[bytes, int, int]:
    """Nearest-neighbour upscale so a small map is still a picture."""
    scale = max(1, -(-MAP_MIN_WIDTH_PX // width))
    cells = width * height
    while scale > 1 and cells * scale * scale > MAP_MAX_PIXELS:
        scale -= 1
    if scale == 1:
        return shades, width, height
    rows = []
    for row in range(height):
        line = shades[row * width : (row + 1) * width]
        wide = bytes(value for value in line for _ in range(scale))
        rows.extend([wide] * scale)
    return b"".join(rows), width * scale, height * scale


def _png(shades: bytes, width: int, height: int) -> bytes:
    """An 8-bit greyscale PNG, standard library only."""

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    raw = b"".join(
        b"\x00" + shades[row * width : (row + 1) * width] for row in range(height)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def _jpeg(shades: bytes, width: int, height: int) -> bytes | None:
    try:
        from io import BytesIO

        from PIL import Image
    except ImportError:
        return None
    image = Image.frombytes("L", (width, height), shades)
    buffer = BytesIO()
    image.save(buffer, format="JPEG", quality=90)
    return buffer.getvalue()


def render_map(capture: Mapping[str, Any]) -> tuple[bytes, str, int, int]:
    """The occupancy grid drawn the way maps are read; (bytes, media type, w, h).

    The same drawing Flyto2 Desktop made from the cells until now: free white,
    walls black, unknown grey, north up, at least ``MAP_MIN_WIDTH_PX`` wide.
    """
    shades, width, height = _map_shades(capture)
    shades, width, height = _scaled(shades, width, height)
    jpeg = _jpeg(shades, width, height)
    if jpeg is not None:
        return jpeg, MEDIA_JPEG, width, height
    return _png(shades, width, height), MEDIA_PNG, width, height


def map_artifact(capture: Mapping[str, Any]) -> dict[str, Any]:
    """The map as a picture, in the contract's artifact transport."""
    data, media_type, _width, _height = render_map(capture)
    return {
        "kind": ARTIFACT_MAP,
        "media_type": media_type,
        "data_base64": base64.b64encode(data).decode("ascii"),
    }


def capture_artifacts(capture: Mapping[str, Any]) -> list[dict[str, Any]]:
    """The artifact a capture result carries, beside its legacy ``capture``."""
    kind = capture.get("kind")
    if kind == ARTIFACT_PHOTO:
        return [photo_artifact(capture)]
    if kind == ARTIFACT_MAP:
        return [map_artifact(capture)]
    raise ValueError(f"unknown capture kind {kind!r}")


__all__ = [
    "ARTIFACT_MAP",
    "ARTIFACT_PHOTO",
    "JUDGED_MOTIONS",
    "MAP_MAX_PIXELS",
    "MAP_MIN_WIDTH_PX",
    "MEDIA_JPEG",
    "MEDIA_PNG",
    "PASSAGE_CLEARANCE",
    "ROBOT_ARRIVAL",
    "STRAIGHT_MOTIONS",
    "arrival_item",
    "capture_artifacts",
    "clearance_item",
    "map_artifact",
    "photo_artifact",
    "recovery_context",
    "render_map",
]
