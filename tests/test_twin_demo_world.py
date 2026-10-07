"""The twin's demo world ships with the repository and reaches the container."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

TWIN = Path(__file__).resolve().parents[1] / "sim" / "twin"
WORLD = TWIN / "worlds" / "flyto_demo_room.world"
MOUNT = "- ./worlds:/opt/flyto-twin/worlds:ro"


def test_compose_mounts_the_worlds_directory_read_only():
    assert MOUNT in (TWIN / "compose.yaml").read_text(encoding="utf-8")


def test_demo_world_is_one_room_and_one_box_ahead_of_the_start_pose():
    world = ET.parse(WORLD).getroot().find("world")
    assert world is not None
    models = {m.get("name"): m for m in world.findall("model")}
    assert set(models) == {"room", "obstacle"}
    box = models["obstacle"].find("link/pose").text.split()
    depth = float(models["obstacle"].find("link/collision/geometry/box/size").text.split()[0])
    # The robot starts at x = -1.8 facing +x; the box's near face is 0.75 m ahead.
    assert round(float(box[0]) - depth / 2 - (-1.8), 3) == 0.75
    assert float(box[1]) == 0.0
