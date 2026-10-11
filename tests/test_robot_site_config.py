"""The demo robot's site configuration, as a reinstall would lay it down.

The robot's Nav2 file and its Wi-Fi unit were changed on the robot itself
(2026-10-08 and 2026-10-10) and lived nowhere else, so a reinstall from this
repository would have brought back a different robot. Both files are now the
robot's own bytes, read over SSH (read-only) on 2026-10-11; the digests below
are what ``sha256sum`` printed on the robot. ``deploy/native_ros2/README.md``
says how to compare them again.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

NATIVE = Path(__file__).resolve().parents[1] / "deploy" / "native_ros2"
NAV2 = NATIVE / "nav2-burger-flyto.yaml"

# sha256 of the files on burger-01, read 2026-10-11 14:46 (+0800).
ROBOT_SHA256 = {
    "nav2-burger-flyto.yaml": (
        "4092dd91bece7b719d8f681925b5d3716a9783621c74847c220fbb80fac9a76d"
    ),
    "wifi-powersave-off.service": (
        "0269fa4c698ceb0e2d45ca46248b5f85cdb7494bcdc603096cf1dbdddb97c8cd"
    ),
}


@pytest.mark.parametrize("name", sorted(ROBOT_SHA256))
def test_the_file_is_the_robots_own_bytes(name):
    digest = hashlib.sha256((NATIVE / name).read_bytes()).hexdigest()
    assert digest == ROBOT_SHA256[name]


def _block(text: str, header: str) -> list[str]:
    """The lines of a top-level YAML section (``header:`` at column 0)."""
    lines = text.splitlines()
    start = lines.index(f"{header}:")
    out: list[str] = []
    for line in lines[start + 1:]:
        if line and not line.startswith((" ", "#")):
            break
        out.append(line)
    return out


def _settings(lines: list[str], indent: int) -> dict[str, str]:
    """``key: value`` pairs at exactly ``indent`` spaces."""
    found: dict[str, str] = {}
    for line in lines:
        stripped = line.lstrip(" ")
        if len(line) - len(stripped) != indent or stripped.startswith("#") or ":" not in stripped:
            continue
        key, _, value = stripped.partition(":")
        found[key] = value.strip()
    return found


def test_the_global_costmap_is_the_rolling_window_the_robot_runs():
    global_costmap = _settings(_block(NAV2.read_text(encoding="utf-8"), "global_costmap"), 6)
    assert global_costmap["rolling_window"] == "true"
    assert global_costmap["width"] == "6"
    assert global_costmap["height"] == "6"
    assert global_costmap["track_unknown_space"] == "false"
    assert global_costmap["plugins"] == '["obstacle_layer", "voxel_layer", "inflation_layer"]'


def test_the_robots_unused_static_layer_block_is_not_a_plugin():
    # The robot dropped static_layer from the plugin list on 2026-10-08 but
    # kept its parameter block; Nav2 loads only what `plugins` names.
    global_costmap = _settings(_block(NAV2.read_text(encoding="utf-8"), "global_costmap"), 6)
    assert "static_layer" in global_costmap
    assert "static_layer" not in global_costmap["plugins"]


def test_the_local_costmap_is_stock():
    local = _settings(_block(NAV2.read_text(encoding="utf-8"), "local_costmap"), 6)
    assert local["rolling_window"] == "true"
    assert local["robot_radius"] == "0.1"


def test_nav2_service_loads_the_robot_file():
    unit = (NATIVE / "nav2.service").read_text(encoding="utf-8")
    assert "params_file:=/etc/ros/nav2-burger-flyto.yaml" in unit
    assert "param/burger.yaml" not in unit


def test_wifi_power_save_is_turned_off_once_per_boot():
    unit = (NATIVE / "wifi-powersave-off.service").read_text(encoding="utf-8")
    assert "Type=oneshot" in unit
    assert "ExecStart=/usr/sbin/iw dev wlan0 set power_save off" in unit
    assert "WantedBy=multi-user.target" in unit


def test_the_install_steps_cover_both_files():
    readme = (NATIVE / "README.md").read_text(encoding="utf-8")
    assert "/etc/ros/nav2-burger-flyto.yaml" in readme
    assert "wifi-powersave-off.service" in readme
    assert "apt install iw" in readme
