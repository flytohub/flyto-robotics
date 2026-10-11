"""The demo robot's site configuration, as a reinstall would lay it down.

The robot's Nav2 file and its Wi-Fi unit were changed on the robot itself
(2026-10-08 and 2026-10-10) and lived nowhere else, so a reinstall from this
repository would have brought back a different robot. These tests pin what
was recorded about them; ``deploy/native_ros2/README.md`` says how they were
reconstructed and how to compare them with the robot.
"""

from __future__ import annotations

from pathlib import Path

NATIVE = Path(__file__).resolve().parents[1] / "deploy" / "native_ros2"
NAV2 = NATIVE / "nav2-burger-flyto.yaml"


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
    assert "static_layer" not in global_costmap


def test_nothing_else_names_a_static_layer_in_the_global_costmap():
    block = "\n".join(_block(NAV2.read_text(encoding="utf-8"), "global_costmap"))
    assert "StaticLayer" not in block


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
