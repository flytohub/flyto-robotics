"""The twin may loosen Nav2 timing only, never a distance, speed or shape."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "sim" / "twin" / "scripts" / "twin_nav2_params.py"
spec = importlib.util.spec_from_file_location("twin_nav2_params", SCRIPT)
twin = importlib.util.module_from_spec(spec)
spec.loader.exec_module(twin)

# The shape of turtlebot3_navigation2's burger.yaml where the overrides land.
BURGER = {
    "bt_navigator": {"ros__parameters": {"transform_tolerance": 0.5}},
    "collision_monitor": {
        "ros__parameters": {
            "transform_tolerance": 0.5,
            "source_timeout": 5.0,
            "PolygonStop": {"type": "circle", "radius": 0.1, "action_type": "stop"},
            "scan": {"source_timeout": 0.2, "type": "scan", "topic": "/scan"},
        }
    },
    "controller_server": {"ros__parameters": {"FollowPath": {"max_vel_x": 0.3}}},
}


def test_overrides_change_only_the_twins_timing():
    merged = twin.apply(BURGER, twin.OVERRIDES)
    monitor = merged["collision_monitor"]["ros__parameters"]
    assert monitor["scan"]["source_timeout"] == 0.5
    assert merged["bt_navigator"]["ros__parameters"]["default_server_timeout"] == 200
    # Everything else is the robot's file, untouched.
    assert monitor["PolygonStop"] == BURGER["collision_monitor"]["ros__parameters"]["PolygonStop"]
    assert merged["controller_server"] == BURGER["controller_server"]
    assert BURGER["collision_monitor"]["ros__parameters"]["scan"]["source_timeout"] == 0.2


@pytest.mark.parametrize(
    "override",
    [
        {"collision_monitor": {"ros__parameters": {"PolygonStop": {"radius": 0.01}}}},
        {"controller_server": {"ros__parameters": {"FollowPath": {"max_vel_x": 1.0}}}},
        {"collision_monitor": {"ros__parameters": {"scan": {"source_timeout": 30.0}}}},
        {"collision_monitor": {"ros__parameters": {"scan": {"source_timeout": 0}}}},
        {"collision_monitor": {"ros__parameters": {"scan": {"source_timeout": "1"}}}},
        {"collison_monitor": {"ros__parameters": {"source_timeout": 0.5}}},
    ],
)
def test_anything_but_a_bounded_timing_change_is_refused(override):
    with pytest.raises(twin.OverrideError):
        twin.apply(BURGER, override)
