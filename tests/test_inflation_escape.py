"""Leaving an obstacle's inflation before a Nav2 goal: geometry and adapter wiring.

Scenes are ray-cast in the robot frame (x forward, y left) into the same
reduced sweep the adapter's observation carries, so the geometry is checked
against what the LiDAR would report, not against the scene itself.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from types import SimpleNamespace
from typing import Any

import pytest

from flyto_robotics import inflation_escape as escape
from flyto_robotics.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
    CallResult,
)
from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    NAVIGATE_THROUGH,
    GenericROS2Adapter,
    RosbridgeROS2Backend,
    StandardInterface,
)

FLOOR = 0.35
BINS = 180
MAX_RANGE = 3.5
TWIN = escape.CostmapGeometry(0.5, 0.1, source="parameters")


# -- scenes ------------------------------------------------------------------


def box(near_x: float, depth: float, width: float, centre_y: float = 0.0):
    """An axis-aligned box ahead of the robot as its four edges."""
    x0, x1 = near_x, near_x + depth
    y0, y1 = centre_y - width / 2, centre_y + width / 2
    return [((x0, y0), (x0, y1)), ((x1, y0), (x1, y1)), ((x0, y0), (x1, y0)), ((x0, y1), (x1, y1))]


def wall_left(y: float):
    return [((-5.0, y), (5.0, y))]


def wall_right(y: float):
    return [((-5.0, -y), (5.0, -y))]


def wall_rear(x: float):
    return [((-x, -5.0), (-x, 5.0))]


def room(size: float = 3.0):
    """Walls far enough to give every sector a return."""
    return wall_left(size) + wall_right(size) + wall_rear(size) + [((size, -5.0), (size, 5.0))]


def _ray(bearing: float, segment) -> float | None:
    (ax, ay), (bx, by) = segment
    dx, dy = math.cos(bearing), math.sin(bearing)
    ex, ey = bx - ax, by - ay
    denominator = dx * ey - dy * ex
    if abs(denominator) < 1e-12:
        return None
    t = (ax * ey - ay * ex) / denominator
    u = (ax * dy - ay * dx) / denominator
    if t <= 0.0 or not 0.0 <= u <= 1.0:
        return None
    return t


def scan(segments, *, bins: int = BINS) -> dict[str, Any]:
    step = math.tau / bins
    ranges: list[float | None] = []
    for index in range(bins):
        hits = [hit for segment in segments if (hit := _ray(index * step, segment)) is not None]
        nearest = min(hits) if hits else None
        ranges.append(round(nearest, 3) if nearest is not None and nearest <= MAX_RANGE else None)
    return {"angle_min_rad": 0.0, "angle_increment_rad": step, "ranges_m": ranges}


def decide(segments, geometry=TWIN, **overrides):
    options = {"floor_m": FLOOR, "max_backoff_m": 0.30, "margin_m": 0.10, "max_lateral_m": 1.0}
    options.update(overrides)
    return escape.plan_escape(scan(segments), geometry, **options)


# The failure seen on the twin: a 0.30 x 0.40 m box 0.40 m ahead.
OBSERVED = box(0.40, 0.30, 0.40) + room()


# -- costmap parameters ------------------------------------------------------


def _double(value):
    return {"type": 3, "double_value": value}


def _text(value):
    return {"type": 4, "string_value": value}


TWIN_PARAMS = {
    "/local_costmap/local_costmap": {
        "plugins": {
            "type": 9,
            "string_array_value": ["obstacle_layer", "voxel_layer", "inflation_layer"],
        },
        "obstacle_layer.plugin": _text("nav2_costmap_2d::ObstacleLayer"),
        "voxel_layer.plugin": _text("nav2_costmap_2d::VoxelLayer"),
        "inflation_layer.plugin": _text("nav2_costmap_2d::InflationLayer"),
        "inflation_layer.enabled": {"type": 1, "bool_value": True},
        "inflation_layer.inflation_radius": _double(0.5),
        "robot_radius": _double(0.1),
        "footprint": _text("[]"),
    },
    "/global_costmap/global_costmap": {
        "plugins": {"type": 9, "string_array_value": ["static_layer", "inflation_layer"]},
        "static_layer.plugin": _text("nav2_costmap_2d::StaticLayer"),
        "inflation_layer.plugin": _text("nav2_costmap_2d::InflationLayer"),
        "inflation_layer.inflation_radius": _double(0.5),
        "robot_radius": _double(0.1),
        "footprint": _text("[]"),
    },
}
NODES = tuple(TWIN_PARAMS)


def reader_for(params):
    def read(node, name):
        if node not in params:
            raise RuntimeError("service unavailable")
        return params[node].get(name)

    return read


def test_geometry_comes_from_the_live_costmap_parameters():
    geometry = escape.read_costmap_geometry(reader_for(TWIN_PARAMS), NODES)
    assert (geometry.inflation_radius_m, geometry.robot_radius_m) == (0.5, 0.1)
    assert geometry.source == "parameters"
    assert geometry.pinned_threshold_m == pytest.approx(0.6)


def test_geometry_takes_the_largest_inflation_and_a_polygon_footprint():
    params = {node: dict(values) for node, values in TWIN_PARAMS.items()}
    params["/global_costmap/global_costmap"]["inflation_layer.inflation_radius"] = _double(0.7)
    params["/local_costmap/local_costmap"]["footprint"] = _text(
        "[[0.15, 0.12], [0.15, -0.12], [-0.15, -0.12], [-0.15, 0.12]]"
    )
    geometry = escape.read_costmap_geometry(reader_for(params), NODES)
    assert geometry.inflation_radius_m == 0.7
    assert geometry.robot_radius_m == pytest.approx(math.hypot(0.15, 0.12))


def test_geometry_ignores_a_disabled_or_implausible_inflation_layer():
    params = {node: dict(values) for node, values in TWIN_PARAMS.items()}
    local = params["/local_costmap/local_costmap"]
    local["inflation_layer.enabled"] = {"type": 1, "bool_value": False}
    local["inflation_layer.inflation_radius"] = _double(9.0)
    params["/global_costmap/global_costmap"]["inflation_layer.inflation_radius"] = _double(-1.0)
    geometry = escape.read_costmap_geometry(reader_for(params), NODES)
    assert geometry.inflation_radius_m == escape.DEFAULT_INFLATION_RADIUS_M
    assert geometry.source == "partial"


@pytest.mark.parametrize("reader", [None, reader_for({})])
def test_geometry_falls_back_to_nav2_defaults(reader):
    geometry = escape.read_costmap_geometry(reader, NODES)
    assert geometry.inflation_radius_m == escape.DEFAULT_INFLATION_RADIUS_M
    assert geometry.robot_radius_m == escape.DEFAULT_ROBOT_RADIUS_M
    assert geometry.source == "fallback"
    assert geometry.notes


@pytest.mark.parametrize(
    "text",
    [
        "",
        "[]",
        "[[0.1, 0.1]]",
        "nonsense",
        "[[0.1, 0.1], [0.1], [0, 0]]",
        "[[1e999, 0], [0, 0], [1, 1]]",
    ],
)
def test_footprint_that_is_not_a_polygon_is_not_used(text):
    assert escape.parse_footprint(text) is None


# -- scan geometry -----------------------------------------------------------


@pytest.mark.parametrize(
    ("segments", "sector", "expected"),
    [
        ([((0.7, -5.0), (0.7, 5.0))], "front", 0.7),
        (wall_left(0.8), "left", 0.8),
        (wall_right(1.2), "right", 1.2),
        (wall_rear(0.6), "rear", 0.6),
    ],
)
def test_sector_minima_read_each_direction(segments, sector, expected):
    minima = escape.sector_minima(escape.sweep_beams(scan(segments)))
    assert minima[sector] == pytest.approx(expected, abs=0.01)
    assert all(
        value is None or value > expected for name, value in minima.items() if name != sector
    )


def test_a_sector_the_lidar_cannot_see_is_not_room():
    beams = escape.sweep_beams(scan(wall_left(0.8)))
    assert escape.sector_minimum(beams, math.pi / 2) == pytest.approx(0.8, abs=0.01)
    assert escape.sector_minimum(beams, -math.pi / 2) is None


def test_backoff_distance_is_clamped():
    assert escape.backoff_distance(0.40, 0.60, 0.30) == pytest.approx(0.20)
    assert escape.backoff_distance(0.20, 0.60, 0.30) == pytest.approx(0.30)
    assert escape.backoff_distance(0.80, 0.60, 0.30) == 0.0


def test_side_choice_prefers_more_room_and_never_an_unreadable_side():
    assert escape.choose_sides(0.6, 1.5) == ("right", "left")
    assert escape.choose_sides(1.5, 0.6) == ("left", "right")
    assert escape.choose_sides(None, 0.4) == ("right", "left")


# -- decisions ---------------------------------------------------------------


def test_a_start_inside_inflation_is_pinned_and_a_free_one_is_not():
    pinned = decide(OBSERVED)
    assert pinned.pinned and pinned.feasible
    assert pinned.clearances["front"] == pytest.approx(0.40, abs=0.01)
    free = decide(box(1.0, 0.30, 0.40) + room())
    assert not free.pinned and free.reason == escape.DECISION_CLEAR


def test_observed_box_backs_off_to_the_threshold():
    decision = decide(OBSERVED)
    assert decision.backoff_m == pytest.approx(0.20, abs=0.01)
    assert decision.backoff_limited_by is None


def test_rear_blocked_means_no_backoff():
    decision = decide(box(0.40, 0.30, 0.40) + wall_left(3) + wall_right(3) + wall_rear(FLOOR))
    assert decision.pinned
    assert decision.backoff_m == 0.0
    assert decision.backoff_limited_by == "rear_clearance"


def test_rear_unreadable_means_no_backoff():
    decision = decide(box(0.40, 0.30, 0.40) + wall_left(3) + wall_right(3))
    assert decision.backoff_m == 0.0
    assert decision.backoff_limited_by == "rear_unreadable"


def test_backoff_never_takes_the_rear_below_the_floor():
    decision = decide(box(0.40, 0.30, 0.40) + wall_left(3) + wall_right(3) + wall_rear(0.45))
    assert decision.backoff_m == pytest.approx(0.45 - FLOOR, abs=0.01)
    assert decision.clearances["rear"] - decision.backoff_m >= FLOOR - 1e-9


def test_side_choice_picks_the_larger_free_side():
    decision = decide(box(0.40, 0.30, 0.40) + wall_left(1.0) + wall_right(2.0) + wall_rear(3))
    assert decision.side == "right" and decision.lateral_offset_m < 0
    mirrored = decide(box(0.40, 0.30, 0.40) + wall_left(2.0) + wall_right(1.0) + wall_rear(3))
    assert mirrored.side == "left" and mirrored.lateral_offset_m > 0


def test_waypoint_clears_the_box_by_robot_radius_and_margin():
    decision = decide(OBSERVED)
    half_width = 0.20
    assert abs(decision.lateral_offset_m) >= half_width + TWIN.robot_radius_m + 0.10 - 0.01
    x, y = decision.waypoint_robot
    # The waypoint sits outside the box grown by the robot's radius.
    assert not (0.40 - 0.1 <= x <= 0.70 + 0.1 and abs(y) <= half_width + 0.1)
    beams = escape.sweep_beams(scan(OBSERVED))
    assert escape.segment_clearance(beams, (x, y), (x, y)) >= FLOOR


def test_no_room_on_either_side_fails_fast_with_named_reason():
    decision = decide(box(0.40, 0.30, 0.40) + wall_left(0.45) + wall_right(0.45) + wall_rear(3))
    assert decision.pinned and not decision.feasible
    assert decision.reason == escape.REASON_NO_ESCAPE_ROOM
    text = decision.describe()
    assert text.startswith("no_escape_room:") and "left 0.45 m" in text and "right 0.45 m" in text
    assert {item["side"] for item in decision.sides_tried} == {"left", "right"}
    assert decision.to_dict()["decision"] == "no_escape_room"


def test_wide_obstacle_beyond_max_lateral_has_no_escape():
    decision = decide(box(0.40, 0.30, 3.0) + room(3.0), max_lateral_m=1.0)
    assert decision.reason == escape.REASON_NO_ESCAPE_ROOM
    assert [item.get("refused") for item in decision.sides_tried] == ["beyond_max_lateral"] * 2


@pytest.mark.parametrize("near", [0.36, 0.40, 0.45, 0.50, 0.55])
@pytest.mark.parametrize("width", [0.2, 0.4, 0.6])
@pytest.mark.parametrize("side_room", [0.6, 0.9, 1.5])
@pytest.mark.parametrize("rear", [0.36, 0.5, 1.0])
def test_floor_is_never_violated(near, width, side_room, rear):
    segments = box(near, 0.30, width) + wall_left(side_room) + wall_right(side_room + 0.3)
    segments += wall_rear(rear)
    decision = decide(segments)
    beams = escape.sweep_beams(scan(segments))
    assert decision.pinned
    if decision.clearances["rear"] is not None:
        assert decision.clearances["rear"] - decision.backoff_m >= FLOOR - 1e-9
    if not decision.feasible:
        return
    back = (-decision.backoff_m, 0.0)
    assert escape.segment_clearance(beams, (0.0, 0.0), back) >= FLOOR - 1e-9
    assert escape.segment_clearance(beams, back, decision.waypoint_robot) >= FLOOR - 1e-9


def test_no_sweep_means_no_decision():
    decision = escape.plan_escape(
        None, TWIN, floor_m=FLOOR, max_backoff_m=0.3, margin_m=0.1, max_lateral_m=1
    )
    assert decision is None


def test_waypoint_pose_is_placed_from_the_start_map_pose_and_faces_the_goal():
    decision = decide(OBSERVED)
    start = {"x": 0.01, "y": 0.0, "yaw": 0.0}
    pose = escape.waypoint_pose(start, decision, (1.20, 0.0))
    assert pose["x"] == pytest.approx(0.01 - decision.backoff_m, abs=1e-3)
    assert pose["y"] == pytest.approx(decision.lateral_offset_m, abs=1e-3)
    assert pose["yaw_radians"] == pytest.approx(math.atan2(-pose["y"], 1.20 - pose["x"]), abs=1e-3)
    turned = escape.to_map({"x": 1.0, "y": 2.0, "yaw": math.pi / 2}, (1.0, 0.0))
    assert turned == pytest.approx((1.0, 3.0))


# -- adapter wiring ----------------------------------------------------------


class EscapeBackend:
    """A ROS graph whose LiDAR sees ``segments`` and whose costmaps are the twin's."""

    def __init__(self, segments, *, through=True, retreat=True, params=TWIN_PARAMS, outcomes=None):
        names = {"motion.navigate", "motion.halt"} | ({"motion.retreat"} if retreat else set())
        self.interfaces = [
            StandardInterface(
                DEFAULT_INTERFACES[name][0],
                DEFAULT_INTERFACES[name][1],
                "geometry_msgs/msg/Twist" if name == "motion.halt" else DEFAULT_INTERFACES[name][2],
            )
            for name in names
        ]
        self.through = through
        self.params = params
        self.calls: list[tuple[str, str, dict]] = []
        self.cancelled: list[str] = []
        self.outcomes = outcomes or {}
        self.payload = {
            "pose": {"frame": "odom", "x": 0.01, "y": 0.0, "yaw": 0.0},
            "map_pose": {"frame": "map", "x": 0.01, "y": 0.0, "yaw": 0.0},
            "range": {"minimum_range_m": 0.40, "sample_count": 360, "sweep": scan(segments)},
            "map_tf_available": True,
        }

    def discover(self):
        return tuple(self.interfaces)

    def observation(self, required=None):
        return dict(self.payload)

    def get_parameter(self, node, name):
        return reader_for(self.params)(node, name)

    def action_available(self, capability_id):
        return capability_id == NAVIGATE_THROUGH and self.through

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        self.calls.append((call_id, capability_id, dict(arguments)))
        outcome = self.outcomes.get(capability_id, OUTCOME_COMPLETED)
        if callable(outcome):
            outcome = outcome(call_id)
        return CallResult(call_id, outcome, evidence={"odom": {"x": 0.0}}, detail=outcome)

    def cancel(self, call_id):
        self.cancelled.append(call_id)
        return CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")

    def safe_stop(self, call_id):
        return CallResult(call_id, OUTCOME_COMPLETED)

    def execution_count(self, call_id):
        return 0


def navigate(adapter, call_id="nav-1", x=1.20, y=0.0):
    return adapter.invoke(
        CallRequest(
            call_id=call_id,
            capability_id="motion.navigate",
            arguments={"x": x, "y": y},
            deadline_seconds=30.0,
        )
    )


def make(backend):
    return GenericROS2Adapter(backend=backend, resource_id="twin")


@pytest.fixture(autouse=True)
def _simulation_marker_off(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    monkeypatch.delenv("FLYTO_ROS2_INFLATION_ESCAPE", raising=False)


def test_free_start_sends_the_goal_unchanged():
    backend = EscapeBackend(box(1.0, 0.3, 0.4) + room())
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_COMPLETED
    assert backend.calls == [("nav-1", "motion.navigate", {"x": 1.20, "y": 0.0})]
    assert result.evidence["navigation_escape"]["pinned"] is False


def test_pinned_start_backs_off_then_routes_through_the_waypoint_to_the_real_goal():
    backend = EscapeBackend(OBSERVED)
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_COMPLETED and result.call_id == "nav-1"
    (backoff_id, backoff_cap, backoff_args), (route_id, route_cap, route_args) = backend.calls
    assert (backoff_id, backoff_cap) == ("nav-1:escape-backoff", "motion.retreat")
    assert backoff_args["distance_m"] == pytest.approx(0.20, abs=0.01)
    # The original goal stays the final pose, under the call's own id.
    assert (route_id, route_cap) == ("nav-1", NAVIGATE_THROUGH)
    assert (route_args["x"], route_args["y"]) == (1.20, 0.0)
    (waypoint,) = route_args["waypoints"]
    assert abs(waypoint["y"]) > 0.20 + 0.1
    escape_evidence = result.evidence["navigation_escape"]
    assert escape_evidence["decision"] == "escape"
    assert escape_evidence["costmap"]["inflation_radius_m"] == 0.5
    assert [leg["leg"] for leg in escape_evidence["legs"]] == ["backoff", "through_poses"]
    assert result.evidence["navigation_target"]["x"] == 1.20


def test_without_navigate_through_poses_the_legs_are_sequential():
    backend = EscapeBackend(OBSERVED, through=False)
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_COMPLETED
    assert [(call, cap) for call, cap, _ in backend.calls] == [
        ("nav-1:escape-backoff", "motion.retreat"),
        ("nav-1:escape-waypoint", "motion.navigate"),
        ("nav-1", "motion.navigate"),
    ]
    assert backend.calls[-1][2] == {"x": 1.20, "y": 0.0}


def test_no_escape_room_refuses_before_any_motion():
    boxed_in = box(0.40, 0.30, 0.40) + wall_left(0.45) + wall_right(0.45) + wall_rear(3)
    backend = EscapeBackend(boxed_in)
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_REFUSED
    assert result.detail.startswith("no_escape_room:")
    assert result.evidence["reason_code"] == "no_escape_room"
    clearances = result.evidence["navigation_escape"]["clearances_m"]
    assert clearances["left"] == pytest.approx(0.45, abs=0.01)
    assert backend.calls == []


def test_a_failed_backoff_ends_the_call_and_names_the_leg():
    backend = EscapeBackend(OBSERVED, outcomes={"motion.retreat": OUTCOME_FAILED})
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_FAILED
    assert result.detail.startswith("inflation escape backoff failed")
    assert len(backend.calls) == 1


def test_without_a_backup_action_the_escape_is_lateral_only():
    backend = EscapeBackend(OBSERVED, retreat=False)
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_COMPLETED
    assert [cap for _, cap, _ in backend.calls] == [NAVIGATE_THROUGH]
    assert result.evidence["navigation_escape"]["backoff_m"] == 0.0


def test_unreadable_costmap_parameters_fall_back_and_still_escape():
    backend = EscapeBackend(OBSERVED, params={})
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_COMPLETED
    costmap = result.evidence["navigation_escape"]["costmap"]
    assert costmap["source"] == "fallback"
    assert costmap["inflation_radius_m"] == escape.DEFAULT_INFLATION_RADIUS_M


def test_escape_can_be_switched_off(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_INFLATION_ESCAPE", "off")
    backend = EscapeBackend(OBSERVED)
    navigate(make(backend))
    assert [cap for _, cap, _ in backend.calls] == ["motion.navigate"]


def test_cancel_during_an_escape_leg_reaches_that_leg_and_stops_the_rest():
    holder: dict[str, Any] = {}

    def retreat(call_id):
        holder["cancel"] = holder["adapter"].cancel("nav-1")
        return OUTCOME_CANCELLED

    backend = EscapeBackend(OBSERVED, outcomes={"motion.retreat": retreat})
    holder["adapter"] = make(backend)
    result = navigate(holder["adapter"])
    assert holder["cancel"].outcome == OUTCOME_CANCELLED
    assert backend.cancelled == ["nav-1:escape-backoff"]
    assert result.outcome == OUTCOME_CANCELLED
    assert len(backend.calls) == 1


def test_rosbridge_sends_waypoints_then_goal_as_one_route():
    goal = RosbridgeROS2Backend._action_goal(
        SimpleNamespace(_duration=RosbridgeROS2Backend._duration),
        NAVIGATE_THROUGH,
        {"x": 1.2, "y": 0.0, "waypoints": [{"x": -0.19, "y": 0.4, "yaw_radians": -0.3}]},
    )
    poses = goal["poses"]
    assert [pose["pose"]["position"]["x"] for pose in poses] == [-0.19, 1.2]
    assert poses[0]["header"]["frame_id"] == "map"
    assert poses[0]["pose"]["orientation"]["z"] == pytest.approx(math.sin(-0.15))


def test_rosbridge_reads_a_parameter_through_get_parameters():
    backend = object.__new__(RosbridgeROS2Backend)
    sent: dict[str, Any] = {}

    def service(name, args, *, timeout, service_type=None):
        sent.update(name=name, args=args, type=service_type)
        if args["names"] == ["missing"]:
            return {"values": []}
        return {"values": [{"type": 3, "double_value": 0.5}]}

    backend._service = service
    value = backend.get_parameter(
        "/local_costmap/local_costmap", "inflation_layer.inflation_radius"
    )
    assert escape.parameter_number(value) == 0.5
    assert sent["name"] == "/local_costmap/local_costmap/get_parameters"
    assert sent["type"] == "rcl_interfaces/srv/GetParameters"
    assert backend.get_parameter("/local_costmap/local_costmap", "missing") is None


def _keys(mapping: Mapping[str, Any]) -> set[str]:
    return set(mapping)


def test_decision_evidence_is_json_shaped():
    import json

    data = decide(OBSERVED).to_dict()
    json.dumps(data, allow_nan=False)
    assert {"pinned", "decision", "pinned_threshold_m", "clearances_m", "costmap"} <= _keys(data)
