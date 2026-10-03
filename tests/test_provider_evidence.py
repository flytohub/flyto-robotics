"""Evidence the adapter produces itself, in the shapes its hosts already read.

The items must equal what Flyto2 Desktop projected from the same observations
(``local/external_capability_dispatch.py`` until 2026-10-04), so a host that
passes them through judges exactly what it judged before. The Desktop side is
transcribed here; if it changes, these transcriptions change with it.
"""

from __future__ import annotations

import base64
import math
import struct
import zlib
from io import BytesIO

import pytest

from flyto_robotics import provider_evidence as pe
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
    CallResult,
)
from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    GenericROS2Adapter,
    StandardInterface,
)

SWEEP = {
    "angle_min_rad": -math.pi,
    "angle_increment_rad": math.pi / 2,
    "ranges_m": [1.0, 0.6, 0.4, 2.0],
}


def bundle(x=0.0, *, yaw=0.0, minimum=0.8, sweep=None, snapshot="s"):
    reading = {"minimum_range_m": minimum, "sample_count": 400}
    if sweep is not None:
        reading["sweep"] = sweep
    return {
        "pose": {"frame": "odom", "x": x, "y": 0.0, "yaw": yaw},
        "range": reading,
        "snapshot": snapshot,
    }


# -- Desktop's projection, transcribed ---------------------------------------


def desktop_clearance(before, floor):
    range_observation = before.get("range")
    try:
        measured = float(range_observation.get("minimum_range_m"))
    except (TypeError, ValueError):
        measured = None
    return {
        "kind": "passage.clearance",
        "usable": measured is not None and measured >= floor,
        "detail": f"minimum LiDAR clearance {measured} m before moving (floor {floor} m)",
        "reason": {
            "code": "lidar_minimum_range",
            "observed": {"minimum_range_m": measured},
            "required": {"minimum_range_m": floor},
            "source": "FLYTO_ROS2_MIN_CLEARANCE_M",
        },
    }


def desktop_arrival(capability_id, before, after, settled):
    def pose(value):
        return dict(value["pose"]) if value and isinstance(value.get("pose"), dict) else None

    def sweep(value):
        reading = value.get("range") if value else None
        found = reading.get("sweep") if isinstance(reading, dict) else None
        return dict(found) if isinstance(found, dict) else None

    return {
        "kind": "robot.arrival",
        "usable": False,
        "detail": "odometry before, after and once settled; judged by Cloud",
        "reason": {
            "code": "odom_displacement",
            "observed": {
                "capability_id": capability_id,
                "before": pose(before),
                "after": pose(after),
                "post_stop": pose(settled),
                "sweeps": {
                    phase: found
                    for phase, found in (("before", sweep(before)), ("post_stop", sweep(settled)))
                    if found is not None
                },
                "snapshots": [
                    item.get("snapshot", "")
                    for item in (before, after, settled)
                    if isinstance(item, dict)
                ],
            },
            "required": {},
            "source": "ros2.observation-bundle.v1",
        },
    }


def desktop_map_jpeg(capture):
    from PIL import Image

    width, height = int(capture["width"]), int(capture["height"])
    cells = base64.b64decode(capture["cells_base64"])
    shades = bytes(205 if value > 100 else 255 - round(value * 2.55) for value in cells)
    image = Image.frombytes("L", (width, height), shades).transpose(Image.FLIP_TOP_BOTTOM)
    scale = max(1, -(-600 // width))
    image = image.resize((width * scale, height * scale), Image.NEAREST)
    buffer = BytesIO()
    image.save(buffer, format="JPEG", quality=90)
    return buffer.getvalue(), image.width, image.height


# -- items --------------------------------------------------------------------


@pytest.mark.parametrize("minimum", [0.8, 0.35, 0.2])
def test_clearance_equals_the_desktop_projection(minimum):
    before = bundle(minimum=minimum)
    assert pe.clearance_item(before, 0.35) == desktop_clearance(before, 0.35)
    assert pe.clearance_item(before, 0.35)["usable"] is (minimum >= 0.35)


def test_no_range_means_no_clearance_claim():
    assert pe.clearance_item({"pose": {"x": 0}}, 0.35) is None
    assert pe.clearance_item(None, 0.35) is None


@pytest.mark.parametrize("capability_id", sorted(pe.JUDGED_MOTIONS))
def test_judged_arrival_equals_the_desktop_projection(capability_id):
    before = bundle(0.0, sweep=SWEEP, snapshot="a")
    after = bundle(0.11, snapshot="b")
    settled = bundle(0.112, sweep=SWEEP, snapshot="c")
    produced = pe.arrival_item(capability_id, before=before, after=after, settled=settled)
    assert produced == desktop_arrival(capability_id, before, after, settled)
    assert produced["usable"] is False


def test_navigation_arrival_reports_the_pose_it_ended_at():
    item = pe.arrival_item("motion.navigate", before=None, after=bundle(1.5), settled=None)
    assert item == {
        "kind": "robot.arrival",
        "usable": True,
        "detail": "observed pose odom: x=1.5, y=0.0, yaw=0.0",
    }
    assert pe.arrival_item("vision.observe", before=None, after=bundle(), settled=None) is None


def test_recovery_context_is_measured_along_the_starting_heading():
    summary = {
        "reason": "obstacle_blocked",
        "capability_id": "motion.advance",
        "start_pose": {"x": 1.0, "y": 1.0, "yaw": math.pi / 2},
        "final_pose": {"x": 1.05, "y": 1.12, "yaw": math.pi / 2},
        "requested_distance_m": 0.3,
        "minimum_range_at_stop_m": 0.33,
        "travel_direction_range_m": 0.33,
        "clearance_floor_m": 0.35,
    }
    context = pe.recovery_context(summary, sweep=SWEEP)
    assert context["reason"] == "obstacle_blocked"
    # A sideways slide is not progress: only the 0.12 m along the heading.
    assert context["travelled_m"] == pytest.approx(0.12)
    assert context["remaining_m"] == pytest.approx(0.18)
    assert context["sweep"] == SWEEP
    assert pe.recovery_context({**summary, "capability_id": "motion.rotate"}) is None
    assert pe.recovery_context(None) is None


def test_a_retreat_travels_backwards():
    summary = {
        "reason": "obstacle_blocked",
        "capability_id": "motion.retreat",
        "start_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "final_pose": {"x": -0.1, "y": 0.0, "yaw": 0.0},
        "requested_distance_m": 0.25,
    }
    context = pe.recovery_context(summary)
    assert context["travelled_m"] == pytest.approx(0.1)
    assert context["remaining_m"] == pytest.approx(0.15)
    assert "sweep" not in context


# -- artifacts ----------------------------------------------------------------


def map_capture(width=3, height=2, cells=(0, 100, 255, 50, 0, 101)):
    return {
        "kind": "map",
        "width": width,
        "height": height,
        "resolution_m": 0.05,
        "origin": {"x": 0.0, "y": 0.0},
        "cells_base64": base64.b64encode(bytes(cells)).decode("ascii"),
    }


def test_the_map_jpeg_is_byte_for_byte_the_desktop_drawing():
    pytest.importorskip("PIL")
    capture = map_capture()
    data, media_type, width, height = pe.render_map(capture)
    expected, expected_width, expected_height = desktop_map_jpeg(capture)
    assert media_type == "image/jpeg"
    assert (width, height) == (expected_width, expected_height) == (600, 400)
    assert data == expected


def _png_pixels(data):
    assert data.startswith(b"\x89PNG\r\n\x1a\n")
    position, idat, header = 8, b"", None
    while position < len(data):
        (length,) = struct.unpack(">I", data[position : position + 4])
        kind = data[position + 4 : position + 8]
        body = data[position + 8 : position + 8 + length]
        (crc,) = struct.unpack(">I", data[position + 8 + length : position + 12 + length])
        assert crc == zlib.crc32(kind + body)
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            idat += body
        position += 12 + length
    width, height = header[0], header[1]
    raw = zlib.decompress(idat)
    rows = [raw[row * (width + 1) : (row + 1) * (width + 1)] for row in range(height)]
    assert all(row[0] == 0 for row in rows)
    return width, height, [row[1:] for row in rows]


def test_without_pillow_the_map_is_a_png_of_the_same_drawing(monkeypatch):
    monkeypatch.setattr(pe, "_jpeg", lambda *_: None)
    data, media_type, width, height = pe.render_map(map_capture())
    assert media_type == "image/png"
    png_width, png_height, rows = _png_pixels(data)
    assert (png_width, png_height) == (width, height) == (600, 400)
    # The grid's first row is the bottom: drawn north up. 0 free white,
    # 100 occupied black, 255 / >100 unknown grey, 50 half.
    assert rows[0][0] == 255 - round(50 * 2.55)
    assert rows[0][200] == 255
    assert rows[0][400] == 205
    assert rows[-1][0] == 255
    assert rows[-1][200] == 0
    assert rows[-1][599] == 205


def test_a_map_whose_cells_do_not_match_its_size_is_refused():
    with pytest.raises(ValueError):
        pe.render_map(map_capture(cells=(0, 1)))


def test_capture_artifacts_follow_the_contract_transport(monkeypatch):
    jpeg = base64.b64encode(b"\xff\xd8 jpeg").decode("ascii")
    photo = {"kind": "photo", "media_type": "image/jpeg", "data_base64": jpeg}
    assert pe.capture_artifacts(photo) == [
        {"kind": "photo", "media_type": "image/jpeg", "data_base64": jpeg}
    ]
    monkeypatch.setattr(pe, "_jpeg", lambda *_: None)
    [drawn] = pe.capture_artifacts(map_capture())
    assert drawn["kind"] == "map" and drawn["media_type"] == "image/png"
    assert base64.b64decode(drawn["data_base64"]).startswith(b"\x89PNG")
    with pytest.raises(ValueError):
        pe.capture_artifacts({"kind": "sound"})


# -- the adapter --------------------------------------------------------------


class Backend:
    """A ROS graph that moves, reports why a motion ended, and can settle."""

    def __init__(self, *, outcome=OUTCOME_COMPLETED, settles=True, capture=None):
        self.interfaces = [
            StandardInterface(*DEFAULT_INTERFACES[capability_id])
            for capability_id in DEFAULT_INTERFACES
            if capability_id != "motion.halt"
        ]
        self.outcome = outcome
        self.x = 0.0
        self.invoked = []
        self.observed = 0
        self.captured = capture
        if settles:
            self.wait_until_stationary = lambda seconds: {"stationary": True, "drifting": False}

    def discover(self):
        return tuple(self.interfaces)

    def observation(self, required=None):
        self.observed += 1
        return {
            "pose": {"frame": "odom", "x": self.x, "y": 0.0, "yaw": 0.0},
            "range": {"minimum_range_m": 0.8, "sample_count": 4, "sweep": SWEEP},
            "camera": None,
            "map_tf_available": True,
        }

    def held_observation(self):
        return {"range": {"minimum_range_m": 0.33, "sample_count": 4, "sweep": SWEEP}}

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        self.invoked.append(call_id)
        self.x = 0.1
        summary = {
            "reason": "completed" if self.outcome == OUTCOME_COMPLETED else "obstacle_blocked",
            "capability_id": capability_id,
            "start_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "final_pose": {"x": 0.1, "y": 0.0, "yaw": 0.0},
            "requested_distance_m": arguments.get("distance_m"),
            "minimum_range_at_stop_m": 0.33,
            "clearance_floor_m": 0.35,
        }
        return CallResult(
            call_id,
            self.outcome,
            evidence={"odom": {"x": 0.1}, "motion_outcome": summary},
            detail="" if self.outcome == OUTCOME_COMPLETED else "obstacle_blocked",
        )

    def capture(self, *, call_id, capability_id, deadline_seconds):
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={"capture": self.captured})

    def cancel(self, call_id):
        return CallResult(call_id, OUTCOME_REFUSED)

    def safe_stop(self, call_id):
        return CallResult(call_id, OUTCOME_COMPLETED)

    def execution_count(self, call_id):
        return self.invoked.count(call_id)


def advance(call_id="call-1", distance=0.3):
    return CallRequest(call_id, "motion.advance", {"distance_m": distance}, 30.0)


@pytest.fixture(autouse=True)
def _hardware(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    monkeypatch.delenv("FLYTO_ROS2_SAFETY_BASIS", raising=False)
    monkeypatch.delenv("FLYTO_ROS2_MIN_CLEARANCE_M", raising=False)


def test_a_completed_motion_carries_its_own_clearance_and_arrival():
    backend = Backend()
    device = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    result = device.invoke(advance())

    assert result.outcome == OUTCOME_COMPLETED
    # Additive: everything the backend reported is still there.
    assert result.evidence["odom"] == {"x": 0.1}
    assert result.evidence["motion_outcome"]["reason"] == "completed"
    clearance, arrival = result.evidence["evidence_items"]
    assert clearance["kind"] == "passage.clearance" and clearance["usable"] is True
    assert clearance["reason"]["required"] == {"minimum_range_m": 0.35}
    observed = arrival["reason"]["observed"]
    assert arrival["kind"] == "robot.arrival" and arrival["usable"] is False
    assert observed["before"]["x"] == 0.0
    assert observed["after"]["x"] == observed["post_stop"]["x"] == 0.1
    assert set(observed["sweeps"]) == {"before", "post_stop"}
    assert "recovery_context" not in result.evidence


def test_a_robot_that_cannot_say_it_settled_leaves_the_arrival_to_the_host():
    device = GenericROS2Adapter(backend=Backend(settles=False), resource_id="robot-1")
    result = device.invoke(advance())
    assert [item["kind"] for item in result.evidence["evidence_items"]] == ["passage.clearance"]


def test_a_blocked_motion_reports_what_a_detour_needs_without_waiting():
    backend = Backend(outcome=OUTCOME_FAILED)
    device = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    observed_before = backend.observed
    result = device.invoke(advance())

    assert result.outcome == OUTCOME_FAILED
    assert result.detail == "obstacle_blocked"
    context = result.evidence["recovery_context"]
    assert context["reason"] == "obstacle_blocked"
    assert context["travelled_m"] == pytest.approx(0.1)
    assert context["remaining_m"] == pytest.approx(0.2)
    assert context["sweep"] == SWEEP
    # Only the clearance: the arrival is the host's, observed after its stop.
    assert [item["kind"] for item in result.evidence["evidence_items"]] == ["passage.clearance"]
    # Preflight and "before" only; nothing waited on after the failure.
    assert backend.observed - observed_before == 2


def test_the_same_call_returns_its_result_not_a_second_motion():
    backend = Backend()
    device = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    first = device.invoke(advance())
    again = device.invoke(advance())
    assert again is first
    assert backend.invoked == ["call-1"]


def test_a_timed_out_call_keeps_the_observation_from_before_it_moved():
    backend = Backend(outcome=OUTCOME_TIMEOUT)
    device = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    device.invoke(advance())
    backend.outcome = OUTCOME_COMPLETED
    resumed = device.invoke(advance())
    arrival = resumed.evidence["evidence_items"][1]
    assert arrival["reason"]["observed"]["before"]["x"] == 0.0


def test_a_refused_motion_is_returned_untouched():
    device = GenericROS2Adapter(backend=Backend(), resource_id="robot-1")
    result = device.invoke(advance(distance=5.0))
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence == {}


def test_a_photo_capture_also_returns_the_contract_artifact():
    jpeg = base64.b64encode(b"\xff\xd8 frame").decode("ascii")
    photo = {"kind": "photo", "media_type": "image/jpeg", "data_base64": jpeg}
    device = GenericROS2Adapter(backend=Backend(capture=photo), resource_id="robot-1")
    result = device.invoke(CallRequest("p", "vision.observe", {}, 10.0))
    assert result.evidence["capture"] == photo
    assert result.evidence["artifacts"] == [
        {"kind": "photo", "media_type": "image/jpeg", "data_base64": jpeg}
    ]


def test_a_map_capture_keeps_its_cells_and_adds_the_drawing():
    capture = map_capture()
    device = GenericROS2Adapter(backend=Backend(capture=capture), resource_id="robot-1")
    result = device.invoke(CallRequest("m", "sensing.map", {}, 10.0))
    assert result.evidence["capture"] == capture
    [artifact] = result.evidence["artifacts"]
    assert artifact["kind"] == "map"
    assert artifact["media_type"] in {"image/jpeg", "image/png"}


def test_a_map_that_cannot_be_drawn_returns_no_artifact():
    broken = map_capture(cells=(1, 2))
    device = GenericROS2Adapter(backend=Backend(capture=broken), resource_id="robot-1")
    result = device.invoke(CallRequest("m", "sensing.map", {}, 10.0))
    assert result.outcome == OUTCOME_COMPLETED
    assert "artifacts" not in result.evidence
