"""The robot's live readings reach an HMI as declared telemetry channels.

The execution host publishes a provider's samples on the channels its manifest
declares (flyto-cloud ``local/resource_state_telemetry.py``): it calls the
discoverer's ``telemetry_sampler`` and drops any channel the manifest does not
declare. These tests pin the channels, what each sample carries, where the
readings come from (battery, odometry, the map transform, Nav2 feedback the
adapter already receives), and that the presentation block of the module pack
rides on the manifest only when the host asks for it.

Nothing here talks to a robot: rosbridge sockets and adapters are fakes.
"""

from __future__ import annotations

import json
import math
import time
from types import SimpleNamespace

import pytest
from test_presence_on_state import RecordingSocket

import flyto_robotics.adapter_provider as provider
from flyto_robotics import live_telemetry as live
from flyto_robotics import motion_outcome
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RosbridgeROS2Backend

# The channel and field names an HMI declaration reads. flyto-modules-robotics
# ``presentation.py`` names the same table (its tests pin a copy); change both
# together or not at all.
CHANNEL_FIELDS = {
    "power.battery": {"percent", "voltage_v"},
    "pose.planar": {
        "x", "y", "heading_deg", "frame", "goal_x", "goal_y",
        "view_x_min", "view_x_max", "view_y_min", "view_y_max",
    },
    "nav.goal": {"state", "distance_m", "progress_pct", "goal_x", "goal_y", "reason"},
}
NAVIGATION_STATES = ("idle", "navigating", "arrived", "blocked", "cancelled", "failed")


# --- the channels -----------------------------------------------------------


def test_the_declared_channels_are_the_shared_table():
    declared = {item["channel_id"]: item for item in live.channel_declarations()}
    assert set(declared) == set(CHANNEL_FIELDS)
    for channel_id, fields in CHANNEL_FIELDS.items():
        item = declared[channel_id]
        assert set(item) == {"channel_id", "display_name", "payload_schema", "max_age_seconds"}
        assert set(item["payload_schema"]["properties"]) == fields
        assert 0.1 <= item["max_age_seconds"] <= 60
    states = declared["nav.goal"]["payload_schema"]["properties"]["state"]["enum"]
    assert tuple(states) == NAVIGATION_STATES == live.NAVIGATION_STATES


def test_declarations_are_fresh_copies():
    first = live.channel_declarations()
    first[0]["payload_schema"]["properties"].clear()
    assert live.channel_declarations()[0]["payload_schema"]["properties"]


# --- battery ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("percentage", "voltage", "expected"),
    [
        (0.64, 11.9, {"percent": 64.0, "voltage_v": 11.9}),
        (1.0, 12.4, {"percent": 100.0, "voltage_v": 12.4}),
        # sensor_msgs/BatteryState: NaN means "not measured", never zero.
        (float("nan"), 11.2, {"percent": None, "voltage_v": 11.2}),
        # Outside the message's 0..1 contract: not shown rather than guessed.
        (64.0, 11.2, {"percent": None, "voltage_v": 11.2}),
        (None, float("nan"), {"percent": None, "voltage_v": None}),
    ],
)
def test_battery_payload_follows_the_battery_state_message(percentage, voltage, expected):
    assert live.battery_payload(percentage, voltage) == expected


# --- pose -------------------------------------------------------------------


def test_the_pose_is_in_the_map_frame_when_localization_is_up():
    odom = {"frame": "odom", "x": 1.0, "y": 0.0, "yaw": 0.0}
    mapped = {"frame": "map", "x": 3.0, "y": -1.0, "yaw": math.pi / 2}
    record = live.NavigationRecord(call_id="c1", goal_x=4.0, goal_y=2.0, updated_at=1.0)
    payload = live.pose_payload(odom, mapped, record)
    assert payload["frame"] == "map"
    assert (payload["x"], payload["y"]) == (3.0, -1.0)
    assert payload["heading_deg"] == pytest.approx(90.0)
    # The goal is drawn only in the frame it was given in.
    assert (payload["goal_x"], payload["goal_y"]) == (4.0, 2.0)
    assert payload["view_x_min"] <= 3.0 <= payload["view_x_max"]
    assert payload["view_x_min"] <= 4.0 <= payload["view_x_max"]
    assert payload["view_y_min"] <= -1.0 and payload["view_y_max"] >= 2.0


def test_without_localization_the_pose_is_odometry_and_carries_no_goal():
    odom = {"frame": "odom", "x": 0.5, "y": 0.25, "yaw": -math.pi / 2}
    record = live.NavigationRecord(call_id="c1", goal_x=4.0, goal_y=2.0, updated_at=1.0)
    payload = live.pose_payload(odom, None, record)
    assert payload["frame"] == "odom"
    assert payload["heading_deg"] == pytest.approx(-90.0)
    assert "goal_x" not in payload and "goal_y" not in payload
    assert payload["view_x_max"] - payload["view_x_min"] >= 2 * live.VIEW_MIN_HALF_SPAN_M


def test_no_pose_without_odometry():
    assert live.pose_payload(None, None, None) is None


# --- navigation -------------------------------------------------------------


def test_navigation_progress_is_measured_from_nav2_feedback():
    record = live.NavigationRecord(call_id="c1", goal_x=2.0, goal_y=0.0, updated_at=0.0)
    assert record.payload()["state"] == "navigating"
    assert record.payload()["progress_pct"] is None
    # Nav2 reports 0 before it has a path; that is no reading of progress.
    record.saw_feedback({"distance_remaining": 0.0}, at=0.5)
    assert record.payload()["progress_pct"] is None
    record.saw_feedback({"distance_remaining": 2.0}, at=1.0)
    record.saw_feedback({"distance_remaining": 0.5}, at=2.0)
    payload = record.payload()
    assert payload["distance_m"] == 0.5
    assert payload["progress_pct"] == 75.0
    assert record.updated_at == 2.0


@pytest.mark.parametrize(
    ("reason", "reached", "state"),
    [
        (motion_outcome.REASON_COMPLETED, False, "arrived"),
        (motion_outcome.REASON_TIMEOUT, True, "arrived"),
        (motion_outcome.REASON_CANCELLED, False, "cancelled"),
        (motion_outcome.REASON_OBSTACLE_BLOCKED, False, "blocked"),
        (motion_outcome.REASON_PATH_BLOCKED, False, "blocked"),
        (motion_outcome.REASON_NO_PATH, False, "blocked"),
        (motion_outcome.REASON_NO_PROGRESS, False, "blocked"),
        (motion_outcome.REASON_SENSOR_STALE, False, "failed"),
        (motion_outcome.REASON_ABORTED_BY_SERVER, False, "failed"),
        ("something new", False, "failed"),
    ],
)
def test_an_ended_navigation_says_how_it_ended(reason, reached, state):
    record = live.NavigationRecord(call_id="c1", goal_x=1.0, goal_y=1.0, updated_at=0.0)
    record.saw_feedback({"distance_remaining": 1.0}, at=1.0)
    record.end(reason, at=3.0, reached=reached)
    payload = record.payload()
    assert payload["state"] == state
    assert payload["reason"] == reason
    assert record.updated_at == 3.0
    if state == "arrived":
        assert payload["progress_pct"] == 100.0
        assert payload["distance_m"] == 0.0


# --- samples ----------------------------------------------------------------


def _readings(**overrides):
    now = time.monotonic()
    base = {
        "battery": {"percent": 80.0, "voltage_v": 12.1},
        "battery_seen_at": now,
        "pose": {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
        "map_pose": None,
        "pose_seen_at": now,
        "navigation": None,
    }
    base.update(overrides)
    return base


def test_samples_cover_each_channel_for_the_resource():
    samples = live.samples("tb3-01", [_readings()])
    by_channel = {item["channel_id"]: item for item in samples}
    assert set(by_channel) == set(CHANNEL_FIELDS)
    for item in samples:
        assert item["resource_id"] == "tb3-01"
        assert item["quality"] == "good"
        assert item["observed_at"].endswith("+00:00")
        assert set(item["payload"]) <= CHANNEL_FIELDS[item["channel_id"]]
    assert by_channel["power.battery"]["payload"] == {"percent": 80.0, "voltage_v": 12.1}
    assert by_channel["nav.goal"]["payload"] == {"state": "idle"}


def test_no_connection_means_no_samples():
    assert live.samples("tb3-01", []) == []


def test_missing_readings_are_left_out_never_invented():
    samples = live.samples("tb3-01", [_readings(battery=None, battery_seen_at=None, pose=None)])
    assert [item["channel_id"] for item in samples] == ["nav.goal"]


def test_readings_merge_across_connections():
    """The presence watch hears battery and odometry; a job's adapter also has
    the map transform and the goal it is driving to."""
    record = live.NavigationRecord(
        call_id="c9", goal_x=2.0, goal_y=1.0, updated_at=time.monotonic()
    )
    watch = _readings(battery_seen_at=time.monotonic() - 3)
    job = _readings(
        battery=None,
        battery_seen_at=None,
        map_pose={"frame": "map", "x": 1.5, "y": 0.5, "yaw": 0.0},
        navigation=record,
    )
    by_channel = {item["channel_id"]: item for item in live.samples("r", [watch, job])}
    assert by_channel["power.battery"]["payload"]["percent"] == 80.0
    assert by_channel["pose.planar"]["payload"]["frame"] == "map"
    assert by_channel["pose.planar"]["payload"]["goal_x"] == 2.0
    assert by_channel["nav.goal"]["payload"]["state"] == "navigating"


# --- the rosbridge backend keeps the readings -------------------------------


BATTERY_MESSAGE = {"percentage": 0.5, "voltage": 11.7, "header": {"frame_id": ""}}


def _rosbridge(presence_only: bool):
    sockets: list[RecordingSocket] = []

    def connect(_url):
        sockets.append(RecordingSocket({}))
        return sockets[-1]

    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=connect, presence_only=presence_only
    )
    return backend, sockets[-1]


@pytest.mark.parametrize("presence_only", [True, False])
def test_both_connections_subscribe_to_the_battery_at_a_low_rate(presence_only):
    backend, socket = _rosbridge(presence_only)
    try:
        battery = [item for item in socket.subscribed if item["topic"] == "/battery_state"]
        assert len(battery) == 1
        assert battery[0]["type"] == "sensor_msgs/msg/BatteryState"
        assert battery[0]["throttle_rate"] >= 1000
    finally:
        backend.disconnect()


def test_the_backend_reports_battery_pose_and_navigation():
    backend, _socket = _rosbridge(False)
    try:
        backend._handle_publish("/battery_state", BATTERY_MESSAGE)
        backend._handle_publish(
            "/odom",
            {
                "pose": {"pose": {"position": {"x": 1.0, "y": 2.0}, "orientation": {"w": 1.0}}},
                "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
            },
        )
        backend._begin_track("c1", "motion.navigate", {"x": 3.0, "y": 4.0, "yaw_radians": 0.0})
        backend._track_feedback("c1", {"distance_remaining": 2.0})
        readings = backend.telemetry_readings()
        assert readings["battery"] == {"percent": 50.0, "voltage_v": 11.7}
        assert readings["pose"]["x"] == 1.0
        assert readings["navigation"].payload()["distance_m"] == 2.0
        assert readings["navigation"].payload()["goal_x"] == 3.0

        backend._motion_summary("c1", status=motion_outcome.STATUS_SUCCEEDED)
        assert backend.telemetry_readings()["navigation"].payload()["state"] == "arrived"
    finally:
        backend.disconnect()


def test_a_cancelled_navigation_is_reported_cancelled():
    backend, _socket = _rosbridge(False)
    try:
        backend._begin_track("c2", "motion.navigate", {"x": 1.0, "y": 0.0})
        backend._forget_track("c2")
        assert backend.telemetry_readings()["navigation"].payload()["state"] == "cancelled"
    finally:
        backend.disconnect()


def test_a_straight_drive_is_not_a_navigation():
    backend, _socket = _rosbridge(False)
    try:
        backend._begin_track("c3", "motion.advance", {"distance_m": 0.2})
        assert backend.telemetry_readings()["navigation"] is None
    finally:
        backend.disconnect()


def test_a_dropped_connection_forgets_the_battery():
    backend, _socket = _rosbridge(True)
    try:
        backend._handle_publish("/battery_state", BATTERY_MESSAGE)
        backend._reset_readings()
        assert backend.telemetry_readings()["battery"] is None
    finally:
        backend.disconnect()


# --- the provider: declared channels, sampler, presentation -----------------


class FakeAdapter:
    resource_id = "robot-1"

    def __init__(self, readings=None, connected=True):
        self._readings = readings
        self.connected = connected

    def describe(self):
        return ()

    def telemetry_readings(self):
        return self._readings

    def disconnect(self):
        pass


def test_the_manifest_declares_the_channels():
    manifest = provider._manifest(FakeAdapter(), resource_name="Robot")
    assert manifest["telemetry_channels"] == live.channel_declarations()


def test_the_sampler_reads_the_watch_and_the_served_adapters(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    watch = FakeAdapter(_readings())
    served = FakeAdapter(_readings(navigation=live.NavigationRecord(
        call_id="c1", goal_x=1.0, goal_y=1.0, updated_at=time.monotonic())))
    gone = FakeAdapter(_readings(), connected=False)
    monkeypatch.setattr(provider, "_watched_adapter", lambda _rid: watch)
    monkeypatch.setattr(provider, "_served_adapters", lambda: [served, gone])
    sampler = provider.discover_resource_manifests.telemetry_sampler
    samples = sampler(execution_host_id="host")
    by_channel = {item["channel_id"]: item for item in samples}
    assert by_channel["nav.goal"]["payload"]["state"] == "navigating"
    assert {item["resource_id"] for item in samples} == {"robot-1"}
    assert provider.discover_resource_manifests.telemetry_interval_seconds == 1.0


def test_the_sampler_is_silent_without_a_connection(monkeypatch):
    monkeypatch.setattr(provider, "_watched_adapter", lambda _rid: None)
    monkeypatch.setattr(provider, "_served_adapters", lambda: [])
    assert provider.telemetry_sampler(execution_host_id="host") == []


def test_a_failing_reading_never_raises_into_the_host(monkeypatch):
    class Broken(FakeAdapter):
        def telemetry_readings(self):
            raise RuntimeError("boom")

    monkeypatch.setattr(provider, "_watched_adapter", lambda _rid: Broken())
    monkeypatch.setattr(provider, "_served_adapters", lambda: [])
    assert provider.telemetry_sampler(execution_host_id="host") == []


def test_build_adapter_remembers_what_it_served(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    class Built:
        connected = True

    built = Built()
    monkeypatch.setattr(provider, "build", lambda _rid: built)
    assert provider.build_adapter("robot-1") is built
    assert built in provider._served_adapters()


BLOCK = {
    "schema": "flyto.resource-presentation.v1",
    "panels": [{"id": "p", "kind": "x", "label": {"en": "P"}}],
}


def test_the_presentation_rides_on_the_manifest_only_when_asked(monkeypatch):
    monkeypatch.setattr(provider, "_pack_presentation", lambda: json.loads(json.dumps(BLOCK)))
    assert "presentation" in provider.MANIFEST_EXTENSIONS
    plain = provider._manifest(FakeAdapter(), resource_name="Robot")
    assert "presentation" not in plain
    asked = provider._manifest(FakeAdapter(), resource_name="Robot", extensions=["presentation"])
    assert asked["presentation"] == BLOCK


def test_no_pack_presentation_means_no_field(monkeypatch):
    monkeypatch.setattr(provider, "_pack_presentation", lambda: None)
    asked = provider._manifest(FakeAdapter(), resource_name="Robot", extensions=["presentation"])
    assert "presentation" not in asked


def test_the_pack_presentation_is_found_by_the_pack_entry_point(monkeypatch):
    calls = []

    class Entry:
        name = provider.MODULE_PACK

        def load(self):
            calls.append("load")
            return lambda: dict(BLOCK)

    class Other:
        name = "automotive"

        def load(self):  # pragma: no cover - must never be loaded
            raise AssertionError("another pack's declaration")

    monkeypatch.setattr(
        provider, "_presentation_entry_points", lambda: [Other(), Entry()]
    )
    assert provider._pack_presentation() == BLOCK
    assert calls == ["load"]


def test_a_broken_pack_presentation_is_left_out(monkeypatch):
    class Entry:
        name = provider.MODULE_PACK

        def load(self):
            raise ImportError("not installed")

    monkeypatch.setattr(provider, "_presentation_entry_points", lambda: [Entry()])
    assert provider._pack_presentation() is None


def test_adapter_exposes_the_backend_readings():
    backend = SimpleNamespace(telemetry_readings=lambda: {"pose": None}, _connected=True)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    assert adapter.telemetry_readings() == {"pose": None}
    backend._connected = False
    assert adapter.telemetry_readings() is None
