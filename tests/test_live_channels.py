"""The robot can be watched live: scan, pose and camera as ``flyto.live-channels.v1``.

flyto-cloud's execution host relays a resource's declared live channels
(``local/live_routes.py``, ``local/live_relay.py``): it reads each channel's
host-side ``source`` through a source-adapter table keyed by scheme
(``local/live_sources.py``), subscribes to one rosbridge topic with a throttle
(``local/live_source_rosbridge.py``) and normalises each message by its type.
Cloud validates the block against its schema
(``services/device/schemas/flyto.live-channels.v1.schema.json``) and serves
``GET /api/spaces/{id}/resources/{rid}/live``. A manifest without the block has
no Live tab.

These tests pin, by value and without importing Cloud, what the robotics
provider declares: which channels, on which topics the robot and its twin
publish, the source form Cloud's rosbridge adapter parses, the rates and kinds
Cloud's schema admits, that a channel is declared only when the graph
publishes its topic with its type, and that nothing declared is a command
path. Nothing here talks to a robot.
"""

from __future__ import annotations

import json
import math
import queue
import re
from urllib.parse import parse_qs, urlsplit

import pytest

import flyto_robotics.adapter_provider as provider
from flyto_robotics import generic_ros2_adapter as ros2
from flyto_robotics import live_channels
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RosbridgeROS2Backend

BRIDGE = "ws://127.0.0.1:19090"

# What rosapi lists on burger-01 and on the twin (the twin presents the
# robot's own interface: same topics, plus /clock). Only the rows these
# channels read, with the types rosapi reports.
ROBOT_TOPICS = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/camera/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
    "/camera/image_raw": "sensor_msgs/msg/Image",
    "/battery_state": "sensor_msgs/msg/BatteryState",
    "/cmd_vel": "geometry_msgs/msg/TwistStamped",
}
TWIN_TOPICS = {**ROBOT_TOPICS, "/clock": "rosgraph_msgs/msg/Clock"}

LABEL_PATTERN = re.compile(r"^[a-z]{2,3}(-[A-Za-z0-9]{2,8})?$")
ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")

# flyto.live-channels.v1, mirrored by value from flyto-cloud's schema.
CLOUD_CHANNEL_FIELDS = {
    "id", "kind", "label_key", "label", "unit", "reading", "transport",
    "max_rate_hz", "source", "floor_m", "fields",
}
CLOUD_TRANSPORTS_BY_KIND = {
    "video": {"whep", "hls", "ws-jpeg"},
    "scan": {"ws-json"},
    "pose": {"ws-json"},
    "telemetry": {"ws-json"},
}
CLOUD_RATE_CAP_HZ = {"ws-json": 20.0, "ws-jpeg": 10.0}
CLOUD_MAX_CANONICAL_BYTES = 8192
CLOUD_MAX_SOURCE = 512
# flyto-cloud local/live_sources.py SOURCE_ADAPTERS: scheme -> wire scheme.
CLOUD_SOURCE_SCHEMES = {"rosbridge": "ws", "rosbridges": "wss"}
# flyto-cloud local/live_source_rosbridge.py MESSAGE_NORMALISERS: the type a
# kind is read as (the bridge spells it with or without ``/msg/``).
CLOUD_NORMALISED_TYPES = {
    "scan": {"sensor_msgs/LaserScan"},
    "pose": {
        "nav_msgs/Odometry",
        "geometry_msgs/PoseStamped",
        "geometry_msgs/PoseWithCovarianceStamped",
    },
    "video": {"sensor_msgs/CompressedImage"},
}

EXPECTED = [
    {
        "id": "scan",
        "kind": "scan",
        "transport": "ws-json",
        "max_rate_hz": 5,
        "label": {"zh-TW": "光達掃描", "en": "LiDAR scan"},
        "floor_m": 0.35,
        "source": "rosbridge://127.0.0.1:19090/scan?type=sensor_msgs/msg/LaserScan",
    },
    {
        "id": "pose",
        "kind": "pose",
        "transport": "ws-json",
        "max_rate_hz": 5,
        "label": {"zh-TW": "里程計位置", "en": "Odometry pose"},
        "source": "rosbridge://127.0.0.1:19090/odom?type=nav_msgs/msg/Odometry",
    },
    {
        "id": "camera",
        "kind": "video",
        "transport": "ws-jpeg",
        "max_rate_hz": 5,
        "label": {"zh-TW": "攝影機", "en": "Camera"},
        "source": (
            "rosbridge://127.0.0.1:19090/camera/image_raw/compressed"
            "?type=sensor_msgs/msg/CompressedImage"
        ),
    },
]


@pytest.fixture(autouse=True)
def default_topics(monkeypatch):
    for name in (
        "FLYTO_ROS2_SCAN_TOPIC",
        "FLYTO_ROS2_ODOM_TOPIC",
        "FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC",
        "FLYTO_ROS2_MIN_CLEARANCE_M",
    ):
        monkeypatch.delenv(name, raising=False)


# --- what is declared ---------------------------------------------------------


def test_the_robot_declares_scan_pose_and_camera_on_its_own_topics():
    assert live_channels.declarations(BRIDGE, ROBOT_TOPICS) == EXPECTED


def test_the_twin_declares_exactly_what_the_robot_declares():
    assert live_channels.declarations(BRIDGE, TWIN_TOPICS) == EXPECTED


def test_declarations_do_not_depend_on_the_deployment_mode(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "simulation")
    simulated = live_channels.declarations(BRIDGE, TWIN_TOPICS)
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    assert live_channels.declarations(BRIDGE, ROBOT_TOPICS) == simulated


def test_a_tls_bridge_is_read_as_rosbridges():
    block = live_channels.declarations("wss://bridge.local:9443", ROBOT_TOPICS)
    assert {urlsplit(item["source"]).scheme for item in block} == {"rosbridges"}
    assert {urlsplit(item["source"]).netloc for item in block} == {"bridge.local:9443"}


def test_the_topics_follow_the_adapters_own_settings(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SCAN_TOPIC", "/scan_filtered")
    monkeypatch.setenv("FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC", "/front/compressed")
    topics = {
        **ROBOT_TOPICS,
        "/scan_filtered": "sensor_msgs/msg/LaserScan",
        "/front/compressed": "sensor_msgs/msg/CompressedImage",
    }
    sources = {item["id"]: item["source"] for item in live_channels.declarations(BRIDGE, topics)}
    assert urlsplit(sources["scan"]).path == "/scan_filtered"
    assert urlsplit(sources["camera"]).path == "/front/compressed"


def test_the_scan_shows_the_floor_the_adapter_enforces(monkeypatch):
    (scan,) = [item for item in live_channels.declarations(BRIDGE, ROBOT_TOPICS)
               if item["id"] == "scan"]
    assert scan["floor_m"] == 0.35 == ros2._clearance_floor()
    monkeypatch.setenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.5")
    (scan,) = [item for item in live_channels.declarations(BRIDGE, ROBOT_TOPICS)
               if item["id"] == "scan"]
    assert scan["floor_m"] == 0.5


def test_the_scan_never_advertises_a_floor_below_the_physical_minimum(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.1")
    (scan,) = [item for item in live_channels.declarations(BRIDGE, ROBOT_TOPICS)
               if item["id"] == "scan"]
    assert scan["floor_m"] == 0.35 == ros2._clearance_floor()


# --- only what the graph serves -------------------------------------------------


def test_a_topic_the_graph_does_not_publish_is_not_declared():
    without_camera = {
        name: kind for name, kind in ROBOT_TOPICS.items() if not name.startswith("/camera")
    }
    ids = [item["id"] for item in live_channels.declarations(BRIDGE, without_camera)]
    assert ids == ["scan", "pose"]


def test_a_topic_of_another_type_is_not_declared():
    wrong = {**ROBOT_TOPICS, "/camera/image_raw/compressed": "sensor_msgs/msg/Image"}
    ids = [item["id"] for item in live_channels.declarations(BRIDGE, wrong)]
    assert "camera" not in ids


def test_a_type_spelled_without_msg_still_matches():
    old = {name: kind.replace("/msg/", "/") for name, kind in ROBOT_TOPICS.items()}
    assert [item["id"] for item in live_channels.declarations(BRIDGE, old)] == [
        "scan", "pose", "camera",
    ]


@pytest.mark.parametrize(
    "bridge",
    [
        None,  # rclpy: no bridge for the host relay to read
        "",
        "http://127.0.0.1:19090",
        "ws://127.0.0.1:19090/bridge",  # Cloud reads host:port only
        "ws://127.0.0.1:19090?x=1",
        "ws:///no-host",
    ],
)
def test_no_bridge_the_relay_can_read_means_no_channels(bridge):
    assert live_channels.declarations(bridge, ROBOT_TOPICS) == []


def test_an_unread_graph_means_no_channels():
    assert live_channels.declarations(BRIDGE, {}) == []


# --- what Cloud accepts, by value ----------------------------------------------


@pytest.mark.parametrize("bridge", [BRIDGE, "wss://bridge.local:9443"])
def test_every_channel_is_one_cloud_admits(bridge):
    block = live_channels.declarations(bridge, ROBOT_TOPICS)
    assert 1 <= len(block) <= 16
    assert len({item["id"] for item in block}) == len(block)
    canonical = json.dumps(block, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                           allow_nan=False).encode("utf-8")
    assert len(canonical) <= CLOUD_MAX_CANONICAL_BYTES
    for item in block:
        assert set(item) <= CLOUD_CHANNEL_FIELDS
        assert ID_PATTERN.match(item["id"])
        assert item["transport"] in CLOUD_TRANSPORTS_BY_KIND[item["kind"]]
        rate = item["max_rate_hz"]
        assert isinstance(rate, (int, float)) and math.isfinite(rate)
        assert 0 < rate <= CLOUD_RATE_CAP_HZ[item["transport"]]
        assert 1 <= len(item["source"]) <= CLOUD_MAX_SOURCE
        assert ("floor_m" in item) == (item["kind"] == "scan")
        assert "fields" not in item
        for language, text in item["label"].items():
            assert LABEL_PATTERN.match(language) and 1 <= len(text) <= 64


@pytest.mark.parametrize("bridge", [BRIDGE, "wss://bridge.local:9443"])
def test_every_source_parses_as_cloud_parses_it(bridge):
    wire = urlsplit(bridge)
    for item in live_channels.declarations(bridge, ROBOT_TOPICS):
        parts = urlsplit(item["source"])
        assert CLOUD_SOURCE_SCHEMES[parts.scheme] == wire.scheme
        assert parts.netloc == wire.netloc
        assert parts.path.strip("/")
        assert ROBOT_TOPICS[parts.path]
        (named,) = parse_qs(parts.query)["type"]
        assert named.replace("/msg/", "/") in CLOUD_NORMALISED_TYPES[item["kind"]]
        assert named == ROBOT_TOPICS[parts.path]


def test_the_camera_rate_is_bounded():
    (camera,) = [item for item in live_channels.declarations(BRIDGE, ROBOT_TOPICS)
                 if item["transport"] == "ws-jpeg"]
    assert camera["kind"] == "video"
    assert camera["max_rate_hz"] <= 5


# --- read only ----------------------------------------------------------------


def _command_paths() -> set[str]:
    paths = {name for kind, name, _type in ros2.DEFAULT_INTERFACES.values() if kind == "action"}
    paths |= {name for kind, name, _type in ros2.ESCAPE_INTERFACES.values()}
    paths.add(ros2.DEFAULT_INTERFACES["motion.halt"][1])
    paths |= set(ros2._command_watch_topics())
    return paths


def test_no_channel_reads_a_command_path():
    commands = _command_paths()
    assert "/cmd_vel" in commands
    for item in live_channels.declarations(BRIDGE, ROBOT_TOPICS):
        assert urlsplit(item["source"]).path not in commands
    assert {row.kind for row in live_channels.LIVE_CHANNELS} <= {"scan", "pose", "video"}


# --- the provider -------------------------------------------------------------


class FakeAdapter:
    resource_id = "robot-1"
    connected = True

    def __init__(self, bridge=BRIDGE, topics=None):
        self._bridge = bridge
        self._topics = dict(ROBOT_TOPICS if topics is None else topics)

    def describe(self):
        return ()

    def transport_status(self):
        return {"transport": "direct", "state": "connected"}

    def bridge_url(self):
        return self._bridge

    def published_topics(self):
        return dict(self._topics)

    def disconnect(self):
        pass


def test_live_channels_is_an_extension_the_host_asks_for():
    assert "live_channels" in provider.MANIFEST_EXTENSIONS
    assert "live_channels" in provider.discover_resource_manifests.manifest_extensions


def test_a_host_that_does_not_ask_gets_no_live_channels():
    assert "live_channels" not in provider._manifest(FakeAdapter(), resource_name="Robot")


def test_the_manifest_carries_the_channels_when_asked():
    manifest = provider._manifest(
        FakeAdapter(), resource_name="Robot", extensions=["live_channels"]
    )
    assert manifest["live_channels"] == EXPECTED


def test_nothing_to_watch_leaves_the_field_out_rather_than_empty():
    # Cloud's schema requires at least one channel: an empty list would make
    # the host refuse the whole manifest and drop the robot.
    for adapter in (FakeAdapter(bridge=None), FakeAdapter(topics={})):
        manifest = provider._manifest(adapter, resource_name="Robot", extensions=["live_channels"])
        assert "live_channels" not in manifest


# --- the adapter exposes the bridge and the graph it read ------------------------


class GraphSocket:
    """Answers rosapi with the robot's topics."""

    def __init__(self, topics):
        self.topics = topics
        self.inbox: queue.Queue = queue.Queue()

    def send(self, payload):
        message = json.loads(payload)
        if message.get("op") != "call_service":
            return
        values = (
            {"topics": list(self.topics), "types": list(self.topics.values())}
            if message["service"] == "/rosapi/topics"
            else {"action_servers": ["/drive_on_heading"]}
        )
        self.inbox.put(json.dumps(
            {"op": "service_response", "id": message["id"], "result": True, "values": values}
        ))

    def pong(self, _data=b""):
        pass

    def recv(self, timeout=None):
        try:
            item = self.inbox.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError from None
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self):
        self.inbox.put(ConnectionError("closed"))


def test_the_rosbridge_adapter_reports_its_bridge_and_the_topics_it_read():
    backend = RosbridgeROS2Backend(
        url=BRIDGE, connection_factory=lambda _url: GraphSocket(ROBOT_TOPICS), presence_only=True
    )
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    try:
        adapter._invalidate_discovery()
        adapter.describe()
        assert adapter.bridge_url() == BRIDGE
        assert adapter.published_topics() == ROBOT_TOPICS
        declared = live_channels.declarations(adapter.bridge_url(), adapter.published_topics())
        assert declared == EXPECTED
    finally:
        adapter.disconnect()
    assert adapter.published_topics() == {}


def test_an_adapter_without_a_bridge_reports_none():
    class Rclpy:
        def discover(self):
            return ()

    adapter = GenericROS2Adapter(backend=Rclpy(), resource_id="robot-1")
    assert adapter.bridge_url() is None
    assert adapter.published_topics() == {}
