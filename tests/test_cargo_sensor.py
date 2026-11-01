"""The robot's own cargo sensor, ``sensing.cargo_present``, over rosbridge.

A delivery is finished only when the moving robot itself still reports the
load (Flyto2 Cloud ``services/space_tasks/cargo_vocabulary.py`` reads the
literal boolean ``cargo_present``). The reading is a ``std_msgs/msg/Bool``
topic named by ``FLYTO_ROS2_CARGO_TOPIC``, read over the same rosbridge
connection as motion and behind the same simulation/hardware graph check.
What is pinned here, with a fake rosbridge:

* A hardware robot declares the capability only on a topic configured for
  it (``FLYTO_ROS2_CARGO_TOPIC``). The shared interface's
  ``/flyto/cargo_present`` is the default only for a served simulation whose
  graph carries the simulator marker (``/clock``), so the simulated tray can
  never answer for a real robot. A topic not on the graph as a Bool is not
  declared, so planning refuses. Evidence always names the deployment mode.
* A fresh ``true`` or ``false`` is answered as exactly that boolean.
* No message, a message older than the freshness window, or a message whose
  ``data`` is not a boolean is a failed call with a typed reason and no
  value: a missing reading is never false and never true. A malformed message
  and a reconnect each discard the last reading.
* A graph that is not the configured deployment refuses the read.
"""

from __future__ import annotations

import json
import queue
import time

import pytest

from flyto_robotics import cargo_sensor
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
)
from flyto_robotics.generic_ros2_adapter import (
    CLOCK_TYPE,
    GenericROS2Adapter,
    RosbridgeROS2Backend,
)

TOPIC = "/flyto/cargo_present"
ODOM = {"/odom": "nav_msgs/msg/Odometry"}
SIM_GRAPH = {**ODOM, "/clock": CLOCK_TYPE, TOPIC: "std_msgs/msg/Bool"}


class FakeBridge:
    """A rosbridge that answers rosapi with a fixed graph and records subscriptions."""

    def __init__(self, topics):
        self.topics = dict(topics)
        self.inbox: queue.Queue = queue.Queue()
        self.subscribed: list[dict] = []

    def send(self, payload):
        message = json.loads(payload)
        if message.get("op") == "subscribe":
            self.subscribed.append(message)
            return
        if message.get("op") != "call_service":
            return
        values = (
            {"topics": list(self.topics), "types": list(self.topics.values())}
            if message["service"] == "/rosapi/topics"
            else {"action_servers": []}
        )
        self.inbox.put(json.dumps(
            {"op": "service_response", "id": message["id"], "result": True, "values": values}
        ))

    def publish(self, topic, msg):
        self.inbox.put(json.dumps({"op": "publish", "topic": topic, "msg": msg}))

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


@pytest.fixture
def twin(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "turtlebot3-twin")
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "simulation")
    monkeypatch.delenv("FLYTO_ROS2_SIM_MARKER_TOPIC", raising=False)
    monkeypatch.setenv(cargo_sensor.TOPIC_ENV, TOPIC)
    # Short, so the failures below do not wait long.
    monkeypatch.setenv(cargo_sensor.MAX_AGE_ENV, "0.3")


@pytest.fixture
def connect():
    made: list[tuple[GenericROS2Adapter, FakeBridge]] = []

    def build(topics=SIM_GRAPH):
        bridge = FakeBridge(topics)
        backend = RosbridgeROS2Backend(
            url="ws://127.0.0.1:1", connection_factory=lambda _url: bridge
        )
        adapter = GenericROS2Adapter(backend=backend, resource_id="turtlebot3-twin")
        made.append((adapter, bridge))
        return adapter, bridge

    yield build
    for adapter, _ in made:
        adapter.disconnect()


def read(adapter, call_id="cargo-1"):
    return adapter.invoke(CallRequest(call_id, cargo_sensor.CAPABILITY_ID, arguments={}))


def wait_for_reading(adapter):
    deadline = time.monotonic() + 2.0
    while adapter.backend._cargo.latest is None and time.monotonic() < deadline:
        time.sleep(0.01)


def declared(adapter):
    return {item.capability_id: item for item in adapter.describe()}


# --- declared only when configured and on the graph ---------------------------


def test_the_shared_identifiers():
    assert cargo_sensor.CAPABILITY_ID == "sensing.cargo_present"
    assert cargo_sensor.MESSAGE_TYPE == "std_msgs/msg/Bool"
    assert cargo_sensor.TOPIC_ENV == "FLYTO_ROS2_CARGO_TOPIC"
    assert cargo_sensor.CONVENTIONAL_TOPIC == TOPIC
    assert cargo_sensor.RESULT_FIELD == "cargo_present"


def test_only_an_explicit_topic_is_configured():
    import os

    saved = os.environ.pop(cargo_sensor.TOPIC_ENV, None)
    try:
        assert cargo_sensor.configured_topic() is None
        os.environ[cargo_sensor.TOPIC_ENV] = "   "
        assert cargo_sensor.configured_topic() is None
        os.environ[cargo_sensor.TOPIC_ENV] = "/tray/loaded"
        assert cargo_sensor.configured_topic() == "/tray/loaded"
    finally:
        os.environ.pop(cargo_sensor.TOPIC_ENV, None)
        if saved is not None:
            os.environ[cargo_sensor.TOPIC_ENV] = saved


@pytest.mark.parametrize(
    ("mode", "marker", "expected"),
    [
        ("simulation", True, TOPIC),
        ("simulation", False, None),
        ("real", True, None),
        ("real", False, None),
    ],
)
def test_the_shared_topic_is_the_default_only_for_a_marked_simulation(
    monkeypatch, mode, marker, expected
):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV, raising=False)
    assert (
        cargo_sensor.served_topic(deployment_mode=mode, sim_marker_on_graph=marker) == expected
    )
    monkeypatch.setenv(cargo_sensor.TOPIC_ENV, "/tray/loaded")
    assert (
        cargo_sensor.served_topic(deployment_mode=mode, sim_marker_on_graph=marker)
        == "/tray/loaded"
    )


def test_an_unset_topic_on_the_simulation_is_the_shared_interface_topic(
    twin, monkeypatch, connect
):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    adapter, bridge = connect()
    declaration = declared(adapter)[cargo_sensor.CAPABILITY_ID]
    assert declaration.required_observations == (f"{TOPIC}:std_msgs/msg/Bool",)
    assert TOPIC in {item["topic"] for item in bridge.subscribed}


def test_an_unset_topic_on_a_robot_without_it_declares_nothing(twin, monkeypatch, connect):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    adapter, _ = connect({**ODOM, "/clock": CLOCK_TYPE})
    assert cargo_sensor.CAPABILITY_ID not in declared(adapter)
    result = read(adapter)
    assert result.outcome == OUTCOME_REFUSED
    assert cargo_sensor.RESULT_FIELD not in result.evidence


def test_a_hardware_robot_never_takes_the_simulated_default(monkeypatch, connect):
    # A real robot whose graph happens to carry /flyto/cargo_present (the
    # simulated tray bridge pointed at it by mistake) declares nothing and
    # never even subscribes to it unless the topic is configured for it.
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV, raising=False)
    adapter, bridge = connect({**ODOM, TOPIC: "std_msgs/msg/Bool"})
    assert cargo_sensor.CAPABILITY_ID not in declared(adapter)
    assert TOPIC not in {item["topic"] for item in bridge.subscribed}
    reading = adapter.backend.cargo_reading(call_id="cargo-1", deadline_seconds=0.1)
    assert reading.outcome == OUTCOME_REFUSED
    assert cargo_sensor.RESULT_FIELD not in reading.evidence
    assert reading.evidence[cargo_sensor.READING_KEY]["deployment_mode"] == "real"


def test_a_simulation_without_the_marker_on_its_graph_takes_no_default(
    twin, monkeypatch, connect
):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    adapter, _ = connect({**ODOM, TOPIC: "std_msgs/msg/Bool"})  # no /clock
    assert cargo_sensor.CAPABILITY_ID not in declared(adapter)


def test_a_simulation_with_the_marker_check_disabled_takes_no_default(
    twin, monkeypatch, connect
):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    adapter, _ = connect()
    assert cargo_sensor.CAPABILITY_ID not in declared(adapter)


def test_a_hardware_robot_with_its_own_configured_sensor_declares_it(monkeypatch, connect):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "/clock")
    monkeypatch.setenv(cargo_sensor.TOPIC_ENV, "/tray/loaded")
    monkeypatch.setenv(cargo_sensor.MAX_AGE_ENV, "0.3")
    adapter, bridge = connect({**ODOM, "/tray/loaded": "std_msgs/msg/Bool"})
    assert cargo_sensor.CAPABILITY_ID in declared(adapter)
    bridge.publish("/tray/loaded", {"data": True})
    wait_for_reading(adapter)
    result = read(adapter)
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence[cargo_sensor.RESULT_FIELD] is True
    assert result.evidence[cargo_sensor.READING_KEY]["deployment_mode"] == "real"


def test_the_publish_rate_contract_is_declared_once():
    # The subscription is volatile, so a value published before the call is
    # not delivered as new: a publisher must repeat it inside the window.
    assert cargo_sensor.MIN_PUBLISH_HZ >= 2.0
    assert 1.0 / cargo_sensor.MIN_PUBLISH_HZ <= cargo_sensor.DEFAULT_MAX_AGE_S / 2


def test_a_configured_topic_missing_from_the_graph_is_not_declared(twin, connect):
    adapter, _ = connect({**ODOM, "/clock": CLOCK_TYPE})
    assert cargo_sensor.CAPABILITY_ID not in declared(adapter)


def test_a_configured_topic_of_another_type_is_not_declared(twin, connect):
    adapter, _ = connect({**ODOM, "/clock": CLOCK_TYPE, TOPIC: "std_msgs/msg/String"})
    assert cargo_sensor.CAPABILITY_ID not in declared(adapter)


def test_a_configured_bool_topic_is_declared_read_only_and_subscribed(twin, connect):
    adapter, bridge = connect()
    declaration = declared(adapter)[cargo_sensor.CAPABILITY_ID]
    assert declaration.safety_class == "read_only"
    assert declaration.requires_safe_stop is False
    assert declaration.required_observations == (f"{TOPIC}:std_msgs/msg/Bool",)
    assert declaration.arguments == ()
    subscription = next(item for item in bridge.subscribed if item["topic"] == TOPIC)
    assert subscription["type"] == "std_msgs/msg/Bool"
    # Never a latched old value: only what is published now.
    assert subscription["qos"]["durability"] == "volatile"


# --- a fresh reading is the answer ------------------------------------------------


@pytest.mark.parametrize("present", [True, False])
def test_a_fresh_reading_is_answered_as_that_boolean(twin, connect, present):
    adapter, bridge = connect()
    adapter.describe()
    bridge.publish(TOPIC, {"data": present})
    wait_for_reading(adapter)
    result = read(adapter)
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence[cargo_sensor.RESULT_FIELD] is present
    reading = result.evidence[cargo_sensor.READING_KEY]
    assert reading["topic"] == TOPIC
    assert reading["message_type"] == "std_msgs/msg/Bool"
    assert 0.0 <= reading["age_s"] <= reading["max_age_s"] == 0.3
    # Simulation evidence is labelled simulation.
    assert reading["deployment_mode"] == "simulation"


def test_a_reading_published_while_the_call_waits_is_answered(twin, connect):
    adapter, bridge = connect()
    adapter.describe()
    bridge.publish(TOPIC, {"data": True})
    wait_for_reading(adapter)
    time.sleep(0.4)  # the first reading is stale now
    bridge.publish(TOPIC, {"data": False})
    result = read(adapter)
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence[cargo_sensor.RESULT_FIELD] is False


# --- no fresh reading is no value -------------------------------------------------


def test_no_message_is_a_typed_failure_without_a_value(twin, connect):
    adapter, _ = connect()
    adapter.describe()
    result = read(adapter)
    assert result.outcome == OUTCOME_FAILED
    assert cargo_sensor.RESULT_FIELD not in result.evidence
    assert result.evidence[cargo_sensor.READING_KEY]["reason"] == cargo_sensor.REASON_MISSING
    assert cargo_sensor.REASON_MISSING in result.detail


def test_a_stale_message_is_a_typed_failure_without_a_value(twin, connect):
    adapter, bridge = connect()
    adapter.describe()
    bridge.publish(TOPIC, {"data": True})
    wait_for_reading(adapter)
    time.sleep(0.4)  # older than the 0.3 s window, and nothing newer comes
    result = read(adapter)
    assert result.outcome == OUTCOME_FAILED
    assert cargo_sensor.RESULT_FIELD not in result.evidence
    reading = result.evidence[cargo_sensor.READING_KEY]
    assert reading["reason"] == cargo_sensor.REASON_STALE
    assert reading["age_s"] > reading["max_age_s"]
    assert cargo_sensor.REASON_STALE in result.detail


@pytest.mark.parametrize("data", [1, 0, "true", None, [True]])
def test_a_message_without_a_boolean_is_no_reading(twin, connect, data):
    adapter, bridge = connect()
    adapter.describe()
    bridge.publish(TOPIC, {"data": data})
    time.sleep(0.05)
    result = read(adapter)
    assert result.outcome == OUTCOME_FAILED
    assert cargo_sensor.RESULT_FIELD not in result.evidence
    assert result.evidence[cargo_sensor.READING_KEY]["reason"] == cargo_sensor.REASON_MISSING


def test_a_malformed_message_invalidates_the_last_reading(twin, connect):
    # A sensor that just sent garbage is not still reporting its last value.
    adapter, bridge = connect()
    adapter.describe()
    bridge.publish(TOPIC, {"data": True})
    wait_for_reading(adapter)
    bridge.publish(TOPIC, {"data": "true"})
    deadline = time.monotonic() + 2.0
    while adapter.backend._cargo.latest is not None and time.monotonic() < deadline:
        time.sleep(0.01)
    result = read(adapter)
    assert result.outcome == OUTCOME_FAILED
    assert cargo_sensor.RESULT_FIELD not in result.evidence
    assert result.evidence[cargo_sensor.READING_KEY]["reason"] == cargo_sensor.REASON_MISSING


def test_a_reconnect_clears_the_last_reading(twin, monkeypatch):
    # A wide window: without the clear, the old reading would still be fresh.
    monkeypatch.setenv(cargo_sensor.MAX_AGE_ENV, "30")
    bridges = [FakeBridge(SIM_GRAPH), FakeBridge(SIM_GRAPH)]
    sockets = iter(bridges)
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: next(sockets)
    )
    adapter = GenericROS2Adapter(backend=backend, resource_id="turtlebot3-twin")
    try:
        adapter.describe()
        bridges[0].publish(TOPIC, {"data": True})
        wait_for_reading(adapter)
        assert read(adapter).evidence[cargo_sensor.RESULT_FIELD] is True
        backend.reconnect()
        adapter.describe()
        result = adapter.invoke(
            CallRequest("cargo-2", cargo_sensor.CAPABILITY_ID, deadline_seconds=0.2)
        )
        assert result.outcome == OUTCOME_FAILED
        assert cargo_sensor.RESULT_FIELD not in result.evidence
        assert result.evidence[cargo_sensor.READING_KEY]["reason"] == cargo_sensor.REASON_MISSING
    finally:
        adapter.disconnect()


def test_describe_never_declares_an_empty_interface(twin, monkeypatch, connect):
    # The cargo topic vanishing from configuration between discovery and
    # declaration must fail loudly, never declare a nameless interface.
    adapter, _ = connect()
    available = adapter._discovered_capabilities()
    assert cargo_sensor.CAPABILITY_ID in available
    monkeypatch.setattr(adapter, "_discovered_capabilities", lambda: available)
    monkeypatch.setattr(adapter, "_cargo_interface", lambda: None)
    with pytest.raises(RuntimeError, match=cargo_sensor.CAPABILITY_ID):
        adapter.describe()


def test_the_call_takes_no_arguments(twin, connect):
    adapter, bridge = connect()
    adapter.describe()
    bridge.publish(TOPIC, {"data": True})
    wait_for_reading(adapter)
    result = adapter.invoke(
        CallRequest("cargo-1", cargo_sensor.CAPABILITY_ID, arguments={"topic": "/other"})
    )
    assert result.outcome == OUTCOME_REFUSED
    assert cargo_sensor.RESULT_FIELD not in result.evidence


# --- the same graph check as motion -------------------------------------------------


def test_a_hardware_graph_under_a_simulation_configuration_refuses_the_read(
    twin, monkeypatch, connect
):
    adapter, bridge = connect({**ODOM, TOPIC: "std_msgs/msg/Bool"})  # no /clock
    assert cargo_sensor.CAPABILITY_ID in declared(adapter)
    bridge.publish(TOPIC, {"data": True})
    wait_for_reading(adapter)
    result = read(adapter)
    assert result.outcome == OUTCOME_REFUSED
    assert "physical hardware" in result.detail
    assert cargo_sensor.RESULT_FIELD not in result.evidence


def test_the_rclpy_transport_does_not_declare_it(twin):
    from flyto_robotics.generic_ros2_adapter import StandardInterface

    class Rclpy:
        def discover(self):
            return (StandardInterface("topic", TOPIC, "std_msgs/msg/Bool"),)

    adapter = GenericROS2Adapter(backend=Rclpy(), resource_id="turtlebot3-twin")
    assert cargo_sensor.CAPABILITY_ID not in {item.capability_id for item in adapter.describe()}


# --- the pure judgement ---------------------------------------------------------------


def test_the_judgement_reads_only_a_literal_boolean_inside_the_window():
    sensor = cargo_sensor.CargoSensor()
    assert sensor.verdict(now=10.0, max_age_s=1.0).reason == cargo_sensor.REASON_MISSING
    assert sensor.record({"data": "true"}, received=9.5) is False
    assert sensor.verdict(now=10.0, max_age_s=1.0).reason == cargo_sensor.REASON_MISSING
    assert sensor.record({"data": False}, received=9.5) is True
    fresh = sensor.verdict(now=10.0, max_age_s=1.0)
    assert (fresh.value, fresh.reason) == (False, None)
    # A malformed message replaces the reading with none at all.
    assert sensor.record({"data": 0}, received=9.6) is False
    assert sensor.verdict(now=10.0, max_age_s=1.0).reason == cargo_sensor.REASON_MISSING
    assert sensor.record({"data": False}, received=9.5) is True
    stale = sensor.verdict(now=11.0, max_age_s=1.0)
    assert (stale.value, stale.reason) == (None, cargo_sensor.REASON_STALE)


@pytest.mark.parametrize("raw", ["abc", "nan", "inf", "-1", "0", "1000"])
def test_an_unusable_freshness_window_falls_back_to_the_default(monkeypatch, raw):
    monkeypatch.setenv(cargo_sensor.MAX_AGE_ENV, raw)
    assert cargo_sensor.max_age_s() == cargo_sensor.DEFAULT_MAX_AGE_S


# --- who subscribes, and the graph each connection judges -----------------------------


def _cargo_subscriptions(bridge):
    return [
        item
        for item in bridge.subscribed
        if item.get("type") == cargo_sensor.MESSAGE_TYPE or item.get("topic") in (None, TOPIC)
    ]


def test_a_hardware_robot_without_a_configured_topic_subscribes_to_no_cargo_topic(
    monkeypatch, connect
):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV, raising=False)
    _adapter, bridge = connect({**ODOM, TOPIC: "std_msgs/msg/Bool"})
    assert bridge.subscribed  # the connection did subscribe to its readings
    assert _cargo_subscriptions(bridge) == []


def test_a_presence_only_connection_subscribes_to_no_cargo_topic(twin, monkeypatch):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    bridge = FakeBridge(SIM_GRAPH)
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: bridge, presence_only=True
    )
    try:
        assert bridge.subscribed
        assert _cargo_subscriptions(bridge) == []
    finally:
        backend.disconnect()


def test_the_cargo_topic_is_read_from_the_graph_on_demand(twin, monkeypatch):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    bridge = FakeBridge(SIM_GRAPH)
    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=lambda _url: bridge)
    try:
        # Nothing has read the graph yet on this connection.
        assert backend._interfaces is None
        assert backend.cargo_topic() == TOPIC
    finally:
        backend.disconnect()


def test_a_reconnect_forgets_the_graph_the_old_connection_read(twin, monkeypatch):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV)
    bridges = [FakeBridge(SIM_GRAPH), FakeBridge(ODOM)]
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: bridges.pop(0)
    )
    try:
        assert backend.cargo_topic() == TOPIC
        backend.reconnect()  # now a graph without the simulator marker
        # Judged before this connection read its graph: never the old marker.
        assert backend._served_cargo_topic() is None
        assert backend.published_topics() == {}
    finally:
        backend.disconnect()
