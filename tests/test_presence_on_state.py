"""A robot that appears, changes or goes is announced on state, not found later.

The execution host used to learn that a robot had come up (rosbridge or the
drivers starting after the host registered, Nav2 activating once the map
loaded) only on its next heartbeat discovery pass, up to a minute later. These
tests drive the adapter's own signals: the transport connecting or dropping,
the first odometry, odometry after a silence, a Nav2 lifecycle transition, and
the silence deadline. Nothing here sleeps; ``time.sleep`` raises.
"""

from __future__ import annotations

import json
import threading
import time
import types
from types import SimpleNamespace

import pytest
from test_observation_on_state import (
    ROSBRIDGE_ODOM,
    FakeNode,
    RosapiSocket,
    _forbidden,
    clock,  # noqa: F401 - fixture
    fake_rclpy,  # noqa: F401 - fixture
    odometry,
)

import flyto_robotics.adapter_provider as provider
import flyto_robotics.generic_ros2_adapter as adapter_module
from flyto_robotics.adapter_contract import OUTCOME_REFUSED, CallRequest
from flyto_robotics.generic_ros2_adapter import (
    SILENT_SECONDS,
    GenericROS2Adapter,
    RclpyROS2Backend,
    RosbridgeROS2Backend,
)


@pytest.fixture
def no_sleep(monkeypatch):
    monkeypatch.setattr(
        adapter_module,
        "time",
        SimpleNamespace(
            monotonic=time.monotonic, monotonic_ns=time.monotonic_ns, sleep=_forbidden
        ),
    )


class RecordingSocket(RosapiSocket):
    """Answers rosapi and records every subscription the backend asks for."""

    def __init__(self, counter):
        super().__init__(counter)
        self.subscribed: list[dict] = []

    def send(self, payload):
        message = json.loads(payload)
        if message.get("op") == "subscribe":
            self.subscribed.append(message)
        super().send(payload)


def _events(backend) -> tuple[list[str], threading.Event]:
    seen: list[str] = []
    arrived = threading.Event()

    def listener(reason: str) -> None:
        seen.append(reason)
        arrived.set()

    backend.add_graph_listener(listener)
    return seen, arrived


def _drain(seen: list[str], arrived: threading.Event, count: int) -> list[str]:
    deadline = time.monotonic() + 2.0
    while len(seen) < count and time.monotonic() < deadline:
        arrived.wait(0.05)
        arrived.clear()
    return list(seen)


# --- the graph changes the backend notices on its own messages -------------


def test_the_first_odometry_is_announced_and_forgets_the_cached_graph(no_sleep):
    queries: dict[str, int] = {}
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket(queries)
    )
    backend.discover()
    assert queries["/rosapi/topics"] == 1
    seen, arrived = _events(backend)

    backend._handle_publish("/odom", ROSBRIDGE_ODOM)
    backend._handle_publish("/odom", ROSBRIDGE_ODOM)

    assert _drain(seen, arrived, 1) == ["odometry_appeared"]
    # The graph is re-read on the next ask, because the robot's drivers just
    # came up behind a rosbridge that was already there.
    backend.discover()
    assert queries["/rosapi/topics"] == 2
    backend.disconnect()


def test_odometry_after_a_silence_is_announced_as_the_robot_returning(no_sleep):
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket({})
    )
    seen, arrived = _events(backend)
    now = time.monotonic()
    with backend._condition:
        backend._update_odometry(ROSBRIDGE_ODOM, now)
        backend._update_odometry(ROSBRIDGE_ODOM, now + 0.1)
        backend._update_odometry(ROSBRIDGE_ODOM, now + 0.1 + SILENT_SECONDS)

    assert _drain(seen, arrived, 2) == ["odometry_appeared", "odometry_returned"]
    backend.disconnect()


def test_a_nav2_lifecycle_transition_is_announced_and_rereads_the_graph(no_sleep):
    queries: dict[str, int] = {}
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket(queries)
    )
    backend.discover()
    seen, arrived = _events(backend)

    backend._handle_publish(
        "/bt_navigator/transition_event",
        {"transition": {"id": 3, "label": "activate"}, "goal_state": {"id": 3}},
    )

    assert _drain(seen, arrived, 1) == ["lifecycle:/bt_navigator/transition_event"]
    backend.discover()
    assert queries["/rosapi/topics"] == 2
    backend.disconnect()


def test_the_first_map_transform_is_announced(no_sleep):
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket({})
    )
    seen, arrived = _events(backend)
    backend._handle_publish(
        "/tf",
        {"transforms": [{"header": {"frame_id": "map"}, "child_frame_id": "odom"}]},
    )
    assert _drain(seen, arrived, 1) == ["map_transform_appeared"]
    backend.disconnect()


def test_a_reconnect_announces_the_first_odometry_again(no_sleep):
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket({})
    )
    seen, arrived = _events(backend)
    backend._handle_publish("/odom", ROSBRIDGE_ODOM)
    _drain(seen, arrived, 1)
    backend.reconnect()
    backend._handle_publish("/odom", ROSBRIDGE_ODOM)
    assert _drain(seen, arrived, 2) == ["odometry_appeared", "odometry_appeared"]
    backend.disconnect()


def test_the_backend_subscribes_to_the_nav2_lifecycle_topics(no_sleep):
    sockets: list[RecordingSocket] = []

    def connect(_url):
        sockets.append(RecordingSocket({}))
        return sockets[-1]

    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=connect)
    topics = {item["topic"]: item["type"] for item in sockets[-1].subscribed}
    assert topics["/bt_navigator/transition_event"] == "lifecycle_msgs/msg/TransitionEvent"
    assert topics["/behavior_server/transition_event"] == "lifecycle_msgs/msg/TransitionEvent"
    assert "/scan" in topics
    backend.disconnect()


def test_a_presence_connection_carries_odometry_and_lifecycle_only(no_sleep):
    sockets: list[RecordingSocket] = []

    def connect(_url):
        sockets.append(RecordingSocket({}))
        return sockets[-1]

    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=connect, presence_only=True
    )
    topics = {item["topic"] for item in sockets[-1].subscribed}
    assert topics == {
        "/odom",
        "/bt_navigator/transition_event",
        "/behavior_server/transition_event",
    }
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    result = adapter.invoke(
        CallRequest(call_id="c1", capability_id="motion.advance", arguments={"distance_m": 0.1})
    )
    assert result.outcome == OUTCOME_REFUSED
    assert "never commands" in result.detail
    backend.disconnect()


def test_rclpy_subscribes_to_lifecycle_and_creates_action_clients_with_the_node(
    fake_rclpy, clock, monkeypatch  # noqa: F811
):
    import sys

    created: list[str] = []

    class FakeActionClient:
        def __init__(self, _node, _type, name):
            created.append(name)

    def module(name, **attributes):
        made = types.ModuleType(name)
        for key, value in attributes.items():
            setattr(made, key, value)
        monkeypatch.setitem(sys.modules, name, made)

    module("rclpy.action", ActionClient=FakeActionClient)
    module("lifecycle_msgs")
    module("lifecycle_msgs.msg", TransitionEvent=object)
    module("nav2_msgs")
    module(
        "nav2_msgs.action",
        BackUp=object,
        DriveOnHeading=object,
        NavigateToPose=object,
        Spin=object,
    )

    backend = RclpyROS2Backend()
    node = FakeNode.instances[-1]
    assert "/bt_navigator/transition_event" in node.callbacks
    assert sorted(created) == ["/backup", "/drive_on_heading", "/navigate_to_pose", "/spin"]

    seen, arrived = _events(backend)
    node.callbacks["/bt_navigator/transition_event"](object())
    node.callbacks["/odom"](odometry())
    assert _drain(seen, arrived, 2) == [
        "lifecycle:/bt_navigator/transition_event",
        "odometry_appeared",
    ]
    backend.disconnect()


# --- the presence watch turns those signals into host notifications --------


class FakeAdapter:
    def __init__(self) -> None:
        self.connected = True
        self.silent = 0.0
        self.connection_listeners: list = []
        self.graph_listeners: list = []
        self.reconnects = 0
        self.reconnect_error: Exception | None = None
        self.disconnected = False

    def add_connection_listener(self, listener) -> None:
        self.connection_listeners.append(listener)

    def add_graph_listener(self, listener) -> None:
        self.graph_listeners.append(listener)

    def silent_seconds(self):
        return self.silent if self.connected else None

    def reopen(self) -> None:
        self.reconnects += 1
        if self.reconnect_error is not None:
            raise self.reconnect_error
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True
        self.connected = False

    def drop(self) -> None:
        self.connected = False
        for listener in self.connection_listeners:
            listener(False)

    def hear(self, reason: str) -> None:
        for listener in self.graph_listeners:
            listener(reason)


class ScriptedWait:
    """Records each wait instead of sleeping; ``on_wait`` decides what happens."""

    def __init__(self, watch_ref, on_wait) -> None:
        self.waits: list[float | None] = []
        self.watch_ref = watch_ref
        self.on_wait = on_wait

    def __call__(self, event, timeout):
        self.waits.append(timeout)
        self.on_wait(self.watch_ref[0], event, timeout, len(self.waits))
        if len(self.waits) > 50:
            raise AssertionError("the watch looped")
        return event.is_set()


def _watch(build, on_wait):
    notes: list[str] = []
    ref: list = [None]
    wait = ScriptedWait(ref, on_wait)
    watch = provider.PresenceWatch(
        resource_id="robot-1",
        notify=notes.append,
        build_adapter=build,
        wait=wait,
        start=False,
    )
    ref[0] = watch
    return watch, notes, wait


def test_a_robot_that_connects_is_announced_at_once():
    adapter = FakeAdapter()

    def on_wait(watch, _event, _timeout, _count):
        watch.close()

    watch, notes, wait = _watch(lambda _rid: adapter, on_wait)
    watch.run()

    assert notes == ["robot_connected"]
    # Followed by the silence deadline, re-armed from the last reading.
    assert wait.waits == [SILENT_SECONDS]
    assert adapter.disconnected


def test_a_robot_that_is_not_up_is_tried_on_back_off_and_announced_once():
    attempts: list[int] = []
    adapter = FakeAdapter()

    def build(_rid):
        attempts.append(1)
        if len(attempts) < 3:
            raise ConnectionRefusedError("rosbridge is not listening yet")
        return adapter

    def on_wait(watch, _event, timeout, count):
        if timeout == SILENT_SECONDS:
            watch.close()

    watch, notes, wait = _watch(build, on_wait)
    watch.run()

    assert notes == ["robot_unreachable", "robot_connected"]
    assert wait.waits[:2] == [1.0, 2.0]
    assert watch.unreachable_resource_ids() == set()


def test_a_drop_is_announced_and_reconnected_without_waiting():
    adapter = FakeAdapter()

    def on_wait(watch, event, _timeout, count):
        if count == 1:
            adapter.drop()
        else:
            watch.close()

    watch, notes, wait = _watch(lambda _rid: adapter, on_wait)
    watch.run()

    assert notes == ["robot_connected", "robot_disconnected", "robot_connected"]
    assert adapter.reconnects == 1
    # The reconnect after the drop came straight away, not after a back-off.
    assert wait.waits == [SILENT_SECONDS, SILENT_SECONDS]


def test_a_failed_reconnect_marks_the_robot_unreachable():
    adapter = FakeAdapter()

    def on_wait(watch, _event, timeout, count):
        if count == 1:
            adapter.reconnect_error = ConnectionRefusedError("gone")
            adapter.drop()
        else:
            watch.close()

    watch, notes, _wait = _watch(lambda _rid: adapter, on_wait)
    watch.run()

    assert notes == ["robot_connected", "robot_disconnected", "robot_unreachable"]
    assert watch.unreachable_resource_ids() == {"robot-1"}


def test_silence_past_the_deadline_is_announced_and_a_reading_clears_it():
    adapter = FakeAdapter()

    def on_wait(watch, event, timeout, count):
        if count == 1:
            # The deadline passed with nothing heard.
            adapter.silent = SILENT_SECONDS + 0.5
        elif count == 2:
            assert timeout is None, "silent: wait for a reading, not a timer"
            assert watch.unreachable_resource_ids() == {"robot-1"}
            adapter.silent = 0.0
            adapter.hear("odometry_returned")
        else:
            watch.close()

    watch, notes, wait = _watch(lambda _rid: adapter, on_wait)
    watch.run()

    assert notes == ["robot_connected", "robot_silent", "odometry_returned"]
    assert wait.waits == [SILENT_SECONDS, None, SILENT_SECONDS]
    assert watch.unreachable_resource_ids() == set()


def test_discovery_reads_the_graph_over_the_watched_connection(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_TRANSPORT", "rosbridge")
    monkeypatch.setenv("FLYTO_ROSBRIDGE_URL", "ws://127.0.0.1:1")
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    monkeypatch.setattr(adapter_module, "time", SimpleNamespace(
        monotonic=time.monotonic, monotonic_ns=time.monotonic_ns, sleep=_forbidden))
    queries: dict[str, int] = {}
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1",
        connection_factory=lambda _url: RosapiSocket(queries),
        presence_only=True,
    )
    watched = GenericROS2Adapter(backend=backend, resource_id="robot-1")
    watch = SimpleNamespace(adapter=watched)
    monkeypatch.setitem(provider._watches, "robot-1", watch)

    def no_build(*_args, **_kwargs):
        raise AssertionError("a pass built its own connection beside the watched one")

    monkeypatch.setattr(provider, "build", no_build)

    first = provider.discover_resource_manifests(execution_host_id="host")
    second = provider.discover_resource_manifests(execution_host_id="host")

    assert first and first[0]["resource_id"] == "robot-1"
    assert "motion.advance" in first[0]["capability_ids"]
    assert second[0]["capability_ids"] == first[0]["capability_ids"]
    # Each pass reads the graph afresh: a pass is asked for because it changed.
    assert queries["/rosapi/topics"] == 2
    assert not backend._presence_only or backend.is_connected()
    backend.disconnect()


def test_the_discoverer_carries_its_watch_for_hosts_to_find():
    assert provider.discover_resource_manifests.watch is provider.watch_resources


def test_no_watch_when_discovery_is_off(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_AUTODISCOVER", "off")
    assert provider.watch_resources(execution_host_id="h", notify=lambda _r: None) is None


def test_a_disconnect_ends_the_event_thread_and_a_new_event_restarts_it(no_sleep):
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket({})
    )
    seen, arrived = _events(backend)
    first = backend._graph_events
    assert first is not None

    backend.disconnect()
    assert backend._graph_events is None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and any(
        thread.name == "ros2-graph-events" and thread.is_alive()
        for thread in threading.enumerate()
    ):
        threading.Event().wait(0.01)
    assert not any(
        thread.name == "ros2-graph-events" and thread.is_alive()
        for thread in threading.enumerate()
    ), "a retired adapter left its event thread behind"

    backend.reconnect()
    backend._handle_publish("/odom", ROSBRIDGE_ODOM)
    assert _drain(seen, arrived, 1) == ["odometry_appeared"]
    backend.disconnect()
