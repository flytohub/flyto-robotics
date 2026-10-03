"""Every adapter wait ends on the ROS message it is waiting for.

The adapter used to spin rclpy to fixed deadlines, re-ask rosapi for the graph
on every observation, and leave the host to sleep a fixed second before the
last pose. These tests replace the clock and ``time.sleep`` inside the adapter
module, and replace each backend's condition with one whose ``wait`` delivers
the next subscription callback. A wait that returns only because a message
arrived consumes exactly the messages it needed; one driven by a deadline
would advance the fake clock instead, and a sleep or spin raises.
"""

from __future__ import annotations

import json
import queue
import sys
import threading
import time
import types
from types import SimpleNamespace

import pytest

import flyto_robotics.generic_ros2_adapter as adapter_module
from flyto_robotics.generic_ros2_adapter import (
    REQUIRE_MAP_TF,
    REQUIRE_POSE,
    REQUIRE_RANGE,
    GenericROS2Adapter,
    RclpyROS2Backend,
    RosbridgeROS2Backend,
)

ODOM_PERIOD = 0.033


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def monotonic(self) -> float:
        return self.now


def _forbidden(*_args, **_kwargs):
    raise AssertionError("the adapter waited on a timer instead of on state")


@pytest.fixture
def clock(monkeypatch):
    fake = FakeClock()
    monkeypatch.setattr(
        adapter_module,
        "time",
        SimpleNamespace(
            monotonic=fake.monotonic,
            monotonic_ns=time.monotonic_ns,
            sleep=_forbidden,
        ),
    )
    return fake


class FeedingCondition(threading.Condition):
    """A condition whose every wait is answered by the next ROS callback.

    With nothing left to deliver, a wait is the cap running out: the fake
    clock jumps to the deadline. Counting both shows whether a call returned
    because of a message or because of time.
    """

    def __init__(self, clock: FakeClock) -> None:
        super().__init__()
        self.clock = clock
        self.feed: list = []
        self.delivered = 0
        self.timed_out = 0

    def wait(self, timeout=None):
        if self.feed:
            self.clock.now += ODOM_PERIOD
            self.feed.pop(0)()
            self.delivered += 1
            return True
        self.timed_out += 1
        self.clock.now += timeout or 0.0
        return False


# --- a fake rclpy that refuses to be spun by the caller ---------------------


class FakeNode:
    def __init__(self, _name: str) -> None:
        self.callbacks: dict[str, object] = {}
        self.topics = [
            ("/odom", ["nav_msgs/msg/Odometry"]),
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ("/cmd_vel", ["geometry_msgs/msg/TwistStamped"]),
        ]
        self.actions = [("/navigate_to_pose", ["nav2_msgs/action/NavigateToPose"])]
        self.graph_reads = 0

    def create_subscription(self, _type, topic, callback, _qos):
        self.callbacks[topic] = callback

    def get_topic_names_and_types(self):
        self.graph_reads += 1
        return self.topics

    def get_action_names_and_types(self):
        return self.actions


class FakeExecutor:
    instances: list[FakeExecutor] = []

    def __init__(self) -> None:
        self.nodes = []
        self.stopped = threading.Event()
        FakeExecutor.instances.append(self)

    def add_node(self, node) -> None:
        self.nodes.append(node)

    def spin(self) -> None:
        self.stopped.wait()

    def shutdown(self) -> None:
        self.stopped.set()


@pytest.fixture
def fake_rclpy(monkeypatch):
    FakeExecutor.instances = []

    def module(name, **attributes):
        created = types.ModuleType(name)
        for key, value in attributes.items():
            setattr(created, key, value)
        monkeypatch.setitem(sys.modules, name, created)
        return created

    def spin_once(*_args, **_kwargs):
        raise AssertionError("the caller spun the node instead of the executor")

    module("rclpy", ok=lambda: True, init=lambda args=None: None, spin_once=spin_once)
    module("rclpy.node", Node=FakeNode)
    module("rclpy.qos", qos_profile_sensor_data=object())
    module("rclpy.executors", SingleThreadedExecutor=FakeExecutor)
    module("nav_msgs")
    module("nav_msgs.msg", Odometry=object)
    module("sensor_msgs")
    module("sensor_msgs.msg", CameraInfo=object, Image=object, LaserScan=object)
    module("tf2_msgs")
    module("tf2_msgs.msg", TFMessage=object)
    return FakeExecutor


def odometry(x=0.0, *, linear=0.0, angular=0.0):
    vector = SimpleNamespace
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(x=x, y=0.0, z=0.0),
                orientation=vector(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vector(x=linear, y=0.0, z=0.0),
                angular=vector(x=0.0, y=0.0, z=angular),
            )
        ),
    )


SCAN = SimpleNamespace(
    ranges=[0.8, 0.9, 1.0],
    range_min=0.1,
    range_max=10.0,
    angle_min=0.0,
    angle_increment=0.1,
)
MAP_TF = SimpleNamespace(
    transforms=[SimpleNamespace(header=SimpleNamespace(frame_id="map"), child_frame_id="odom")]
)


def rclpy_backend(clock):
    backend = RclpyROS2Backend()
    condition = FeedingCondition(clock)
    backend._condition = condition
    return backend, condition


# --- (a) observation returns on the first odom + scan callback --------------


def test_rclpy_observation_returns_on_the_callbacks_it_needs(fake_rclpy, clock):
    backend, condition = rclpy_backend(clock)
    condition.feed = [
        lambda: backend._on_odometry(odometry()),
        lambda: backend._on_scan(SCAN),
        lambda: pytest.fail("the observation waited past the readings it needed"),
    ]

    observation = backend.observation()

    assert observation["pose"]["x"] == 0.0
    assert observation["range"]["minimum_range_m"] == 0.8
    assert condition.delivered == 2
    assert condition.timed_out == 0
    # Two odometry periods, not a 0.25 s spin plus 0.1 s spins to a deadline.
    assert clock.now - 100.0 == pytest.approx(2 * ODOM_PERIOD)


def test_a_warm_rclpy_backend_answers_without_waiting(fake_rclpy, clock):
    backend, condition = rclpy_backend(clock)
    backend._on_odometry(odometry())
    backend._on_scan(SCAN)

    observation = backend.observation()

    assert observation["pose"] is not None and observation["range"] is not None
    assert condition.delivered == condition.timed_out == 0
    assert clock.now == 100.0


def test_rclpy_callbacks_run_on_a_background_executor_until_disconnect(fake_rclpy, clock):
    backend, _condition = rclpy_backend(clock)
    executor = fake_rclpy.instances[-1]
    assert backend._node in executor.nodes
    assert backend._executor_thread.is_alive()

    backend.disconnect()

    assert executor.stopped.is_set()
    assert backend._executor_thread is None
    backend.reconnect()
    assert fake_rclpy.instances[-1] is not executor
    assert backend._executor_thread.is_alive()
    backend.disconnect()


def test_rclpy_discover_reads_the_graph_without_spinning(fake_rclpy, clock):
    backend, _condition = rclpy_backend(clock)

    found = backend.discover()

    assert {item.name for item in found} >= {"/cmd_vel", "/navigate_to_pose"}
    assert clock.now == 100.0


def test_a_goal_future_is_awaited_by_its_done_callback(fake_rclpy, clock):
    backend, _condition = rclpy_backend(clock)

    class Future:
        def __init__(self):
            self.callbacks = []
            self.finished = False

        def done(self):
            return self.finished

        def add_done_callback(self, callback):
            self.callbacks.append(callback)
            # The executor thread completes the goal as soon as it is asked.
            threading.Thread(target=self.finish).start()

        def finish(self):
            self.finished = True
            for callback in self.callbacks:
                callback(self)

    started = time.monotonic()
    assert backend._wait_future(Future(), clock.now + 30.0) is True
    assert time.monotonic() - started < 1.0


def test_rosbridge_observation_returns_on_the_messages_it_needs(clock):
    backend = quiet_backend()
    condition = FeedingCondition(clock)
    backend._condition = condition
    condition.feed = [
        lambda: backend._update_odometry(ROSBRIDGE_ODOM, clock.now),
        lambda: backend._update_scan(ROSBRIDGE_SCAN, clock.now),
    ]

    observation = backend.observation()

    assert observation["pose"] is not None and observation["range"] is not None
    assert condition.delivered == 2 and condition.timed_out == 0
    backend.disconnect()


# --- (b) a navigate preflight waits for the first map->odom transform -------


def test_navigate_preflight_waits_for_the_map_transform_instead_of_refusing(
    fake_rclpy, clock, monkeypatch
):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "lidar_clearance")
    backend, condition = rclpy_backend(clock)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    # Odometry and LiDAR arrive first; the transform follows on the next /tf.
    condition.feed = [
        lambda: backend._on_odometry(odometry()),
        lambda: backend._on_scan(SCAN),
        lambda: backend._on_tf(MAP_TF),
    ]

    assert adapter._motion_preflight("motion.navigate") is None
    assert condition.delivered == 3
    assert condition.timed_out == 0


def test_the_default_observation_does_not_hold_motion_for_the_map(fake_rclpy, clock):
    backend, condition = rclpy_backend(clock)
    backend._on_odometry(odometry())
    backend._on_scan(SCAN)
    condition.feed = [lambda: pytest.fail("an advance must not wait for the map")]

    observation = backend.observation()

    assert observation["map_tf_available"] is False
    assert condition.delivered == 0


def test_navigate_still_refuses_once_the_wait_ends_without_a_transform(
    fake_rclpy, clock, monkeypatch
):
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.5")
    backend, condition = rclpy_backend(clock)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    backend._on_odometry(odometry())
    backend._on_scan(SCAN)

    refusal = adapter._motion_preflight("motion.navigate")

    assert refusal == "fresh map-to-odom transform is required before navigation"
    assert condition.timed_out == 1
    assert clock.now - 100.0 == pytest.approx(0.5)


def test_the_preflight_asks_only_for_what_navigation_needs(fake_rclpy, clock):
    backend, _condition = rclpy_backend(clock)
    assert backend._missing((REQUIRE_POSE, REQUIRE_RANGE, REQUIRE_MAP_TF)) == [
        REQUIRE_POSE,
        REQUIRE_RANGE,
        REQUIRE_MAP_TF,
    ]


# --- (c) settle on odometry, not on a fixed second --------------------------


def test_stationary_returns_on_consecutive_still_odometry(fake_rclpy, clock):
    backend, condition = rclpy_backend(clock)
    backend._on_odometry(odometry(0.10, linear=0.12))
    condition.feed = [
        lambda: backend._on_odometry(odometry(0.104, linear=0.08)),
        lambda: backend._on_odometry(odometry(0.105, linear=0.0)),
        lambda: backend._on_odometry(odometry(0.105, linear=0.0)),
        lambda: backend._on_odometry(odometry(0.105, linear=0.0)),
        lambda: pytest.fail("waited past the samples that showed it stopped"),
    ]

    settled = backend.wait_until_stationary(1.0)

    assert settled["stationary"] is True and settled["drifting"] is False
    assert settled["still_samples"] == 3
    assert condition.delivered == 4 and condition.timed_out == 0
    assert settled["waited_seconds"] == pytest.approx(4 * ODOM_PERIOD)
    assert settled["waited_seconds"] < 1.0


def test_a_robot_still_moving_at_the_cap_is_reported_drifting(fake_rclpy, clock):
    backend, condition = rclpy_backend(clock)
    backend._on_odometry(odometry(0.0, linear=0.05))
    moving = [
        (lambda step=step: backend._on_odometry(odometry(0.002 * step, linear=0.05)))
        for step in range(1, 100)
    ]
    condition.feed = list(moving)

    settled = backend.wait_until_stationary(1.0)

    assert settled["stationary"] is False and settled["drifting"] is True
    assert settled["still_samples"] == 0
    # It stopped at the cap: about 1 s of odometry, not every queued sample.
    assert 0.95 <= settled["waited_seconds"] <= 1.0 + ODOM_PERIOD
    assert condition.feed


def test_stationary_judges_pose_when_odometry_carries_no_twist(clock):
    backend = quiet_backend()
    condition = FeedingCondition(clock)
    backend._condition = condition

    def pose_only(x):
        return {"pose": {"pose": {"position": {"x": x, "y": 0.0}, "orientation": {"w": 1.0}}}}

    backend._update_odometry(pose_only(0.10), clock.now)
    condition.feed = [
        lambda: backend._update_odometry(pose_only(0.12), clock.now),
        lambda: backend._update_odometry(pose_only(0.1205), clock.now),
        lambda: backend._update_odometry(pose_only(0.1205), clock.now),
        lambda: backend._update_odometry(pose_only(0.1206), clock.now),
    ]

    settled = backend.wait_until_stationary(1.0)

    assert settled["stationary"] is True
    assert condition.delivered == 4 and condition.timed_out == 0
    backend.disconnect()


def test_the_adapter_exposes_the_settle_wait_for_the_host(fake_rclpy, clock):
    backend, condition = rclpy_backend(clock)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    backend._on_odometry(odometry(linear=0.0))
    condition.feed = [lambda: backend._on_odometry(odometry(linear=0.0)) for _ in range(3)]

    assert adapter.wait_until_stationary(1.0)["stationary"] is True
    assert adapter.supports_shared_connection is True


# --- (d) rosbridge discovery is cached until a reconnect ---------------------

ROSBRIDGE_ODOM = {
    "pose": {"pose": {"position": {"x": 0.0, "y": 0.0}, "orientation": {"w": 1.0}}},
    "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
}
ROSBRIDGE_SCAN = {"ranges": [0.8, 0.9, 1.0], "range_min": 0.1, "range_max": 100.0}


class QuietSocket:
    """A rosbridge connection nothing on the graph ever answers on."""

    def __init__(self):
        self._closed = threading.Event()

    def send(self, _payload):
        pass

    def pong(self, _data=b""):
        pass

    def recv(self, timeout=None):
        self._closed.wait(timeout if timeout is not None else 0.2)
        raise TimeoutError

    def close(self):
        self._closed.set()


def quiet_backend():
    return RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: QuietSocket()
    )


class RosapiSocket:
    """Answers rosapi graph queries and counts them."""

    def __init__(self, counter):
        self.counter = counter
        self.inbox: queue.Queue = queue.Queue()

    def send(self, payload):
        message = json.loads(payload)
        if message.get("op") != "call_service":
            return
        self.counter[message["service"]] = self.counter.get(message["service"], 0) + 1
        values = (
            {
                "topics": ["/odom", "/scan", "/cmd_vel"],
                "types": [
                    "nav_msgs/msg/Odometry",
                    "sensor_msgs/msg/LaserScan",
                    "geometry_msgs/msg/TwistStamped",
                ],
            }
            if message["service"] == "/rosapi/topics"
            else {"action_servers": ["/drive_on_heading"]}
        )
        self.inbox.put(
            json.dumps(
                {"op": "service_response", "id": message["id"], "result": True, "values": values}
            )
        )

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

    def drop(self):
        self.inbox.put(ConnectionError("rosbridge went away"))

    def close(self):
        self.inbox.put(ConnectionError("closed"))


@pytest.fixture
def no_sleep(monkeypatch):
    monkeypatch.setattr(
        adapter_module,
        "time",
        SimpleNamespace(
            monotonic=time.monotonic, monotonic_ns=time.monotonic_ns, sleep=_forbidden
        ),
    )


def test_rosbridge_discovery_is_served_from_cache_until_reconnect(no_sleep, monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    queries: dict[str, int] = {}
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket(queries)
    )
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    with backend._condition:
        backend._update_odometry(ROSBRIDGE_ODOM, time.monotonic())
        backend._update_scan(ROSBRIDGE_SCAN, time.monotonic())

    adapter.describe()
    for phase in ("before", "after", "post_stop"):
        adapter.observe(phase=phase, execution_id="call-1")
    assert adapter._deployment_mismatch() is None
    assert adapter._motion_preflight("motion.advance") is None

    assert queries == {"/rosapi/topics": 1, "/rosapi/action_servers": 1}

    backend.reconnect()
    adapter.observe(phase="before", execution_id="call-1")
    assert queries == {"/rosapi/topics": 2, "/rosapi/action_servers": 2}
    backend.disconnect()


def test_a_missing_capability_rereads_the_graph_once(no_sleep):
    queries: dict[str, int] = {}
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: RosapiSocket(queries)
    )
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter.describe()
    assert queries["/rosapi/topics"] == 1

    # motion.navigate is not on this graph: the miss asks rosapi again, since
    # Nav2 may have started since, and then refuses.
    from flyto_robotics.adapter_contract import OUTCOME_REFUSED, CallRequest

    result = adapter.invoke(
        CallRequest(
            call_id="c1", capability_id="motion.navigate", arguments={"x": 1.0, "y": 0.0}
        )
    )

    assert result.outcome == OUTCOME_REFUSED
    assert queries["/rosapi/topics"] == 2
    backend.disconnect()


def test_a_dropped_socket_is_announced_and_forgets_the_graph(no_sleep):
    queries: dict[str, int] = {}
    sockets: list[RosapiSocket] = []

    def connect(_url):
        sockets.append(RosapiSocket(queries))
        return sockets[-1]

    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=connect)
    events: list[bool] = []
    dropped = threading.Event()

    def listener(connected):
        events.append(connected)
        if not connected:
            dropped.set()

    backend.add_connection_listener(listener)
    backend.discover()
    assert backend._interfaces is not None

    sockets[-1].drop()

    assert dropped.wait(2.0), "the drop was not announced"
    assert events == [False]
    assert backend._interfaces is None
    assert backend.discover() == ()

    backend.reconnect()
    assert events == [False, True]
    backend.disconnect()


def test_disconnect_wakes_the_keepalive_at_once(no_sleep):
    backend = quiet_backend()
    stop = backend._keepalive_stop

    backend.disconnect()

    assert stop.is_set()
