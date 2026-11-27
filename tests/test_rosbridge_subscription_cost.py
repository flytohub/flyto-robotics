"""What a rosbridge connection reads, at what rate, and while what.

On 2026-10-12 rosbridge on burger-01 used 75 % of a core (load ~10 on a 4-core
Pi 4B) and the braking guard saw scans up to 0.965 s late while the sensor kept
10 Hz. Measured read-only through the robot's tunnel the same evening, one
adapter connection carried 884 KiB/s, 756 KiB/s of it a raw camera frame a
second that only an unread observation digest used.

On 2026-11-09, with the robot completely idle and no job running, rosbridge
still used 52 % of a core (516 ticks in 10 s). One connection -- flyto-cloud's
warm adapter, kept for the life of the host -- carried ~121 KB/s of scans,
transforms and odometry nobody was reading; the presence watch carried
~1.9 KB/s. These tests pin each row of ``ROSBRIDGE_SUBSCRIPTIONS`` to the calls
that need it (``CALL_READINGS``) and the freshness every refusal reads.
"""

from __future__ import annotations

import json
import time

import pytest
from test_inflation_escape import OBSERVED, EscapeBackend, box, make, navigate, room
from test_observation_on_state import ROSBRIDGE_ODOM, ROSBRIDGE_SCAN

from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics.adapter_contract import OUTCOME_COMPLETED, CallRequest
from flyto_robotics.generic_ros2_adapter import (
    CALL_READINGS,
    DEFAULT_INTERFACES,
    HOST_INTERFACES,
    READ_MOTION,
    READ_STATE,
    REQUIRE_RANGE,
    REQUIRE_SCAN_TIMING,
    ROSBRIDGE_SUBSCRIPTIONS,
    GenericROS2Adapter,
    RosbridgeROS2Backend,
)
from tests.test_presence_on_state import RecordingSocket, no_sleep  # noqa: F401 - fixture

#: Measured on 2026-10-12, read-only, KiB/s per topic on one connection as
#: 0.8.1 subscribed them (odometry at the robot's ~18 Hz, the others as sent).
MEASURED_KIB_S = {
    "/camera/image_raw": 756.1,
    "/scan": 98.0,
    "/odom": 13.2,
    "/tf": 10.0,
    "/camera/camera_info": 6.2,
    "/battery_state": 0.4,
}
ODOM_HZ = 18.1

IDLE_TOPICS = {
    "/odom",
    "/battery_state",
    "/bt_navigator/transition_event",
    "/behavior_server/transition_event",
    "/collision_monitor_state",
}
CALL_TOPICS = {"/odom", "/scan", "/tf"}
MOTION_ONLY_TOPICS = {"/cmd_vel_nav", "/cmd_vel"}


class LedgerSocket(RecordingSocket):
    """Records subscribes and unsubscribes; ``held()`` is what rosbridge holds now."""

    def __init__(self, counter):
        super().__init__(counter)
        self.unsubscribed: list[dict] = []
        self.ids: dict[str, dict] = {}

    def send(self, payload):
        message = json.loads(payload)
        if message.get("op") == "unsubscribe":
            self.unsubscribed.append(message)
            self.ids.pop(message["id"], None)
        elif message.get("op") == "subscribe":
            self.ids[message["id"]] = message
        super().send(payload)

    def held(self) -> dict[str, list[dict]]:
        topics: dict[str, list[dict]] = {}
        for request in self.ids.values():
            topics.setdefault(request["topic"], []).append(request)
        return topics


def connection(*, presence_only=False):
    sockets: list[LedgerSocket] = []

    def connect(_url):
        sockets.append(LedgerSocket({}))
        return sockets[-1]

    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=connect, presence_only=presence_only
    )
    return backend, sockets


def effective(requests: list[dict]) -> dict:
    """What rosbridge sends for one topic: the smallest throttle and queue any id asks."""
    return {
        "throttle_rate": min(item.get("throttle_rate", 0) for item in requests),
        "queue_length": min(item.get("queue_length", 0) for item in requests),
    }


def kib_s(held: dict[str, list[dict]]) -> float:
    total = 0.0
    for topic, rate in MEASURED_KIB_S.items():
        if topic not in held:
            continue
        throttle = effective(held[topic])["throttle_rate"]
        scale = min(1.0, (1000.0 / throttle) / ODOM_HZ) if topic == "/odom" and throttle else 1.0
        total += rate * scale
    return total


@pytest.fixture(autouse=True)
def _hardware(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.25")


# -- an idle connection ------------------------------------------------------------


@pytest.mark.usefixtures("no_sleep")
def test_an_idle_full_connection_reads_no_lidar_transforms_or_commands():
    backend, sockets = connection()
    held = sockets[-1].held()
    backend.disconnect()
    assert set(held) == IDLE_TOPICS
    assert effective(held["/odom"]) == {"throttle_rate": 500, "queue_length": 1}


@pytest.mark.usefixtures("no_sleep")
def test_an_idle_full_connection_costs_what_the_presence_watch_costs():
    full, full_sockets = connection()
    presence, presence_sockets = connection(presence_only=True)
    idle, watch = full_sockets[-1].held(), presence_sockets[-1].held()
    full.disconnect()
    presence.disconnect()
    # The collision monitor publishes only on a change: nothing while idle.
    assert set(idle) - set(watch) == {"/collision_monitor_state"}
    assert kib_s(idle) == pytest.approx(kib_s(watch))
    # 2026-11-09 measured the presence watch at ~1.9 KB/s and the warm full
    # connection at ~121 KB/s; idle, the full connection is now the former.
    assert kib_s(idle) < 2.5


@pytest.mark.usefixtures("no_sleep")
def test_a_presence_connection_never_takes_a_calls_readings():
    backend, sockets = connection(presence_only=True)
    with backend.hold_readings(READ_MOTION):
        held = set(sockets[-1].held())
    backend.disconnect()
    assert held.isdisjoint(CALL_TOPICS - {"/odom"} | MOTION_ONLY_TOPICS)


# -- while a call runs ------------------------------------------------------------


@pytest.mark.usefixtures("no_sleep")
def test_a_motion_holds_lidar_transforms_fast_odometry_and_the_command_watch():
    backend, sockets = connection()
    with backend.hold_readings(READ_MOTION):
        held = sockets[-1].held()
        assert set(held) >= CALL_TOPICS | MOTION_ONLY_TOPICS
        # The guard reads every scan newest first, odometry at 10 Hz newest first.
        assert effective(held["/scan"]) == {"throttle_rate": 0, "queue_length": 1}
        assert effective(held["/odom"]) == {"throttle_rate": 100, "queue_length": 1}
        assert effective(held["/tf"])["throttle_rate"] == 0
        # No camera, even during a call: vision.observe captures one compressed
        # frame on its own subscription, and the HMI's live camera is its own.
        assert not {"/camera/image_raw", "/camera/camera_info"} & set(held)
        assert DEFAULT_INTERFACES["vision.observe"][1] == "/camera/image_raw/compressed"
        during = kib_s(held)
    after = sockets[-1].held()
    backend.disconnect()
    assert set(after) == IDLE_TOPICS
    assert effective(after["/odom"])["throttle_rate"] == 500
    # Dropped by the ids that made them; the idle odometry id stays.
    dropped = {item["topic"] for item in sockets[-1].unsubscribed}
    assert dropped == {"/scan", "/tf", "/odom"} | MOTION_ONLY_TOPICS
    # During a call the connection costs what 0.8.2 measured: a seventh of 0.8.1.
    assert during < sum(MEASURED_KIB_S.values()) / 7.0


@pytest.mark.usefixtures("no_sleep")
def test_a_state_read_takes_no_command_watch():
    backend, sockets = connection()
    with backend.hold_readings(READ_STATE):
        held = set(sockets[-1].held())
    backend.disconnect()
    assert held >= CALL_TOPICS
    assert held.isdisjoint(MOTION_ONLY_TOPICS)


@pytest.mark.usefixtures("no_sleep")
def test_readings_stay_until_the_last_call_holding_them_ends():
    backend, sockets = connection()
    first = backend.hold_readings(READ_MOTION)
    first.__enter__()
    with backend.hold_readings(READ_STATE):
        pass
    still = set(sockets[-1].held())
    first.__exit__(None, None, None)
    released = set(sockets[-1].held())
    backend.disconnect()
    assert still >= CALL_TOPICS | MOTION_ONLY_TOPICS
    assert released == IDLE_TOPICS


@pytest.mark.usefixtures("no_sleep")
def test_a_goal_left_running_keeps_the_guard_reading_until_it_ends():
    backend, sockets = connection()
    with backend.hold_readings(READ_MOTION):
        # A call that ran out of time: its goal is still driving.
        backend._active_actions["adv-1"] = "/drive_on_heading"
    assert "/scan" in sockets[-1].held()
    backend._handle_message(
        {"op": "action_result", "id": "adv-1", "status": 4, "result": True, "values": {}}
    )
    after = set(sockets[-1].held())
    backend.disconnect()
    assert after == IDLE_TOPICS


@pytest.mark.usefixtures("no_sleep")
def test_a_reconnect_during_a_call_takes_its_readings_on_the_new_socket():
    backend, sockets = connection()
    with backend.hold_readings(READ_MOTION):
        backend.reconnect()
        assert len(sockets) == 2
        assert set(sockets[-1].held()) >= CALL_TOPICS | MOTION_ONLY_TOPICS
    after = set(sockets[-1].held())
    backend.disconnect()
    assert after == IDLE_TOPICS


# -- which calls need which rows ----------------------------------------------------


def test_every_capability_says_what_it_reads():
    capabilities = set(DEFAULT_INTERFACES) | set(HOST_INTERFACES)
    assert capabilities <= set(CALL_READINGS)
    # A stop goes out at once; it never waits for a subscription.
    assert CALL_READINGS["motion.halt"] is None
    for capability, reading in CALL_READINGS.items():
        assert reading in (None, READ_MOTION, READ_STATE), capability
    # Anything not listed holds the most.
    assert adapter_module._call_reading("motion.something_new") == READ_MOTION


def test_every_row_names_its_reader_and_its_holders():
    for row in ROSBRIDGE_SUBSCRIPTIONS:
        assert row.reader.strip(), row.key
        assert row.held_by, row.key


class HoldingBackend(EscapeBackend):
    """Records the readings held around each call the adapter makes."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.holding: list[str] = []
        self.held_at_invoke: list[tuple[str, ...]] = []
        self.held_at_observation: list[tuple[str, ...]] = []

    def hold_readings(self, reading):
        backend = self

        class Hold:
            def __enter__(self):
                if reading is not None:
                    backend.holding.append(reading)

            def __exit__(self, *_exc):
                if reading is not None:
                    backend.holding.remove(reading)

        return Hold()

    def invoke(self, **kwargs):
        self.held_at_invoke.append(tuple(self.holding))
        return super().invoke(**kwargs)

    def observation(self, required=None):
        self.held_at_observation.append(tuple(self.holding))
        return super().observation(required)


def test_the_adapter_holds_a_motions_readings_for_the_whole_call(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    backend = HoldingBackend(box(1.0, 0.3, 0.4) + room())
    result = navigate(make(backend), x=0.50)
    assert result.outcome == OUTCOME_COMPLETED
    assert backend.held_at_invoke == [(READ_MOTION,)]
    # The preflight, the before and the after observations all read under it.
    assert backend.held_at_observation
    assert all(READ_MOTION in held for held in backend.held_at_observation)
    assert backend.holding == []


def test_a_stop_holds_nothing(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    backend = HoldingBackend(OBSERVED)
    stopped = []
    backend.safe_stop = lambda call_id: stopped.append(tuple(backend.holding)) or (
        EscapeBackend.safe_stop(backend, call_id)
    )
    make(backend).invoke(
        CallRequest(call_id="halt-1", capability_id="motion.halt", arguments={},
                    deadline_seconds=5.0)
    )
    assert stopped == [()]


def test_a_hosts_observation_and_stillness_wait_read_under_a_hold(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    backend = HoldingBackend(OBSERVED)
    seen: list[tuple[str, ...]] = []
    backend.wait_until_stationary = lambda _s: seen.append(tuple(backend.holding)) or {
        "stationary": True
    }
    adapter = make(backend)
    adapter.observe(phase="before", execution_id="call-1")
    adapter.wait_until_stationary(1.0)
    assert backend.held_at_observation[-1] == (READ_STATE,)
    assert seen == [(READ_STATE,)]


# -- freshness: a call judges only what arrived after it asked ---------------------


@pytest.mark.usefixtures("no_sleep")
def test_a_scan_held_from_before_the_call_never_admits_a_motion(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "lidar_clearance")
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    backend, _sockets = connection()
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    with backend.hold_readings(READ_MOTION):
        pass
    # Readings from an earlier call, well inside the observation max age.
    with backend._condition:
        backend._update_odometry(ROSBRIDGE_ODOM, time.monotonic())
        backend._update_scan(ROSBRIDGE_SCAN, time.monotonic())
    with backend.hold_readings(READ_MOTION):
        # The robot sends nothing after the subscription: the refusals fire.
        refusal = adapter._motion_preflight("motion.advance")
    backend.disconnect()
    assert refusal == "fresh odometry is required before motion"


@pytest.mark.usefixtures("no_sleep")
def test_a_motion_waits_for_scans_sent_after_it_asked(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SAFETY_BASIS", "lidar_clearance")
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "3")
    backend, _sockets = connection()
    with backend._condition:
        backend._update_scan(ROSBRIDGE_SCAN, time.monotonic())
    with backend.hold_readings(READ_MOTION):
        assert REQUIRE_RANGE in backend._missing((REQUIRE_RANGE,))
        assert REQUIRE_SCAN_TIMING in backend._missing((REQUIRE_SCAN_TIMING,))
        for _ in range(4):
            with backend._condition:
                backend._update_scan(ROSBRIDGE_SCAN, time.monotonic())
            time.sleep(0.01)
        assert backend._missing((REQUIRE_RANGE, REQUIRE_SCAN_TIMING)) == []
    backend.disconnect()


@pytest.mark.usefixtures("no_sleep")
def test_the_time_between_two_calls_is_never_a_scan_interval():
    backend, _sockets = connection()
    for start in (100.0, 102.0):  # two calls, 1.7 s apart with no scan between
        with backend.hold_readings(READ_MOTION):
            for index in range(4):
                with backend._condition:
                    backend._update_scan(ROSBRIDGE_SCAN, start + 0.1 * index)
    worst = backend._latency.scan_period_s
    backend.disconnect()
    assert worst == pytest.approx(0.1)


@pytest.mark.usefixtures("no_sleep")
def test_only_a_transport_that_holds_per_call_waits_for_scan_timing():
    assert RosbridgeROS2Backend.HOLDS_READINGS_PER_CALL is True
    assert getattr(adapter_module.RclpyROS2Backend, "HOLDS_READINGS_PER_CALL", False) is False


@pytest.mark.usefixtures("no_sleep")
def test_reading_the_backend_directly_holds_what_it_waits_for():
    # A tool or probe that reads the backend outside a call (the twin's
    # scenario runner) gets readings sent after it asked, and leaves the
    # connection idle again.
    backend, sockets = connection()
    taken: list[set[str]] = []
    original = backend._missing

    def missing(required):
        taken.append(set(sockets[-1].held()))
        return original(required)

    backend._missing = missing
    backend.observation()
    after = set(sockets[-1].held())
    backend.disconnect()
    assert taken and all(held >= CALL_TOPICS for held in taken)
    assert after == IDLE_TOPICS
