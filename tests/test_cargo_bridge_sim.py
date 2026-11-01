"""SIMULATION cargo bridge: symbolic tray state published as std_msgs/Bool.

No Docker and no network: rosbridge is a fake websocket and the arm host's
simulation control is a fake fetch function.
"""

from __future__ import annotations

import json
import queue
import threading
import time
import urllib.request

import pytest

from sim.fetch_return.arm_place_signal import ArmTruthWatcher
from sim.fetch_return.cargo_bridge import CargoBridge, serve_control
from sim.fetch_return.cargo_tray import CargoTray, Station, parked_pose
from sim.fetch_return.rosbridge_link import BOOL_TYPE, RosbridgeLink

STATION = Station(x=0.40, y=0.0, tolerance_m=0.10)


def odom(x: float, y: float, *, vx: float = 0.0, wz: float = 0.0) -> dict:
    return {
        "header": {"frame_id": "odom"},
        "pose": {"pose": {"position": {"x": x, "y": y, "z": 0.0}}},
        "twist": {"twist": {"linear": {"x": vx, "y": 0.0}, "angular": {"z": wz}}},
    }


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


# --- the tray state ------------------------------------------------------


def test_tray_starts_empty_and_is_labelled_simulation():
    tray = CargoTray(STATION, clock=FakeClock())
    assert tray.cargo_present is False
    assert tray.snapshot()["simulation"] is True


def test_place_while_parked_at_station_loads_the_tray():
    clock = FakeClock()
    tray = CargoTray(STATION, clock=clock)
    tray.observe_odom(odom(0.43, 0.02))
    event = tray.report_place(source="test")
    assert event["accepted"] is True
    assert tray.cargo_present is True


def test_place_without_any_pose_is_refused_never_assumed():
    tray = CargoTray(STATION, clock=FakeClock())
    event = tray.report_place(source="test")
    assert event == {**event, "accepted": False, "reason": "robot_pose_unknown"}
    assert tray.cargo_present is False


def test_place_with_stale_pose_is_refused():
    clock = FakeClock()
    tray = CargoTray(STATION, clock=clock, pose_max_age_s=1.0)
    tray.observe_odom(odom(0.40, 0.0))
    clock.now += 1.5
    event = tray.report_place(source="test")
    assert (event["accepted"], event["reason"]) == (False, "robot_pose_unknown")
    assert tray.cargo_present is False


def test_place_away_from_station_is_refused():
    tray = CargoTray(STATION, clock=FakeClock())
    tray.observe_odom(odom(0.0, 0.0))
    event = tray.report_place(source="test")
    assert (event["accepted"], event["reason"]) == (False, "robot_not_at_station")
    assert tray.cargo_present is False


def test_place_while_robot_still_moving_is_refused():
    tray = CargoTray(STATION, clock=FakeClock())
    tray.observe_odom(odom(0.40, 0.0, vx=0.08))
    event = tray.report_place(source="test")
    assert (event["accepted"], event["reason"]) == (False, "robot_not_parked")


def test_malformed_odometry_is_unknown_not_a_pose():
    assert parked_pose({"pose": {"pose": {"position": {"x": "a"}}}}) is None
    assert parked_pose({"pose": {"pose": {"position": {"x": 0.4, "y": 0.0}}}}) is None
    assert parked_pose({}) is None
    tray = CargoTray(STATION, clock=FakeClock())
    tray.observe_odom({"pose": {}})
    assert tray.report_place(source="test")["reason"] == "robot_pose_unknown"


def test_cargo_stays_with_the_robot_and_only_unload_clears_it():
    clock = FakeClock()
    tray = CargoTray(STATION, clock=clock)
    tray.observe_odom(odom(0.40, 0.0))
    tray.report_place(source="test")
    tray.observe_odom(odom(-1.5, 0.3, vx=0.2))
    clock.now += 30
    assert tray.cargo_present is True
    tray.unload(source="test")
    assert tray.cargo_present is False


# --- the arm side signal -------------------------------------------------


def test_arm_watcher_reports_only_a_rising_edge_into_the_tray():
    readings = iter(
        [
            {"inside": False},
            {"inside": False},
            {"inside": True},
            {"inside": True},
            {"inside": False},
            {"inside": True},
        ]
    )
    watcher = ArmTruthWatcher(fetch=lambda: next(readings))
    edges = [watcher.poll() for _ in range(6)]
    assert edges == [False, False, True, False, False, True]


def test_arm_watcher_never_counts_a_block_already_inside_at_start():
    watcher = ArmTruthWatcher(fetch=lambda: {"inside": True})
    assert [watcher.poll(), watcher.poll()] == [False, False]


def test_arm_watcher_treats_unreachable_or_unknown_as_no_signal():
    readings = iter([{"inside": False}, OSError("down"), {"held": True}, {"inside": "yes"}])

    def fetch():
        item = next(readings)
        if isinstance(item, Exception):
            raise item
        return item

    watcher = ArmTruthWatcher(fetch=fetch)
    assert [watcher.poll() for _ in range(4)] == [False, False, False, False]
    assert watcher.last_error is not None


def test_arm_watcher_unknown_reading_does_not_reset_the_baseline():
    readings = iter([{"inside": False}, OSError("down"), {"inside": True}])

    def fetch():
        item = next(readings)
        if isinstance(item, Exception):
            raise item
        return item

    watcher = ArmTruthWatcher(fetch=fetch)
    assert [watcher.poll() for _ in range(3)] == [False, False, True]


# --- rosbridge -----------------------------------------------------------


class FakeWebSocket:
    """Records what the bridge sends; the test pushes what rosbridge would."""

    def __init__(self) -> None:
        self.sent: list[dict] = []
        self.inbox: queue.Queue[str] = queue.Queue()
        self.closed = False

    def send(self, raw: str) -> None:
        self.sent.append(json.loads(raw))

    def recv(self, timeout: float | None = None) -> str:
        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty as error:
            raise TimeoutError from error

    def close(self) -> None:
        self.closed = True

    def push(self, payload: dict) -> None:
        self.inbox.put(json.dumps(payload))


def test_link_advertises_the_cargo_topic_latched_as_std_msgs_bool():
    ws = FakeWebSocket()
    link = RosbridgeLink("ws://fake", connection_factory=lambda url: ws)
    link.connect()
    link.advertise("/flyto/cargo_present", BOOL_TYPE, latch=True)
    link.publish("/flyto/cargo_present", {"data": True})
    link.close()
    advertise, publish = ws.sent[0], ws.sent[1]
    assert advertise == {
        "op": "advertise",
        "topic": "/flyto/cargo_present",
        "type": "std_msgs/msg/Bool",
        "latch": True,
    }
    assert publish == {"op": "publish", "topic": "/flyto/cargo_present", "msg": {"data": True}}


def test_link_routes_subscribed_messages_to_their_handler():
    ws = FakeWebSocket()
    got: list[dict] = []
    link = RosbridgeLink("ws://fake", connection_factory=lambda url: ws)
    link.subscribe("/odom", "nav_msgs/msg/Odometry", got.append)
    link.connect()
    ws.push({"op": "publish", "topic": "/odom", "msg": odom(1.0, 2.0)})
    ws.push({"op": "publish", "topic": "/other", "msg": {}})
    deadline = time.monotonic() + 2
    while not got and time.monotonic() < deadline:
        time.sleep(0.01)
    link.close()
    assert got == [odom(1.0, 2.0)]
    assert any(m.get("op") == "subscribe" and m["topic"] == "/odom" for m in ws.sent)


def test_link_re_advertises_and_resubscribes_after_reconnect():
    sockets = [FakeWebSocket(), FakeWebSocket()]
    link = RosbridgeLink("ws://fake", connection_factory=lambda url: sockets.pop(0))
    link.subscribe("/odom", "nav_msgs/msg/Odometry", lambda m: None)
    link.connect()
    link.advertise("/flyto/cargo_present", BOOL_TYPE, latch=True)
    second = sockets[0]
    link.connect()
    ops = [(m["op"], m["topic"]) for m in second.sent]
    link.close()
    assert ("subscribe", "/odom") in ops
    assert ("advertise", "/flyto/cargo_present") in ops


# --- the bridge ----------------------------------------------------------


def _bridge(ws: FakeWebSocket, *, arm=None, topic: str = "/flyto/cargo_present"):
    clock = FakeClock()
    tray = CargoTray(STATION, clock=clock)
    link = RosbridgeLink("ws://fake", connection_factory=lambda url: ws)
    return CargoBridge(link=link, tray=tray, topic=topic, arm=arm), tray, clock


def _published(ws: FakeWebSocket, topic: str) -> list[bool]:
    return [m["msg"]["data"] for m in ws.sent if m["op"] == "publish" and m["topic"] == topic]


def test_bridge_publishes_current_state_on_the_configured_topic(monkeypatch):
    ws = FakeWebSocket()
    bridge, tray, _ = _bridge(ws, topic="/sim/cargo")
    bridge.start()
    bridge.tick()
    tray.observe_odom(odom(0.40, 0.0))
    tray.report_place(source="test")
    bridge.tick()
    bridge.stop()
    assert _published(ws, "/sim/cargo") == [False, True]


def test_topic_default_comes_from_env(monkeypatch):
    from sim.fetch_return import cargo_bridge

    monkeypatch.delenv("FLYTO_ROS2_CARGO_TOPIC", raising=False)
    assert cargo_bridge.configured_topic() is None
    monkeypatch.setenv("FLYTO_ROS2_CARGO_TOPIC", "/robot/tray")
    assert cargo_bridge.configured_topic() == "/robot/tray"


def test_bridge_odom_and_arm_edge_load_the_tray_only_when_parked():
    ws = FakeWebSocket()
    readings = iter([{"inside": False}, {"inside": True}])
    arm = ArmTruthWatcher(fetch=lambda: next(readings))
    bridge, tray, _ = _bridge(ws, arm=arm)
    bridge.start()
    ws.push({"op": "publish", "topic": "/odom", "msg": odom(0.41, 0.01)})
    deadline = time.monotonic() + 2
    while tray.snapshot()["robot_pose"] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    bridge.tick()
    bridge.tick()
    bridge.stop()
    assert _published(ws, "/flyto/cargo_present") == [False, True]
    assert tray.snapshot()["events"][-1]["source"] == "arm_sim_truth"


def test_bridge_arm_edge_away_from_station_is_recorded_and_refused():
    ws = FakeWebSocket()
    readings = iter([{"inside": False}, {"inside": True}])
    arm = ArmTruthWatcher(fetch=lambda: next(readings))
    bridge, tray, _ = _bridge(ws, arm=arm)
    tray.observe_odom(odom(-1.0, 0.0))
    bridge.start()
    bridge.tick()
    bridge.tick()
    bridge.stop()
    assert _published(ws, "/flyto/cargo_present") == [False, False]
    assert tray.snapshot()["events"][-1]["reason"] == "robot_not_at_station"


def test_control_endpoint_reports_place_and_unload_over_loopback_http():
    ws = FakeWebSocket()
    bridge, tray, _ = _bridge(ws)
    tray.observe_odom(odom(0.40, 0.0))
    server = serve_control(bridge, host="127.0.0.1", port=0)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()

    def post(path: str) -> dict:
        req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=b"{}", method="POST")
        with urllib.request.urlopen(req, timeout=2) as response:
            return json.loads(response.read())

    try:
        placed = post("/sim/cargo/place")
        assert placed["event"]["accepted"] is True
        assert placed["state"]["cargo_present"] is True
        assert placed["state"]["simulation"] is True
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/sim/cargo", timeout=2) as r:
            assert json.loads(r.read())["cargo_present"] is True
        assert post("/sim/cargo/unload")["state"]["cargo_present"] is False
    finally:
        server.shutdown()
        server.server_close()


def test_control_endpoint_refuses_a_non_loopback_bind():
    ws = FakeWebSocket()
    bridge, _, _ = _bridge(ws)
    with pytest.raises(ValueError):
        serve_control(bridge, host="0.0.0.0", port=0)


def test_bridge_over_a_real_websocket_fake_rosbridge():
    """The production websocket path against a local server speaking rosbridge v2."""
    serve = pytest.importorskip("websockets.sync.server").serve

    received: list[dict] = []

    def handler(connection):
        for raw in connection:
            message = json.loads(raw)
            received.append(message)
            if message.get("op") == "subscribe" and message["topic"] == "/odom":
                connection.send(
                    json.dumps({"op": "publish", "topic": "/odom", "msg": odom(0.40, 0.0)})
                )

    with serve(handler, "127.0.0.1", 0) as server:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        port = server.socket.getsockname()[1]
        tray = CargoTray(STATION)
        bridge = CargoBridge(
            link=RosbridgeLink(f"ws://127.0.0.1:{port}"), tray=tray, topic="/flyto/cargo_present"
        )
        bridge.start()
        deadline = time.monotonic() + 3
        while tray.snapshot()["robot_pose"] is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert tray.report_place(source="test")["accepted"] is True
        assert bridge.tick() is True
        deadline = time.monotonic() + 3
        while not any(m.get("op") == "publish" for m in received):
            assert time.monotonic() < deadline
            time.sleep(0.02)
        bridge.stop()
        server.shutdown()
    ops = [(m["op"], m.get("topic")) for m in received]
    assert ("advertise", "/flyto/cargo_present") in ops
    assert {"op": "publish", "topic": "/flyto/cargo_present", "msg": {"data": True}} in received
