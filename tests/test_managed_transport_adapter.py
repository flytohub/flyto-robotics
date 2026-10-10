"""The adapter owns its SSH forward: fail fast while down, operator refresh.

The forward runs on fake ssh seams (tests/test_ssh_transport.py) and the ROS
side is a fake backend; nothing here opens a socket or moves anything.
"""

from __future__ import annotations

import io
import json
import threading
import time

import pytest
from test_ssh_transport import FakeSsh, _env, _wait_for

from flyto_robotics import adapter_provider as provider
from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics import ssh_transport as st
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
    CallResult,
)
from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    GenericROS2Adapter,
    StandardInterface,
)


class LinkBackend:
    """A ROS link that can drop, and whose stop can be made to fail."""

    def __init__(self) -> None:
        self.interfaces = [
            StandardInterface(*DEFAULT_INTERFACES["motion.navigate"]),
            StandardInterface("topic", "/odom", "nav_msgs/msg/Odometry"),
            StandardInterface("topic", "/scan", "sensor_msgs/msg/LaserScan"),
        ]
        self._connected = True
        self.reconnects = 0
        self.reconnect_error: Exception | None = None
        self.invoked: list[str] = []
        self.stops = 0
        self.stop_outcome = OUTCOME_COMPLETED
        self.release = threading.Event()

    def is_connected(self) -> bool:
        return self._connected

    def discover(self):
        return tuple(self.interfaces)

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        self.invoked.append(call_id)
        return CallResult(call_id, OUTCOME_COMPLETED)

    def cancel(self, call_id):
        return CallResult(call_id, OUTCOME_REFUSED, detail="no active goal")

    def safe_stop(self, call_id):
        self.stops += 1
        if self.stop_outcome == OUTCOME_COMPLETED:
            self.release.set()
        return CallResult(call_id, self.stop_outcome, detail="cmd_vel")

    def execution_count(self, call_id):
        return 0

    def observation(self, required=None):
        return {}

    def disconnect(self):
        self._connected = False

    def reconnect(self):
        # Like the rosbridge backend: the old session goes first.
        self._connected = False
        self.reconnects += 1
        if self.reconnect_error is not None:
            raise self.reconnect_error
        self._connected = True


@pytest.fixture
def link(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    fake = FakeSsh()
    config = st.config_from_env(_env(FLYTO_ROS2_SSH_LINGER_S="0"))
    tunnel = st.acquire(config, seams=fake.seams())
    assert tunnel.wait_connected(2.0)
    backend = LinkBackend()
    device = GenericROS2Adapter(backend=backend, resource_id="robot-1", transport=tunnel)
    yield device, backend, tunnel, fake
    st.close_all()


def _navigate(call_id: str = "go") -> CallRequest:
    return CallRequest(call_id, "motion.navigate", {"x": 1.0, "y": 0.0})


# -- while the forward is down, calls fail fast and typed -----------------------------


def test_a_call_while_the_forward_is_down_is_refused_typed_and_sends_nothing(link):
    device, backend, tunnel, fake = link
    fake.script = ["Host key verification failed."]
    fake.last.exit("Connection reset by peer")
    _wait_for(lambda: tunnel.status()["state"] == "failed")

    started = time.monotonic()
    result = device.invoke(_navigate())

    assert time.monotonic() - started < 0.5
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == "transport_unavailable"
    assert result.evidence["transport"]["state"] == "failed"
    assert result.detail.startswith("transport_unavailable:")
    assert backend.invoked == []
    stop = device.safe_stop()
    assert stop.outcome == OUTCOME_FAILED
    assert stop.evidence["reason_code"] == "transport_unavailable"
    assert device.cancel("go").evidence["reason_code"] == "transport_unavailable"
    assert backend.stops == 0


def test_a_ros_link_the_forward_dropped_is_reopened_on_the_next_call(link):
    device, backend, _tunnel, _fake = link
    backend.disconnect()
    backend.reconnect_error = ConnectionRefusedError("rosbridge not up yet")
    first = device.invoke(CallRequest("look", "places.list", {}))
    assert first.evidence["reason_code"] == "transport_unavailable"
    assert "rosbridge did not answer" in first.detail

    backend.reconnect_error = None
    device.invoke(CallRequest("look-2", "places.list", {}))
    assert backend.reconnects == 2 and backend.is_connected()


def test_status_reports_state_since_attempts_and_last_error(link):
    device, _backend, tunnel, fake = link
    status = device.transport_status()
    assert status["state"] == "connected" and status["rosbridge_ok"] is True
    for key in ("since", "attempts", "last_error", "host", "forwards"):
        assert key in status
    fake.last.exit("ssh: connect to host flyto-robot.local port 22: Connection refused")
    fake.script = ["Connection refused"] * 50
    _wait_for(lambda: tunnel.status()["attempts"] >= 2)
    status = device.transport_status()
    assert status["state"] == "reconnecting"
    assert "refused SSH" in status["last_error"]


# -- the operator refresh ----------------------------------------------------------


def test_reconnect_rebuilds_the_forward_and_rereads_the_robot(link):
    device, backend, _tunnel, fake = link
    before = fake.last

    status = device.reconnect()

    assert before.stopped and fake.last is not before
    assert fake.resolved == ["flyto-robot.local"] * 2
    assert backend.reconnects == 1
    assert status["state"] == "connected"
    assert status["refused"] is False
    assert status["rosbridge_ok"] is True
    assert status["topics_seen"] == 2
    assert status["served_identity"] == {"resource_id": "robot-1", "deployment_mode": "real"}
    assert status["safe_stop"] is None
    for key in ("since", "attempts", "last_error"):
        assert key in status
    # Idempotent: again is the same refresh, not an error.
    again = device.reconnect()
    assert again["state"] == "connected" and len(fake.processes) == 3


def test_reconnect_reports_a_robot_whose_rosbridge_does_not_answer(link):
    device, backend, _tunnel, _fake = link
    backend.reconnect_error = ConnectionRefusedError("rosbridge down")
    status = device.reconnect()
    assert status["state"] == "connected"  # the forward itself is up
    assert status["rosbridge_ok"] is False
    assert "rosbridge down" in status["rosbridge_error"]
    assert status["topics_seen"] is None


def _start_motion(device: GenericROS2Adapter, backend: LinkBackend) -> threading.Thread:
    running = threading.Event()

    def moving(request):
        running.set()
        backend.release.wait(5.0)
        return CallResult(request.call_id, OUTCOME_COMPLETED)

    device._invoke = moving  # the motion pipeline itself is tested elsewhere
    worker = threading.Thread(target=device.invoke, args=(_navigate("moving"),))
    worker.start()
    assert running.wait(2.0)
    return worker


def test_reconnect_during_a_motion_stops_it_first(link):
    device, backend, _tunnel, fake = link
    worker = _start_motion(device, backend)

    status = device.reconnect()
    worker.join(2.0)

    assert backend.stops == 1
    assert status["refused"] is False
    assert status["safe_stop"]["confirmed"] is True
    assert status["safe_stop"]["calls"] == ["moving"]
    assert len(fake.processes) == 2  # refreshed after the stop


def test_reconnect_is_refused_when_the_stop_is_not_confirmed(link):
    device, backend, tunnel, fake = link
    backend.stop_outcome = OUTCOME_FAILED
    worker = _start_motion(device, backend)
    try:
        status = device.reconnect()
    finally:
        backend.release.set()
        worker.join(2.0)

    assert status["refused"] is True
    assert status["reason_code"] == "safe_stop_unconfirmed"
    assert status["safe_stop"]["confirmed"] is False
    assert len(fake.processes) == 1 and tunnel.connected  # the link left as it was
    assert device.transport_status()["accepting_calls"] is True


def test_calls_are_refused_while_a_refresh_drains_the_link(link):
    device, _backend, tunnel, _fake = link
    tunnel.calls.pause()
    try:
        result = device.invoke(_navigate("late"))
    finally:
        tunnel.calls.resume()
    assert result.evidence["reason_code"] == "transport_unavailable"
    assert "being refreshed" in result.detail


# -- built from the environment ----------------------------------------------------


@pytest.fixture
def ssh_env(monkeypatch):
    fake = FakeSsh()
    seams = fake.seams()
    monkeypatch.setattr(st, "Seams", lambda: seams)
    monkeypatch.setenv("FLYTO_ROS2_SSH_HOST", "ubuntu@flyto-robot.local")
    monkeypatch.setenv("FLYTO_ROS2_SSH_LINGER_S", "0")
    monkeypatch.setenv("FLYTO_ROS2_SSH_READY_TIMEOUT_S", "0.2")
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    monkeypatch.delenv("FLYTO_ROS2_TRANSPORT", raising=False)
    monkeypatch.delenv("FLYTO_ROSBRIDGE_URL", raising=False)
    yield fake
    st.close_all()


def test_an_ssh_host_implies_rosbridge_on_the_forwarded_port(ssh_env, monkeypatch):
    urls: list[str] = []

    def connect(url):
        urls.append(url)
        raise ConnectionRefusedError("no rosbridge in tests")

    monkeypatch.setattr(adapter_module.RosbridgeROS2Backend, "_connect", staticmethod(connect))
    with pytest.raises(ConnectionRefusedError):
        adapter_module.build("robot-1")
    tunnel_port = int(ssh_env.processes[0].argv[ssh_env.processes[0].argv.index("-L") + 1]
                      .split(":")[1])
    assert urls == [f"ws://127.0.0.1:{tunnel_port}"]
    # The failed build released its hold: the forward went with it.
    _wait_for(lambda: st.current() is None)


def test_a_build_while_the_forward_is_down_fails_fast_typed(ssh_env):
    ssh_env.resolve_error = OSError("mDNS: no answer")
    device = adapter_module.build("robot-1")
    try:
        assert device.transport is not None and not device.connected
        result = device.invoke(_navigate())
        assert result.evidence["reason_code"] == "transport_unavailable"
        assert result.evidence["transport"]["error_code"] == "unresolved"
    finally:
        device.disconnect()
    _wait_for(lambda: st.current() is None)


def test_ssh_with_the_dds_transport_is_refused(ssh_env, monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_TRANSPORT", "rclpy")
    with pytest.raises(st.TransportConfigError, match="rosbridge"):
        adapter_module.build("robot-1")


# -- the provider's status surface and process protocol ----------------------------


def test_provider_status_without_ssh_with_a_refused_config_and_stopped(monkeypatch):
    monkeypatch.delenv("FLYTO_ROS2_SSH_HOST", raising=False)
    assert provider.transport_status() is None
    monkeypatch.setenv("FLYTO_ROS2_SSH_HOST", "ubuntu@robot")
    monkeypatch.setenv("FLYTO_ROS2_SSH_FORWARDS", "camera=10.0.0.5:8080")
    refused = provider.transport_status()
    assert refused["state"] == "failed" and refused["error_code"] == "config_refused"
    monkeypatch.delenv("FLYTO_ROS2_SSH_FORWARDS")
    assert provider.transport_status()["state"] == "stopped"
    assert provider.discover_resource_manifests.transport_status is provider.transport_status
    assert provider.discover_resource_manifests.reconnect is provider.reconnect_resource


def test_the_process_protocol_serves_reconnect_and_transport_status(link, monkeypatch, capsys):
    device, _backend, _tunnel, _fake = link
    monkeypatch.setattr(provider, "build", lambda _rid: device)
    requests = [
        {"id": 1, "op": "transport_status"},
        {"id": 2, "op": "reconnect"},
        {"id": 3, "op": "close"},
    ]
    monkeypatch.setattr(
        "sys.stdin", io.StringIO("".join(json.dumps(item) + "\n" for item in requests))
    )
    assert provider._serve(provider.ADAPTER_ID, "robot-1") == 0
    replies = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert replies[0]["ok"] and replies[0]["result"]["state"] == "connected"
    assert replies[1]["ok"] and replies[1]["result"]["refused"] is False
    assert replies[1]["result"]["topics_seen"] == 2


def test_reconnect_resource_refuses_another_resource(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "robot-1")
    with pytest.raises(provider.ResourceNotServed):
        provider.reconnect_resource("robot-2")


def test_the_presence_watch_is_woken_and_told_when_the_forward_returns():
    notes: list[str] = []
    watch = provider.PresenceWatch(resource_id="robot-1", notify=notes.append, start=False)
    watch._on_transport({"state": "reconnecting"})
    assert not watch._changed.is_set()
    watch._on_transport({"state": "connected"})
    assert watch._changed.is_set()
    assert notes == ["transport_reconnecting", "transport_connected"]
