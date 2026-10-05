"""The managed SSH forward: configuration, refusals, supervision, teardown.

No network and no ssh binary: the process, the port probe, name resolution
and the back-off are fakes handed to the tunnel as its seams.
"""

from __future__ import annotations

import threading
import time

import pytest

from flyto_robotics import ssh_transport as st


def _env(**extra: str) -> dict[str, str]:
    return {"FLYTO_ROS2_SSH_HOST": "ubuntu@flyto-robot.local", **extra}


def _wait_for(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition not reached")
        time.sleep(0.005)


class FakeProcess:
    def __init__(self, argv: list[str], on_exit, stderr: str = "") -> None:
        self.argv = argv
        self._on_exit = on_exit
        self._alive = True
        self._stderr = stderr
        self.stopped = False

    def alive(self) -> bool:
        return self._alive

    def stop(self) -> None:
        self.stopped = True
        self._alive = False

    def error_text(self) -> str:
        return self._stderr

    def exit(self, stderr: str = "") -> None:
        """The robot rebooted, Wi-Fi dropped: ssh exits on its own."""
        self._stderr = stderr
        self._alive = False
        self._on_exit()


class FakeSsh:
    """Seams for one tunnel. ``script`` is what each next ssh does at once."""

    def __init__(self) -> None:
        self.processes: list[FakeProcess] = []
        self.script: list[str | None] = []  # None: stays up; text: exits with it
        self.resolved: list[str] = []
        self.resolve_error: Exception | None = None
        self.delays: list[int] = []
        self.reaped: list[str] = []
        self.next_port = 40000
        self.lock = threading.Lock()

    def spawn(self, argv, on_exit):
        process = FakeProcess(argv, on_exit)
        with self.lock:
            self.processes.append(process)
            exits = self.script.pop(0) if self.script else None
        if exits is not None:
            process.exit(exits)
        return process

    def probe(self, _port: int) -> bool:
        with self.lock:
            return bool(self.processes) and self.processes[-1].alive()

    def resolve(self, hostname: str) -> str:
        self.resolved.append(hostname)
        if self.resolve_error is not None:
            raise self.resolve_error
        return "192.0.2.34"

    def free_port(self) -> int:
        self.next_port += 1
        return self.next_port

    def backoff(self, attempts: int) -> float:
        self.delays.append(attempts)
        return 0.001

    def seams(self) -> st.Seams:
        return st.Seams(
            spawn=self.spawn,
            probe=self.probe,
            resolve=self.resolve,
            free_port=self.free_port,
            reap=lambda path, _host: self.reaped.append(path),
            control_path=lambda _config: "/nonexistent/flyto-test.sock",
            backoff=self.backoff,
            poll_s=0.001,
        )

    @property
    def last(self) -> FakeProcess:
        return self.processes[-1]


def _tunnel(fake: FakeSsh, **env: str) -> st.ManagedTunnel:
    config = st.config_from_env(_env(**env))
    assert config is not None
    return st.ManagedTunnel(config, fake.seams())


@pytest.fixture
def fake():
    return FakeSsh()


# -- configuration and refusals -------------------------------------------------------


def test_no_host_means_no_managed_transport():
    assert st.config_from_env({}) is None


def test_rosbridge_is_always_forwarded_and_camera_ports_are_data():
    config = st.config_from_env(
        _env(FLYTO_ROS2_SSH_FORWARDS="camera=8080,evidence=127.0.0.1:9000")
    )
    assert config is not None
    assert {item.name: item.remote_port for item in config.forwards} == {
        "camera": 8080,
        "evidence": 9000,
        "rosbridge": 9090,
    }
    assert all(item.local_port == 0 for item in config.forwards)


@pytest.mark.parametrize(
    "env",
    [
        {"FLYTO_ROS2_SSH_FORWARDS": "camera=192.168.0.5:8080"},
        {"FLYTO_ROS2_SSH_FORWARDS": "rosbridge=0.0.0.0:9090"},
        {"FLYTO_ROS2_SSH_LOCAL_PORTS": "rosbridge=0.0.0.0:19090"},
        {"FLYTO_ROS2_SSH_LOCAL_PORTS": "rosbridge=192.168.0.2:19090"},
        {"FLYTO_ROS2_SSH_LOCAL_PORTS": "camera=8080"},
        {"FLYTO_ROS2_SSH_FORWARDS": "rosbridge=99999"},
        {"FLYTO_ROS2_SSH_FORWARDS": "camera"},
        {"FLYTO_ROS2_SSH_HOST": "-oProxyCommand=touch /tmp/x"},
        {"FLYTO_ROS2_SSH_HOST": "ubuntu@robot; rm -rf /"},
        {"FLYTO_ROS2_SSH_IDENTITY": "/nonexistent/key"},
    ],
)
def test_non_loopback_binds_and_unsafe_hosts_are_refused(env):
    with pytest.raises(st.TransportConfigError):
        st.config_from_env(_env(**env))


def test_an_explicit_rosbridge_url_must_be_the_derived_one():
    with pytest.raises(st.TransportConfigError, match="contradicts"):
        st.config_from_env(_env(FLYTO_ROSBRIDGE_URL="ws://127.0.0.1:19090"))
    pinned = st.config_from_env(
        _env(
            FLYTO_ROSBRIDGE_URL="ws://127.0.0.1:19090",
            FLYTO_ROS2_SSH_LOCAL_PORTS="rosbridge=19090",
        )
    )
    assert pinned is not None and pinned.forward("rosbridge").local_port == 19090


def test_the_command_line_is_key_only_strict_loopback_and_runs_nothing():
    config = st.config_from_env(_env(FLYTO_ROS2_SSH_FORWARDS="camera=8080"))
    assert config is not None
    argv = st.ssh_argv(config, {"rosbridge": 41000, "camera": 41001}, "/run/x.sock")
    joined = " ".join(argv)
    assert argv[:3] == ["ssh", "-N", "-T"]
    for option in (
        "BatchMode=yes",
        "StrictHostKeyChecking=yes",
        "PasswordAuthentication=no",
        "ExitOnForwardFailure=yes",
        "ControlMaster=yes",
        "ControlPath=/run/x.sock",
    ):
        assert option in argv
    assert "accept-new" not in joined
    assert "-L" in argv and "127.0.0.1:41000:127.0.0.1:9090" in argv
    assert "127.0.0.1:41001:127.0.0.1:8080" in argv
    assert "-R" not in argv and "-D" not in argv
    # The host is the last word, after "--": no remote command can follow.
    assert argv[-2:] == ["--", "ubuntu@flyto-robot.local"]


def test_an_unknown_host_key_fails_with_how_to_trust_it_once():
    config = st.config_from_env(_env())
    assert config is not None
    code, fatal, text = st.classify(
        "No ED25519 host key is known for flyto-robot.local and you have requested "
        "strict checking.\nHost key verification failed.",
        config,
    )
    assert (code, fatal) == ("host_key_unknown", True)
    assert "ssh-keyscan -t ed25519 flyto-robot.local" in text
    assert "known_hosts" in text
    changed = st.classify("@ WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED! @", config)
    assert changed[:2] == ("host_key_changed", True) and "ssh-keygen -R" in changed[2]
    assert st.classify("Permission denied (publickey).", config)[:2] == ("auth_refused", True)
    assert st.classify("connect to host x port 22: Connection refused", config)[1] is False


def test_the_backoff_is_exponential_capped_and_jittered():
    full = [st.backoff_delay(n, lambda: 1.0) for n in range(1, 9)]
    assert full == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0, 30.0]
    assert st.backoff_delay(3, lambda: 0.0) == 2.0  # never below half


# -- supervision ----------------------------------------------------------------------


def test_the_forward_connects_and_reports_it(fake):
    tunnel = _tunnel(fake)
    try:
        assert tunnel.wait_connected(2.0)
        status = tunnel.status()
        assert status["state"] == "connected"
        assert status["attempts"] == 0
        assert status["resolved_address"] == "192.0.2.34"
        local = tunnel.local_port("rosbridge")
        assert tunnel.url() == f"ws://127.0.0.1:{local}"
        assert status["forwards"]["rosbridge"] == {
            "local": f"127.0.0.1:{local}",
            "remote": "127.0.0.1:9090",
        }
        assert status["since"].endswith("+00:00")
    finally:
        tunnel.close()


def test_an_exit_reconnects_with_backoff_and_resolves_the_host_again(fake):
    fake.script = [None, "ssh: connect to host flyto-robot.local port 22: Connection refused",
                   "ssh: connect to host flyto-robot.local port 22: Connection refused", None]
    states: list[str] = []
    tunnel = _tunnel(fake)
    tunnel.add_listener(lambda status: states.append(status["state"]))
    try:
        assert tunnel.wait_connected(2.0)
        first = fake.last
        first.exit("Timeout, server flyto-robot.local not responding.")  # robot rebooted
        _wait_for(lambda: len(fake.processes) == 4 and tunnel.connected)
        assert first is not fake.last
        # One back-off per failed attempt, growing: 1, 2, then 3 failures in a row.
        assert fake.delays == [1, 2, 3]
        assert fake.resolved == ["flyto-robot.local"] * 4
        assert tunnel.status()["attempts"] == 0
        assert "reconnecting" in states and states[-1] == "connected"
    finally:
        tunnel.close()


def test_an_unresolvable_host_is_retried_not_fatal(fake):
    fake.resolve_error = OSError("nodename nor servname provided")
    tunnel = _tunnel(fake)
    try:
        _wait_for(lambda: len(fake.delays) >= 2)
        status = tunnel.status()
        assert status["state"] == "reconnecting"
        assert status["error_code"] == "unresolved"
        assert not fake.processes  # ssh is not started for a name that does not resolve
        fake.resolve_error = None
        assert tunnel.wait_connected(2.0)
    finally:
        tunnel.close()


def test_an_untrusted_host_key_is_failed_and_waits_for_the_operator(fake):
    fake.script = ["Host key verification failed."]
    tunnel = _tunnel(fake)
    try:
        _wait_for(lambda: tunnel.status()["state"] == "failed")
        time.sleep(0.05)
        assert len(fake.processes) == 1  # never retried on its own
        assert fake.delays == []
        assert "ssh-keyscan" in tunnel.status()["last_error"]
        tunnel.refresh()  # the operator trusted the key and pressed reconnect
        assert tunnel.wait_connected(2.0)
        assert len(fake.processes) == 2
    finally:
        tunnel.close()


def test_a_busy_ephemeral_port_is_picked_again(fake):
    fake.script = ["bind [127.0.0.1]:40001: Address already in use\n"
                   "channel_setup_fwd_listener_tcpip: cannot listen to port: 40001"]
    tunnel = _tunnel(fake)
    try:
        assert tunnel.wait_connected(2.0)
        busy, fresh = fake.processes[0].argv, fake.last.argv
        assert busy != fresh
        assert f"127.0.0.1:{tunnel.local_port('rosbridge')}:127.0.0.1:9090" in fresh
        assert tunnel.url() == f"ws://127.0.0.1:{tunnel.local_port('rosbridge')}"
    finally:
        tunnel.close()


def test_refresh_tears_the_forward_down_and_builds_it_again(fake):
    tunnel = _tunnel(fake)
    try:
        assert tunnel.wait_connected(2.0)
        first = fake.last
        tunnel.refresh()
        assert tunnel.status()["state"] == "reconnecting"  # at once: calls fail fast
        _wait_for(lambda: len(fake.processes) == 2 and tunnel.connected)
        assert first.stopped
        assert fake.resolved == ["flyto-robot.local"] * 2
    finally:
        tunnel.close()


def test_close_stops_ssh_and_reports_stopped(fake):
    tunnel = _tunnel(fake)
    assert tunnel.wait_connected(2.0)
    process = fake.last
    tunnel.close()
    assert process.stopped
    assert tunnel.status()["state"] == "stopped"


def test_the_last_holder_tears_it_down_after_the_linger(fake):
    config = st.config_from_env(_env(FLYTO_ROS2_SSH_LINGER_S="0.05"))
    first = st.acquire(config, seams=fake.seams())
    second = st.acquire(config)
    try:
        assert first is second
        assert first.wait_connected(2.0)
        first.release()
        time.sleep(0.1)
        assert st.current(config) is first and not fake.last.stopped  # still held
        again = st.acquire(config)  # re-held within the linger: the same forward
        second.release()
        time.sleep(0.1)
        assert again is first and not fake.last.stopped
        again.release()
        _wait_for(lambda: fake.last.stopped)
        assert st.current(config) is None
    finally:
        st.close_all()


def test_call_gate_pauses_and_reports_actuating_calls():
    gate = st.CallGate()
    assert gate.begin("move", actuating=True)
    assert gate.begin("look", actuating=False)
    assert gate.pause() == ["move"]
    assert not gate.begin("another", actuating=True)
    gate.resume()
    gate.end("move")
    assert gate.pause() == []
