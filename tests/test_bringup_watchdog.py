from __future__ import annotations

import socket
from pathlib import Path

import pytest

from flyto_robotics.bringup_watchdog import (
    WatchdogTicker,
    evaluate_bringup_watchdog,
    sd_notify,
)

UNIT_PATH = (
    Path(__file__).resolve().parents[1]
    / "deploy"
    / "systemd"
    / "turtlebot3-bringup.service"
)


def _decision(**overrides: object) -> str:
    defaults: dict[str, object] = {
        "ready_sent": False,
        "odom_seen": False,
        "last_odom_age": float("inf"),
        "startup_elapsed": 0.0,
        "startup_grace": 10.0,
        "freshness_window": 5.0,
    }
    defaults.update(overrides)
    return evaluate_bringup_watchdog(**defaults)  # type: ignore[arg-type]


def test_waits_during_startup_grace_before_any_odom() -> None:
    assert _decision(startup_elapsed=9.9) == "wait"


def test_arms_on_first_odom_even_immediately() -> None:
    assert _decision(odom_seen=True, last_odom_age=0.0, startup_elapsed=0.1) == "arm"


def test_arms_at_grace_expiry_without_odom_so_the_timer_can_starve() -> None:
    # A bringup that never produces /odom must still be handed to systemd's
    # watchdog timer; "wait" forever would recreate the silent-hang blind spot.
    assert _decision(startup_elapsed=10.0) == "arm"


def test_pings_while_fresh_and_starves_when_stale() -> None:
    assert _decision(ready_sent=True, odom_seen=True, last_odom_age=4.9) == "ping"
    assert _decision(ready_sent=True, odom_seen=True, last_odom_age=5.1) == "starve"


def test_starves_after_arming_without_any_odom() -> None:
    assert _decision(ready_sent=True) == "starve"


class _Recorder:
    def __init__(self) -> None:
        self.notifications: list[str] = []
        self.log_lines: list[str] = []

    def notify(self, message: str) -> bool:
        self.notifications.append(message)
        return True

    def log(self, line: str) -> None:
        self.log_lines.append(line)


def _ticker(recorder: _Recorder) -> WatchdogTicker:
    return WatchdogTicker(
        started_at=100.0,
        notify=recorder.notify,
        log=recorder.log,
        startup_grace=10.0,
        freshness_window=5.0,
    )


def test_ticker_full_lifecycle_matches_the_2026_08_07_hang() -> None:
    recorder = _Recorder()
    ticker = _ticker(recorder)

    # Cold start: ~8s before the first odometry message, no notifications yet.
    assert ticker.tick(101.0) == "wait"
    assert ticker.tick(108.0) == "wait"
    assert recorder.notifications == []

    # First /odom: READY and the first ping go out together.
    ticker.record_odom(108.5)
    assert ticker.tick(109.0) == "arm"
    assert recorder.notifications == ["READY=1\nWATCHDOG=1"]

    # Healthy: fresh odom, one ping per tick.
    ticker.record_odom(109.8)
    assert ticker.tick(110.0) == "ping"
    assert recorder.notifications[-1] == "WATCHDOG=1"

    # The hang: process alive, /odom silent. Pings stop; nothing else is sent.
    sent_before_hang = len(recorder.notifications)
    assert ticker.tick(115.0) == "starve"
    assert ticker.tick(116.0) == "starve"
    assert len(recorder.notifications) == sent_before_hang
    assert any("withholding watchdog pings" in line for line in recorder.log_lines)

    # Recovery within WatchdogSec resumes pinging without re-sending READY.
    ticker.record_odom(116.5)
    assert ticker.tick(117.0) == "ping"
    assert recorder.notifications[-1] == "WATCHDOG=1"
    assert recorder.notifications.count("READY=1\nWATCHDOG=1") == 1
    assert any("fresh again" in line for line in recorder.log_lines)


def test_ticker_arms_without_odom_after_grace_then_starves() -> None:
    recorder = _Recorder()
    ticker = _ticker(recorder)
    assert ticker.tick(110.0) == "arm"
    assert recorder.notifications == ["READY=1\nWATCHDOG=1"]
    assert ticker.tick(111.0) == "starve"
    assert recorder.notifications == ["READY=1\nWATCHDOG=1"]


def test_ticker_starve_and_recovery_log_only_on_transitions() -> None:
    recorder = _Recorder()
    ticker = _ticker(recorder)
    ticker.record_odom(100.5)
    ticker.tick(101.0)
    ticker.tick(110.0)
    ticker.tick(111.0)
    stale_lines = [line for line in recorder.log_lines if "withholding" in line]
    assert len(stale_lines) == 1


def test_sd_notify_writes_datagram_to_unix_socket(tmp_path, monkeypatch) -> None:
    # Bind by relative path: AF_UNIX paths are capped at ~104 bytes on macOS
    # and pytest's tmp_path can exceed that on its own.
    monkeypatch.chdir(tmp_path)
    socket_path = "notify.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(socket_path)
    server.settimeout(2.0)
    try:
        assert sd_notify("READY=1\nWATCHDOG=1", socket_path=socket_path) is True
        assert server.recv(1024) == b"READY=1\nWATCHDOG=1"
    finally:
        server.close()


@pytest.mark.parametrize("socket_path", [None, ""])
def test_sd_notify_is_a_no_op_outside_systemd(socket_path) -> None:
    assert sd_notify("WATCHDOG=1", socket_path=socket_path) is False


def test_sd_notify_reports_failure_for_missing_socket(tmp_path) -> None:
    assert sd_notify("WATCHDOG=1", socket_path=str(tmp_path / "gone.sock")) is False


# --------------------------------------------------------------------------
# turtlebot3-bringup.service unit contract
#
# These assert the *parsed* unit, not its text. A directive only takes effect
# in the section systemd reads it from — StartLimitBurst= in [Service] is
# silently ignored, WatchdogSec= in [Unit] likewise — so a substring check for
# "StartLimitBurst=3" would happily pass on a unit that does nothing. Everything
# below therefore goes through _parse_unit and asserts section + key + value.
#
# configparser is deliberately not used: systemd allows a key to repeat within a
# section with cumulative meaning (two ExecStartPre=, four Environment= here),
# and configparser collapses repeats to the last one.
# --------------------------------------------------------------------------


def _parse_unit(text: str) -> dict[str, list[tuple[str, str]]]:
    """Parse systemd unit text into {section: [(key, value), ...]}.

    Repeated keys are preserved in file order. Comments are whole-line only,
    matching systemd: there is no trailing-comment syntax, and the Exec* lines
    carry shell that must survive intact. Only the first '=' splits a line, so
    an Environment=KEY=VALUE pair keeps its own '='. A trailing backslash
    continues a directive onto the next line.
    """
    sections: dict[str, list[tuple[str, str]]] = {}
    current: str | None = None
    pending = ""
    for raw in text.splitlines():
        line = raw.strip()
        if not pending and (not line or line.startswith(("#", ";"))):
            continue
        if not pending and line.startswith("[") and line.endswith("]"):
            current = line[1:-1]
            sections.setdefault(current, [])
            continue
        line = pending + line
        pending = ""
        if line.endswith("\\"):
            pending = line[:-1]
            continue
        if current is None or "=" not in line:
            continue
        key, _, value = line.partition("=")
        sections[current].append((key.strip(), value.strip()))
    return sections


@pytest.fixture(scope="module")
def unit() -> dict[str, list[tuple[str, str]]]:
    return _parse_unit(UNIT_PATH.read_text())


def _values(unit: dict[str, list[tuple[str, str]]], section: str, key: str) -> list[str]:
    return [value for name, value in unit.get(section, []) if name == key]


def _only(unit: dict[str, list[tuple[str, str]]], section: str, key: str) -> str:
    values = _values(unit, section, key)
    assert len(values) == 1, f"expected exactly one {section}/{key}, got {values}"
    return values[0]


def test_unit_parser_keeps_sections_repeats_and_values_with_equals_signs() -> None:
    # Guards the assertions below: if the parser silently dropped repeats or
    # split on the wrong '=', every contract test would pass vacuously.
    parsed = _parse_unit(
        "# comment\n"
        "[Unit]\n"
        "StartLimitBurst=3\n"
        "\n"
        "[Service]\n"
        "; also a comment\n"
        "Environment=A=1\n"
        "Environment=B=2\n"
        "ExecStart=/bin/bash -lc 'x \\\n"
        "y'\n"
    )
    assert parsed["Unit"] == [("StartLimitBurst", "3")]
    assert _values(parsed, "Service", "Environment") == ["A=1", "B=2"]
    assert _values(parsed, "Service", "ExecStart") == ["/bin/bash -lc 'x y'"]
    assert "StartLimitBurst" not in dict(parsed["Service"])


def test_start_rate_limit_is_in_unit_section_and_parks_after_three_starts(unit) -> None:
    # The NRestarts=193 finding: turtlebot3_node kept failing the OpenCR /
    # Dynamixel motor-bus handshake, supervised shutdown took the healthy LDS
    # lidar down with it (the flapping /scan an operator actually saw), and
    # Restart=always retried it forever. Burst 20 never tripped because one
    # failure cycle is far too long for 20 of them to fit in 300s. Burst 3 fits.
    assert _only(unit, "Unit", "StartLimitIntervalSec") == "300"
    assert _only(unit, "Unit", "StartLimitBurst") == "3"


def test_start_rate_limit_is_not_placed_in_the_service_section(unit) -> None:
    # systemd reads both keys from [Unit] only; in [Service] they are accepted
    # and ignored, which would silently restore the unbounded retry loop.
    service_keys = {key for key, _ in unit["Service"]}
    assert "StartLimitIntervalSec" not in service_keys
    assert "StartLimitBurst" not in service_keys


def test_notify_watchdog_contract_is_intact_in_the_service_section(unit) -> None:
    # The silent-hang recovery path, verified on the robot 2026-08-07. Bounding
    # the restart rate must not weaken it.
    assert _only(unit, "Service", "Type") == "notify"
    assert _only(unit, "Service", "NotifyAccess") == "all"
    assert _only(unit, "Service", "WatchdogSec") == "15"


def test_restart_behaviour_is_retained(unit) -> None:
    assert _only(unit, "Service", "Restart") == "always"
    assert _only(unit, "Service", "RestartSec") == "5"


def test_shutdown_signalling_is_retained(unit) -> None:
    # SIGINT so rclpy shuts the launch group down cleanly; 20s before systemd
    # escalates to SIGKILL.
    assert _only(unit, "Service", "KillSignal") == "SIGINT"
    assert _only(unit, "Service", "TimeoutStopSec") == "20"


def test_serial_device_waits_run_before_launch_in_order(unit) -> None:
    pre = _values(unit, "Service", "ExecStartPre")
    assert len(pre) == 2
    assert "/dev/ttyACM0" in pre[0] and "/dev/tb3_lidar" in pre[0]
    assert pre[1].endswith("/bin/sleep 2")
    # The device wait can legitimately burn ~62s, so the start timeout must stay
    # above the default 90s or a slow cold boot fails before it ever tries.
    assert int(_only(unit, "Service", "TimeoutStartSec")) >= 120


def test_exec_start_uses_the_supervised_whole_group_launch(unit) -> None:
    # Whole-group supervision is what turns "one node died" into a unit failure
    # systemd can count. Without it the restart limiter has nothing to count and
    # the unit sits `active` with a dead turtlebot3_node.
    exec_start = _only(unit, "Service", "ExecStart")
    assert "turtlebot3_bringup_supervised.launch.py" in exec_start
    assert "robot.launch.py" not in exec_start
