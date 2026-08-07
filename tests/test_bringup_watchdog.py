from __future__ import annotations

import socket

import pytest

from flyto_robotics.bringup_watchdog import (
    WatchdogTicker,
    evaluate_bringup_watchdog,
    sd_notify,
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
