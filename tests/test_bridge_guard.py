from __future__ import annotations

import inspect
import signal

import pytest

from flyto_robotics.bridge_guard import (
    ProcessGroupSupervisor,
    StartupSignalGuard,
    TerminationSignalInbox,
    infer_expected_markers,
    run,
    shell_exit_code,
)


class FakeProcess:
    def __init__(self, *, pid: int = 321, returncode: int | None = None) -> None:
        self.pid = pid
        self.returncode = returncode
        self.wait_calls = 0

    def poll(self) -> int | None:
        return self.returncode

    def wait(self) -> int:
        self.wait_calls += 1
        if self.returncode is None:
            self.returncode = -signal.SIGKILL
        return self.returncode


def make_guard(
    *, expected_markers: int = 5, deadline: float = 8.0, quiet: float = 0.35
) -> tuple[StartupSignalGuard, list[tuple[int, str]], list[str]]:
    forwarded: list[tuple[int, str]] = []
    forced: list[str] = []
    guard = StartupSignalGuard(
        expected_markers=expected_markers,
        started_at=10.0,
        startup_deadline_seconds=deadline,
        quiet_period_seconds=quiet,
        forward_signal=lambda signum, reason: forwarded.append((signum, reason)),
        force_stop=lambda: forced.append("forced"),
    )
    return guard, forwarded, forced


def test_infer_expected_markers_stops_at_ros_arguments() -> None:
    assert infer_expected_markers(
        [
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "--ros-args",
            "-r",
            "__node:=bridge",
        ]
    ) == 2
    assert infer_expected_markers(["--ros-args", "-p", "config_file:=bridge.yaml"]) == 0


def test_first_signal_is_buffered_until_every_explicit_bridge_is_ready() -> None:
    guard, forwarded, forced = make_guard(expected_markers=2)
    assert guard.handle_signal(signal.SIGINT, 10.1) == "buffered"
    guard.observe_output("Creating GZ_TO_ROS Bridge\n", 10.2)
    assert forwarded == []
    guard.observe_output("Creating ROS_TO_GZ Bridge\n", 10.3)
    assert forwarded == []
    guard.tick(10.64)
    assert forwarded == []
    guard.tick(10.66)
    assert forwarded == [(signal.SIGINT, "ready")]
    guard.tick(20.0)
    assert forwarded == [(signal.SIGINT, "ready")]
    assert forced == []


def test_config_driven_bridge_waits_for_quiet_period() -> None:
    guard, forwarded, _ = make_guard(expected_markers=0, quiet=0.35)
    guard.observe_output("Creating GZ_TO_ROS Bridge\n", 10.1)
    assert guard.handle_signal(signal.SIGTERM, 10.2) == "buffered"
    guard.tick(10.44)
    assert forwarded == []
    guard.tick(10.46)
    assert forwarded == [(signal.SIGTERM, "ready")]


def test_pending_signal_is_forwarded_at_bounded_startup_deadline() -> None:
    guard, forwarded, _ = make_guard(expected_markers=5, deadline=2.0)
    assert guard.handle_signal(signal.SIGTERM, 10.5) == "buffered"
    guard.tick(11.99)
    assert forwarded == []
    guard.tick(12.0)
    assert forwarded == [(signal.SIGTERM, "startup-deadline")]


def test_repeated_signal_forces_child_stop_without_second_forward() -> None:
    guard, forwarded, forced = make_guard(expected_markers=5)
    assert guard.handle_signal(signal.SIGINT, 10.1) == "buffered"
    assert guard.handle_signal(signal.SIGTERM, 10.2) == "forced"
    assert forwarded == []
    assert forced == ["forced"]


def test_termination_signal_inbox_defers_and_preserves_signal_order() -> None:
    inbox = TerminationSignalInbox()
    inbox.record(signal.SIGINT)
    inbox.record(signal.SIGTERM)

    assert inbox.drain() == (signal.SIGINT, signal.SIGTERM)
    assert inbox.drain() == ()


def test_process_group_supervisor_forwards_then_forces_at_grace_deadline() -> None:
    process = FakeProcess()
    now = [10.0]
    sent: list[tuple[int, int]] = []
    supervisor = ProcessGroupSupervisor(  # type: ignore[arg-type]
        process,
        shutdown_grace_seconds=2.0,
        monotonic=lambda: now[0],
        kill_group=lambda pid, signum: sent.append((pid, signum)),
    )

    supervisor.forward(signal.SIGTERM)
    now[0] = 11.99
    assert supervisor.tick() is False
    now[0] = 12.0
    assert supervisor.tick() is True
    assert supervisor.tick() is False
    assert sent == [(321, signal.SIGTERM), (321, signal.SIGKILL)]


def test_process_group_supervisor_cleanup_forces_and_reaps_once() -> None:
    process = FakeProcess()
    sent: list[tuple[int, int]] = []
    supervisor = ProcessGroupSupervisor(  # type: ignore[arg-type]
        process,
        shutdown_grace_seconds=2.0,
        kill_group=lambda pid, signum: sent.append((pid, signum)),
    )

    supervisor.cleanup()
    assert sent == [(321, signal.SIGKILL)]
    assert process.wait_calls == 1
    assert supervisor.reap() == -signal.SIGKILL
    assert process.wait_calls == 2


def test_os_signal_handler_only_records_for_main_loop_processing() -> None:
    source = inspect.getsource(run)
    handler_source = source[
        source.index("def handle_termination(") :
        source.index("def process_pending_signals(")
    ]

    assert "signal_inbox.record(signum)" in handler_source
    assert "guard.handle_signal" not in handler_source
    assert "print(" not in handler_source


@pytest.mark.parametrize(("return_code", "expected"), [(0, 0), (23, 23), (-11, 139)])
def test_shell_exit_code_preserves_child_outcome(return_code: int, expected: int) -> None:
    assert shell_exit_code(return_code) == expected
