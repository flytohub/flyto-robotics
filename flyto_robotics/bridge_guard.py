"""Protect ros_gz_bridge startup from concurrent rclcpp shutdown signals."""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable, Sequence
from contextlib import suppress
from pathlib import Path

READY_MARKER = re.compile(r"Creating .* Bridge")
DEFAULT_STARTUP_DEADLINE_SECONDS = 8.0
DEFAULT_QUIET_PERIOD_SECONDS = 0.35
DEFAULT_SHUTDOWN_GRACE_SECONDS = 5.0


def infer_expected_markers(arguments: Sequence[str]) -> int:
    """Count explicit bridge specifications before the ROS argument boundary."""

    bridge_arguments = []
    for argument in arguments:
        if argument == "--ros-args":
            break
        bridge_arguments.append(argument)
    return sum(
        "@" in argument and ("[" in argument or "]" in argument)
        for argument in bridge_arguments
    )


def shell_exit_code(return_code: int) -> int:
    """Convert a negative subprocess signal return into its shell exit code."""

    return 128 + abs(return_code) if return_code < 0 else return_code


class StartupSignalGuard:
    """Buffer one termination signal until bridge initialization is quiescent."""

    def __init__(
        self,
        *,
        expected_markers: int,
        started_at: float,
        startup_deadline_seconds: float,
        quiet_period_seconds: float,
        forward_signal: Callable[[int, str], None],
        force_stop: Callable[[], None],
    ) -> None:
        if expected_markers < 0:
            raise ValueError("expected_markers cannot be negative")
        if startup_deadline_seconds <= 0:
            raise ValueError("startup_deadline_seconds must be positive")
        if quiet_period_seconds < 0:
            raise ValueError("quiet_period_seconds cannot be negative")
        self.expected_markers = expected_markers
        self.started_at = started_at
        self.startup_deadline_seconds = startup_deadline_seconds
        self.quiet_period_seconds = quiet_period_seconds
        self._forward_signal = forward_signal
        self._force_stop = force_stop
        self._marker_count = 0
        self._last_marker_at: float | None = None
        self._pending_signal: int | None = None
        self._signal_count = 0
        self._forwarded = False
        self._lock = threading.Lock()

    @property
    def marker_count(self) -> int:
        with self._lock:
            return self._marker_count

    def _ready_unlocked(self, now: float) -> bool:
        marker_target_reached = (
            self._marker_count >= self.expected_markers
            if self.expected_markers
            else self._marker_count > 0
        )
        return (
            marker_target_reached
            and self._last_marker_at is not None
            and now - self._last_marker_at >= self.quiet_period_seconds
        )

    def _take_forward_unlocked(self, now: float) -> tuple[int, str] | None:
        if self._pending_signal is None or self._forwarded:
            return None
        if self._ready_unlocked(now):
            reason = "ready"
        elif now - self.started_at >= self.startup_deadline_seconds:
            reason = "startup-deadline"
        else:
            return None
        self._forwarded = True
        return self._pending_signal, reason

    def observe_output(self, output: str, now: float) -> None:
        marker_matches = len(READY_MARKER.findall(output))
        action: tuple[int, str] | None = None
        with self._lock:
            if marker_matches:
                self._marker_count += marker_matches
                self._last_marker_at = now
            action = self._take_forward_unlocked(now)
        if action is not None:
            self._forward_signal(*action)

    def handle_signal(self, signum: int, now: float) -> str:
        action: tuple[int, str] | None = None
        force = False
        with self._lock:
            self._signal_count += 1
            if self._signal_count > 1:
                force = True
                outcome = "forced"
            elif self._ready_unlocked(now):
                self._pending_signal = signum
                action = self._take_forward_unlocked(now)
                outcome = "forwarded"
            else:
                self._pending_signal = signum
                outcome = "buffered"
        if force:
            self._force_stop()
        elif action is not None:
            self._forward_signal(*action)
        return outcome

    def tick(self, now: float) -> None:
        with self._lock:
            action = self._take_forward_unlocked(now)
        if action is not None:
            self._forward_signal(*action)


class TerminationSignalInbox:
    """Record OS termination signals without entering the guarded state machine."""

    def __init__(self) -> None:
        self._signals: deque[int] = deque()

    def record(self, signum: int) -> None:
        self._signals.append(signum)

    def drain(self) -> tuple[int, ...]:
        signals: list[int] = []
        while self._signals:
            signals.append(self._signals.popleft())
        return tuple(signals)


class ProcessGroupSupervisor:
    """Forward shutdown to one child process group and always reap its leader."""

    def __init__(
        self,
        child: subprocess.Popen[bytes],
        *,
        shutdown_grace_seconds: float,
        monotonic: Callable[[], float] = time.monotonic,
        kill_group: Callable[[int, int], None] = os.killpg,
    ) -> None:
        if shutdown_grace_seconds <= 0:
            raise ValueError("shutdown_grace_seconds must be positive")
        self.child = child
        self.shutdown_grace_seconds = shutdown_grace_seconds
        self._monotonic = monotonic
        self._kill_group = kill_group
        self._shutdown_deadline: float | None = None
        self._forced = False
        self._lock = threading.Lock()

    def _send_unlocked(self, signum: int) -> None:
        with suppress(ProcessLookupError):
            self._kill_group(self.child.pid, signum)

    def forward(self, signum: int) -> None:
        with self._lock:
            self._send_unlocked(signum)
            if self._shutdown_deadline is None:
                self._shutdown_deadline = (
                    self._monotonic() + self.shutdown_grace_seconds
                )

    def force(self) -> None:
        with self._lock:
            if not self._forced:
                self._send_unlocked(signal.SIGKILL)
                self._forced = True

    def tick(self) -> bool:
        """Force an overdue shutdown and report whether a kill was issued."""

        with self._lock:
            overdue = (
                self._shutdown_deadline is not None
                and self._monotonic() >= self._shutdown_deadline
                and self.child.poll() is None
                and not self._forced
            )
        if overdue:
            self.force()
        return overdue

    def reap(self) -> int:
        """Wait for the direct child and return its exact subprocess status."""

        return self.child.wait()

    def cleanup(self) -> None:
        """Fail closed on exceptional exits and synchronously reap the child."""

        if self.child.poll() is None:
            self.force()
        self.child.wait()


def _positive_float_environment(name: str, default: float) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise ValueError(f"{name} must be a positive number") from exc
    if value <= 0:
        raise ValueError(f"{name} must be a positive number")
    return value


def _nonnegative_float_environment(name: str, default: float) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise ValueError(f"{name} must be a non-negative number") from exc
    if value < 0:
        raise ValueError(f"{name} must be a non-negative number")
    return value


def _parameter_bridge_executable() -> str:
    configured = os.environ.get("FLYTO_ROBOTICS_PARAMETER_BRIDGE")
    if configured:
        return configured
    discovered = shutil.which("parameter_bridge")
    if discovered:
        return discovered
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    candidate = Path(f"/opt/ros/{ros_distro}/lib/ros_gz_bridge/parameter_bridge")
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return str(candidate)
    raise FileNotFoundError("could not locate ros_gz_bridge parameter_bridge")


def run(arguments: Sequence[str]) -> int:
    """Run parameter_bridge behind a bounded startup signal barrier."""

    startup_deadline = _positive_float_environment(
        "FLYTO_ROBOTICS_BRIDGE_STARTUP_DEADLINE_SECONDS",
        DEFAULT_STARTUP_DEADLINE_SECONDS,
    )
    quiet_period = _positive_float_environment(
        "FLYTO_ROBOTICS_BRIDGE_QUIET_PERIOD_SECONDS",
        DEFAULT_QUIET_PERIOD_SECONDS,
    )
    shutdown_grace = _positive_float_environment(
        "FLYTO_ROBOTICS_BRIDGE_SHUTDOWN_GRACE_SECONDS",
        DEFAULT_SHUTDOWN_GRACE_SECONDS,
    )
    prespawn_delay = _nonnegative_float_environment(
        "FLYTO_ROBOTICS_BRIDGE_PRESPAWN_DELAY_SECONDS",
        0.0,
    )
    command = [_parameter_bridge_executable(), *arguments]
    started_at = time.monotonic()
    child_holder: list[subprocess.Popen[bytes] | None] = [None]
    supervisor_holder: list[ProcessGroupSupervisor | None] = [None]
    force_before_spawn = threading.Event()
    signal_inbox = TerminationSignalInbox()

    def forward_signal(signum: int, reason: str) -> None:
        print(
            f"[flyto-bridge-guard] forwarding {signal.Signals(signum).name} "
            f"reason={reason}",
            file=sys.stderr,
            flush=True,
        )
        child = child_holder[0]
        supervisor = supervisor_holder[0]
        if child is None or supervisor is None:
            return
        supervisor.forward(signum)

    def force_stop() -> None:
        print(
            "[flyto-bridge-guard] repeated termination signal; forcing child stop",
            file=sys.stderr,
            flush=True,
        )
        child = child_holder[0]
        supervisor = supervisor_holder[0]
        if child is None or supervisor is None:
            force_before_spawn.set()
            return
        supervisor.force()

    guard = StartupSignalGuard(
        expected_markers=infer_expected_markers(arguments),
        started_at=started_at,
        startup_deadline_seconds=startup_deadline,
        quiet_period_seconds=quiet_period,
        forward_signal=forward_signal,
        force_stop=force_stop,
    )

    def handle_termination(signum: int, _frame: object) -> None:
        signal_inbox.record(signum)

    def process_pending_signals() -> None:
        for signum in signal_inbox.drain():
            outcome = guard.handle_signal(signum, time.monotonic())
            if outcome == "buffered":
                print(
                    f"[flyto-bridge-guard] buffered {signal.Signals(signum).name} "
                    f"during startup markers={guard.marker_count}",
                    file=sys.stderr,
                    flush=True,
                )

    previous_handlers = {
        signum: signal.signal(signum, handle_termination)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    print(
        f"[flyto-bridge-guard] armed expected_markers={guard.expected_markers}",
        file=sys.stderr,
        flush=True,
    )
    if prespawn_delay:
        prespawn_deadline = time.monotonic() + prespawn_delay
        while time.monotonic() < prespawn_deadline:
            process_pending_signals()
            time.sleep(min(0.01, max(0.0, prespawn_deadline - time.monotonic())))
    process_pending_signals()
    child = subprocess.Popen(  # noqa: S603 - command path is explicitly resolved above
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
        start_new_session=True,
    )
    child_holder[0] = child
    supervisor = ProcessGroupSupervisor(
        child,
        shutdown_grace_seconds=shutdown_grace,
    )
    supervisor_holder[0] = supervisor
    if force_before_spawn.is_set():
        force_stop()

    def relay_output() -> None:
        assert child.stdout is not None
        while True:
            raw_line = child.stdout.readline()
            if not raw_line:
                break
            sys.stdout.buffer.write(raw_line)
            sys.stdout.buffer.flush()
            guard.observe_output(raw_line.decode("utf-8", "replace"), time.monotonic())

    relay = threading.Thread(target=relay_output, name="bridge-output-relay", daemon=True)
    relay.start()
    try:
        while child.poll() is None:
            process_pending_signals()
            guard.tick(time.monotonic())
            if supervisor.tick():
                print(
                    "[flyto-bridge-guard] graceful shutdown deadline exceeded; "
                    "forcing child stop",
                    file=sys.stderr,
                    flush=True,
                )
            time.sleep(0.01)
        process_pending_signals()
        relay.join(timeout=1.0)
        return shell_exit_code(supervisor.reap())
    finally:
        supervisor.cleanup()
        relay.join(timeout=1.0)
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)


def main() -> int:
    try:
        return run(sys.argv[1:])
    except (FileNotFoundError, OSError, ValueError) as exc:
        print(f"flyto bridge guard: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
