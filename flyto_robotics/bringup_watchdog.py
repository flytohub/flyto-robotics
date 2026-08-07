"""Freshness watchdog for turtlebot3-bringup: restart on silence, not just death.

The supervised launch wrapper (turtlebot3_bringup_supervised.launch.py) turns
"one bringup process died" into "the whole group exits" so systemd's
Restart=always can recover it. That mechanism is scoped to processes that
exit. On 2026-08-07 the OpenCR bridge hung *while alive* right after a real
motor command: same PID, state Sl, zero further log lines, and every topic it
publishes — /odom, /battery_state — silent at once. systemd saw an active
service; OnProcessExit had nothing to catch; the robot sat dead until a
manual restart.

The check that failure mode needs is not "is the process running" but "is it
still doing anything". This node answers that with the same freshness pattern
mission.py's evaluate_sensor_gate already applies to mission-level sensor
trust, wired into systemd's own watchdog protocol:

- It runs inside the same launch group as bringup, so it shares the service's
  cgroup and $NOTIFY_SOCKET (the unit sets NotifyAccess=all because this is a
  sibling process, not the service's main PID).
- READY=1 is deferred until the first /odom message, or a generous startup
  grace matching mission.py's sensor_startup_grace_seconds — the normal ~8s
  cold-start window before odometry begins must not itself trip the watchdog.
- While /odom stays fresh it pings WATCHDOG=1. When /odom goes stale it
  simply stops pinging: systemd's watchdog timer kills the whole cgroup and
  Restart=always brings it back. No new privilege, no `systemctl restart`
  from inside the service.

The decision logic is a pure function (evaluate_bringup_watchdog) plus a
clock-free ticker (WatchdogTicker) so the exact wait/arm/ping/starve
transitions are unit-testable without a robot, the same way mission.py tests
evaluate_sensor_gate.
"""

from __future__ import annotations

import argparse
import os
import socket
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Literal

WatchdogDecision = Literal["wait", "arm", "ping", "starve"]

# Matches mission.py's sensor_startup_grace_seconds: the established bound for
# how long a healthy cold start may take before odometry begins (~8s observed).
DEFAULT_STARTUP_GRACE_SECONDS = 10.0
# /odom runs at 20 Hz when healthy; 5s of silence is ~100 missed messages —
# far beyond scheduler jitter, far quicker than an operator noticing by hand.
DEFAULT_FRESHNESS_WINDOW_SECONDS = 5.0
DEFAULT_TICK_SECONDS = 1.0


def evaluate_bringup_watchdog(
    *,
    ready_sent: bool,
    odom_seen: bool,
    last_odom_age: float,
    startup_elapsed: float,
    startup_grace: float,
    freshness_window: float,
) -> WatchdogDecision:
    """Classify what to tell systemd this tick.

    Before READY: wait for the first /odom message, but never longer than the
    startup grace — a bringup that produces no odometry at all still gets
    armed, then starves the timer, so systemd restarts it after
    startup_grace + WatchdogSec instead of never.

    After READY: ping only while /odom is fresh. Going stale stops the pings;
    recovery within systemd's WatchdogSec resumes them harmlessly.
    """
    if not ready_sent:
        if odom_seen or startup_elapsed >= startup_grace:
            return "arm"
        return "wait"
    if odom_seen and last_odom_age <= freshness_window:
        return "ping"
    return "starve"


def sd_notify(message: str, *, socket_path: str | None) -> bool:
    """Send one sd_notify datagram; False when not under systemd or on error.

    Implemented directly (an abstract-namespace-aware AF_UNIX datagram) so the
    robot needs no sdnotify/pystemd dependency for a ~10-line protocol.
    """
    if not socket_path:
        return False
    target = socket_path
    if target.startswith("@"):
        target = "\0" + target[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.sendto(message.encode("utf-8"), target)
    except OSError:
        return False
    return True


@dataclass
class WatchdogTicker:
    """Clock-free state machine between /odom callbacks and sd_notify calls.

    Callers inject `now` (monotonic seconds) and a notify callable, so every
    transition — including the log lines an operator will grep for at 2am —
    is testable without ROS or systemd.
    """

    started_at: float
    notify: Callable[[str], bool]
    log: Callable[[str], None]
    startup_grace: float = DEFAULT_STARTUP_GRACE_SECONDS
    freshness_window: float = DEFAULT_FRESHNESS_WINDOW_SECONDS
    ready_sent: bool = field(default=False, init=False)
    starving: bool = field(default=False, init=False)
    last_odom_at: float | None = field(default=None, init=False)

    def record_odom(self, now: float) -> None:
        self.last_odom_at = now

    def tick(self, now: float) -> WatchdogDecision:
        odom_seen = self.last_odom_at is not None
        decision = evaluate_bringup_watchdog(
            ready_sent=self.ready_sent,
            odom_seen=odom_seen,
            last_odom_age=(now - self.last_odom_at) if odom_seen else float("inf"),
            startup_elapsed=now - self.started_at,
            startup_grace=self.startup_grace,
            freshness_window=self.freshness_window,
        )
        if decision == "arm":
            self.ready_sent = True
            if odom_seen:
                self.log("bringup_watchdog: first /odom seen; READY=1, watchdog armed.")
            else:
                self.log(
                    "bringup_watchdog: no /odom within "
                    f"{self.startup_grace:.0f}s startup grace; arming anyway so "
                    "systemd's watchdog timer can restart a bringup that never "
                    "produced odometry."
                )
            self.notify("READY=1\nWATCHDOG=1")
        elif decision == "ping":
            if self.starving:
                self.starving = False
                self.log("bringup_watchdog: /odom is fresh again; resuming watchdog pings.")
            self.notify("WATCHDOG=1")
        elif decision == "starve":
            if not self.starving:
                self.starving = True
                self.log(
                    "bringup_watchdog: /odom stale beyond "
                    f"{self.freshness_window:.0f}s; withholding watchdog pings so "
                    "systemd restarts the whole bringup group. The 2026-08-07 "
                    "silent hang looked exactly like this: process alive, every "
                    "topic dead."
                )
        return decision


def run_watchdog(
    *,
    odom_topic: str = "/odom",
    startup_grace: float = DEFAULT_STARTUP_GRACE_SECONDS,
    freshness_window: float = DEFAULT_FRESHNESS_WINDOW_SECONDS,
    tick_seconds: float = DEFAULT_TICK_SECONDS,
) -> None:
    """Spin the ROS node. ROS imports stay in here so tests never need them."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("flyto_bringup_watchdog")
    socket_path = os.environ.get("NOTIFY_SOCKET")
    if not socket_path:
        node.get_logger().warning(
            "NOTIFY_SOCKET is not set; running observe-only. Under systemd this "
            "means the unit is not Type=notify and stale /odom will be logged "
            "but never trigger a restart."
        )
    ticker = WatchdogTicker(
        started_at=time.monotonic(),
        notify=lambda message: sd_notify(message, socket_path=socket_path),
        log=lambda line: node.get_logger().info(line),
        startup_grace=startup_grace,
        freshness_window=freshness_window,
    )
    node.create_subscription(
        Odometry,
        odom_topic,
        lambda _message: ticker.record_odom(time.monotonic()),
        10,
    )
    node.create_timer(tick_seconds, lambda: ticker.tick(time.monotonic()))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # SIGINT arrives as KeyboardInterrupt, SIGTERM as rclpy's external
        # shutdown; both are the launch group closing down normally, and a
        # traceback here would read as the watchdog crashing during every
        # clean stop.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="flyto-bringup-watchdog")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--startup-grace-seconds",
        type=float,
        default=DEFAULT_STARTUP_GRACE_SECONDS,
    )
    parser.add_argument(
        "--freshness-window-seconds",
        type=float,
        default=DEFAULT_FRESHNESS_WINDOW_SECONDS,
    )
    parser.add_argument("--tick-seconds", type=float, default=DEFAULT_TICK_SECONDS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    run_watchdog(
        odom_topic=args.odom_topic,
        startup_grace=args.startup_grace_seconds,
        freshness_window=args.freshness_window_seconds,
        tick_seconds=args.tick_seconds,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
