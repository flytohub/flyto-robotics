"""How fast a simulator's clock ran against the wall during one motion.

A simulator publishes its own time on ``/clock``; every Nav2 timer, behavior
duration and progress window on a simulated robot runs on that time. When the
simulator falls behind real time (a CPU-starved renderer), every phase of a
motion takes longer in wall time by the same factor, with no stall and no
recovery to show for it. Twin, 2026-10-06 22:01 local: the back-off, the
escape waypoint and the goal leg each took 1.8-2.4x their usual wall time
(BackUp 9.1 s against ~5 s for the same 0.20 m), so a task looked slow while
the robot never stopped.

This module turns ``/clock`` samples into the real-time factor (simulated
seconds per wall second) over a motion: the mean over the whole motion and
the minimum over windows of ``WINDOW_S``, and flags ``sim_slow`` below
``SIM_SLOW_RTF``. It never reads ROS; the adapter feeds it samples. A real
deployment has no simulator clock and is reported as not applicable.
"""

from __future__ import annotations

import math
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

#: Below this real-time factor a motion is flagged ``sim_slow``. Basis: at
#: 0.5 every wall-clock duration of the motion is at least doubled. The twin
#: normally runs at about 0.7 (CPU-only LiDAR rendering, measured 2026-10-04,
#: see sim/twin/scripts/twin_nav2_params.py), which stays above it; the slow
#: 22:01 run of 2026-10-06 stretched every phase 1.8-2.4x against those
#: usual runs, i.e. about 0.3-0.4, which falls below it.
SIM_SLOW_RTF = 0.5
SIM_SLOW_BASIS = (
    "below 0.5 simulated seconds per wall second every wall-clock duration of "
    "the motion is at least doubled; the twin usually runs near 0.7"
)
#: The minimum is taken over windows at least this long. The adapter samples
#: the clock at most every 0.1 s, so a 1 s window keeps sampling jitter to
#: about a tenth of the factor.
WINDOW_S = 1.0
#: The transport is asked for a clock sample at most this often (ms).
CLOCK_THROTTLE_MS = 100
#: Samples one motion keeps; beyond it every other sample is dropped.
MAX_SAMPLES = 2048

NOT_APPLICABLE_REAL = "real deployment: no simulator clock"


def slow_threshold() -> float:
    """``SIM_SLOW_RTF``, or ``FLYTO_ROS2_SIM_SLOW_RTF`` when set to a factor in (0, 1]."""
    raw = os.getenv("FLYTO_ROS2_SIM_SLOW_RTF", "").strip()
    try:
        value = float(raw) if raw else SIM_SLOW_RTF
    except ValueError:
        return SIM_SLOW_RTF
    return value if math.isfinite(value) and 0.0 < value <= 1.0 else SIM_SLOW_RTF


def clock_seconds(message: Any) -> float | None:
    """Simulated seconds of a ``rosgraph_msgs/Clock`` message, or None."""
    if not isinstance(message, Mapping):
        return None
    stamp = message.get("clock")
    if not isinstance(stamp, Mapping):
        return None
    try:
        seconds = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
    except (TypeError, ValueError):
        return None
    return seconds if math.isfinite(seconds) and seconds >= 0.0 else None


@dataclass
class ClockRecord:
    """``/clock`` samples, (simulated seconds, wall monotonic seconds), in order."""

    topic: str
    samples: list[tuple[float, float]] = field(default_factory=list)
    #: The simulated clock went backwards (a simulator reset) during the motion.
    reset_seen: bool = False

    def add(self, sim_s: float, wall_s: float) -> None:
        if self.samples:
            last_sim, last_wall = self.samples[-1]
            if wall_s <= last_wall:
                return
            if sim_s < last_sim:
                # Time after a reset is not comparable with time before it.
                self.reset_seen = True
                self.samples = []
        self.samples.append((sim_s, wall_s))
        if len(self.samples) > MAX_SAMPLES:
            self.samples = self.samples[::2]

    def summary(self, *, threshold: float | None = None) -> dict[str, Any]:
        limit = slow_threshold() if threshold is None else threshold
        base: dict[str, Any] = {"applicable": True, "clock_topic": self.topic}
        if self.reset_seen:
            base["clock_reset_seen"] = True
        if len(self.samples) < 2:
            return {
                **base,
                "measured": False,
                "reason": "fewer than two clock samples during the motion",
                "samples": len(self.samples),
            }
        first_sim, first_wall = self.samples[0]
        last_sim, last_wall = self.samples[-1]
        wall = last_wall - first_wall
        sim = last_sim - first_sim
        mean = sim / wall
        windows = _window_factors(self.samples, WINDOW_S)
        minimum = min(windows) if windows else mean
        return {
            **base,
            "measured": True,
            "samples": len(self.samples),
            "wall_s": round(wall, 3),
            "sim_s": round(sim, 3),
            "rtf_mean": round(mean, 3),
            "rtf_min": round(minimum, 3),
            "window_s": WINDOW_S,
            "windows": len(windows),
            "sim_slow": minimum < limit,
            "sim_slow_threshold": limit,
            "basis": SIM_SLOW_BASIS,
        }


def _window_factors(samples: list[tuple[float, float]], window_s: float) -> list[float]:
    """Real-time factor over consecutive spans of at least ``window_s`` wall seconds."""
    factors: list[float] = []
    start = 0
    for index in range(1, len(samples)):
        wall = samples[index][1] - samples[start][1]
        if wall >= window_s:
            factors.append((samples[index][0] - samples[start][0]) / wall)
            start = index
    return factors


def not_applicable(reason: str = NOT_APPLICABLE_REAL) -> dict[str, Any]:
    return {"applicable": False, "reason": reason}


def combine(parts: list[Mapping[str, Any]], *, threshold: float | None = None) -> dict[str, Any]:
    """One record over several motions (an escape's legs) from their summaries."""
    measured = [part for part in parts if part.get("measured")]
    if not parts or not any(part.get("applicable") for part in parts):
        reason = next((str(p.get("reason")) for p in parts if p.get("reason")), None)
        return not_applicable(reason or NOT_APPLICABLE_REAL)
    if not measured:
        return {"applicable": True, "measured": False, "reason": "no leg measured the clock"}
    limit = slow_threshold() if threshold is None else threshold
    wall = sum(float(part["wall_s"]) for part in measured)
    sim = sum(float(part["sim_s"]) for part in measured)
    minimum = min(float(part["rtf_min"]) for part in measured)
    mean = sim / wall if wall > 0 else minimum
    return {
        "applicable": True,
        "measured": True,
        "legs_measured": len(measured),
        "wall_s": round(wall, 3),
        "sim_s": round(sim, 3),
        "rtf_mean": round(mean, 3),
        "rtf_min": round(minimum, 3),
        "sim_slow": minimum < limit,
        "sim_slow_threshold": limit,
        "basis": SIM_SLOW_BASIS,
    }
