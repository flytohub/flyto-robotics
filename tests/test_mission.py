from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from flyto_robotics.mission import (
    DEFAULT_SENSOR_FRESHNESS_TIMEOUT_SECONDS,
    DEFAULT_SENSOR_STABILIZATION_SECONDS,
    DEFAULT_SENSOR_STARTUP_GRACE_SECONDS,
    evaluate_sensor_gate,
    relative_move_reached,
    relative_move_tolerance,
    unready_sensors,
)

SOURCE_DIR = Path(__file__).resolve().parents[1] / "flyto_robotics"


@pytest.mark.parametrize(
    ("samples_present", "oldest_sample_age"),
    [(False, float("inf")), (True, 1.1)],
)
def test_sensor_gate_waits_for_clean_startup_samples(
    samples_present: bool,
    oldest_sample_age: float,
) -> None:
    assert (
        evaluate_sensor_gate(
            samples_present=samples_present,
            oldest_sample_age=oldest_sample_age,
            ready_duration=0.0,
            startup_elapsed=5.0,
            startup_grace=10.0,
            freshness_timeout=1.0,
            stabilization_seconds=1.0,
            control_started=False,
        )
        == "wait"
    )


def test_sensor_gate_requires_a_stable_fresh_window_before_motion() -> None:
    common = {
        "samples_present": True,
        "oldest_sample_age": 0.1,
        "startup_elapsed": 2.0,
        "startup_grace": 10.0,
        "freshness_timeout": 1.0,
        "stabilization_seconds": 1.0,
        "control_started": False,
    }
    assert evaluate_sensor_gate(**common, ready_duration=0.9) == "wait"
    assert evaluate_sensor_gate(**common, ready_duration=1.0) == "ready"


def test_sensor_gate_fails_closed_at_the_correct_lifecycle_phase() -> None:
    common = {
        "samples_present": True,
        "oldest_sample_age": 1.1,
        "ready_duration": 0.0,
        "startup_grace": 10.0,
        "freshness_timeout": 1.0,
        "stabilization_seconds": 1.0,
    }
    assert (
        evaluate_sensor_gate(
            **common,
            startup_elapsed=10.1,
            control_started=False,
        )
        == "fail_not_ready"
    )
    assert (
        evaluate_sensor_gate(
            **common,
            startup_elapsed=20.0,
            control_started=True,
        )
        == "fail_stale"
    )


class TestRelativeMoveArrival:
    """The arrival box scales with the move, and stays symmetric in sign.

    The box is how far short of the target the move may stop and still be
    called done. A fixed 0.03 m box is a fair ceiling on a 0.40 m jog, but it
    is 60% of a 0.05 m move and three times a 0.01 m nudge — completing both
    before the robot has meaningfully travelled. These are pure policy
    assertions: no controller, no plan, no odometry.
    """

    # (commanded distance magnitude, expected tolerance)
    MAGNITUDES = [(0.01, 0.001), (0.05, 0.005), (0.10, 0.010), (0.40, 0.030)]
    SIGNED = [
        pytest.param(sign * distance, tolerance, id=f"{sign * distance:+.2f}m")
        for distance, tolerance in MAGNITUDES
        for sign in (1, -1)
    ]

    @pytest.mark.parametrize(("target", "expected"), SIGNED)
    def test_tolerance_is_a_tenth_of_the_move_within_bounds(
        self, target: float, expected: float
    ) -> None:
        assert relative_move_tolerance(target) == pytest.approx(expected)

    @pytest.mark.parametrize(("target", "tolerance"), SIGNED)
    def test_progress_exactly_at_the_boundary_has_arrived(
        self, target: float, tolerance: float
    ) -> None:
        boundary = target - tolerance if target > 0 else target + tolerance
        assert relative_move_reached(boundary, target)

    @pytest.mark.parametrize(("target", "tolerance"), SIGNED)
    def test_a_micrometre_short_of_the_boundary_has_not(
        self, target: float, tolerance: float
    ) -> None:
        boundary = target - tolerance if target > 0 else target + tolerance
        outside = boundary - 0.000001 if target > 0 else boundary + 0.000001
        assert not relative_move_reached(outside, target)

    @pytest.mark.parametrize("target", [0.01, -0.01])
    def test_the_origin_is_never_already_arrived(self, target: float) -> None:
        """The 0.03 m constant completed this step before the wheels turned."""
        assert not relative_move_reached(0.0, target)


class TestUnreadySensors:
    """Naming what the gate is waiting on.

    A run that failed because odometry was 0.1s late and a run that failed
    because the lidar died look identical to an operator otherwise.
    """

    NOW = 100.0
    TIMEOUT = 1.0

    def blocking(self, **last_seen: float | None) -> list[str]:
        return unready_sensors(
            last_seen=last_seen, now=self.NOW, freshness_timeout=self.TIMEOUT
        )

    def test_a_sensor_that_never_arrived_is_named_as_such(self) -> None:
        assert self.blocking(odometry=None) == ["odometry: never arrived"]

    def test_a_stale_sensor_reports_how_stale(self) -> None:
        (line,) = self.blocking(scan=self.NOW - 3.5)
        assert line == "scan: last sample 3.5s ago"

    def test_a_fresh_sensor_is_not_named(self) -> None:
        assert self.blocking(odometry=self.NOW - 0.2) == []

    def test_a_sample_exactly_at_the_timeout_is_still_fresh(self) -> None:
        assert self.blocking(odometry=self.NOW - self.TIMEOUT) == []

    def test_every_blocking_sensor_is_named_not_just_the_first(self) -> None:
        """An operator who fixes only the first one comes straight back."""
        assert self.blocking(
            odometry=None, scan=self.NOW - 9.0, camera=self.NOW - 0.1
        ) == ["odometry: never arrived", "scan: last sample 9.0s ago"]

    def test_all_sensors_healthy_returns_nothing(self) -> None:
        """Not a failure to explain — it means the gate wants stabilization.

        The caller distinguishes these; an empty list must not be reported as
        "no sensors", which would send the operator looking in the wrong place.
        """
        assert self.blocking(odometry=self.NOW, scan=self.NOW) == []

    def test_ordering_follows_the_caller(self) -> None:
        assert self.blocking(scan=None, odometry=None) == [
            "scan: never arrived",
            "odometry: never arrived",
        ]


class TestSensorWindowDefaults:
    """The two windows must stay asymmetric, and the reason is not obvious.

    Someone tuning "the sensor timeout" will reach for whichever constant they
    find first. These assertions exist to make the wrong one fail loudly.
    """

    def test_startup_grace_is_far_more_patient_than_the_freshness_timeout(
        self,
    ) -> None:
        """Waiting is free before anything moves; it is not, during.

        Every tick inside the startup grace publishes a stop, so extending it
        costs only how fast a broken robot is reported. The freshness timeout
        governs a robot already under power, where the same slack would mean
        steering on second-old obstacle data.
        """
        assert (
            DEFAULT_SENSOR_STARTUP_GRACE_SECONDS
            >= 10 * DEFAULT_SENSOR_FRESHNESS_TIMEOUT_SECONDS
        )

    def test_the_grace_clears_the_measured_discovery_tail(self) -> None:
        """Measured on the lab robot: p95 3.58s, max 3.60s, 9.1s degraded.

        The effective discovery budget is the grace minus the stabilization
        window that has to fit inside it, which is what made a 10s grace mean
        9s of discovery — and 9.1s was observed. It missed by 0.1 second.
        """
        budget = (
            DEFAULT_SENSOR_STARTUP_GRACE_SECONDS
            - DEFAULT_SENSOR_STABILIZATION_SECONDS
        )
        assert budget > 9.1, "the worst latency seen here must not be a coin flip"

    def test_the_grace_leaves_the_shortest_job_something_to_drive_with(self) -> None:
        """Discovery is spent out of the mission's own budget.

        The gate does not tick the controller while it waits, but `started_at`
        was stamped at construction, so the wait is already on the clock when
        the first tick lands. A grace at or above the mission timeout would
        leave no time to move, or could never fire at all — which would make
        raising it look like a fix while changing nothing.
        """
        timeouts = []
        for job in sorted((SOURCE_DIR.parent / "examples/jobs").glob("*.json")):
            data = json.loads(job.read_text())
            timeout = (data.get("safety") or {}).get("mission_timeout_seconds")
            if timeout is None:
                timeout = data.get("mission_timeout_seconds")
            if timeout is not None:
                timeouts.append((job.name, float(timeout)))

        assert timeouts, "no example job declares a mission timeout"
        name, shortest = min(timeouts, key=lambda pair: pair[1])
        assert shortest / 2 >= DEFAULT_SENSOR_STARTUP_GRACE_SECONDS, (
            f"{name} allows {shortest}s for the whole mission; a "
            f"{DEFAULT_SENSOR_STARTUP_GRACE_SECONDS}s startup grace leaves too "
            "little of it to actually move"
        )

    def test_stabilization_fits_inside_the_grace(self) -> None:
        assert (
            DEFAULT_SENSOR_STABILIZATION_SECONDS
            < DEFAULT_SENSOR_STARTUP_GRACE_SECONDS
        )

    def test_both_ros_nodes_declare_the_same_grace(self) -> None:
        """They used to disagree: 10.0s in one, 5.0s in the other.

        The shortcut node — the one behind the arrow cards — had the tighter
        budget, against a measured 3.60s worst case. Neither file may hardcode
        its own number again.
        """
        for name in ("ros2_node.py", "shortcut_ros2_node.py"):
            source = (SOURCE_DIR / name).read_text()
            declaration = re.search(
                r'declare_parameter\(\s*\n?\s*"sensor_startup_grace_seconds",\s*([^\n)]+)',
                source,
            )
            assert declaration, f"{name} does not declare the grace"
            assert (
                declaration.group(1).strip()
                == "DEFAULT_SENSOR_STARTUP_GRACE_SECONDS"
            ), f"{name} hardcodes its own grace instead of sharing the constant"
