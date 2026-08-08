from __future__ import annotations

import pytest

from flyto_robotics.mission import evaluate_sensor_gate, unready_sensors


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
