from __future__ import annotations

import pytest

from flyto_robotics.mission import evaluate_sensor_gate


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
