"""The stop-and-resume record a demonstration rests on, actually executed.

This logic had seven tests before, and every one of them read the driver's
source and asserted that certain strings appeared in it — one of them asserting
a particular line break. They passed whether or not the logic worked, and the
moment the same behaviour moved to another file they all went red without a
single behaviour changing. That is the opposite of a test.
"""

from __future__ import annotations

import math

import pytest

from flyto_robotics.lab_safety_observation import (
    MOTION_RESUMED,
    STOP_OBSERVED,
    CommandedVelocity,
    SafetyObservation,
)

STOP_DISTANCE = 0.25
DRIVING = CommandedVelocity(linear_x=0.20, angular_z=0.0)
HALTED = CommandedVelocity(linear_x=0.0, angular_z=0.0)
CLEAR = 2.0
NEAR = 0.10


def observer() -> SafetyObservation:
    return SafetyObservation(stop_distance=STOP_DISTANCE)


def approach(obs: SafetyObservation) -> str | None:
    """Drive, then meet an obstacle and halt — the sequence being recorded."""
    obs.observe(
        obstacle_entered=False, obstacle_exited=False, minimum_range=CLEAR, command=DRIVING
    )
    return obs.observe(
        obstacle_entered=True, obstacle_exited=False, minimum_range=NEAR, command=HALTED
    )


class TestReadingACommand:
    @pytest.mark.parametrize(
        ("linear", "angular", "zero"),
        [(0.0, 0.0, True), (0.0005, 0.0, True), (0.0, 0.0005, True), (0.2, 0.0, False)],
    )
    def test_float_noise_on_the_wire_still_counts_as_stopped(self, linear, angular, zero):
        assert CommandedVelocity(linear, angular).is_zero is zero

    def test_a_drifting_command_is_not_under_way(self):
        assert CommandedVelocity(0.005, 0.0).is_moving is False
        assert CommandedVelocity(0.02, 0.0).is_moving is True


class TestRecordingAStop:
    def test_the_full_sequence_records_a_stop(self):
        assert approach(observer()) == STOP_OBSERVED

    def test_a_robot_that_never_moved_was_not_stopped_by_anything(self):
        """The precondition that makes the record mean something."""
        obs = observer()
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=False,
                minimum_range=NEAR,
                command=HALTED,
            )
            is None
        )
        assert obs.stop_observed is False

    def test_a_halt_with_the_obstacle_still_far_is_not_a_safety_stop(self):
        obs = observer()
        obs.observe(
            obstacle_entered=False, obstacle_exited=False, minimum_range=CLEAR, command=DRIVING
        )
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=False,
                minimum_range=1.0,
                command=HALTED,
            )
            is None
        )

    def test_still_moving_is_not_a_stop(self):
        obs = observer()
        obs.observe(
            obstacle_entered=False, obstacle_exited=False, minimum_range=CLEAR, command=DRIVING
        )
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=False,
                minimum_range=NEAR,
                command=DRIVING,
            )
            is None
        )

    def test_the_stop_is_recorded_once_however_many_ticks_follow(self):
        """One event, described once. A per-tick record would report dozens."""
        obs = observer()
        assert approach(obs) == STOP_OBSERVED
        for _ in range(5):
            assert (
                obs.observe(
                    obstacle_entered=True,
                    obstacle_exited=False,
                    minimum_range=NEAR,
                    command=HALTED,
                )
                is None
            )

    def test_an_unmeasurable_range_records_nothing(self):
        """Infinity is nothing within reach, so no proximity stop to attribute.

        Unlike the obstacle guard, this observes rather than commands — not
        recording an event cannot move a robot.
        """
        obs = observer()
        obs.observe(
            obstacle_entered=False, obstacle_exited=False, minimum_range=CLEAR, command=DRIVING
        )
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=False,
                minimum_range=math.inf,
                command=HALTED,
            )
            is None
        )


class TestRecordingAResume:
    def resumed(self, obs: SafetyObservation) -> str | None:
        return obs.observe(
            obstacle_entered=True, obstacle_exited=True, minimum_range=CLEAR, command=DRIVING
        )

    def test_a_resume_after_a_stop_is_recorded(self):
        obs = observer()
        approach(obs)
        assert self.resumed(obs) == MOTION_RESUMED

    def test_there_is_no_resume_without_a_stop_first(self):
        obs = observer()
        assert self.resumed(obs) is None

    def test_the_obstacle_must_have_gone(self):
        obs = observer()
        approach(obs)
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=False,
                minimum_range=CLEAR,
                command=DRIVING,
            )
            is None
        )

    def test_the_way_must_actually_be_clear_again(self):
        obs = observer()
        approach(obs)
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=True,
                minimum_range=NEAR,
                command=DRIVING,
            )
            is None
        )

    def test_the_resume_is_recorded_once(self):
        obs = observer()
        approach(obs)
        assert self.resumed(obs) == MOTION_RESUMED
        assert self.resumed(obs) is None


class TestNoCommandYet:
    def test_nothing_is_recorded_before_the_first_command(self):
        obs = observer()
        assert (
            obs.observe(
                obstacle_entered=True,
                obstacle_exited=False,
                minimum_range=NEAR,
                command=None,
            )
            is None
        )
        assert obs.motion_before_obstacle is False
