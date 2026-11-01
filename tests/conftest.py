"""Shared test isolation."""

from __future__ import annotations

import pytest

from flyto_robotics import generic_ros2_adapter


@pytest.fixture(autouse=True)
def _no_halt_hold_between_tests():
    """The halt hold is process-wide (one robot per host); each test starts unheld."""
    generic_ros2_adapter.HALT_HOLD.release()
    yield
    generic_ros2_adapter.HALT_HOLD.release()
