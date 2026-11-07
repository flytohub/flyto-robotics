"""Shared test isolation."""

from __future__ import annotations

import pytest

from flyto_robotics import generic_ros2_adapter


@pytest.fixture(autouse=True)
def _a_fresh_halt_hold_per_test(monkeypatch, tmp_path):
    """The halt hold is process-wide (one robot per host) and kept in a file so
    it survives a host restart. Each test gets its own file and its own hold
    (unheld, nothing admitted, epoch 0), as a freshly started host would."""
    monkeypatch.setenv(generic_ros2_adapter.HALT_HOLD_FILE_ENV, str(tmp_path / "halt-hold.json"))
    monkeypatch.setattr(generic_ros2_adapter, "HALT_HOLD", generic_ros2_adapter._HaltHold())
