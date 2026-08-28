"""Validation tests for the per-robot mapping settings contract."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from deploy.executors.mapping_settings import (
    MappingSettings,
    MappingSettingsError,
)

PROFILE = (
    Path(__file__).resolve().parents[1]
    / "deploy/executors/turtlebot3-mapping.json"
)


def _settings_values(**overrides) -> dict:
    values = json.loads(PROFILE.read_text(encoding="utf-8"))
    values.update(overrides)
    return values


def test_the_shipped_turtlebot3_settings_are_valid():
    settings = MappingSettings.from_file(PROFILE)
    assert settings.slam_unit == "flyto-slam.service"
    assert settings.checks_battery


@pytest.mark.parametrize(
    "overrides",
    [
        {"slam_unit": "flyto-slamXservice"},
        {"slam_unit": "flyto-slam.service/extra"},
        {"map_dir": "relative/maps"},
        {"map_dir": "/home/ubuntu/../root"},
        {"map_dir": "/home//ubuntu/maps"},
        {"battery_topic": "battery_state"},
        {"battery_topic": "/"},
        {"ros_domain_id": True},
        {"ros_domain_id": 233},
    ],
)
def test_invalid_machine_facts_are_refused(overrides):
    with pytest.raises(MappingSettingsError):
        MappingSettings.from_mapping(_settings_values(**overrides))


@pytest.mark.parametrize(
    "overrides",
    [
        {"battery_topic": None},
        {"min_mapping_volts": None},
    ],
)
def test_half_a_battery_check_is_refused(overrides):
    with pytest.raises(MappingSettingsError, match="must be set together"):
        MappingSettings.from_mapping(_settings_values(**overrides))


def test_unknown_settings_are_refused_as_typos():
    with pytest.raises(MappingSettingsError, match="unknown_setting"):
        MappingSettings.from_mapping(_settings_values(unknown_setting=True))
