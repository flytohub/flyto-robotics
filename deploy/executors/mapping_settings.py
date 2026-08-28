"""What the mapping executor needs to know about the machine it is running on.

Everything here was a constant in `flyto_mapping_executor.py` until a second
robot was contemplated. `SLAM_UNIT`, `BRINGUP_UNIT`, `ROS_DOMAIN_ID`, `MAP_DIR`
and the battery floor are facts about *one* TurtleBot3, and a second robot with
a different SLAM unit or a 4S pack would have meant forking the file. The camera
gateway built the same day read seventeen of its settings from the environment
with validation and bounds; this read none of its own.

## Why a file and not the environment

The device-executor registry starts an executor with `env={}` — no PATH, no
inherited anything — so the `EnvironmentFile` pattern the camera gateway uses is
not available. What *is* available is the manifest: it carries the full argv,
so the config path is passed as `--config`, and the manifest is already the
per-robot data file. One robot, one manifest, one settings file beside it.

## Unknown keys are refused, not ignored

The same rule `safety_profile.py` applies, for the same reason: a typo in a
settings file that is silently ignored leaves the operator believing they
changed something. A misspelled `slam_unit` must fail loudly at load, not
quietly fall back to a default that happens to be right on the machine it was
written on.

## What is deliberately *not* configurable

The map-name pattern, the prepared-payload marker and the contract version are
not settings. They are the boundary this executor defends and the protocol it
speaks, and a machine cannot be given permission to relax either. The saver's
timeouts are also fixed: their ordering relative to the manifest timeout is an
invariant (`saver < shell < parent`), and a settings file able to invert it
would reintroduce the orphaned-saver defect it exists to prevent.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

CONTRACT_VERSION = "flyto.robotics.mapping-executor.v1"

MAX_SETTINGS_BYTES = 16_384

# systemd's own rule, minus the template and instance forms this has no use for.
UNIT_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9:_.-]{0,127}\.(?:service|target)\Z")
# A ROS topic, bounded the same way the camera gateway bounds its own.
TOPIC = re.compile(r"/[A-Za-z0-9_][A-Za-z0-9_/]{0,253}\Z")

# A 3S LiPo sits near 11.1 V and a 6S near 22.2 V; a quadruped's pack is
# different again. Bounded only against values that cannot be a pack voltage at
# all, because the right floor for a given robot is a thing to measure, not a
# thing this file can know.
MIN_PLAUSIBLE_VOLTS = 3.0
MAX_PLAUSIBLE_VOLTS = 60.0

# ROS 2's own range.
MAX_ROS_DOMAIN_ID = 232

REQUIRED = frozenset({"contract_version", "slam_unit", "map_dir"})
OPTIONAL = frozenset({
    "readiness_unit", "navigation_unit", "ros_setup", "ros_domain_id",
    "battery_topic", "min_mapping_volts",
})


class MappingSettingsError(ValueError):
    """A settings file that cannot be trusted to describe a machine."""


def _text(value, pattern: re.Pattern, field: str) -> str:
    if not isinstance(value, str) or pattern.fullmatch(value) is None:
        raise MappingSettingsError(f"{field} is not a valid {pattern.pattern[:20]}…")
    return value


def _absolute(value, field: str) -> str:
    if not isinstance(value, str):
        raise MappingSettingsError(f"{field} must be an absolute path with no dot-segments")
    parts = value.split("/")
    if (not value.startswith("/") or value == "/" or len(value) > 4096
            or not value.isascii() or any(ord(c) < 32 for c in value)
            or any(part in {"", ".", ".."} for part in parts[1:])):
        raise MappingSettingsError(f"{field} must be an absolute path with no dot-segments")
    return value


@dataclass(frozen=True)
class MappingSettings:
    """One machine's answer to every question the executor asks about it."""

    #: The unit that holds SLAM between `mapping.start` and `mapping.save`.
    slam_unit: str
    #: Where a recorded map is published.
    map_dir: str
    #: What must be running before a mapping run can begin — the driver stack
    #: that produces scan and odometry. `None` states that this machine has no
    #: such unit, which is a claim an operator makes deliberately: the executor
    #: then cannot refuse `sensors_unavailable` and a run may start against a
    #: robot whose sensors are down.
    readiness_unit: str | None = None
    #: The navigation stack that must NOT be running, because it and SLAM both
    #: publish the map. `None` on a machine that has no such stack.
    navigation_unit: str | None = None
    #: Sourced before the save command. `None` on a machine whose saver needs no
    #: environment.
    ros_setup: str | None = None
    ros_domain_id: int | None = None
    #: `None` states that this machine reports no readable pack. The executor
    #: then starts without a battery check and says so in its evidence, rather
    #: than silently skipping a safety precondition.
    battery_topic: str | None = None
    min_mapping_volts: float | None = None

    @property
    def checks_battery(self) -> bool:
        return self.battery_topic is not None and self.min_mapping_volts is not None

    @classmethod
    def from_file(cls, path: Path) -> MappingSettings:
        try:
            raw = path.read_bytes()
        except OSError as exc:
            raise MappingSettingsError(f"settings unreadable: {path}") from exc
        if len(raw) > MAX_SETTINGS_BYTES:
            raise MappingSettingsError("settings file is too large to be settings")
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as exc:
            raise MappingSettingsError("settings must be UTF-8 JSON") from exc
        return cls.from_mapping(value)

    @classmethod
    def from_mapping(cls, value) -> MappingSettings:
        if not isinstance(value, dict):
            raise MappingSettingsError("settings must be a JSON object")

        missing = REQUIRED - set(value)
        if missing:
            raise MappingSettingsError(f"settings is missing {', '.join(sorted(missing))}")
        unknown = set(value) - REQUIRED - OPTIONAL
        if unknown:
            raise MappingSettingsError(
                f"settings cannot set {', '.join(sorted(unknown))}; "
                f"a key this file does not know is a typo, not a feature"
            )
        if value["contract_version"] != CONTRACT_VERSION:
            raise MappingSettingsError(
                f"settings contract_version must be {CONTRACT_VERSION}")

        slam_unit = _text(value["slam_unit"], UNIT_NAME, "slam_unit")
        map_dir = _absolute(value["map_dir"], "map_dir")

        readiness = value.get("readiness_unit")
        if readiness is not None:
            readiness = _text(readiness, UNIT_NAME, "readiness_unit")
        navigation = value.get("navigation_unit")
        if navigation is not None:
            navigation = _text(navigation, UNIT_NAME, "navigation_unit")
        if navigation is not None and navigation == slam_unit:
            raise MappingSettingsError(
                "navigation_unit and slam_unit cannot be the same unit; the "
                "executor would refuse every run as navigation_running")

        ros_setup = value.get("ros_setup")
        if ros_setup is not None:
            ros_setup = _absolute(ros_setup, "ros_setup")

        domain = value.get("ros_domain_id")
        if domain is not None:
            if isinstance(domain, bool) or not isinstance(domain, int):
                raise MappingSettingsError("ros_domain_id must be an integer")
            if not 0 <= domain <= MAX_ROS_DOMAIN_ID:
                raise MappingSettingsError(
                    f"ros_domain_id must be 0-{MAX_ROS_DOMAIN_ID}")

        topic = value.get("battery_topic")
        if topic is not None:
            topic = _text(topic, TOPIC, "battery_topic")
        volts = value.get("min_mapping_volts")
        if volts is not None:
            if isinstance(volts, bool) or not isinstance(volts, (int, float)):
                raise MappingSettingsError("min_mapping_volts must be a number")
            volts = float(volts)
            if not MIN_PLAUSIBLE_VOLTS <= volts <= MAX_PLAUSIBLE_VOLTS:
                raise MappingSettingsError(
                    f"min_mapping_volts must be {MIN_PLAUSIBLE_VOLTS}-"
                    f"{MAX_PLAUSIBLE_VOLTS}; a value outside that is not a pack")
        # Half a battery check is worse than none: a topic with no floor reads
        # the pack and compares it to nothing, and a floor with no topic is a
        # threshold that never runs. Either both, or an explicit neither.
        if (topic is None) != (volts is None):
            raise MappingSettingsError(
                "battery_topic and min_mapping_volts must be set together, or "
                "both omitted to state that this machine reports no pack")

        return cls(
            slam_unit=slam_unit,
            map_dir=map_dir,
            readiness_unit=readiness,
            navigation_unit=navigation,
            ros_setup=ros_setup,
            ros_domain_id=domain,
            battery_topic=topic,
            min_mapping_volts=volts,
        )
