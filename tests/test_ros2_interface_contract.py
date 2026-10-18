"""One table decides which ROS 2 interfaces the adapter drives and pairing approves.

The generic ROS 2 adapter drives Nav2 actions for its motions, and the ROS 2
pairing manifest approves the action types a stack may be mapped onto. Both
used to keep their own copy of that mapping, and they disagreed: the adapter
drove ``DriveOnHeading``, ``BackUp`` and ``Spin``, while pairing approved only
``NavigateToPose`` for Nav2, so a manifest could not describe the motions the
adapter really performs. These tests hold the two to one table: every action
the adapter can drive is approved for its stack, and the stack approves
nothing the adapter does not drive.
"""

from __future__ import annotations

import copy
import hashlib
import json
import sys
import types
from pathlib import Path

import pytest

from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    ESCAPE_INTERFACES,
    RclpyROS2Backend,
)
from flyto_robotics.ros2_pairing import (
    _STACK_INTERFACES,
    Ros2PairingError,
    load_ros2_adapter_manifest,
    parse_ros2_adapter_manifest,
)

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "examples/ros2-adapters/flyto2-standard.json"


def _driven_actions() -> dict[str, str]:
    """Every action the adapter can send a goal to, by capability."""
    return {
        capability_id: interface_type
        for capability_id, (kind, _name, interface_type) in {
            **DEFAULT_INTERFACES,
            **ESCAPE_INTERFACES,
        }.items()
        if kind == "action"
    }


def _resign(value: dict) -> dict:
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    value["snapshot"] = hashlib.sha256(
        json.dumps(
            unsigned, ensure_ascii=False, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
    ).hexdigest()
    return value


@pytest.mark.parametrize(
    ("capability_id", "interface_type"), sorted(_driven_actions().items())
)
def test_every_action_the_adapter_drives_is_approved_for_nav2(
    capability_id: str, interface_type: str
) -> None:
    assert interface_type in _STACK_INTERFACES["nav2"], capability_id


def test_nav2_approves_nothing_the_adapter_does_not_drive() -> None:
    assert _STACK_INTERFACES["nav2"] == frozenset(_driven_actions().values())


def test_a_topic_the_adapter_reads_or_stops_on_is_never_an_approved_mapping() -> None:
    topics = {
        interface_type
        for kind, _name, interface_type in DEFAULT_INTERFACES.values()
        if kind == "topic"
    }
    approved = frozenset().union(*_STACK_INTERFACES.values())
    assert topics and not topics & approved


def test_a_relative_drive_can_be_paired_onto_the_action_the_adapter_uses() -> None:
    manifest = copy.deepcopy(load_ros2_adapter_manifest(MANIFEST))
    _kind, name, interface_type = DEFAULT_INTERFACES["motion.advance"]
    adapter = copy.deepcopy(manifest["adapters"][0])
    adapter.update(
        adapter_id="ros2.nav2.drive_on_heading.v1",
        capability_ids=["robotics.motion.move_relative@1"],
        interface={"kind": "action", "name": name, "type": interface_type},
    )
    manifest["adapters"].append(adapter)
    parsed = parse_ros2_adapter_manifest(_resign(manifest))
    assert parsed["adapters"][-1]["interface"]["type"] == interface_type


def test_an_action_outside_the_table_is_still_refused_for_nav2() -> None:
    manifest = copy.deepcopy(load_ros2_adapter_manifest(MANIFEST))
    manifest["adapters"][0]["interface"]["type"] = "nav2_msgs/action/FollowPath"
    with pytest.raises(Ros2PairingError, match="not approved for nav2"):
        parse_ros2_adapter_manifest(_resign(manifest))


def test_the_action_class_is_resolved_from_the_table_type(monkeypatch) -> None:
    # Each fake class is distinct, so a second map that pointed a capability
    # at the wrong class would be caught.
    classes = {
        interface_type.rsplit("/", 1)[1]: type(interface_type.rsplit("/", 1)[1], (), {})
        for interface_type in _driven_actions().values()
    }
    package = types.ModuleType("nav2_msgs")
    actions = types.ModuleType("nav2_msgs.action")
    for class_name, made in classes.items():
        setattr(actions, class_name, made)
    monkeypatch.setitem(sys.modules, "nav2_msgs", package)
    monkeypatch.setitem(sys.modules, "nav2_msgs.action", actions)

    backend = RclpyROS2Backend.__new__(RclpyROS2Backend)
    for capability_id, interface_type in _driven_actions().items():
        resolved = backend._action_type(capability_id)
        assert resolved is classes[interface_type.rsplit("/", 1)[1]], capability_id
