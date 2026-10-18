"""ROS 2 semantic mapping: a graph interface is evidence, never an authorization.

The ROS 2 pairing manifest maps a Flyto2 capability onto a standard ROS 2
interface. The approved interfaces per stack (``_STACK_INTERFACES``) are
derived from one declarative table, ``ros2_interfaces.INTERFACES``; these
tests hold every stack to the rule that an interface it does not approve is
refused, whatever the manifest claims.
"""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import pytest

from flyto_robotics.ros2_pairing import (
    _STACK_INTERFACES,
    Ros2PairingError,
    load_ros2_adapter_manifest,
    parse_ros2_adapter_manifest,
)

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "examples/ros2-adapters/flyto2-standard.json"


def _resign(value: dict) -> dict:
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    value["snapshot"] = hashlib.sha256(
        json.dumps(
            unsigned, ensure_ascii=False, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
    ).hexdigest()
    return value


def _with_interface(**changes: str) -> dict:
    manifest = copy.deepcopy(load_ros2_adapter_manifest(MANIFEST))
    adapter = manifest["adapters"][0]
    stack = changes.pop("stack", None)
    if stack is not None:
        adapter["stack"] = stack
    adapter["interface"].update(changes)
    return _resign(manifest)


def _foreign_pairs() -> list[tuple[str, str]]:
    return sorted(
        (stack, interface_type)
        for stack in _STACK_INTERFACES
        for other, types in _STACK_INTERFACES.items()
        if other != stack
        for interface_type in types
        if interface_type not in _STACK_INTERFACES[stack]
    )


def test_the_standard_nav2_mapping_is_admitted_forward() -> None:
    manifest = parse_ros2_adapter_manifest(load_ros2_adapter_manifest(MANIFEST))
    adapter = manifest["adapters"][0]
    assert adapter["stack"] == "nav2"
    assert adapter["interface"]["type"] in _STACK_INTERFACES["nav2"]
    assert manifest["direct_actuation"] is False


@pytest.mark.parametrize(("stack", "interface_type"), _foreign_pairs())
def test_an_interface_approved_for_another_stack_is_refused(
    stack: str, interface_type: str
) -> None:
    with pytest.raises(Ros2PairingError, match=f"not approved for {stack}"):
        parse_ros2_adapter_manifest(_with_interface(stack=stack, type=interface_type))


@pytest.mark.parametrize("interface_type", [
    # Real Nav2 actions the generic adapter never drives.
    "nav2_msgs/action/FollowPath",
    "nav2_msgs/action/ComputePathToPose",
    "example_vendor_msgs/action/NavigateToPose",
])
def test_a_plausible_but_unapproved_nav2_interface_is_refused(interface_type: str) -> None:
    # A look-alike name or another package's action of the same name is a
    # description; only the table approves an interface.
    with pytest.raises(Ros2PairingError, match="not approved for nav2"):
        parse_ros2_adapter_manifest(_with_interface(type=interface_type))


def test_an_unknown_stack_is_refused() -> None:
    with pytest.raises(Ros2PairingError, match="stack is unsupported"):
        parse_ros2_adapter_manifest(_with_interface(stack="vendor_stack"))


def test_interface_kind_and_type_must_agree() -> None:
    with pytest.raises(Ros2PairingError, match="kind and type do not match"):
        parse_ros2_adapter_manifest(_with_interface(kind="service"))


@pytest.mark.parametrize("interface_type", [
    "geometry_msgs/msg/Twist",
    "std_msgs/msg/Float64",
])
def test_a_topic_message_is_not_an_action_mapping(interface_type: str) -> None:
    with pytest.raises(Ros2PairingError):
        parse_ros2_adapter_manifest(_with_interface(type=interface_type))
