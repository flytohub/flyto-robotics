from __future__ import annotations

import copy
import hashlib
import json

import pytest

from flyto_robotics.cli import PROJECT_ROOT
from flyto_robotics.resource_binding import (
    ResourceBindingError,
    load_resource_plan,
    parse_resource_plan,
    select_resource_binding,
)

EXAMPLE = (
    PROJECT_ROOT
    / "examples/resource-plans/gazebo-shortcut-forward-30cm.json"
)


def _snapshot(plan: dict[str, object]) -> str:
    unsigned = {key: value for key, value in plan.items() if key != "snapshot"}
    return hashlib.sha256(
        json.dumps(
            unsigned,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
            default=str,
        ).encode("utf-8")
    ).hexdigest()


def _resign(plan: dict[str, object]) -> dict[str, object]:
    plan["snapshot"] = _snapshot(plan)
    return plan


def test_exact_gazebo_resource_binding_matches_workflow_and_robot() -> None:
    plan = load_resource_plan(EXAMPLE)

    binding = select_resource_binding(
        plan,
        workflow_id="shortcut.forward.30cm.v1",
        resource_id="flyto-rover-sim-001",
        capability_id="mobility.move_relative",
        allowed_adapter_ids=("robotics.gazebo",),
        target_space_id="gazebo-lab",
    )

    assert binding.endpoint_id == "gazebo-rover-motion"
    assert binding.adapter_id == "robotics.gazebo"
    assert binding.plan_snapshot == plan["snapshot"]


def test_resource_plan_rejects_snapshot_tampering_and_raw_motor_fields() -> None:
    plan = load_resource_plan(EXAMPLE)
    tampered = copy.deepcopy(plan)
    tampered["selected"][0]["adapter_id"] = "attacker.adapter"  # type: ignore[index]
    with pytest.raises(ResourceBindingError, match="snapshot does not match"):
        parse_resource_plan(tampered)

    raw_motor = copy.deepcopy(plan)
    raw_motor["selected"][0]["linear_x"] = 2.0  # type: ignore[index]
    _resign(raw_motor)
    with pytest.raises(ResourceBindingError, match="unsupported fields: linear_x"):
        parse_resource_plan(raw_motor)


@pytest.mark.parametrize(
    ("workflow_id", "resource_id", "capability_id", "adapter_id"),
    (
        (
            "shortcut.backward.30cm.v1",
            "flyto-rover-sim-001",
            "mobility.move_relative",
            "robotics.gazebo",
        ),
        (
            "shortcut.forward.30cm.v1",
            "another-robot",
            "mobility.move_relative",
            "robotics.gazebo",
        ),
        (
            "shortcut.forward.30cm.v1",
            "flyto-rover-sim-001",
            "vision.observe",
            "robotics.gazebo",
        ),
        (
            "shortcut.forward.30cm.v1",
            "flyto-rover-sim-001",
            "mobility.move_relative",
            "camera.onvif",
        ),
    ),
)
def test_resource_binding_fails_closed_on_cross_context_reuse(
    workflow_id: str,
    resource_id: str,
    capability_id: str,
    adapter_id: str,
) -> None:
    plan = load_resource_plan(EXAMPLE)

    with pytest.raises(ResourceBindingError):
        select_resource_binding(
            plan,
            workflow_id=workflow_id,
            resource_id=resource_id,
            capability_id=capability_id,
            allowed_adapter_ids=(adapter_id,),
            target_space_id="gazebo-lab",
        )


def test_resource_identifiers_are_language_neutral_but_bounded() -> None:
    plan = load_resource_plan(EXAMPLE)
    localized = copy.deepcopy(plan)
    selected = localized["selected"][0]  # type: ignore[index]
    selected["endpoint_id"] = "病房乙-移動端點"
    selected["capability_id"] = "移動.相對距離"
    _resign(localized)

    parsed = parse_resource_plan(localized)

    assert parsed["selected"][0]["endpoint_id"] == "病房乙-移動端點"
