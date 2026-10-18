"""The arm adapter's capabilities as ``flyto.capability-contract.v1`` rows.

Built from an :class:`~flyto_robotics.arm_adapter.ArmProfile`, so the bounds a
host enforces and the tolerances it judges with are exactly the ones the
adapter enforces itself:

* ``manipulation.state``: read only, no evidence.
* ``manipulation.move_joints``: one ``number`` parameter per joint (``<joint>_rad``)
  whose ``min``/``max`` are the profile's joint limits, and one ``distance_to``
  evidence spec per joint on ``joint_positions`` in frame ``joints``. The
  tolerance is the adapter's arrival tolerance plus its settle drift, so the
  contract accepts no end pose the adapter would not have reported as arrived.
* ``motion.halt``: ``role: safe_stop``. In ``hold`` mode one ``delta`` spec per
  joint says the joint did not move more than the stop's drift tolerance
  between the hold being written and the end of the confirmation window. A
  ``torque_off`` stop is confirmed by the torque read-back the adapter reports
  in ``stop_confirmation``, which no measure op expresses, so it declares no
  evidence.
* ``manipulation.grasp`` (only when the profile declares a gripper): no
  parameters. Two ``delta`` specs on ``object_held``, from the moment the jaws
  stopped (``before``) to the end of the hold window (``after``): ``held`` went
  from 0 to 1 exactly, and the jaw position moved no more than the hold's drift
  tolerance. A grasp that closed fully reports ``held`` 0 and is never usable.
* ``manipulation.release`` (only with a gripper): one ``number`` parameter,
  ``open_to_rad``, bounded from the release position to the open limit, and one
  ``distance_to`` spec on ``gripper_position`` (phases ``after`` + ``settled``).

The rows have the shape of a ``flyto.pack.v1`` module row (``module_id``,
``provides_capability``, ``params_schema``, ``timeout_ms``, ``contract``;
flyto-core ``docs/specs/PACK_MANIFEST_SPEC.md``). This repository does not
import flyto-core; ``tests/test_arm_contracts.py`` validates the rows with
flyto-core's ``validate_contract`` when it is importable.
"""

from __future__ import annotations

import math
from typing import Any

from .arm_adapter import (
    ARGUMENT_SUFFIX,
    CAPABILITY_GRASP,
    CAPABILITY_HALT,
    CAPABILITY_MOVE,
    CAPABILITY_RELEASE,
    CAPABILITY_STATE,
    FRAME,
    GRIPPER_OBSERVATION,
    HELD_FIELD,
    HELD_OBSERVATION,
    OBSERVATION,
    RELEASE_ARGUMENT,
    STOP_HOLD,
    ArmProfile,
    GripperProfile,
)

CONTRACT_SCHEMA = "flyto.capability-contract.v1"
# Headroom over the adapter's own budget, for the host's deadline.
DEADLINE_MARGIN_MS = 2000


def _ms(seconds: float) -> int:
    return int(math.ceil(seconds * 1000)) + DEADLINE_MARGIN_MS


def state_row(profile: ArmProfile) -> dict[str, Any]:
    return {
        "module_id": CAPABILITY_STATE,
        "provides_capability": CAPABILITY_STATE,
        "label": {"en": "Arm state"},
        "description": "Read every joint's position and torque state. Writes nothing.",
        "params_schema": {},
        "timeout_ms": DEADLINE_MARGIN_MS * 2,
        "contract": {
            "schema": CONTRACT_SCHEMA,
            "actuates": False,
            "safety_class": "read_only",
            "requires_safe_stop": False,
            "cancellable": True,
            "idempotent": True,
        },
    }


def move_row(profile: ArmProfile) -> dict[str, Any]:
    params: dict[str, Any] = {}
    evidence: list[dict[str, Any]] = []
    tolerance = profile.position_tolerance_rad + profile.settle_max_drift_rad
    for joint in profile.joints:
        argument = joint.name + ARGUMENT_SUFFIX
        params[argument] = {
            "type": "number",
            "required": True,
            "min": joint.min_rad,
            "max": joint.max_rad,
            "unit": "rad",
            "description": f"Target position of joint {joint.name}",
        }
        evidence.append(
            {
                "kind": f"joint.{joint.name}.arrival",
                "observe": OBSERVATION,
                "phases": ["after", "settled"],
                "measure": {"op": "distance_to", "fields": [joint.name], "frame": FRAME},
                "expect": {"arguments": {joint.name: argument}},
                "tolerance": {"absolute": tolerance},
                "settle": {"max_drift": profile.settle_max_drift_rad},
                "unit": "rad",
            }
        )
    budget_s = profile.move_deadline_s + (
        profile.settle_samples * profile.control_period_s
        + profile.stop_confirm_samples * profile.stop_confirm_period_s
    )
    return {
        "module_id": CAPABILITY_MOVE,
        "provides_capability": CAPABILITY_MOVE,
        "label": {"en": "Move arm joints"},
        "description": (
            "Move every joint to an absolute target within its declared limits, "
            "at the profile's bounded speed. Out-of-limit targets are refused."
        ),
        "params_schema": params,
        "timeout_ms": _ms(budget_s),
        "contract": {
            "schema": CONTRACT_SCHEMA,
            "actuates": True,
            "safety_class": "movement",
            "requires_safe_stop": True,
            "cancellable": True,
            # A lost connection mid-move leaves the result unknown; a retry
            # could move the arm again, so it is not declared idempotent.
            "idempotent": False,
            "effects": ["arm.joints-moved"],
            "requires": ["arm.limits-accepted", "arm.workspace-clear"],
            "evidence": evidence,
            "expected_duration_ms": _ms(budget_s),
        },
    }


def halt_row(profile: ArmProfile) -> dict[str, Any]:
    evidence: list[dict[str, Any]] = []
    if profile.stop_mode == STOP_HOLD:
        for joint in profile.joints:
            evidence.append(
                {
                    "kind": f"joint.{joint.name}.held",
                    "observe": OBSERVATION,
                    "phases": ["before", "after"],
                    "measure": {"op": "delta", "fields": [joint.name], "frame": FRAME},
                    "expect": {"value": 0},
                    "tolerance": {"absolute": profile.stop_max_drift_rad},
                    "unit": "rad",
                }
            )
    contract: dict[str, Any] = {
        "schema": CONTRACT_SCHEMA,
        "actuates": True,
        "safety_class": "controlled",
        "requires_safe_stop": False,
        "cancellable": False,
        "idempotent": True,
        "role": "safe_stop",
        "effects": ["arm.stopped"],
        "expected_duration_ms": _ms(
            profile.stop_confirm_samples * profile.stop_confirm_period_s
        ),
    }
    if evidence:
        contract["evidence"] = evidence
    return {
        "module_id": CAPABILITY_HALT,
        "provides_capability": CAPABILITY_HALT,
        "label": {"en": "Stop the arm"},
        "description": (
            f"Stop the arm ({profile.stop_mode}); completed only when the read-back confirms it."
        ),
        "params_schema": {},
        "timeout_ms": contract["expected_duration_ms"],
        "contract": contract,
    }


def _grip_budget_s(profile: ArmProfile, gripper: GripperProfile) -> float:
    return profile.move_deadline_s + (
        gripper.hold_samples * profile.control_period_s
        + profile.stop_confirm_samples * profile.stop_confirm_period_s
    )


def _grip_contract(
    profile: ArmProfile, gripper: GripperProfile, effects: list[str], evidence: list
) -> dict[str, Any]:
    return {
        "schema": CONTRACT_SCHEMA,
        "actuates": True,
        "safety_class": "movement",
        "requires_safe_stop": True,
        "cancellable": True,
        # A lost connection mid-grip leaves the result unknown; a retry could
        # move the jaws again, so neither is declared idempotent.
        "idempotent": False,
        "effects": effects,
        "requires": ["arm.limits-accepted", "arm.workspace-clear"],
        "evidence": evidence,
        "expected_duration_ms": _ms(_grip_budget_s(profile, gripper)),
    }


def grasp_row(profile: ArmProfile, gripper: GripperProfile) -> dict[str, Any]:
    joint = gripper.joint
    evidence = [
        {
            "kind": "gripper.object_held",
            "observe": HELD_OBSERVATION,
            "phases": ["before", "after"],
            "measure": {"op": "delta", "fields": [HELD_FIELD]},
            # 0 when the jaws stopped, 1 once the hold window confirmed it.
            "expect": {"value": 1},
            "tolerance": {"absolute": 0},
        },
        {
            "kind": "gripper.hold_stable",
            "observe": HELD_OBSERVATION,
            "phases": ["before", "after"],
            "measure": {"op": "delta", "fields": [joint]},
            "expect": {"value": 0},
            "tolerance": {"absolute": gripper.hold_max_drift_rad},
            "unit": "rad",
        },
    ]
    return {
        "module_id": CAPABILITY_GRASP,
        "provides_capability": CAPABILITY_GRASP,
        "label": {"en": "Grasp"},
        "description": (
            "Close the gripper toward its closed limit; completed only when read-back shows "
            "it stopped short of closed on an object and held it."
        ),
        "params_schema": {},
        "timeout_ms": _ms(_grip_budget_s(profile, gripper)),
        "contract": _grip_contract(profile, gripper, ["gripper.object-held"], evidence),
    }


def release_row(profile: ArmProfile, gripper: GripperProfile) -> dict[str, Any]:
    low, high = gripper.release_range
    joint = gripper.joint
    evidence = [
        {
            "kind": "gripper.opened",
            "observe": GRIPPER_OBSERVATION,
            "phases": ["after", "settled"],
            "measure": {"op": "distance_to", "fields": [joint], "frame": FRAME},
            "expect": {"arguments": {joint: RELEASE_ARGUMENT}},
            "tolerance": {
                "absolute": gripper.position_tolerance_rad + gripper.hold_max_drift_rad
            },
            "settle": {"max_drift": gripper.hold_max_drift_rad},
            "unit": "rad",
        }
    ]
    return {
        "module_id": CAPABILITY_RELEASE,
        "provides_capability": CAPABILITY_RELEASE,
        "label": {"en": "Release"},
        "description": (
            "Open the gripper to a position between its release position and its open "
            "limit; completed only when read-back shows it there."
        ),
        "params_schema": {
            RELEASE_ARGUMENT: {
                "type": "number",
                "required": True,
                "min": low,
                "max": high,
                "unit": "rad",
                "description": f"Position to open gripper joint {joint} to",
            }
        },
        "timeout_ms": _ms(_grip_budget_s(profile, gripper)),
        "contract": _grip_contract(profile, gripper, ["gripper.opened"], evidence),
    }


def capability_rows(profile: ArmProfile) -> list[dict[str, Any]]:
    """Every capability this adapter serves for ``profile``, in a stable order."""
    rows = [state_row(profile), move_row(profile), halt_row(profile)]
    if profile.gripper is not None:
        rows += [grasp_row(profile, profile.gripper), release_row(profile, profile.gripper)]
    return rows
