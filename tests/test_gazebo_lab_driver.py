from __future__ import annotations

import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER_FILE = ROOT / "flyto_robotics/gazebo_lab_driver.py"


def _load_geometry_function():
    tree = ast.parse(DRIVER_FILE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "point_ahead_from_quaternion"
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {"math": math}
    exec(compile(module, str(DRIVER_FILE), "exec"), namespace)
    return namespace["point_ahead_from_quaternion"]


def _quaternion_for_yaw(yaw: float) -> SimpleNamespace:
    return SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def test_obstacle_geometry_uses_live_heading_on_curved_routes() -> None:
    point_ahead = _load_geometry_function()

    obstacle_x, obstacle_y, yaw = point_ahead(
        x=-2.24,
        y=-0.23,
        lead_distance=0.8,
        orientation=_quaternion_for_yaw(-math.pi / 2.0),
    )

    assert obstacle_x == pytest.approx(-2.24)
    assert obstacle_y == pytest.approx(-1.03)
    assert yaw == pytest.approx(-math.pi / 2.0)


def test_tick_uses_live_world_yaw_and_records_path_length() -> None:
    source = DRIVER_FILE.read_text(encoding="utf-8")

    assert 'robot_yaw = float(self.latest_world_pose["yaw"])' in source
    assert "robot_y = self.obstacle_active_y" in source
    assert '"world_path_length": round(self.world_path_length, 6)' in source
    assert "self.world_path_length += math.hypot" in source


def test_replay_delay_is_short_but_configurable() -> None:
    source = DRIVER_FILE.read_text(encoding="utf-8")

    assert 'self.declare_parameter("replay_initial_delay_seconds", 0.05)' in source
    assert "self._elapsed() + self.replay_initial_delay_seconds" in source


def test_delivery_gate_uses_verified_qr_before_existing_human_decision() -> None:
    source = DRIVER_FILE.read_text(encoding="utf-8")

    qr_build = source.index("token = build_signed_qr_confirmation(")
    qr_verify = source.index("confirmation = self.qr_authenticator.verify(")
    decision_build = source.index("decision = qr_confirmation_to_human_decision(")
    publish = source.index("self.decision_publisher.publish(message)")

    assert qr_build < qr_verify < decision_build < publish
    assert '"qr_confirmation_verified"' in source
    assert '"qr_confirmation_replay_rejected"' in source
    assert '"raw_token_persisted": False' in source
    assert "FLYTO_ROBOTICS_QR_SECRET" in source
    assert 'self._capture("approval")' in source


def test_guarded_handoff_completes_before_qr_approval_is_published() -> None:
    source = DRIVER_FILE.read_text(encoding="utf-8")

    handoff = source.index(
        'self._record(\n                "guarded_handoff_approved"'
    )
    publish_after_handoff = source.index(
        "self._publish_approval(approval_id)",
        handoff,
    )

    assert handoff < publish_after_handoff
    assert '"item_rejected"' in source
    assert '"recipient_rejected"' in source
    assert "container_locked=event[\"container_locked\"]" in source


@pytest.mark.parametrize(
    "scenario_name",
    ("ai4all-branching.json", "careflow-adversarial.json"),
)
def test_gazebo_scenarios_require_the_verified_qr_actor(
    scenario_name: str,
) -> None:
    scenario = json.loads(
        (ROOT / "scenarios/gazebo" / scenario_name).read_text(encoding="utf-8")
    )

    assert scenario["expectations"]["required_actor_ids"] == [
        "qr.ward-b.receiver"
    ]
    assert "delivery QR" in scenario["description"]


def test_medication_scenario_requires_patient_qr_and_failure_captures() -> None:
    scenario = json.loads(
        (
            ROOT
            / "scenarios/gazebo/ai4all-medication-handoff.json"
        ).read_text(encoding="utf-8")
    )

    assert scenario["expectations"]["required_actor_ids"] == ["qr.patient-12"]
    assert {
        "item_rejected",
        "recipient_rejected",
    }.issubset(scenario["expectations"]["required_capture_labels"])
