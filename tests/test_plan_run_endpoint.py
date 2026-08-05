"""POST /v1/plans — a caller-authored plan runs through every existing guard."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from flyto_robotics.contracts import load_job
from flyto_robotics.delivery_gateway import (
    DeliveryGateway,
    DeliveryGatewayError,
    DeliverySessionConflictError,
    parse_plan_run_request,
)

TOKEN = "test-only-delivery-gateway-token-with-32-bytes"
QR_SECRET = "test-only-delivery-qr-secret-with-32-bytes"
ROOT = Path(__file__).resolve().parents[1]
JOB_PATH = ROOT / "examples" / "jobs" / "tb3-lab-shortcut.json"
FORWARD_PLAN = ROOT / "examples" / "plans" / "shortcut-forward-40cm.json"


def gateway() -> DeliveryGateway:
    return DeliveryGateway(
        token=TOKEN,
        qr_secret=QR_SECRET,
        job=load_job(JOB_PATH),
        port=0,
        time_scale=50.0,
        confirmation_timeout_seconds=10.0,
    )


def plan_body(**plan_overrides: object) -> dict[str, object]:
    plan = copy.deepcopy(json.loads(FORWARD_PLAN.read_text()))
    plan.update(plan_overrides)
    return {
        "contract_version": "flyto.cloud.plan-run-request.v1",
        "request_id": "req-plan-0001",
        "plan": plan,
        "requested_at": "2026-08-05T00:00:00Z",
    }


# -- the wrapper contract ------------------------------------------------


def test_a_valid_wrapper_is_accepted():
    parsed = parse_plan_run_request(plan_body())
    assert parsed["request_id"] == "req-plan-0001"
    assert parsed["plan"]["plan_id"] == "shortcut.forward.40cm.v1"


@pytest.mark.parametrize(
    "smuggled", ["speed", "cmd_vel_topic", "token", "host", "parameters"]
)
def test_an_unknown_wrapper_field_is_refused(smuggled):
    body = plan_body()
    body[smuggled] = "anything"
    with pytest.raises(DeliveryGatewayError):
        parse_plan_run_request(body)


def test_a_wrong_contract_version_is_refused():
    body = plan_body()
    body["contract_version"] = "flyto.cloud.plan-run-request.v2"
    with pytest.raises(DeliveryGatewayError):
        parse_plan_run_request(body)


def test_a_plan_that_is_not_an_object_is_refused():
    body = plan_body()
    body["plan"] = "shortcut-forward-40cm.json"
    with pytest.raises(DeliveryGatewayError):
        parse_plan_run_request(body)


# -- the plan itself ----------------------------------------------------


def test_a_valid_plan_starts_a_session():
    payload = gateway().start_plan(plan_body())
    assert payload["session_id"].startswith("pln-")
    assert payload["goal"] == "向前移動四十公分後安全停止"


def test_a_plan_for_another_robot_is_refused():
    """The same guard run-ros applies, so a plan cannot be aimed at a stranger."""
    with pytest.raises(DeliveryGatewayError, match="robot_id"):
        gateway().start_plan(plan_body(robot_id="flyto-rover-sim-001"))


def test_a_plan_naming_an_unregistered_capability_is_refused():
    """parse_plan is the boundary that treats a plan as hostile input."""
    body = plan_body()
    body["plan"]["steps"][0]["capability"] = "teleport"
    with pytest.raises(DeliveryGatewayError, match="plan_invalid"):
        gateway().start_plan(body)


def test_a_plan_that_moves_must_end_with_safe_stop():
    """Otherwise a caller could leave the robot driving by omitting one step."""
    body = plan_body()
    body["plan"]["steps"] = [body["plan"]["steps"][0]]
    with pytest.raises(DeliveryGatewayError, match="safe_stop"):
        gateway().start_plan(body)


def test_an_out_of_range_argument_is_refused():
    body = plan_body()
    body["plan"]["steps"][0]["arguments"]["speed"] = 99.0
    with pytest.raises(DeliveryGatewayError, match="plan_invalid"):
        gateway().start_plan(body)


def test_only_one_mission_runs_at_a_time():
    """One process owns the robot, which is why the HTTP hop exists at all."""
    live = gateway()
    live.start_plan(plan_body())
    with pytest.raises(DeliverySessionConflictError):
        live.start_plan(plan_body(plan_id="shortcut.forward.40cm.v1"))
