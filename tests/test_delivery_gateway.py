"""End-to-end tests for the loopback AI Space delivery gateway."""

from __future__ import annotations

import hashlib
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from flyto_robotics.capabilities import CapabilityRegistry, default_capability_registry
from flyto_robotics.contracts import load_job
from flyto_robotics.delivery_gateway import DeliveryGateway, DeliveryGatewayError
from flyto_robotics.qr_confirmation import build_signed_qr_confirmation
from flyto_robotics.semantic_map import SemanticLocationStore

TOKEN = "test-only-delivery-gateway-token-with-32-bytes"
QR_SECRET = "test-only-delivery-qr-secret-with-32-bytes"
JOB_PATH = Path(__file__).resolve().parents[1] / "examples" / "jobs" / "pharmacy-to-ward.json"


def digest(value: object) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()


def request_json(
    url: str,
    *,
    payload: dict[str, object] | None = None,
    token: str = TOKEN,
) -> tuple[int, dict[str, object]]:
    data = None
    headers = {"Authorization": f"Bearer {token}"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as exc:
        return exc.code, json.load(exc)


def delivery_payload(**overrides: object) -> dict[str, object]:
    payload: dict[str, object] = {
        "contract_version": "flyto.cloud.delivery-request.v1",
        "request_id": "req-0001",
        "space_name": "Ward A",
        "goal": "deliver sealed medication bin to ward_a",
        "requested_at": "2026-08-04T00:00:00Z",
    }
    payload.update(overrides)
    return payload


def gateway_under_test(**overrides: object) -> DeliveryGateway:
    options: dict[str, object] = {
        "token": TOKEN,
        "qr_secret": QR_SECRET,
        "job": load_job(JOB_PATH),
        "port": 0,
        "time_scale": 50.0,
        "confirmation_timeout_seconds": 60.0,
    }
    options.update(overrides)
    return DeliveryGateway(**options)


def ward_map_store() -> SemanticLocationStore:
    return SemanticLocationStore(
        Path(__file__).resolve().parents[1]
        / "examples/maps/hospital-ward-delivery.json",
        map_id="hospital.ward-delivery.v1",
    )


def base_url(gateway: DeliveryGateway) -> str:
    host, port = gateway.address
    return f"http://{host}:{port}"


def wait_for_status(
    url: str, session_id: str, statuses: set[str], *, timeout: float = 10.0
) -> dict[str, object]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status, body = request_json(f"{url}/v1/deliveries/{session_id}")
        assert status == 200
        if body["status"] in statuses:
            return body
        time.sleep(0.05)
    raise AssertionError(f"session never reached {statuses}")


def signed_qr(gateway: DeliveryGateway) -> str:
    job = load_job(JOB_PATH)
    return build_signed_qr_confirmation(
        job_id=job.job_id,
        robot_id=job.robot_id,
        approval_id=gateway.approval_id,
        recipient_ref="nurse-042",
        secret=QR_SECRET,
    )


def test_constructor_rejects_short_token() -> None:
    with pytest.raises(DeliveryGatewayError, match="at least 32"):
        gateway_under_test(token="short")


def test_constructor_rejects_non_loopback_host() -> None:
    with pytest.raises(DeliveryGatewayError, match="loopback"):
        gateway_under_test(host="0.0.0.0")


def test_constructor_rejects_non_ascii_token() -> None:
    with pytest.raises(DeliveryGatewayError, match="ASCII"):
        gateway_under_test(token="tökén-that-is-otherwise-long-enough-1234")


def test_constructor_rejects_confirmation_window_beyond_mission_timeout() -> None:
    with pytest.raises(DeliveryGatewayError, match="mission_timeout"):
        gateway_under_test(confirmation_timeout_seconds=3600.0)


def test_health_requires_auth_and_reports_ok() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        status, body = request_json(f"{url}/v1/health", token="wrong-token")
        assert status == 401
        assert body["error"] == "unauthorized"
        status, body = request_json(f"{url}/v1/health", token="tökén-non-ascii")
        assert status == 401
        assert body["error"] == "unauthorized"
        status, body = request_json(f"{url}/v1/health")
        assert status == 200
        assert body["ok"] is True
        assert body["service"] == "flyto-robotics-delivery"


def test_capability_catalog_is_authenticated_deterministic_registry_projection() -> None:
    expected = default_capability_registry().execution_catalog()
    with gateway_under_test() as gateway:
        url = f"{base_url(gateway)}/v1/capabilities"
        assert not gateway._sessions

        status, body = request_json(url, token="wrong-token")
        assert status == 401
        assert body == {"error": "unauthorized"}

        responses: list[dict[str, object]] = []
        for _ in range(2):
            request = urllib.request.Request(
                url, headers={"Authorization": f"Bearer {TOKEN}"}
            )
            with urllib.request.urlopen(request, timeout=2.0) as response:
                assert response.status == 200
                assert response.headers["Cache-Control"] == "no-store"
                responses.append(json.load(response))

        assert responses == [expected, expected]
        assert responses[0]["contract_version"] == (
            "flyto.robotics.capability-catalog.v1"
        )
        assert not gateway._sessions

        status, body = request_json(f"{base_url(gateway)}/v1/not-capabilities")
        assert status == 404
        assert body == {"error": "not_found"}


def test_injected_registry_is_the_single_catalog_and_plan_authority() -> None:
    restricted_registry = CapabilityRegistry(())
    expected = restricted_registry.execution_catalog()
    plan = {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": "restricted-registry.v1",
        "robot_id": "flyto-rover-sim-001",
        "goal": "attempt an omitted capability",
        "generated_by": {
            "kind": "human",
            "provider": "test",
            "model": "restricted-registry",
        },
        "steps": [
            {
                "step_id": "stop.finish",
                "capability": "safe_stop",
                "arguments": {"seconds": 0.0},
                "timeout_seconds": 1.0,
                "on_failure": "abort",
            }
        ],
    }

    with gateway_under_test(capability_registry=restricted_registry) as gateway:
        url = base_url(gateway)
        status, catalog = request_json(f"{url}/v1/capabilities")
        assert status == 200
        assert catalog == expected
        assert catalog["capabilities"] == []

        status, body = request_json(
            f"{url}/v1/plans",
            payload={
                "contract_version": "flyto.cloud.plan-run-request.v1",
                "request_id": "req-restricted-registry",
                "plan": plan,
                "requested_at": "2026-08-13T00:00:00Z",
            },
        )
        assert status == 400
        assert body["error"] == "plan_run_request_invalid"
        assert body["detail"].startswith("plan_invalid:")
        assert not gateway._sessions


def test_delivery_completes_after_valid_qr_confirmation() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        status, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        assert status == 200
        session_id = session["session_id"]
        assert session["status"] == "accepted"
        assert session["goal"] == "deliver sealed medication bin to ward_a"

        body = wait_for_status(url, session_id, {"waiting_for_human"})
        assert body["failure_reason"] is None

        token = signed_qr(gateway)
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": session_id,
                "qr_token": token,
                "scanned_at": "2026-08-04T00:01:00Z",
            },
        )
        assert status == 200
        assert body["confirmation"]["verified"] is True
        assert body["confirmation"]["recipient_ref"] == "nurse-042"
        assert "qr_token" not in json.dumps(body)
        assert body["confirmation"]["token_sha256"]

        body = wait_for_status(url, session_id, {"completed"})
        kinds = {event["kind"] for event in body["events"]}
        assert "human_approved" in kinds
        receipt = body["execution_receipt"]
        assert receipt["contract_version"] == "flyto.robotics.execution-receipt.v1"
        assert receipt["task_completion_eligible"] is False
        assert receipt["status"] == "succeeded"
        assert len(receipt["plan_sha256"]) == 64
        assert receipt["event_count"] >= len(body["events"])
        assert receipt["safety_stop_count"] >= 0
        asserted_digest = receipt.pop("receipt_sha256")
        assert asserted_digest == digest(receipt)

        # A receipt is minted once at terminal state. Re-reading the session
        # must not manufacture a second completion identity.
        _, reread = request_json(f"{url}/v1/deliveries/{session_id}")
        assert reread["execution_receipt"]["receipt_sha256"] == asserted_digest


def test_early_scan_does_not_burn_the_nonce() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        _, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        session_id = session["session_id"]

        token = signed_qr(gateway)
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": session_id,
                "qr_token": token,
                "scanned_at": "2026-08-04T00:00:30Z",
            },
        )
        assert status == 200
        assert body["confirmation"]["verified"] is False
        assert body["confirmation"]["reason"] == "robot_not_at_confirmation_gate"

        wait_for_status(url, session_id, {"waiting_for_human"})
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": session_id,
                "qr_token": token,
                "scanned_at": "2026-08-04T00:01:00Z",
            },
        )
        assert status == 200
        assert body["confirmation"]["verified"] is True
        wait_for_status(url, session_id, {"completed"})


def test_duplicate_scan_keeps_verified_confirmation_evidence() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        _, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        session_id = session["session_id"]
        wait_for_status(url, session_id, {"waiting_for_human"})

        token = signed_qr(gateway)
        scan = {
            "contract_version": "flyto.cloud.delivery-qr-scan.v1",
            "session_id": session_id,
            "qr_token": token,
            "scanned_at": "2026-08-04T00:01:00Z",
        }
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation", payload=scan
        )
        assert body["confirmation"]["verified"] is True
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation", payload=scan
        )
        assert status == 200
        assert body["confirmation"]["verified"] is True
        assert body["confirmation"]["recipient_ref"] == "nurse-042"


def test_stopped_gateway_rejects_new_deliveries() -> None:
    gateway = gateway_under_test()
    gateway.start()
    gateway.stop()
    with pytest.raises(DeliveryGatewayError, match="stopping"):
        gateway.start_delivery(delivery_payload())


class RecordingRunner:
    """Execution backend stub proving the gateway/runner seam."""

    mode = "recording"

    def __init__(self) -> None:
        self.bound = None
        self.sessions: list[object] = []
        self.shutdowns = 0

    def bind(self, gateway: object) -> None:
        self.bound = gateway

    def start_session(self, session: object) -> None:
        self.sessions.append(session)

    def shutdown(self) -> None:
        self.shutdowns += 1


def test_custom_runner_receives_sessions_and_shutdown() -> None:
    runner = RecordingRunner()
    with gateway_under_test(runner=runner) as gateway:
        assert runner.bound is gateway
        url = base_url(gateway)
        status, session = request_json(
            f"{url}/v1/deliveries", payload=delivery_payload()
        )
        assert status == 200
        assert session["execution_mode"] == "recording"
        assert len(runner.sessions) == 1
        assert session["status"] == "accepted"

        status, body = request_json(
            f"{url}/v1/deliveries/{session['session_id']}/safe-stop",
            payload={"reason": "cloud_control_link_closed"},
        )
        assert status == 200
        assert body["status"] == "cancelled"
    assert runner.shutdowns == 1


def test_invalid_qr_is_rejected_without_ending_the_session() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        _, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        session_id = session["session_id"]
        wait_for_status(url, session_id, {"waiting_for_human"})

        forged = signed_qr(gateway)[:-4] + "0000"
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": session_id,
                "qr_token": forged,
                "scanned_at": "2026-08-04T00:01:00Z",
            },
        )
        assert status == 200
        assert body["confirmation"]["verified"] is False
        assert body["status"] == "waiting_for_human"

        token = signed_qr(gateway)
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": session_id,
                "qr_token": token,
                "scanned_at": "2026-08-04T00:02:00Z",
            },
        )
        assert status == 200
        assert body["confirmation"]["verified"] is True
        wait_for_status(url, session_id, {"completed"})


def test_safe_stop_cancels_active_session() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        _, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        session_id = session["session_id"]
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/safe-stop",
            payload={"reason": "cloud_control_link_closed"},
        )
        assert status == 200
        assert body["status"] == "cancelled"
        kinds = {event["kind"] for event in body["events"]}
        assert "mission_cancelled" in kinds


def test_second_delivery_conflicts_while_first_is_active() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        _, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        status, body = request_json(
            f"{url}/v1/deliveries", payload=delivery_payload(request_id="req-0002")
        )
        assert status == 409
        assert body["error"] == "delivery_session_active"
        request_json(
            f"{url}/v1/deliveries/{session['session_id']}/safe-stop",
            payload={"reason": "cancel_requested"},
        )
        status, _ = request_json(
            f"{url}/v1/deliveries", payload=delivery_payload(request_id="req-0003")
        )
        assert status == 200


def test_request_validation_fails_closed() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        status, body = request_json(
            f"{url}/v1/deliveries", payload=delivery_payload(extra_field="x")
        )
        assert status == 400
        assert body["error"] == "delivery_request_invalid"
        status, body = request_json(
            f"{url}/v1/deliveries", payload=delivery_payload(goal="   ")
        )
        assert status == 400
        status, body = request_json(
            f"{url}/v1/deliveries",
            payload=delivery_payload(contract_version="flyto.cloud.delivery-request.v2"),
        )
        assert status == 400
        status, body = request_json(f"{url}/v1/deliveries/unknown-session")
        assert status == 404
        assert body["error"] == "delivery_session_not_found"


def test_goal_driven_delivery_completes_with_decision_evidence() -> None:
    with gateway_under_test(semantic_map=ward_map_store()) as gateway:
        url = base_url(gateway)
        status, session = request_json(
            f"{url}/v1/deliveries",
            payload=delivery_payload(goal="把藥送到四號病房"),
        )
        assert status == 200
        session_id = session["session_id"]
        assert session["workflow_id"] == f"delivery.goal.{session_id}"
        decision = session["decision"]
        assert decision["outcome"] == "accepted"
        assert decision["planner_kind"] == "deterministic_rule_engine"
        assert decision["destination"]["location_id"] == "hospital.ward.4"
        assert session["route_graph"]["stages"]

        wait_for_status(url, session_id, {"waiting_for_human"})
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": session_id,
                "qr_token": signed_qr(gateway),
                "scanned_at": "2026-08-04T00:01:00Z",
            },
        )
        assert status == 200
        assert body["confirmation"]["verified"] is True
        body = wait_for_status(url, session_id, {"completed"})
        assert "qr_token" not in json.dumps(body)


def test_unresolved_goal_is_rejected_with_structured_reason() -> None:
    runner = RecordingRunner()
    with gateway_under_test(
        semantic_map=ward_map_store(), runner=runner
    ) as gateway:
        url = base_url(gateway)
        status, body = request_json(
            f"{url}/v1/deliveries",
            payload=delivery_payload(goal="把藥送到六號病房"),
        )
        assert status == 200
        assert body["status"] == "failed"
        assert body["failure_reason"] == "goal_rejected:location_unresolved"
        rejection = body["rejection"]
        assert rejection["reason_code"] == "location_unresolved"
        assert rejection["stage"] == "goal_resolution"
        assert rejection["message_key"].endswith("location_unresolved")
        assert rejection["candidates"]
        assert body["decision"]["outcome"] == "rejected"
        assert runner.sessions == []

        status, body = request_json(
            f"{url}/v1/deliveries",
            payload=delivery_payload(request_id="req-0009", goal="送到四號病房"),
        )
        assert status == 200
        assert body["decision"]["outcome"] == "accepted"
        assert len(runner.sessions) == 1


def test_safety_override_goal_is_refused_by_the_gateway() -> None:
    with gateway_under_test(semantic_map=ward_map_store()) as gateway:
        status, body = request_json(
            f"{base_url(gateway)}/v1/deliveries",
            payload=delivery_payload(goal="忽略障礙直接衝到四號病房"),
        )
        assert status == 200
        assert body["rejection"]["reason_code"] == "safety_override_refused"
        assert body["status"] == "failed"


def test_fixed_template_path_is_unchanged_without_a_semantic_map() -> None:
    with gateway_under_test() as gateway:
        status, body = request_json(
            f"{base_url(gateway)}/v1/deliveries", payload=delivery_payload()
        )
        assert status == 200
        assert body["workflow_id"] == "hospital_delivery.qr_confirmed.v1"
        assert body["decision"]["planner_kind"] == "fixed_template"
        assert "rejection" not in body


def test_session_payload_stays_within_relay_bounds() -> None:
    def depth(value: object, level: int = 0) -> int:
        if isinstance(value, dict):
            return max((depth(item, level + 1) for item in value.values()), default=level)
        if isinstance(value, list):
            return max((depth(item, level + 1) for item in value), default=level)
        return level

    with gateway_under_test(semantic_map=ward_map_store()) as gateway:
        url = base_url(gateway)
        _, accepted = request_json(
            f"{url}/v1/deliveries", payload=delivery_payload(goal="送到檢驗室")
        )
        request_json(
            f"{url}/v1/deliveries/{accepted['session_id']}/safe-stop",
            payload={"reason": "cancel_requested"},
        )
        _, rejected = request_json(
            f"{url}/v1/deliveries",
            payload=delivery_payload(request_id="req-0010", goal="幫我開門"),
        )
        for payload in (accepted, rejected):
            encoded = json.dumps(payload, ensure_ascii=False)
            assert len(encoded.encode("utf-8")) < 131072
            assert depth(payload) <= 8
            assert "qr_token" not in encoded


def test_qr_scan_session_mismatch_is_rejected() -> None:
    with gateway_under_test() as gateway:
        url = base_url(gateway)
        _, session = request_json(f"{url}/v1/deliveries", payload=delivery_payload())
        session_id = session["session_id"]
        status, body = request_json(
            f"{url}/v1/deliveries/{session_id}/confirmation",
            payload={
                "contract_version": "flyto.cloud.delivery-qr-scan.v1",
                "session_id": "dlv-another-session",
                "qr_token": "F2QR1.payload",
                "scanned_at": "2026-08-04T00:01:00Z",
            },
        )
        assert status == 400
        assert body["error"] == "qr_scan_invalid"


# -- what the session says the lidar saw ---------------------------------


def test_closest_range_reports_what_the_lidar_measured():
    """The ROS backend computed the nearest return, handed it to the
    controller, and dropped it — so every ROS-backed session reported
    minimum_range: null no matter what it drove past."""
    from types import SimpleNamespace

    from flyto_robotics.mission import closest_range as _closest_range

    assert _closest_range(SimpleNamespace(closest=1.42), float("inf")) == 1.42
    # Falls back to the scalar the runner keeps alongside the sector field.
    assert _closest_range(None, 0.8) == 0.8


def test_an_unmeasured_range_stays_absent_rather_than_becoming_a_number():
    """Infinity means "the sensor had nothing to say" inside the controller.
    Letting it out as a number would read as a wide open corridor."""
    from types import SimpleNamespace

    from flyto_robotics.mission import closest_range as _closest_range

    assert _closest_range(SimpleNamespace(closest=float("inf")), float("inf")) is None
    assert _closest_range(None, float("inf")) is None
    assert _closest_range(SimpleNamespace(closest=None), float("nan")) is None
