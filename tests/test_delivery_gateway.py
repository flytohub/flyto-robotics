"""End-to-end tests for the loopback AI Space delivery gateway."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from flyto_robotics.contracts import load_job
from flyto_robotics.delivery_gateway import DeliveryGateway, DeliveryGatewayError
from flyto_robotics.qr_confirmation import build_signed_qr_confirmation

TOKEN = "test-only-delivery-gateway-token-with-32-bytes"
QR_SECRET = "test-only-delivery-qr-secret-with-32-bytes"
JOB_PATH = Path(__file__).resolve().parents[1] / "examples" / "jobs" / "pharmacy-to-ward.json"


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
