from __future__ import annotations

import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from flyto_robotics.input_gateway import InputGateway, InputGatewayError

TOKEN = "test-only-input-gateway-token-with-32-bytes"


def input_payload(**overrides: object) -> dict[str, object]:
    payload: dict[str, object] = {
        "contract_version": "flyto.robotics.input-event.v1",
        "event_id": "event.1",
        "source_id": "keyboard.main",
        "control_id": "ArrowUp",
        "session_id": "session.1",
        "phase": "press",
        "sequence": 1,
    }
    payload.update(overrides)
    return payload


def request_json(
    url: str,
    *,
    payload: dict[str, object] | None = None,
    token: str = TOKEN,
) -> tuple[int, dict[str, object]]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = Request(
        url,
        data=data,
        method="GET" if payload is None else "POST",
        headers={
            "Authorization": f"Bearer {token}",
            **({"Content-Type": "application/json"} if payload is not None else {}),
        },
    )
    try:
        with urlopen(request, timeout=2.0) as response:
            return response.status, json.load(response)
    except HTTPError as exc:
        return exc.code, json.load(exc)


def test_gateway_accepts_valid_event_and_waits_for_control_ack() -> None:
    with InputGateway(token=TOKEN, port=0) as gateway:
        host, port = gateway.address

        def acknowledge() -> None:
            while True:
                pending = gateway.drain()
                if pending:
                    pending[0].acknowledge(
                        action="start_workflow",
                        reason="input_pressed",
                        workflow_id="shortcut.forward.30cm.v1",
                        robot_state="accepted",
                    )
                    return

        worker = threading.Thread(target=acknowledge)
        worker.start()
        status, result = request_json(
            f"http://{host}:{port}/v1/input-events",
            payload=input_payload(),
        )
        worker.join(timeout=2.0)

    assert status == 200
    assert result["accepted"] is True
    assert result["action"] == "start_workflow"
    assert result["workflow_id"] == "shortcut.forward.30cm.v1"


def test_gateway_rejects_unauthorized_or_motor_shaped_payloads() -> None:
    with InputGateway(token=TOKEN, port=0, ack_timeout_seconds=0.05) as gateway:
        host, port = gateway.address
        url = f"http://{host}:{port}/v1/input-events"
        unauthorized, unauthorized_body = request_json(
            url,
            payload=input_payload(),
            token="wrong-token",
        )
        unsafe, unsafe_body = request_json(
            url,
            payload=input_payload(linear_x=1.0),
        )

    assert unauthorized == 401
    assert unauthorized_body["error"] == "unauthorized"
    assert unsafe == 400
    assert unsafe_body["error"] == "input_event_invalid"


def test_gateway_fails_closed_when_control_thread_does_not_acknowledge() -> None:
    with InputGateway(token=TOKEN, port=0, ack_timeout_seconds=0.05) as gateway:
        host, port = gateway.address
        status, body = request_json(
            f"http://{host}:{port}/v1/input-events",
            payload=input_payload(),
        )

    assert status == 503
    assert body["error"] == "input_ack_timeout"


def test_gateway_health_is_authenticated_and_loopback_only() -> None:
    with InputGateway(token=TOKEN, port=0) as gateway:
        host, port = gateway.address
        status, body = request_json(f"http://{host}:{port}/v1/health")

    assert status == 200
    assert body["service"] == "flyto-robotics-input"
    with pytest.raises(InputGatewayError, match="loopback"):
        InputGateway(token=TOKEN, host="0.0.0.0")
    with pytest.raises(InputGatewayError, match="at least 32"):
        InputGateway(token="short")
