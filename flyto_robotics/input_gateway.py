"""Authenticated loopback HTTP transport for shortcut input events."""

from __future__ import annotations

import hmac
import ipaddress
import json
import queue
import threading
from collections.abc import Mapping
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from .input_runtime import InputEvent, InputValidationError, parse_input_event

INPUT_ACK_CONTRACT_VERSION = "flyto.robotics.input-ack.v1"
MAX_INPUT_BODY_BYTES = 8192
MAX_PENDING_INPUTS = 256


class InputGatewayError(ValueError):
    """Raised when the local gateway cannot be configured safely."""


@dataclass
class QueuedInput:
    """One accepted event awaiting the deterministic ROS control thread."""

    event: InputEvent
    _completed: threading.Event = field(default_factory=threading.Event)
    _response: dict[str, object] | None = None

    def acknowledge(
        self,
        *,
        action: str,
        reason: str,
        workflow_id: str | None,
        robot_state: str,
    ) -> None:
        self._response = {
            "contract_version": INPUT_ACK_CONTRACT_VERSION,
            "accepted": True,
            "event_id": self.event.event_id,
            "action": action,
            "reason": reason,
            "workflow_id": workflow_id,
            "robot_state": robot_state,
        }
        self._completed.set()

    def wait(self, timeout_seconds: float) -> dict[str, object] | None:
        if not self._completed.wait(timeout_seconds):
            return None
        return self._response


class InputGateway:
    """Small local-only HTTP server; the browser never receives its token."""

    def __init__(
        self,
        *,
        token: str,
        host: str = "127.0.0.1",
        port: int = 8765,
        ack_timeout_seconds: float = 0.75,
    ) -> None:
        token_bytes = token.encode("utf-8")
        if len(token_bytes) < 32:
            raise InputGatewayError("input gateway token must be at least 32 bytes")
        try:
            address = ipaddress.ip_address(host)
        except ValueError as exc:
            raise InputGatewayError("input gateway host must be a loopback IP") from exc
        if not address.is_loopback:
            raise InputGatewayError("input gateway may bind only to loopback")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise InputGatewayError("input gateway port must be between 0 and 65535")
        if not 0.05 <= ack_timeout_seconds <= 2.0:
            raise InputGatewayError("ack timeout must be between 0.05 and 2.0 seconds")

        self._token = token
        self._host = host
        self._port = port
        self._ack_timeout_seconds = ack_timeout_seconds
        self._pending: queue.Queue[QueuedInput] = queue.Queue(
            maxsize=MAX_PENDING_INPUTS
        )
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        if self._server is None:
            return self._host, self._port
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    def start(self) -> None:
        if self._server is not None:
            return
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "FlytoRoboticsInput/1"

            def log_message(self, _format: str, *args: object) -> None:
                return

            def _send_json(
                self,
                status: HTTPStatus,
                payload: Mapping[str, object],
            ) -> None:
                body = json.dumps(
                    dict(payload),
                    ensure_ascii=False,
                    separators=(",", ":"),
                    sort_keys=True,
                ).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _authorized(self) -> bool:
                supplied = self.headers.get("Authorization", "")
                expected = f"Bearer {gateway._token}"
                return hmac.compare_digest(supplied, expected)

            def do_GET(self) -> None:  # noqa: N802
                if self.path != "/v1/health":
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
                    return
                if not self._authorized():
                    self._send_json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
                    return
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "service": "flyto-robotics-input",
                        "contract_version": INPUT_ACK_CONTRACT_VERSION,
                    },
                )

            def do_POST(self) -> None:  # noqa: N802
                if self.path != "/v1/input-events":
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
                    return
                if not self._authorized():
                    self._send_json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
                    return
                if self.headers.get_content_type() != "application/json":
                    self._send_json(
                        HTTPStatus.UNSUPPORTED_MEDIA_TYPE,
                        {"error": "application_json_required"},
                    )
                    return
                try:
                    content_length = int(self.headers.get("Content-Length", ""))
                except ValueError:
                    content_length = -1
                if not 1 <= content_length <= MAX_INPUT_BODY_BYTES:
                    self._send_json(
                        HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                        {"error": "input_event_size_invalid"},
                    )
                    return
                try:
                    decoded: Any = json.loads(
                        self.rfile.read(content_length).decode("utf-8")
                    )
                    event = parse_input_event(decoded)
                except (UnicodeError, json.JSONDecodeError, InputValidationError) as exc:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "input_event_invalid", "detail": str(exc)[:160]},
                    )
                    return
                pending = QueuedInput(event)
                try:
                    gateway._pending.put_nowait(pending)
                except queue.Full:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "input_queue_full"},
                    )
                    return
                response = pending.wait(gateway._ack_timeout_seconds)
                if response is None:
                    self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {
                            "error": "input_ack_timeout",
                            "detail": "control thread did not acknowledge the event",
                        },
                    )
                    return
                self._send_json(HTTPStatus.OK, response)

        self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="flyto-robotics-input-gateway",
            daemon=True,
        )
        self._thread.start()

    def drain(self, *, maximum: int = 32) -> tuple[QueuedInput, ...]:
        if not 1 <= maximum <= 256:
            raise InputGatewayError("drain maximum must be between 1 and 256")
        drained: list[QueuedInput] = []
        for _ in range(maximum):
            try:
                drained.append(self._pending.get_nowait())
            except queue.Empty:
                break
        return tuple(drained)

    def stop(self) -> None:
        server = self._server
        thread = self._thread
        self._server = None
        self._thread = None
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2.0)

    def __enter__(self) -> InputGateway:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()
