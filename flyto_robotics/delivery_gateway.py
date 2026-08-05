"""Loopback delivery gateway bridging Flyto Cloud AI Space to the mission runtime."""

from __future__ import annotations

import hmac
import ipaddress
import json
import math
import re
import threading
import time
import uuid
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from .ai_planner import PlanValidationError, compile_workflow, parse_plan
from .contracts import DeliveryJob
from .goal_planner import (
    DeterministicDeliveryGoalPlanner,
    FixedTemplateGoalPlanner,
    GoalDecision,
)
from .input_runtime import MOTION_PRIMITIVES
from .mission import MissionController, Pose2D, normalize_angle
from .qr_confirmation import (
    QRConfirmationAuthenticator,
    QRConfirmationValidationError,
    qr_token_sha256,
)
from .semantic_map import SemanticLocationMap, SemanticLocationStore
from .workflow import (
    MissionState,
    PrimitiveKind,
    WorkflowPlan,
    WorkflowStep,
)

DELIVERY_REQUEST_CONTRACT_VERSION = "flyto.cloud.delivery-request.v1"
QR_SCAN_CONTRACT_VERSION = "flyto.cloud.delivery-qr-scan.v1"
DELIVERY_SESSION_CONTRACT_VERSION = "flyto.robotics.delivery-session.v1"
DELIVERY_SERVICE_NAME = "flyto-robotics-delivery"

SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
QR_TOKEN_PREFIX = "F2QR1."
# A 16 KiB qr_token can inflate up to ~6x under JSON string escaping; the body
# bound must stay above that so an oversized-but-relay-legal scan is rejected
# as evidence, not as a session-killing 413.
MAX_DELIVERY_BODY_BYTES = 131072
MAX_QR_TOKEN_BYTES = 16384
MAX_GOAL_BYTES = 2000
MAX_SPACE_NAME_BYTES = 200
MAX_TIMESTAMP_BYTES = 64
MAX_SESSION_EVENTS = 32
MAX_RETAINED_SESSIONS = 16
SIMULATION_TIMESTEP_SECONDS = 0.05

DELIVERY_REQUEST_FIELDS = frozenset(
    {"contract_version", "request_id", "space_name", "goal", "requested_at"}
)
QR_SCAN_FIELDS = frozenset({"contract_version", "session_id", "qr_token", "scanned_at"})
SAFE_STOP_FIELDS = frozenset({"reason"})

SESSION_PATH = re.compile(r"^/v1/deliveries/([A-Za-z0-9][A-Za-z0-9._:-]{0,127})$")
CONFIRMATION_PATH = re.compile(
    r"^/v1/deliveries/([A-Za-z0-9][A-Za-z0-9._:-]{0,127})/confirmation$"
)
PLAN_RUN_REQUEST_CONTRACT_VERSION = "flyto.cloud.plan-run-request.v1"
PLAN_RUN_REQUEST_FIELDS = frozenset(
    {"contract_version", "request_id", "plan", "requested_at"}
)

SAFE_STOP_PATH = re.compile(
    r"^/v1/deliveries/([A-Za-z0-9][A-Za-z0-9._:-]{0,127})/safe-stop$"
)


class DeliveryGatewayError(ValueError):
    """Raised when a delivery request violates the gateway contract."""


class DeliverySessionConflictError(DeliveryGatewayError):
    """Raised when a new delivery is requested while one is still active."""


def delivery_confirmation_workflow(
    job: DeliveryJob,
    *,
    approval_id: str,
    confirmation_timeout_seconds: float,
) -> WorkflowPlan:
    """Compile a delivery that gates the dropoff behind one signed QR scan."""
    return WorkflowPlan(
        workflow_id="hospital_delivery.qr_confirmed.v1",
        steps=(
            WorkflowStep(
                step_id="navigate.pickup",
                kind=PrimitiveKind.NAVIGATE,
                active_state=MissionState.NAVIGATING_TO_PICKUP,
                station=job.pickup,
                timeout_seconds=job.safety.mission_timeout_seconds,
            ),
            WorkflowStep(
                step_id="dwell.pickup",
                kind=PrimitiveKind.DWELL,
                active_state=MissionState.WAITING_FOR_PICKUP,
                station=job.pickup,
                dwell_seconds=job.safety.pickup_dwell_seconds,
                timeout_seconds=max(1.0, job.safety.pickup_dwell_seconds + 1.0),
            ),
            WorkflowStep(
                step_id="navigate.dropoff",
                kind=PrimitiveKind.NAVIGATE,
                active_state=MissionState.NAVIGATING_TO_DROPOFF,
                station=job.dropoff,
                timeout_seconds=job.safety.mission_timeout_seconds,
            ),
            WorkflowStep(
                step_id="confirm.dropoff",
                kind=PrimitiveKind.ASK_HUMAN,
                active_state=MissionState.WAITING_FOR_HUMAN,
                station=job.dropoff,
                arguments=(
                    ("approval_id", approval_id),
                    ("prompt_key", "delivery.qr_confirmation"),
                ),
                timeout_seconds=confirmation_timeout_seconds,
            ),
            WorkflowStep(
                step_id="dwell.dropoff",
                kind=PrimitiveKind.DWELL,
                active_state=MissionState.WAITING_FOR_DROPOFF,
                station=job.dropoff,
                dwell_seconds=job.safety.dropoff_dwell_seconds,
                timeout_seconds=max(1.0, job.safety.dropoff_dwell_seconds + 1.0),
            ),
        ),
    )


def _bounded_text(value: Any, field_name: str, *, minimum: int, maximum: int) -> str:
    if not isinstance(value, str):
        raise DeliveryGatewayError(f"{field_name} must be a string")
    encoded = len(value.encode("utf-8"))
    if not minimum <= encoded <= maximum:
        raise DeliveryGatewayError(
            f"{field_name} must be between {minimum} and {maximum} bytes"
        )
    return value


def _identifier(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not SAFE_IDENTIFIER.fullmatch(value):
        raise DeliveryGatewayError(f"{field_name} must be a safe identifier")
    return value


def _exact_fields(data: Any, allowed: frozenset[str], field_name: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise DeliveryGatewayError(f"{field_name} must be an object")
    if set(data) != allowed:
        raise DeliveryGatewayError(
            f"{field_name} must contain exactly: {', '.join(sorted(allowed))}"
        )
    return data


def parse_delivery_request(value: Any) -> dict[str, str]:
    """Validate one flyto.cloud.delivery-request.v1 payload, fail closed."""
    data = _exact_fields(value, DELIVERY_REQUEST_FIELDS, "delivery request")
    if data["contract_version"] != DELIVERY_REQUEST_CONTRACT_VERSION:
        raise DeliveryGatewayError("delivery request contract_version is unsupported")
    goal = _bounded_text(data["goal"], "goal", minimum=1, maximum=MAX_GOAL_BYTES).strip()
    if not goal:
        raise DeliveryGatewayError("goal must not be blank")
    return {
        "request_id": _identifier(data["request_id"], "request_id"),
        "space_name": _bounded_text(
            data["space_name"], "space_name", minimum=0, maximum=MAX_SPACE_NAME_BYTES
        ),
        "goal": goal,
        "requested_at": _bounded_text(
            data["requested_at"], "requested_at", minimum=1, maximum=MAX_TIMESTAMP_BYTES
        ),
    }


def parse_plan_run_request(value: Any) -> dict[str, Any]:
    """Validate one flyto.cloud.plan-run-request.v1 payload, fail closed.

    The wrapper is checked here; the plan inside it is checked by ``parse_plan``
    against the frozen capability registry, which is the boundary that already
    treats a plan as hostile input. Keeping the two separate means this function
    never has to know what a capability is.
    """
    data = _exact_fields(value, PLAN_RUN_REQUEST_FIELDS, "plan run request")
    if data["contract_version"] != PLAN_RUN_REQUEST_CONTRACT_VERSION:
        raise DeliveryGatewayError("plan run request contract_version is unsupported")
    if not isinstance(data["plan"], dict):
        raise DeliveryGatewayError("plan must be an object")
    return {
        "request_id": _identifier(data["request_id"], "request_id"),
        "plan": data["plan"],
        "requested_at": _bounded_text(
            data["requested_at"], "requested_at", minimum=1, maximum=MAX_TIMESTAMP_BYTES
        ),
    }


def parse_qr_scan(value: Any, *, session_id: str) -> dict[str, str]:
    """Validate one flyto.cloud.delivery-qr-scan.v1 payload, fail closed."""
    data = _exact_fields(value, QR_SCAN_FIELDS, "qr scan")
    if data["contract_version"] != QR_SCAN_CONTRACT_VERSION:
        raise DeliveryGatewayError("qr scan contract_version is unsupported")
    if _identifier(data["session_id"], "session_id") != session_id:
        raise DeliveryGatewayError("qr scan session_id does not match the request path")
    token = _bounded_text(
        data["qr_token"],
        "qr_token",
        minimum=len(QR_TOKEN_PREFIX) + 1,
        maximum=MAX_QR_TOKEN_BYTES,
    )
    if not token.startswith(QR_TOKEN_PREFIX):
        raise DeliveryGatewayError("qr_token must carry the F2QR1 prefix")
    return {
        "session_id": session_id,
        "qr_token": token,
        "scanned_at": _bounded_text(
            data["scanned_at"], "scanned_at", minimum=1, maximum=MAX_TIMESTAMP_BYTES
        ),
    }


class DeliverySession:
    """One delivery mission executed by the gateway's mission runner."""

    def __init__(
        self,
        *,
        session_id: str,
        request: dict[str, str],
        controller: MissionController,
        approval_id: str,
        decision: GoalDecision | None = None,
    ) -> None:
        self.session_id = session_id
        self.request_id = request["request_id"]
        self.space_name = request["space_name"]
        self.goal = request["goal"]
        self.requested_at = request["requested_at"]
        self.controller = controller
        self.approval_id = approval_id
        self.decision = decision
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.sim_now = 0.0
        self.confirmation: dict[str, Any] | None = None
        self.thread: threading.Thread | None = None
        # Observations the operator watches; the controller acts on its own
        # copies, so these are strictly for the room.
        self.minimum_range: float | None = None
        self.scan: dict[str, Any] | None = None


class SimulatedDeliveryRunner:
    """Default execution backend: deterministic planar kinematics in real time."""

    mode = "simulated_planar"

    def __init__(self, *, time_scale: float = 1.0) -> None:
        if (
            isinstance(time_scale, bool)
            or not isinstance(time_scale, (int, float))
            or not 0.1 <= float(time_scale) <= 100.0
        ):
            raise DeliveryGatewayError("time_scale must be between 0.1 and 100.0")
        self._time_scale = float(time_scale)
        self._gateway: DeliveryGateway | None = None
        self._threads: list[threading.Thread] = []

    def bind(self, gateway: DeliveryGateway) -> None:
        self._gateway = gateway

    def start_session(self, session: DeliverySession) -> None:
        if self._gateway is None:
            raise DeliveryGatewayError("mission runner is not bound to a gateway")
        self._threads = [thread for thread in self._threads if thread.is_alive()]
        session.thread = threading.Thread(
            target=self._run_mission,
            args=(session,),
            name=f"flyto-robotics-delivery-mission-{session.session_id}",
            daemon=True,
        )
        self._threads.append(session.thread)
        session.thread.start()

    def shutdown(self) -> None:
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()

    def _run_mission(self, session: DeliverySession) -> None:
        gateway = self._gateway
        if gateway is None:
            return
        wall_step = SIMULATION_TIMESTEP_SECONDS / self._time_scale
        while True:
            with gateway.lock:
                if session.controller.terminal:
                    return
                if gateway.stopping:
                    session.controller.cancel_for_safety(
                        session.sim_now, reason="gateway_shutdown"
                    )
                    return
                command = session.controller.tick(
                    session.pose, minimum_range=math.inf, now=session.sim_now
                )
                session.pose = Pose2D(
                    x=session.pose.x
                    + command.linear_x
                    * math.cos(session.pose.yaw)
                    * SIMULATION_TIMESTEP_SECONDS,
                    y=session.pose.y
                    + command.linear_x
                    * math.sin(session.pose.yaw)
                    * SIMULATION_TIMESTEP_SECONDS,
                    yaw=normalize_angle(
                        session.pose.yaw
                        + command.angular_z * SIMULATION_TIMESTEP_SECONDS
                    ),
                )
                session.sim_now += SIMULATION_TIMESTEP_SECONDS
            time.sleep(wall_step)


class DeliveryGateway:
    """Small local-only HTTP adapter; the browser never receives its token."""

    def __init__(
        self,
        *,
        token: str,
        qr_secret: str | bytes,
        job: DeliveryJob,
        host: str = "127.0.0.1",
        port: int = 8766,
        time_scale: float = 1.0,
        confirmation_timeout_seconds: float = 180.0,
        runner: Any | None = None,
        semantic_map: SemanticLocationMap | SemanticLocationStore | None = None,
    ) -> None:
        token_bytes = token.encode("utf-8")
        if len(token_bytes) < 32:
            raise DeliveryGatewayError("delivery gateway token must be at least 32 bytes")
        if not token.isascii():
            raise DeliveryGatewayError("delivery gateway token must be ASCII")
        try:
            address = ipaddress.ip_address(host)
        except ValueError as exc:
            raise DeliveryGatewayError(
                "delivery gateway host must be a literal loopback IP"
            ) from exc
        if not address.is_loopback:
            raise DeliveryGatewayError("delivery gateway host must be a literal loopback IP")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise DeliveryGatewayError("delivery gateway port must be between 0 and 65535")
        if (
            isinstance(confirmation_timeout_seconds, bool)
            or not isinstance(confirmation_timeout_seconds, (int, float))
            or not 5.0 <= float(confirmation_timeout_seconds) <= 3600.0
        ):
            raise DeliveryGatewayError(
                "confirmation_timeout_seconds must be between 5.0 and 3600.0"
            )
        if float(confirmation_timeout_seconds) > job.safety.mission_timeout_seconds:
            raise DeliveryGatewayError(
                "confirmation_timeout_seconds cannot exceed the job "
                "mission_timeout_seconds"
            )
        self._token = token
        self._expected_authorization = f"Bearer {token}".encode()
        self._authenticator = QRConfirmationAuthenticator(qr_secret)
        self._job = job
        self._host = host
        self._port = port
        self._confirmation_timeout_seconds = float(confirmation_timeout_seconds)
        self._runner = runner or SimulatedDeliveryRunner(time_scale=time_scale)
        self._semantic_map = semantic_map
        self._planner: Any = (
            DeterministicDeliveryGoalPlanner(semantic_map=semantic_map)
            if semantic_map is not None
            else FixedTemplateGoalPlanner()
        )
        self._approval_id = f"{job.job_id}.dropoff"
        self._lock = threading.RLock()
        self._sessions: OrderedDict[str, DeliverySession] = OrderedDict()
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._stopping = False
        self._runner.bind(self)

    @property
    def lock(self) -> threading.RLock:
        return self._lock

    @property
    def stopping(self) -> bool:
        return self._stopping

    @property
    def address(self) -> tuple[str, int]:
        if self._server is None:
            raise DeliveryGatewayError("delivery gateway is not running")
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    @property
    def approval_id(self) -> str:
        return self._approval_id

    def __enter__(self) -> DeliveryGateway:
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()

    # ------------------------------------------------------------------ HTTP

    def start(self) -> None:
        if self._server is not None:
            return
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "FlytoRoboticsDelivery/1"
            timeout = 10.0

            def log_message(self, *_args: object) -> None:
                return

            def _authorized(self) -> bool:
                supplied = self.headers.get("Authorization", "")
                return hmac.compare_digest(
                    supplied.encode("utf-8", "replace"),
                    gateway._expected_authorization,
                )

            def _send_json(self, status: int, payload: dict[str, Any]) -> None:
                body = json.dumps(
                    payload, ensure_ascii=False, separators=(",", ":"), sort_keys=True
                ).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _read_json_body(self) -> Any:
                if self.headers.get("Content-Type", "").split(";")[0].strip() != (
                    "application/json"
                ):
                    self._send_json(415, {"error": "application_json_required"})
                    return None
                try:
                    length = int(self.headers.get("Content-Length", ""))
                except ValueError:
                    length = -1
                if not 1 <= length <= MAX_DELIVERY_BODY_BYTES:
                    self._send_json(413, {"error": "delivery_request_size_invalid"})
                    return None
                try:
                    return json.loads(self.rfile.read(length).decode("utf-8"))
                except (UnicodeError, json.JSONDecodeError) as exc:
                    self._send_json(
                        400,
                        {"error": "delivery_request_invalid", "detail": str(exc)[:160]},
                    )
                    return None

            def do_GET(self) -> None:  # noqa: N802 - http.server contract
                if not self._authorized():
                    self._send_json(401, {"error": "unauthorized"})
                    return
                if self.path == "/v1/health":
                    self._send_json(
                        200,
                        {
                            "ok": True,
                            "service": DELIVERY_SERVICE_NAME,
                            "contract_version": DELIVERY_SESSION_CONTRACT_VERSION,
                        },
                    )
                    return
                match = SESSION_PATH.fullmatch(self.path)
                if match:
                    payload = gateway.session_payload(match.group(1))
                    if payload is None:
                        self._send_json(404, {"error": "delivery_session_not_found"})
                        return
                    self._send_json(200, payload)
                    return
                self._send_json(404, {"error": "not_found"})

            def do_POST(self) -> None:  # noqa: N802 - http.server contract
                if not self._authorized():
                    self._send_json(401, {"error": "unauthorized"})
                    return
                if self.path == "/v1/plans":
                    body = self._read_json_body()
                    if body is None:
                        return
                    try:
                        payload = gateway.start_plan(body)
                    except DeliverySessionConflictError:
                        self._send_json(409, {"error": "delivery_session_active"})
                        return
                    except DeliveryGatewayError as exc:
                        self._send_json(
                            400,
                            {"error": "plan_run_request_invalid", "detail": str(exc)[:160]},
                        )
                        return
                    self._send_json(200, payload)
                    return
                if self.path == "/v1/deliveries":
                    body = self._read_json_body()
                    if body is None:
                        return
                    try:
                        payload = gateway.start_delivery(body)
                    except DeliverySessionConflictError:
                        self._send_json(409, {"error": "delivery_session_active"})
                        return
                    except DeliveryGatewayError as exc:
                        self._send_json(
                            400,
                            {"error": "delivery_request_invalid", "detail": str(exc)[:160]},
                        )
                        return
                    self._send_json(200, payload)
                    return
                match = CONFIRMATION_PATH.fullmatch(self.path)
                if match:
                    body = self._read_json_body()
                    if body is None:
                        return
                    try:
                        payload = gateway.confirm_delivery(match.group(1), body)
                    except DeliveryGatewayError as exc:
                        self._send_json(
                            400,
                            {"error": "qr_scan_invalid", "detail": str(exc)[:160]},
                        )
                        return
                    if payload is None:
                        self._send_json(404, {"error": "delivery_session_not_found"})
                        return
                    self._send_json(200, payload)
                    return
                match = SAFE_STOP_PATH.fullmatch(self.path)
                if match:
                    body = self._read_json_body()
                    if body is None:
                        return
                    try:
                        payload = gateway.safe_stop(match.group(1), body)
                    except DeliveryGatewayError as exc:
                        self._send_json(
                            400,
                            {"error": "safe_stop_invalid", "detail": str(exc)[:160]},
                        )
                        return
                    if payload is None:
                        self._send_json(404, {"error": "delivery_session_not_found"})
                        return
                    self._send_json(200, payload)
                    return
                self._send_json(404, {"error": "not_found"})

        self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        self._stopping = False
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="flyto-robotics-delivery-gateway",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stopping = True
        server = self._server
        thread = self._thread
        self._server = None
        self._thread = None
        # Stop accepting HTTP requests before cancelling missions so a request
        # racing shutdown cannot create a session the cancellation pass misses.
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2.0)
        self._runner.shutdown()
        with self._lock:
            for session in self._sessions.values():
                if not session.controller.terminal:
                    session.controller.cancel_for_safety(
                        session.sim_now, reason="gateway_shutdown"
                    )

    # --------------------------------------------------------------- sessions

    def start_delivery(self, body: Any) -> dict[str, Any]:
        request = parse_delivery_request(body)
        with self._lock:
            if self._stopping:
                raise DeliveryGatewayError("delivery gateway is stopping")
            for session in self._sessions.values():
                if not session.controller.terminal:
                    raise DeliverySessionConflictError(
                        "an active delivery session already exists"
                    )
            session_id = f"dlv-{uuid.uuid4().hex[:12]}"
            goal_decision = self._planner.plan_delivery(
                job=self._job,
                goal=request["goal"],
                session_id=session_id,
                approval_id=self._approval_id,
                confirmation_timeout_seconds=self._confirmation_timeout_seconds,
                execution_mode=str(getattr(self._runner, "mode", "unknown")),
            )
            workflow = goal_decision.workflow or delivery_confirmation_workflow(
                self._job,
                approval_id=self._approval_id,
                confirmation_timeout_seconds=self._confirmation_timeout_seconds,
            )
            controller = MissionController(self._job, workflow=workflow)
            session = DeliverySession(
                session_id=session_id,
                request=request,
                controller=controller,
                approval_id=self._approval_id,
                decision=goal_decision,
            )
            self._sessions[session_id] = session
            while len(self._sessions) > MAX_RETAINED_SESSIONS:
                oldest_id, oldest = next(iter(self._sessions.items()))
                if not oldest.controller.terminal:
                    break
                del self._sessions[oldest_id]
            if not goal_decision.accepted:
                # Fail closed without ever moving: the session is born terminal
                # so the relay reports a rejection instead of dropping the link.
                controller.fail(
                    f"goal_rejected:{goal_decision.reason_code}", session.sim_now
                )
                return self._session_payload(session)
            self._runner.start_session(session)
            return self._session_payload(session)

    def start_plan(self, body: Any) -> dict[str, Any]:
        """Run one caller-supplied plan, with every existing guard in place.

        This is the same path ``run-ros`` takes, given an entry point. It exists
        so a workflow step can author its own motion — a distance typed into a
        builder rather than a card pre-registered on the robot — without any
        caller gaining a way around validation.

        The plan is compiled against the frozen capability registry, its
        robot_id must match this gateway's job, and a plan that moves must end
        in safe_stop. Running out-of-process is the point: if the caller dies
        mid-mission this gateway still owns the final stop.
        """
        request = parse_plan_run_request(body)
        with self._lock:
            if self._stopping:
                raise DeliveryGatewayError("delivery gateway is stopping")
            for session in self._sessions.values():
                if not session.controller.terminal:
                    raise DeliverySessionConflictError(
                        "an active delivery session already exists"
                    )

            try:
                plan = parse_plan(request["plan"])
            except PlanValidationError as exc:
                raise DeliveryGatewayError(f"plan_invalid:{exc}") from exc
            if plan.robot_id != self._job.robot_id:
                raise DeliveryGatewayError(
                    "plan robot_id does not match this robot"
                )
            try:
                workflow = compile_workflow(plan, semantic_map=self._semantic_map)
            except PlanValidationError as exc:
                raise DeliveryGatewayError(f"plan_uncompilable:{exc}") from exc
            if (
                any(step.kind in MOTION_PRIMITIVES for step in workflow.steps)
                and workflow.steps[-1].kind != PrimitiveKind.SAFE_STOP
            ):
                raise DeliveryGatewayError(
                    "a plan that moves must end with safe_stop"
                )

            session_id = f"pln-{uuid.uuid4().hex[:12]}"
            controller = MissionController(self._job, workflow=workflow)
            session = DeliverySession(
                session_id=session_id,
                request={
                    "request_id": request["request_id"],
                    "space_name": "",
                    "goal": plan.goal,
                    "requested_at": request["requested_at"],
                },
                controller=controller,
                approval_id=self._approval_id,
            )
            self._sessions[session_id] = session
            while len(self._sessions) > MAX_RETAINED_SESSIONS:
                oldest_id, oldest = next(iter(self._sessions.items()))
                if not oldest.controller.terminal:
                    break
                del self._sessions[oldest_id]
            self._runner.start_session(session)
            return self._session_payload(session)

    def session_payload(self, session_id: str) -> dict[str, Any] | None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None
            return self._session_payload(session)

    def confirm_delivery(self, session_id: str, body: Any) -> dict[str, Any] | None:
        scan = parse_qr_scan(body, session_id=session_id)
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None
            controller = session.controller
            evidence: dict[str, Any] = {
                "token_sha256": qr_token_sha256(scan["qr_token"]),
                "scanned_at": scan["scanned_at"],
            }
            # Verification consumes the single-use nonce, so a scan may only
            # reach the authenticator while the ask_human gate is open;
            # otherwise a premature scan would burn the token for good.
            gate_open = (
                not controller.terminal
                and controller.state is MissionState.WAITING_FOR_HUMAN
                and session.approval_id not in controller.human_decisions
            )
            if not gate_open:
                if not controller.terminal:
                    controller.record_human_decision_rejection(
                        reason="robot_not_at_confirmation_gate", now=session.sim_now
                    )
                evidence.update(
                    {"verified": False, "reason": "robot_not_at_confirmation_gate"}
                )
            else:
                try:
                    verified = self._authenticator.verify(
                        scan["qr_token"],
                        expected_job_id=controller.job.job_id,
                        expected_robot_id=controller.job.robot_id,
                        expected_approval_id=session.approval_id,
                    )
                    controller.submit_human_decision(
                        approval_id=session.approval_id,
                        approved=True,
                        actor_id=f"qr.{verified.recipient_ref}"[:128],
                        now=session.sim_now,
                    )
                    evidence.update(
                        {
                            "verified": True,
                            "confirmation_id": verified.confirmation_id,
                            "recipient_ref": verified.recipient_ref,
                        }
                    )
                except (QRConfirmationValidationError, ValueError) as exc:
                    if not controller.terminal:
                        controller.record_human_decision_rejection(
                            reason=str(exc)[:128], now=session.sim_now
                        )
                    evidence.update({"verified": False, "reason": str(exc)[:160]})
            if not (session.confirmation and session.confirmation.get("verified")):
                session.confirmation = evidence
            return self._session_payload(session)

    def safe_stop(self, session_id: str, body: Any) -> dict[str, Any] | None:
        data = _exact_fields(body, SAFE_STOP_FIELDS, "safe stop")
        reason = _identifier(data["reason"], "reason")
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None
            if not session.controller.terminal:
                session.controller.cancel_for_safety(session.sim_now, reason=reason)
            return self._session_payload(session)

    def _session_payload(self, session: DeliverySession) -> dict[str, Any]:
        controller = session.controller
        decision = session.decision
        payload: dict[str, Any] = {
            "contract_version": DELIVERY_SESSION_CONTRACT_VERSION,
            "session_id": session.session_id,
            "status": controller.state.value,
            "request_id": session.request_id,
            "space_name": session.space_name,
            "goal": session.goal,
            "requested_at": session.requested_at,
            "job_id": controller.job.job_id,
            "robot_id": controller.job.robot_id,
            "workflow_id": controller.workflow.workflow_id,
            "approval_id": session.approval_id,
            "execution_mode": str(getattr(self._runner, "mode", "unknown")),
            "failure_reason": controller.failure_reason,
            "elapsed_seconds": round(
                max(0.0, session.sim_now - controller.started_at), 3
            ),
            "pose": {
                "x": round(session.pose.x, 3),
                "y": round(session.pose.y, 3),
                "yaw": round(session.pose.yaw, 3),
            },
            "confirmation": session.confirmation,
            "minimum_range": (
                None
                if session.minimum_range is None or session.minimum_range == float("inf")
                else round(session.minimum_range, 3)
            ),
            "scan": session.scan,
            "events": [
                event.to_dict() for event in controller.events[-MAX_SESSION_EVENTS:]
            ],
        }
        if decision is not None:
            payload["decision"] = decision.decision
            if decision.rejection is not None:
                payload["rejection"] = decision.rejection
            if decision.route_graph is not None:
                payload["route_graph"] = decision.route_graph
        return payload
