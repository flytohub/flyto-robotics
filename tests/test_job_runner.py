"""The robot-side job runner, with both ends faked.

Everything this does is HTTP against two services, so both are stood up as real
loopback servers rather than mocked: a fake Flyto2 device API and a fake robot
gateway. That makes the tests exercise the actual requests — headers, paths,
bodies — instead of a description of them.
"""

from __future__ import annotations

import importlib.util
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

RUNNER_PATH = Path(__file__).resolve().parents[1] / "deploy" / "flyto_job_runner.py"


def load_runner(monkeypatch, tmp_path, *, cloud: str, gateway: str):
    """Import the runner fresh with its environment pointed at the fakes."""
    monkeypatch.setenv("FLYTO_CLOUD_URL", cloud)
    monkeypatch.setenv("FLYTO_ROBOTICS_GATEWAY_URL", gateway)
    monkeypatch.setenv("FLYTO_RUNNER_DATA_DIR", str(tmp_path / "data"))
    monkeypatch.setenv("FLYTO_ROBOTICS_DELIVERY_TOKEN", "t" * 40)
    monkeypatch.setenv("FLYTO_PLAN_ROOT", str(tmp_path / "plans"))
    spec = importlib.util.spec_from_file_location("flyto_job_runner", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Fake:
    """A loopback HTTP server that records what it was asked."""

    def __init__(self, routes):
        self.routes = routes
        self.seen: list[tuple[str, dict, dict]] = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def _serve(self):
                length = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(length) if length else b""
                try:
                    body = json.loads(raw) if raw else {}
                except Exception:
                    body = {}
                outer.seen.append((self.path, dict(self.headers), body))
                for prefix, responder in outer.routes.items():
                    if self.path.startswith(prefix):
                        status, payload = responder(self.path, body)
                        data = json.dumps(payload).encode()
                        self.send_response(status)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                self.send_response(404)
                self.end_headers()

            do_POST = _serve
            do_GET = _serve

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    @property
    def url(self) -> str:
        host, port = self.server.server_address
        return f"http://{host}:{port}"

    def paths(self) -> list[str]:
        return [path for path, _, _ in self.seen]

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def gateway():
    """A robot gateway that accepts a plan and succeeds on the first poll."""
    state = {"plans": []}

    def start(path, body):
        state["plans"].append(body)
        return 200, {"session_id": "pln-test", "state": "running"}

    def session(path, body):
        # Exactly what DeliveryGateway._session_payload emits: "status" (a
        # MissionState value), "pose", and the lidar's own "minimum_range".
        return 200, {"session_id": "pln-test", "status": "completed",
                     "pose": {"x": 0.37, "y": 0.0, "yaw": 1.55},
                     "minimum_range": 1.42}

    fake = Fake({"/v1/plans": start, "/v1/deliveries/": session})
    fake.state = state
    yield fake
    fake.close()


PLAN = {
    "contract_version": "flyto.robotics.plan.v1",
    "plan_id": "shortcut.forward.40cm.v1",
    "robot_id": "flyto-tb3-lab-001",
    "goal": "move forward",
    "steps": [{"step_id": "s", "capability": "move_relative",
               "arguments": {"distance_m": 0.4}}],
}


# -- pairing -------------------------------------------------------------


def test_pairing_stores_the_credential_once_and_never_the_code(monkeypatch, tmp_path):
    cloud = Fake({"/api/devices/pair/claim":
                  lambda p, b: (200, {"device_id": "dev-1", "device_secret": "s-1"})})
    try:
        monkeypatch.setenv("FLYTO_PAIRING_CODE", "CODE-123")
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway="http://127.0.0.1:1")
        credentials = runner._pair()
        assert credentials == {"device_id": "dev-1", "device_secret": "s-1"}

        stored = runner.CREDENTIAL_FILE
        assert stored.exists()
        assert oct(stored.stat().st_mode)[-3:] == "600", "a credential must not be world readable"
        # The one-time code is popped from the environment and never written.
        assert "CODE-123" not in stored.read_text()
        assert monkeypatch  # keep the fixture alive
        import os
        assert "FLYTO_PAIRING_CODE" not in os.environ
    finally:
        cloud.close()


def test_pairing_without_a_code_says_where_to_get_one(monkeypatch, tmp_path):
    runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1")
    monkeypatch.delenv("FLYTO_PAIRING_CODE", raising=False)
    with pytest.raises(runner.RunnerError, match="Pair a device"):
        runner._pair()


def test_the_device_credential_is_the_only_thing_sent_afterwards(monkeypatch, tmp_path):
    runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1")
    headers = runner._headers({"device_id": "dev-1", "device_secret": "s-1"})
    assert headers == {"Authorization": "Bearer device:dev-1.s-1"}


# -- recognising a job ---------------------------------------------------


def test_a_plan_carried_inline_is_used(monkeypatch, tmp_path):
    runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1")
    job = {"job_id": "j1", "steps": [{"params": {"plan": PLAN}}]}
    assert runner._plan_from(job)["plan_id"] == PLAN["plan_id"]


def test_a_plan_named_by_file_is_read_from_the_plans_directory(monkeypatch, tmp_path):
    runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1")
    root = tmp_path / "plans"
    root.mkdir(parents=True, exist_ok=True)
    (root / "forward.json").write_text(json.dumps(PLAN))
    job = {"job_id": "j1", "steps": [{"params": {"plan_path": "forward.json"}}]}
    assert runner._plan_from(job)["plan_id"] == PLAN["plan_id"]


@pytest.mark.parametrize(
    "reference",
    ["../../../etc/passwd", "/etc/passwd", "../flyto-delivery.env", "subdir/../../secret"],
)
def test_a_job_cannot_name_a_file_outside_the_plans_directory(monkeypatch, tmp_path, reference):
    """A job arrives over the network; it must not choose what this reads."""
    runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1")
    (tmp_path / "plans").mkdir(parents=True, exist_ok=True)
    job = {"job_id": "j1", "steps": [{"params": {"plan_path": reference}}]}
    assert runner._plan_from(job) is None


def test_a_job_with_no_plan_is_not_a_robot_job(monkeypatch, tmp_path):
    runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1")
    assert runner._plan_from({"job_id": "j1", "steps": [{"params": {"url": "https://x"}}]}) is None
    assert runner._plan_from({"job_id": "j1"}) is None


# -- carrying one out ----------------------------------------------------


def make_cloud(completions):
    def claim(path, body):
        return 200, {"lease_id": "lease-1"}

    def complete(path, body):
        completions.append(body)
        return 200, {"ok": True}

    return Fake({"/api/devices/jobs/": lambda p, b: (
        (200, {"lease_id": "lease-1"}) if p.endswith("/claim") else complete(p, b)
    )})


def test_a_robot_job_is_claimed_run_and_reported_succeeded(monkeypatch, tmp_path, gateway):
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        runner._handle({"job_id": "j1", "steps": [{"params": {"plan": PLAN}}]},
                       {"device_id": "dev-1", "device_secret": "s-1"})

        assert "/api/devices/jobs/j1/claim" in cloud.paths()
        # The plan reached the gateway, wrapped in the contract it expects.
        sent = gateway.state["plans"][0]
        assert sent["contract_version"] == "flyto.cloud.plan-run-request.v1"
        assert sent["plan"]["plan_id"] == PLAN["plan_id"]
        done = completions[-1]
        # The body the real /complete route accepts: status matches
        # ^(success|failed)$, and evidence rides in variables["evidence"]
        # where the Space task sweep reads it. "succeeded" plus a "result"
        # field passed the fake but 422'd against the real contract.
        assert done["status"] == "success"
        kinds = {item["kind"]: item for item in done["variables"]["evidence"]}
        assert set(kinds) == {"arrival.pose", "clearance.measurement"}
        assert kinds["arrival.pose"]["usable"] is True
        assert "0.37" in kinds["arrival.pose"]["detail"]
        # The passage-inspection half: a measurement, not a picture.
        assert "1.42" in kinds["clearance.measurement"]["detail"]
    finally:
        cloud.close()


def test_a_job_this_device_cannot_run_is_failed_with_a_reason(monkeypatch, tmp_path, gateway):
    """Not skipped silently: an unrunnable job that looks pending forever is worse."""
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        runner._handle({"job_id": "j2", "steps": [{"params": {"url": "https://x"}}]},
                       {"device_id": "dev-1", "device_secret": "s-1"})
        assert completions[-1]["status"] == "failed"
        assert "robot plans only" in completions[-1]["error_message"]
        assert gateway.state["plans"] == [], "nothing should reach the robot"
    finally:
        cloud.close()


def test_a_gateway_refusal_fails_the_job_rather_than_retrying(monkeypatch, tmp_path):
    """Re-running something that may already have moved a robot is how a retry
    becomes a collision."""
    completions: list[dict] = []
    cloud = make_cloud(completions)
    refusing = Fake({"/v1/plans": lambda p, b: (400, {"detail": "a plan that moves must end with safe_stop"})})
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=refusing.url)
        runner._handle({"job_id": "j3", "steps": [{"params": {"plan": PLAN}}]},
                       {"device_id": "dev-1", "device_secret": "s-1"})
        assert completions[-1]["status"] == "failed"
        assert len([p for p in refusing.paths() if p == "/v1/plans"]) == 1, "sent once, not retried"
    finally:
        cloud.close()
        refusing.close()


def test_an_unknown_outcome_is_not_reported_as_success(monkeypatch, tmp_path):
    """The gateway still owns the robot; not knowing is not the same as failing
    to move, and it is certainly not success."""
    completions: list[dict] = []
    cloud = make_cloud(completions)
    stuck = Fake({
        "/v1/plans": lambda p, b: (200, {"session_id": "pln-1", "status": "navigating"}),
        "/v1/deliveries/": lambda p, b: (200, {"session_id": "pln-1", "status": "navigating"}),
    })
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=stuck.url)
        runner.MISSION_WATCH_SECONDS = 0.2
        runner.GATEWAY_POLL_SECONDS = 0.05
        runner._handle({"job_id": "j4", "steps": [{"params": {"plan": PLAN}}]},
                       {"device_id": "dev-1", "device_secret": "s-1"})
        done = completions[-1]
        assert done["status"] == "failed"
        assert "unknown" in done["error_message"]
        assert "evidence" not in (done.get("variables") or {}), \
            "an unknown outcome must not claim an arrival"
    finally:
        cloud.close()
        stuck.close()


def test_the_runner_needs_no_third_party_package():
    """The reason this exists: the cloud's own runner pulls twenty-one of them,
    because its import closure reaches the whole backend."""
    import ast

    # Named rather than derived from sys.stdlib_module_names, which needs 3.10+
    # and would make this test silently weaker on an older interpreter. These
    # are exactly what connected_runner.py drags in.
    FORBIDDEN = {
        "RestrictedPython", "anthropic", "boto3", "botocore", "core",
        "cryptography", "dotenv", "fastapi", "firebase_admin", "flyto_blueprint",
        "google", "httpx", "hvac", "jwt", "psutil", "pydantic", "redis",
        "requests", "stripe", "websockets", "yaml", "playwright", "rclpy",
    }
    tree = ast.parse(RUNNER_PATH.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    assert not (imported & FORBIDDEN), f"stdlib only, but found {imported & FORBIDDEN}"


def test_a_failed_mission_reports_no_evidence_at_all(monkeypatch, tmp_path):
    """A mission that did not finish must not have its last known pose written
    down as an arrival, nor its last range reading as a clearance."""
    completions: list[dict] = []
    cloud = make_cloud(completions)
    failing = Fake({
        "/v1/plans": lambda p, b: (200, {"session_id": "pln-2", "status": "running"}),
        "/v1/deliveries/": lambda p, b: (200, {
            "session_id": "pln-2", "status": "failed",
            "failure_reason": "obstacle_stop",
            "pose": {"x": 0.1}, "minimum_range": 0.18,
        }),
    })
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=failing.url)
        runner._handle({"job_id": "j5", "steps": [{"params": {"plan": PLAN}}]},
                       {"device_id": "dev-1", "device_secret": "s-1"})
        done = completions[-1]
        assert done["status"] == "failed"
        assert "obstacle_stop" in done["error_message"]
        assert "evidence" not in done.get("variables", {})
    finally:
        cloud.close()
        failing.close()


def test_the_gateways_own_terminal_state_is_recognised(monkeypatch, tmp_path):
    """MissionState.COMPLETED is "completed". Waiting for "succeeded" — which
    no gateway sends — made every real mission time out as outcome unknown."""
    from pathlib import Path as _Path
    source = _Path(__file__).resolve().parents[1] / "deploy" / "flyto_job_runner.py"
    text = source.read_text()
    assert '"completed"' in text
