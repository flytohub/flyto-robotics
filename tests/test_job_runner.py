"""The robot-side job runner, with both ends faked.

Everything this does is HTTP against two services, so both are stood up as real
loopback servers rather than mocked: a fake Flyto2 device API and a fake robot
gateway. That makes the tests exercise the actual requests — headers, paths,
bodies — instead of a description of them.
"""

from __future__ import annotations

import builtins
import hashlib
import importlib.util
import json
import os
import sys
import threading
import types
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

RUNNER_PATH = Path(__file__).resolve().parents[1] / "deploy" / "flyto_job_runner.py"


def load_runner(monkeypatch, tmp_path, *, cloud: str, gateway: str):
    """Import the runner fresh with its environment pointed at the fakes."""
    # Resolved: on macOS tempfile hands back /var/folders/... and /var is a
    # symlink to /private/var, which the event journal correctly refuses to
    # write through. An unresolved path would fail every journal assertion here
    # for a reason that has nothing to do with what is being tested.
    tmp_path = tmp_path.resolve()
    monkeypatch.setenv("FLYTO_CLOUD_URL", cloud)
    monkeypatch.setenv("FLYTO_ROBOTICS_GATEWAY_URL", gateway)
    monkeypatch.setenv("FLYTO_RUNNER_DATA_DIR", str(tmp_path / "data"))
    monkeypatch.delenv("FLYTO_DEVICE_EVENT_JOURNAL", raising=False)
    monkeypatch.delenv("FLYTO_ROBOT_RESOURCE_ID", raising=False)
    monkeypatch.setenv("FLYTO_ROBOTICS_DELIVERY_TOKEN", "t" * 40)
    monkeypatch.setenv("FLYTO_PLAN_ROOT", str(tmp_path / "plans"))
    spec = importlib.util.spec_from_file_location("flyto_job_runner", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Fake:
    """A loopback HTTP server that records what it was asked."""

    def __init__(self, routes, *, pass_headers=False):
        self.routes = routes
        self.pass_headers = pass_headers
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
                        if outer.pass_headers:
                            status, payload = responder(
                                self.path,
                                body,
                                {k.lower(): v for k, v in self.headers.items()},
                            )
                        else:
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
        dispatched = state["plans"][-1]
        receipt = {
            "contract_version": "flyto.robotics.execution-receipt.v1",
            "request_id": dispatched["request_id"],
            "session_id": "pln-test",
            "job_id": "robot-job-test",
            "robot_id": dispatched["plan"]["robot_id"],
            "workflow_id": "robot-workflow-test",
            "status": "succeeded",
            "plan_sha256": _digest(dispatched["plan"]),
            "mission_result_sha256": "a" * 64,
            "events_sha256": "b" * 64,
            "event_count": 4,
            "safety_stop_count": 1,
            "final_pose": {"x": 0.37, "y": 0.0, "yaw": 1.55},
            "minimum_range": 1.42,
            "elapsed_seconds": 3.5,
            "task_completion_eligible": False,
        }
        receipt["receipt_sha256"] = _digest(receipt)
        return 200, {
            "contract_version": "flyto.robotics.delivery-session.v2",
            "session_id": "pln-test",
            "status": "completed",
            "pose": {"x": 0.37, "y": 0.0, "yaw": 1.55},
            "minimum_range": 1.42,
            "execution_receipt": receipt,
        }

    fake = Fake({"/v1/plans": start, "/v1/deliveries/": session})
    fake.state = state
    yield fake
    fake.close()


PLAN = {
    "contract_version": "flyto.robotics.plan.v1",
    "plan_id": "shortcut.forward.40cm.v1",
    "robot_id": "flyto-tb3-lab-001",
    "goal": "move forward",
    "steps": [{"step_id": "s", "capability": "move_relative", "arguments": {"distance_m": 0.4}}],
}


def _digest(value: object) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()


def _cloud_job(job_id: str, steps: list[dict], *, device_id: str = "dev-1") -> dict:
    steps = json.loads(json.dumps(steps))
    trace_id = f"trace-{job_id}"
    handoff = {
        "contract_version": "flyto.cloud.device-job-handoff.v1",
        "device_id": device_id,
        "trace_id": trace_id,
        "workflow_sha256": _digest(steps),
        "task_completion_authority": "flyto.space.evidence.v1",
    }
    handoff["handoff_sha256"] = _digest(handoff)
    return {
        "job_id": job_id,
        "device_id": device_id,
        "steps": steps,
        "input_params": {
            "_flyto_trace_id": trace_id,
            "_flyto_device_handoff": handoff,
        },
    }


# -- pairing -------------------------------------------------------------


class PairResponse:
    def __init__(self, body: bytes):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def read(self, limit: int) -> bytes:
        return self.body[:limit]


def test_explicit_pair_mode_is_bounded_isolated_and_idempotent(
    monkeypatch, tmp_path, capsys
):
    code = "one-time-code-that-must-not-escape"
    secret = "returned-device-secret"
    runner = load_runner(monkeypatch, tmp_path, cloud="https://cloud.invalid", gateway="unused")
    calls = []

    def urlopen(request, timeout):
        calls.append((request, timeout))
        return PairResponse(
            json.dumps({"device_id": "dev-1", "device_secret": secret}).encode()
        )

    monkeypatch.setattr(runner.urllib.request, "urlopen", urlopen)
    for forbidden in ("_post", "_handle", "_executor_registry"):
        monkeypatch.setattr(
            runner,
            forbidden,
            lambda *_a, _name=forbidden, **_k: pytest.fail(f"pair called {_name}"),
        )
    monkeypatch.setenv("FLYTO_PAIRING_CODE", code)
    assert runner.pair_main() == 0
    first = capsys.readouterr()
    assert json.loads(first.out) == {"ok": True, "status": "paired"}
    assert first.err == ""
    assert len(first.out) < 128
    assert code not in first.out + first.err
    assert "FLYTO_PAIRING_CODE" not in os.environ
    assert len(calls) == 1
    assert calls[0][0].full_url == "https://cloud.invalid/api/devices/pair/claim"
    assert code in calls[0][0].data.decode()

    stored = runner.CREDENTIAL_FILE
    assert json.loads(stored.read_text()) == {
        "device_id": "dev-1",
        "device_secret": secret,
    }
    assert stored.stat().st_mode & 0o777 == 0o600
    assert stored.parent.stat().st_mode & 0o777 == 0o700
    assert code not in stored.read_text()

    monkeypatch.setenv("FLYTO_PAIRING_CODE", "another-code")
    assert runner.pair_main() == 0
    second = capsys.readouterr()
    assert json.loads(second.out) == {"ok": True, "status": "already_paired"}
    assert second.err == ""
    assert len(calls) == 1


@pytest.mark.parametrize(
    "body",
    [
        b"not-json",
        b"{}",
        b'{"device_id":"dev","device_secret":"s","extra":true}',
        b'{"device_id":"dev","device_id":"other","device_secret":"s"}',
        b'{"device_id":"dev","device_secret":"s","number":NaN}',
        json.dumps({"device_id": "dev", "device_secret": "s" * 513}).encode(),
        b"x" * 4097,
    ],
)
def test_pair_mode_closes_malformed_and_oversized_responses(
    monkeypatch, tmp_path, capsys, body
):
    runner = load_runner(monkeypatch, tmp_path, cloud="https://cloud.invalid", gateway="unused")
    monkeypatch.setenv("FLYTO_PAIRING_CODE", "private-code")
    monkeypatch.setattr(
        runner.urllib.request, "urlopen", lambda *_a, **_k: PairResponse(body)
    )
    assert runner.pair_main() != 0
    output = capsys.readouterr()
    assert json.loads(output.out) == runner.PAIR_ERRORS["response"]
    assert output.err == ""
    assert len(output.out) < 256
    assert not runner.CREDENTIAL_FILE.exists()


def test_pair_mode_missing_code_and_bad_existing_file_are_content_free(
    monkeypatch, tmp_path, capsys
):
    runner = load_runner(monkeypatch, tmp_path, cloud="https://cloud.invalid", gateway="unused")
    assert runner.pair_main() != 0
    assert json.loads(capsys.readouterr().out) == runner.PAIR_ERRORS["missing_code"]

    runner.DATA_DIR.mkdir(mode=0o700)
    runner.CREDENTIAL_FILE.write_text("private malformed material")
    runner.CREDENTIAL_FILE.chmod(0o644)
    monkeypatch.setenv("FLYTO_PAIRING_CODE", "private-code")
    assert runner.pair_main() != 0
    output = capsys.readouterr()
    assert json.loads(output.out) == runner.PAIR_ERRORS["existing_credential"]
    assert "private" not in output.out + output.err


@pytest.mark.parametrize(
    "content",
    [
        b"x" * 4097,
        b'{"device_id":"dev","device_secret":"s","extra":true}',
        b'{"device_id":"dev","device_id":"other","device_secret":"s"}',
        b'{"device_id":"dev","device_secret":"bad\\u0000value"}',
        b'{"device_id":"dev","device_secret":"s","number":Infinity}',
    ],
)
def test_pair_mode_rejects_bounded_stored_credential_faults_without_network(
    monkeypatch, tmp_path, capsys, content
):
    runner = load_runner(monkeypatch, tmp_path, cloud="https://cloud.invalid", gateway="unused")
    runner.DATA_DIR.mkdir(mode=0o700)
    runner.CREDENTIAL_FILE.write_bytes(content)
    runner.CREDENTIAL_FILE.chmod(0o600)
    monkeypatch.setenv("FLYTO_PAIRING_CODE", "private-code")
    monkeypatch.setattr(
        runner.urllib.request,
        "urlopen",
        lambda *_a, **_k: pytest.fail("invalid stored credential reached network"),
    )
    assert runner.pair_main() != 0
    assert json.loads(capsys.readouterr().out) == runner.PAIR_ERRORS["existing_credential"]


def test_pair_mode_rejects_symlinked_stored_and_systemd_credentials(
    monkeypatch, tmp_path, capsys
):
    runner = load_runner(monkeypatch, tmp_path, cloud="https://cloud.invalid", gateway="unused")
    target = tmp_path / "target"
    target.write_text(json.dumps({"device_id": "dev", "device_secret": "secret"}))
    target.chmod(0o600)
    runner.DATA_DIR.mkdir(mode=0o700)
    runner.CREDENTIAL_FILE.symlink_to(target)
    monkeypatch.setenv("FLYTO_PAIRING_CODE", "private-code")
    monkeypatch.setattr(
        runner.urllib.request,
        "urlopen",
        lambda *_a, **_k: pytest.fail("symlink reached network"),
    )
    assert runner.pair_main() != 0
    assert json.loads(capsys.readouterr().out) == runner.PAIR_ERRORS["existing_credential"]

    runner.CREDENTIAL_FILE.unlink()
    systemd = tmp_path / "systemd"
    systemd.mkdir()
    (systemd / runner.SYSTEMD_CREDENTIAL_NAME).symlink_to(target)
    monkeypatch.setenv("CREDENTIALS_DIRECTORY", str(systemd))
    assert runner.pair_main() != 0
    assert json.loads(capsys.readouterr().out) == runner.PAIR_ERRORS["existing_credential"]


@pytest.mark.parametrize(
    ("variable", "value", "error_key"),
    [
        ("FLYTO_PAIRING_CODE", "x" * 257, "invalid_code"),
        ("FLYTO_PAIRING_CODE", "code\ncontrol", "invalid_code"),
        ("FLYTO_PAIRING_CODE", "café", "invalid_code"),
        ("FLYTO_RUNNER_NAME", "x" * 129, "invalid_name"),
        ("FLYTO_RUNNER_NAME", "name\tcontrol", "invalid_name"),
        ("FLYTO_RUNNER_NAME", "機器人", "invalid_name"),
    ],
)
def test_pair_mode_rejects_unbounded_request_inputs_before_network(
    monkeypatch, tmp_path, capsys, variable, value, error_key
):
    runner = load_runner(monkeypatch, tmp_path, cloud="https://cloud.invalid", gateway="unused")
    monkeypatch.setenv("FLYTO_PAIRING_CODE", "valid-code")
    monkeypatch.setenv(variable, value)
    monkeypatch.setattr(
        runner.urllib.request,
        "urlopen",
        lambda *_a, **_k: pytest.fail("invalid request input reached network"),
    )
    assert runner.pair_main() != 0
    output = capsys.readouterr()
    assert json.loads(output.out) == runner.PAIR_ERRORS[error_key]
    assert value not in output.out + output.err
    assert "FLYTO_PAIRING_CODE" not in os.environ


def test_pairing_stores_the_credential_once_and_never_the_code(monkeypatch, tmp_path):
    cloud = Fake(
        {
            "/api/devices/pair/claim": lambda p, b: (
                200,
                {"device_id": "dev-1", "device_secret": "s-1"},
            )
        }
    )
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
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    monkeypatch.delenv("FLYTO_PAIRING_CODE", raising=False)
    with pytest.raises(runner.RunnerError, match="Pair a device"):
        runner._pair()


def test_the_device_credential_is_the_only_thing_sent_afterwards(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    headers = runner._headers({"device_id": "dev-1", "device_secret": "s-1"})
    assert headers == {"Authorization": "Bearer device:dev-1.s-1"}


# -- recognising a job ---------------------------------------------------


def test_a_plan_carried_inline_is_used(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    job = {"job_id": "j1", "steps": [{"params": {"plan": PLAN}}]}
    assert runner._plan_from(job)["plan_id"] == PLAN["plan_id"]


def test_a_plan_named_by_file_is_read_from_the_plans_directory(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
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
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    (tmp_path / "plans").mkdir(parents=True, exist_ok=True)
    job = {"job_id": "j1", "steps": [{"params": {"plan_path": reference}}]}
    assert runner._plan_from(job) is None


def test_a_job_with_no_plan_is_not_a_robot_job(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    assert runner._plan_from({"job_id": "j1", "steps": [{"params": {"url": "https://x"}}]}) is None
    assert runner._plan_from({"job_id": "j1"}) is None


# -- carrying one out ----------------------------------------------------


# What api/devices/routes_jobs.py actually reads a lease from. The fake used to
# accept a completion carrying no lease at all, which is how the runner shipped
# sending a header name — "X-Flyto-Lease" — that no route reads: every mission
# ran, every report was refused 409, and these tests stayed green throughout.
LEASE_HEADER = "x-flyto2-job-lease"


def make_cloud(completions):
    def route(path, body, headers):
        if path.endswith("/claim"):
            return 200, {"lease_id": "lease-1"}
        if headers.get(LEASE_HEADER) != "lease-1":
            return 409, {"ok": False, "error": "Job lease is missing or invalid"}
        completions.append(body)
        return 200, {"ok": True}

    return Fake({"/api/devices/jobs/": route}, pass_headers=True)


class FakeExecutorRegistry:
    def __init__(self, module_ids, result):
        self.module_metadata = {module_id: object() for module_id in module_ids}
        self.result = result
        self.calls = []
        self.handle = object()

    def prepare(self, module_id, params):
        self.calls.append(("prepare", module_id, params))
        return self.handle

    def execute(self, handle):
        self.calls.append(("execute", handle))
        return self.result

    def discard(self, handle):
        self.calls.append(("discard", handle))


def test_generic_shape_accepts_arguments_and_rejects_conflicting_dual_form(
    monkeypatch, tmp_path
):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    registry = FakeExecutorRegistry({"future.inspect"}, {})
    assert runner._generic_step(
        {"steps": [{"module": "future.inspect", "arguments": {"sample": 1}}]}, registry
    ) == ("future.inspect", {"sample": 1})
    with pytest.raises(runner.RunnerError):
        runner._generic_step(
            {"steps": [{
                "module": "future.inspect",
                "params": {"sample": 1},
                "arguments": {"sample": 2},
            }]},
            registry,
        )


def test_registry_cannot_own_robotics_or_collide_with_a_legacy_plan(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    robotics = FakeExecutorRegistry({"robotics.turn"}, {})
    with pytest.raises(runner.RunnerError):
        runner._generic_step(
            {"steps": [{"module": "robotics.turn", "params": {"degrees": 90}}]}, robotics
        )
    generic = FakeExecutorRegistry({"future.inspect"}, {})
    selected = runner._generic_step(
        {"steps": [
            {"params": {"plan": PLAN}},
            {"module": "future.inspect", "params": {}},
        ]},
        generic,
    )
    assert selected == ("future.inspect", {})
    assert generic.calls == []


def test_manifest_directory_must_be_normalized_absolute(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    runner.DEVICE_EXECUTOR_MANIFEST_DIR = Path("/etc/flyto/../flyto/device-executors")
    with pytest.raises(runner.RunnerError, match="device_executor_registry_unavailable"):
        runner._executor_registry()


@pytest.mark.parametrize(
    "reason_code",
    ["device_executor_started", "device_executor_succeeded", "device_executor_failed"],
)
def test_generic_replay_gate_recognises_started_and_terminal_records(
    monkeypatch, tmp_path, reason_code
):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    runner._append_event(
        {"device_id": "dev-1", "device_secret": "synthetic"},
        status="started" if reason_code.endswith("started") else "failed",
        severity="info",
        reason_code=reason_code,
        job_id="replay-1",
    )
    assert runner._generic_replay_seen("replay-1") is True
    assert runner._generic_replay_seen("different-run") is False


def test_generic_dispatch_is_module_neutral(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    registry = FakeExecutorRegistry({"vision.observe", "future.inspect", "later.measure"}, {})
    for module_id in registry.module_metadata:
        if module_id.startswith("robotics."):
            continue
        assert runner._generic_step(
            {"steps": [{"module_id": module_id, "params": {"synthetic": True}}]}, registry
        ) == (module_id, {"synthetic": True})


def install_in_memory_cloud(monkeypatch, runner, completions, *, completion_error=None):
    """Claim and complete without opening a socket; retain the lease evidence."""
    def post(base, path, body, headers=None, timeout=15.0):
        if path.endswith("/claim"):
            return {"lease_id": "generic-lease"}
        if path.endswith("/complete"):
            completions.append((body, dict(headers or {})))
            if completion_error is not None:
                raise completion_error
            return {"ok": True}
        raise AssertionError("unexpected transport call")

    monkeypatch.setattr(runner, "_post", post)


def generic_job(job_id="generic-memory", module_id="future.inspect"):
    return {"job_id": job_id, "steps": [{"module": module_id, "params": {"safe": True}}]}


def test_handle_blocks_robotics_registry_and_mixed_plan_without_provider_calls(
    monkeypatch, tmp_path
):
    credentials = {"device_id": "dev-1", "device_secret": "synthetic"}
    for job, owned in (
        (generic_job("owned-robotics", "robotics.turn"), {"robotics.turn"}),
        ({"job_id": "mixed", "steps": [
            {"params": {"plan": PLAN}},
            {"module": "future.inspect", "params": {}},
        ]}, {"future.inspect"}),
    ):
        runner = load_runner(
            monkeypatch, tmp_path / job["job_id"],
            cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1",
        )
        completions = []
        install_in_memory_cloud(monkeypatch, runner, completions)
        registry = FakeExecutorRegistry(owned, {})
        runner._device_executor_registry = registry
        runner._handle(job, credentials)
        assert registry.calls == []
        assert completions[-1][0]["variables"]["detail"] == "device_executor_registry_error"


@pytest.mark.parametrize("operation", ["prepare", "execute"])
def test_handle_provider_exceptions_are_private(monkeypatch, tmp_path, operation):
    secret = "Bearer synthetic-private-provider-value"

    class PrivateFailureRegistry(FakeExecutorRegistry):
        def prepare(self, module_id, params):
            if operation == "prepare":
                raise RuntimeError(secret)
            return super().prepare(module_id, params)

        def execute(self, handle):
            if operation == "execute":
                raise RuntimeError(secret)
            return super().execute(handle)

    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    completions = []
    install_in_memory_cloud(monkeypatch, runner, completions)
    registry = PrivateFailureRegistry(
        {"future.inspect"}, {"status": "succeeded", "reason_code": "ok", "evidence": []}
    )
    runner._device_executor_registry = registry
    runner._handle(generic_job(f"private-{operation}"), {
        "device_id": "dev-1", "device_secret": "synthetic",
    })
    serialized = json.dumps(completions) + json.dumps(records(runner))
    assert secret not in serialized
    expected = (
        "device_executor_registry_error" if operation == "prepare" else "device_executor_failed"
    )
    assert completions[-1][0]["variables"]["detail"] == expected


def test_handle_started_journal_failure_discards_and_executes_zero(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    completions = []
    install_in_memory_cloud(monkeypatch, runner, completions)
    registry = FakeExecutorRegistry(
        {"future.inspect"}, {"status": "succeeded", "reason_code": "ok", "evidence": []}
    )
    runner._device_executor_registry = registry
    monkeypatch.setattr(
        runner, "_append_event", lambda *args, **kwargs: (_ for _ in ()).throw(OSError("private"))
    )
    with pytest.raises(OSError):
        runner._handle(generic_job("journal-failure"), {
            "device_id": "dev-1", "device_secret": "synthetic",
        })
    assert registry.calls == [
        ("prepare", "future.inspect", {"safe": True}), ("discard", registry.handle),
    ]
    assert completions == []


def test_generic_handle_propagates_claim_lease_to_completion(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    completions = []
    install_in_memory_cloud(monkeypatch, runner, completions)
    registry = FakeExecutorRegistry(
        {"future.inspect"}, {"status": "succeeded", "reason_code": "ok", "evidence": []}
    )
    runner._device_executor_registry = registry
    runner._handle(generic_job("lease-generic"), {
        "device_id": "dev-1", "device_secret": "synthetic",
    })
    assert completions[-1][1][runner.LEASE_HEADER] == "generic-lease"


@pytest.mark.parametrize("prior_code", ["device_executor_started", "device_executor_succeeded"])
def test_fresh_runner_handle_refuses_prior_generic_history_before_prepare(
    monkeypatch, tmp_path, prior_code
):
    credentials = {"device_id": "dev-1", "device_secret": "synthetic"}
    first = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    first._append_event(
        credentials,
        status="started" if prior_code.endswith("started") else "succeeded",
        severity="info",
        reason_code=prior_code,
        job_id="durable-replay",
    )
    fresh = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    completions = []
    install_in_memory_cloud(monkeypatch, fresh, completions)
    registry = FakeExecutorRegistry({"future.inspect"}, {})
    fresh._device_executor_registry = registry
    fresh._handle(generic_job("durable-replay"), credentials)
    assert registry.calls == []
    assert completions[-1][0]["variables"]["detail"] == "device_executor_replay_refused"


def test_fresh_runner_refuses_after_execute_when_completion_transport_failed(
    monkeypatch, tmp_path
):
    credentials = {"device_id": "dev-1", "device_secret": "synthetic"}
    first = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    install_in_memory_cloud(
        monkeypatch, first, [], completion_error=urllib.error.URLError("synthetic")
    )
    first_registry = FakeExecutorRegistry(
        {"future.inspect"}, {"status": "succeeded", "reason_code": "ok", "evidence": []}
    )
    first._device_executor_registry = first_registry
    with pytest.raises(urllib.error.URLError):
        first._handle(generic_job("transport-replay"), credentials)
    fresh = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    completions = []
    install_in_memory_cloud(monkeypatch, fresh, completions)
    fresh_registry = FakeExecutorRegistry({"future.inspect"}, {})
    fresh._device_executor_registry = fresh_registry
    fresh._handle(generic_job("transport-replay"), credentials)
    assert fresh_registry.calls == []
    assert completions[-1][0]["variables"]["detail"] == "device_executor_replay_refused"


def test_handle_journal_read_failure_refuses_before_prepare(monkeypatch, tmp_path):
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    completions = []
    install_in_memory_cloud(monkeypatch, runner, completions)
    monkeypatch.setattr(
        runner, "_generic_replay_seen", lambda job_id: (_ for _ in ()).throw(OSError("private"))
    )
    registry = FakeExecutorRegistry({"future.inspect"}, {})
    runner._device_executor_registry = registry
    runner._handle(generic_job("read-failure"), {
        "device_id": "dev-1", "device_secret": "synthetic",
    })
    assert registry.calls == []
    assert completions[-1][0]["variables"]["detail"] == "device_executor_replay_refused"
    assert records(runner)[-1]["reason_code"] == "device_executor_replay_refused"


def test_registry_owned_steps_route_generically_with_validated_evidence(monkeypatch, tmp_path):
    completions = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway="http://127.0.0.1:1")
        evidence = [{
            "kind": "observation.text",
            "usable": True,
            "detail": "synthetic result",
            "source": {"provider": "fixture", "source_id": "sample-1"},
        }]
        registry = FakeExecutorRegistry(
            {"vision.observe", "future.inspect"},
            {"status": "succeeded", "reason_code": "ok", "evidence": evidence},
        )
        runner._device_executor_registry = registry
        runner._handle(
            {"job_id": "generic-1", "steps": [{"module": "future.inspect", "params": {"x": 1}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        assert registry.calls[:2] == [
            ("prepare", "future.inspect", {"x": 1}),
            ("execute", registry.handle),
        ]
        assert completions == [{"status": "success", "variables": {
            "detail": "device_executor_succeeded", "evidence": evidence,
        }}]
    finally:
        cloud.close()


@pytest.mark.parametrize("provider_status", ["failed", "refused"])
def test_generic_failure_never_reports_evidence(monkeypatch, tmp_path, provider_status):
    completions = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway="http://127.0.0.1:1")
        registry = FakeExecutorRegistry(
            {"vision.observe"},
            {"status": provider_status, "reason_code": "private", "evidence": []},
        )
        runner._device_executor_registry = registry
        runner._handle(
            {"job_id": "generic-2", "steps": [{"module": "vision.observe", "params": {}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        done = completions[-1]
        assert done["status"] == "failed"
        assert "evidence" not in done["variables"]
        assert done["error_message"] == f"device_executor_{provider_status}"
    finally:
        cloud.close()


def test_generic_start_is_journaled_before_execute_and_stop_discards(monkeypatch, tmp_path):
    completions = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway="http://127.0.0.1:1")
        registry = FakeExecutorRegistry(
            {"future.inspect"}, {"status": "succeeded", "reason_code": "ok", "evidence": []}
        )
        runner._device_executor_registry = registry
        original = runner._append_event

        def stop_after_journal(*args, **kwargs):
            original(*args, **kwargs)
            runner._stopping = True

        monkeypatch.setattr(runner, "_append_event", stop_after_journal)
        runner._handle(
            {"job_id": "generic-stop", "steps": [{"module": "future.inspect", "params": {}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        assert registry.calls == [
            ("prepare", "future.inspect", {}), ("discard", registry.handle),
        ]
        assert completions == []
    finally:
        cloud.close()


def test_runner_source_has_no_vision_specific_dispatch_branch():
    source = RUNNER_PATH.read_text()
    assert '== "vision.observe"' not in source
    assert "if module_id == 'vision.observe'" not in source


def test_a_robot_job_is_claimed_run_and_reported_succeeded(monkeypatch, tmp_path, gateway):
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        runner._handle(
            _cloud_job("j1", [{"params": {"plan": PLAN}}]),
            {"device_id": "dev-1", "device_secret": "s-1"},
        )

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
        receipt = done["variables"]["execution_receipt"]
        assert receipt["contract_version"] == "flyto.robotics.execution-receipt.v1"
        assert receipt["task_completion_eligible"] is False
        assert receipt["plan_sha256"] == _digest(PLAN)
    finally:
        cloud.close()


def test_tampered_cloud_device_handoff_is_refused_before_gateway(
    monkeypatch, tmp_path, gateway
):
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        dispatched = _cloud_job("j-handoff", [{"params": {"plan": PLAN}}])
        dispatched["steps"][0]["params"]["plan"]["goal"] = "tampered after dispatch"
        runner._handle(
            dispatched,
            {"device_id": "dev-1", "device_secret": "s-1"},
        )

        assert gateway.state["plans"] == []
        assert completions[-1]["status"] == "failed"
        assert completions[-1]["error_message"] == "device_handoff_invalid"
        assert "evidence" not in completions[-1].get("variables", {})
    finally:
        cloud.close()


def test_trace_bearing_cloud_job_without_handoff_is_refused_before_gateway(
    monkeypatch, tmp_path, gateway
):
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        runner._handle(
            {
                "job_id": "j-no-handoff",
                "device_id": "dev-1",
                "steps": [{"params": {"plan": PLAN}}],
                "input_params": {"_flyto_trace_id": "trace-j-no-handoff"},
            },
            {"device_id": "dev-1", "device_secret": "s-1"},
        )

        assert gateway.state["plans"] == []
        assert completions[-1]["status"] == "failed"
        assert completions[-1]["error_message"] == "device_handoff_invalid"
        assert "evidence" not in completions[-1].get("variables", {})
    finally:
        cloud.close()


def test_a_v2_gateway_without_a_terminal_receipt_fails_closed(monkeypatch, tmp_path):
    completions: list[dict] = []
    cloud = make_cloud(completions)
    missing = Fake(
        {
            "/v1/plans": lambda _p, _b: (
                200,
                {
                    "contract_version": "flyto.robotics.delivery-session.v2",
                    "session_id": "pln-no-receipt",
                    "status": "completed",
                },
            )
        }
    )
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=missing.url)
        runner._handle(
            {"job_id": "j-no-receipt", "steps": [{"params": {"plan": PLAN}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        done = completions[-1]
        assert done["status"] == "failed"
        assert done["error_message"] == "execution_receipt_missing"
        assert "evidence" not in done.get("variables", {})
    finally:
        cloud.close()
        missing.close()


def test_a_tampered_terminal_receipt_fails_closed(monkeypatch, tmp_path, gateway):
    completions: list[dict] = []
    cloud = make_cloud(completions)
    original = gateway.routes["/v1/deliveries/"]

    def tampered(path, body):
        status, payload = original(path, body)
        payload["execution_receipt"]["event_count"] += 1
        return status, payload

    gateway.routes["/v1/deliveries/"] = tampered
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        runner._handle(
            {"job_id": "j-tampered", "steps": [{"params": {"plan": PLAN}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        done = completions[-1]
        assert done["status"] == "failed"
        assert done["error_message"] == "execution_receipt_invalid"
        assert "evidence" not in done.get("variables", {})
    finally:
        cloud.close()


def test_a_job_this_device_cannot_run_is_failed_with_a_reason(monkeypatch, tmp_path, gateway):
    """Not skipped silently: an unrunnable job that looks pending forever is worse."""
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        runner._handle(
            {"job_id": "j2", "steps": [{"params": {"url": "https://x"}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
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
    refusing = Fake(
        {"/v1/plans": lambda p, b: (400, {"detail": "a plan that moves must end with safe_stop"})}
    )
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=refusing.url)
        runner._handle(
            {"job_id": "j3", "steps": [{"params": {"plan": PLAN}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
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
    stuck = Fake(
        {
            "/v1/plans": lambda p, b: (200, {"session_id": "pln-1", "status": "navigating"}),
            "/v1/deliveries/": lambda p, b: (200, {"session_id": "pln-1", "status": "navigating"}),
        }
    )
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=stuck.url)
        # The gateway in this test never states a bound, so the fallback is
        # what applies — renaming the constant without renaming this line is
        # how the suite hung for five minutes instead of failing.
        runner.DEFAULT_MISSION_WATCH_SECONDS = 0.2
        runner.GATEWAY_POLL_SECONDS = 0.05
        runner._handle(
            {"job_id": "j4", "steps": [{"params": {"plan": PLAN}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        done = completions[-1]
        assert done["status"] == "failed"
        assert "unknown" in done["error_message"]
        assert "evidence" not in (done.get("variables") or {}), (
            "an unknown outcome must not claim an arrival"
        )
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
        "RestrictedPython",
        "anthropic",
        "boto3",
        "botocore",
        "core",
        "cryptography",
        "dotenv",
        "fastapi",
        "firebase_admin",
        "flyto_blueprint",
        "google",
        "httpx",
        "hvac",
        "jwt",
        "psutil",
        "pydantic",
        "redis",
        "requests",
        "stripe",
        "websockets",
        "yaml",
        "playwright",
        "rclpy",
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
    failing = Fake(
        {
            "/v1/plans": lambda p, b: (200, {"session_id": "pln-2", "status": "running"}),
            "/v1/deliveries/": lambda p, b: (
                200,
                {
                    "session_id": "pln-2",
                    "status": "failed",
                    "failure_reason": "obstacle_stop",
                    "pose": {"x": 0.1},
                    "minimum_range": 0.18,
                },
            ),
        }
    )
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=failing.url)
        runner._handle(
            {"job_id": "j5", "steps": [{"params": {"plan": PLAN}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
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


def test_the_lease_header_is_the_one_the_device_api_reads(monkeypatch, tmp_path, gateway):
    """The claim's lease must go back under the header the route reads.

    A guessed header name is indistinguishable from sending none: the mission
    runs, the robot moves, and the completion is refused 409 — which is what
    happened on the real API while these tests were green against a fake that
    never checked.
    """
    completions: list[dict] = []
    cloud = make_cloud(completions)
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
        assert runner.LEASE_HEADER == "x-flyto2-job-lease"
        runner._handle(
            {"job_id": "j6", "steps": [{"params": {"plan": PLAN}}]},
            {"device_id": "dev-1", "device_secret": "s-1"},
        )
        assert completions, "the completion was accepted, so the lease was recognised"
        sent = [h for p, h, _ in cloud.seen if p.endswith("/complete")][0]
        lowered = {k.lower(): v for k, v in sent.items()}
        assert lowered.get("x-flyto2-job-lease") == "lease-1"
    finally:
        cloud.close()


# -- a step authored on the canvas ---------------------------------------


TURN_STEP = {"id": "sweep", "module": "robotics.turn", "params": {"degrees": 90}}


def install_robotics_package(monkeypatch, *, trusted=None, catalog=None):
    """Install only the public 0.1.1 surface the runner consumes."""
    package = types.ModuleType("flyto_modules_robotics")
    package.__path__ = []
    gateway_module = types.ModuleType("flyto_modules_robotics.gateway")
    steps_module = types.ModuleType("flyto_modules_robotics.steps")

    class GatewayError(RuntimeError):
        pass

    class GatewayRefused(GatewayError):
        pass

    class CapabilityCatalogError(ValueError):
        pass

    class PlanBuildError(ValueError):
        pass

    def fetch_catalog():
        if catalog is not None:
            return catalog()
        request = urllib.request.Request(
            f"{os.environ['FLYTO_ROBOTICS_GATEWAY_URL']}/v1/capabilities",
            headers={"Authorization": f"Bearer {os.environ['FLYTO_ROBOTICS_DELIVERY_TOKEN']}"},
        )
        try:
            with urllib.request.urlopen(request, timeout=5.0) as response:
                payload = json.load(response)
        except urllib.error.HTTPError:
            raise GatewayRefused() from None
        except urllib.error.URLError:
            raise GatewayError() from None
        if not isinstance(payload, dict) or payload.get("contract_version") != "catalog.v1":
            raise CapabilityCatalogError()
        return ("immutable-catalog", payload.get("revision"))

    def default_trusted(module_id, params, *, robot_id, catalog):
        if module_id != "robotics.turn":
            return None
        return {
            "contract_version": "flyto.robotics.plan.v1",
            "plan_id": "trusted.turn.v1",
            "robot_id": robot_id,
            "steps": [
                {
                    "step_id": "turn",
                    "capability": "turn_relative",
                    "arguments": {
                        "degrees": params["degrees"],
                        "clockwise": params.get("clockwise", False),
                    },
                },
                {"step_id": "stop", "capability": "safe_stop", "arguments": {}},
            ],
        }

    gateway_module.GatewayError = GatewayError
    gateway_module.GatewayRefused = GatewayRefused
    gateway_module.CapabilityCatalogError = CapabilityCatalogError
    gateway_module.capability_catalog = fetch_catalog
    steps_module.PlanBuildError = PlanBuildError
    steps_module.step_module_id = lambda step: str(step.get("module") or "")
    steps_module.trusted_plan_for_step = trusted or default_trusted
    monkeypatch.setitem(sys.modules, "flyto_modules_robotics", package)
    monkeypatch.setitem(sys.modules, "flyto_modules_robotics.gateway", gateway_module)
    monkeypatch.setitem(sys.modules, "flyto_modules_robotics.steps", steps_module)
    return gateway_module, steps_module


def capability_gateway(status=200):
    return Fake(
        {
            "/v1/capabilities": lambda p, b: (
                status,
                {"contract_version": "catalog.v1", "revision": 7},
            )
        }
    )


def test_a_canvas_authored_motion_step_becomes_a_trusted_plan(monkeypatch, tmp_path):
    """The runner holds no table of its own: it asks the package that owns the
    module identifiers, so the canvas and the robot cannot disagree about what
    a step means."""
    seen = {}

    def trusted(module_id, params, *, robot_id, catalog):
        seen.update(module_id=module_id, params=params, robot_id=robot_id, catalog=catalog)
        return {
            "contract_version": "flyto.robotics.plan.v1",
            "plan_id": "trusted",
            "robot_id": robot_id,
            "steps": [{"capability": "safe_stop"}],
        }

    install_robotics_package(monkeypatch, trusted=trusted)
    monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
    gateway = capability_gateway()
    try:
        runner = load_runner(monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway=gateway.url)
        params = {"degrees": 90, "clockwise": True}
        plan = runner._plan_from(
            {"job_id": "j", "steps": [{"module": "robotics.turn", "params": params}]}
        )
        assert plan["robot_id"] == "flyto-tb3-lab-001"
        assert seen == {
            "module_id": "robotics.turn",
            "params": params,
            "robot_id": "flyto-tb3-lab-001",
            "catalog": ("immutable-catalog", 7),
        }
        assert gateway.paths() == ["/v1/capabilities"]
    finally:
        gateway.close()


@pytest.mark.parametrize(
    "step",
    [
        {"module": "robotics.move", "params": {"distance_m": 0.4, "reverse": True}},
        {"module": "robotics.turn", "params": {"degrees": 90, "clockwise": True}},
    ],
)
def test_canvas_motion_values_and_direction_flags_pass_unchanged(monkeypatch, tmp_path, step):
    received = []

    def trusted(module_id, params, *, robot_id, catalog):
        received.append((module_id, params, robot_id, catalog))
        return {"contract_version": "plan.v1", "plan_id": "trusted", "robot_id": robot_id}

    install_robotics_package(monkeypatch, trusted=trusted, catalog=lambda: ("catalog",))
    monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    runner._plan_from({"steps": [step]})
    assert received == [
        (step["module"], step["params"], "flyto-tb3-lab-001", ("catalog",))
    ]


def test_without_a_robot_id_the_step_is_refused_rather_than_guessed(monkeypatch, tmp_path):
    """The gateway checks a plan's robot_id against its own job, so a guess
    here only becomes a refusal further from the cause."""
    install_robotics_package(monkeypatch)
    monkeypatch.delenv("FLYTO_ROBOTICS_ROBOT_ID", raising=False)
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    assert runner._plan_from({"job_id": "j", "steps": [TURN_STEP]}) is None


def test_a_step_the_package_does_not_know_is_still_not_a_robot_job(monkeypatch, tmp_path):
    install_robotics_package(monkeypatch)
    monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    assert (
        runner._plan_from(
            {"job_id": "j", "steps": [{"module": "browser.click", "params": {"selector": "#go"}}]}
        )
        is None
    )


def test_missing_root_package_keeps_job_plan_unsupported(monkeypatch, tmp_path):
    for name in tuple(sys.modules):
        if name == "flyto_modules_robotics" or name.startswith("flyto_modules_robotics."):
            monkeypatch.delitem(sys.modules, name, raising=False)
    real_import = builtins.__import__

    def absent(name, *args, **kwargs):
        if name == "flyto_modules_robotics" or name.startswith("flyto_modules_robotics."):
            raise ModuleNotFoundError("optional package absent", name="flyto_modules_robotics")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", absent)
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    assert runner._plan_from({"steps": [TURN_STEP]}) is None


def test_internal_package_import_failure_is_fixed_trusted_fault(monkeypatch, tmp_path):
    install_robotics_package(monkeypatch)
    real_import = builtins.__import__

    def broken(name, *args, **kwargs):
        if name == "flyto_modules_robotics.gateway":
            raise ImportError(SENTINEL_SECRET)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", broken)
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    with pytest.raises(runner.AuthoredPlanRefused) as raised:
        runner._plan_from({"steps": [TURN_STEP]})
    assert raised.value.reason_code == "trusted_plan_construction_failed"
    assert SENTINEL_SECRET not in str(raised.value)


@pytest.mark.parametrize(
    ("module_name", "symbol"),
    [
        ("flyto_modules_robotics.gateway", "capability_catalog"),
        ("flyto_modules_robotics.steps", "trusted_plan_for_step"),
    ],
)
def test_missing_011_public_symbol_is_fixed_trusted_fault(
    monkeypatch, tmp_path, module_name, symbol
):
    _, steps_module = install_robotics_package(monkeypatch)
    target = sys.modules[module_name] if "gateway" in module_name else steps_module
    monkeypatch.delattr(target, symbol)
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    with pytest.raises(runner.AuthoredPlanRefused) as raised:
        runner._plan_from({"steps": [TURN_STEP]})
    assert raised.value.reason_code == "trusted_plan_construction_failed"


def test_an_unbuildable_motion_step_is_refused_not_approximated(monkeypatch, tmp_path):
    """A distance out of bounds must not become the nearest allowed distance."""
    gateway_module, _ = install_robotics_package(monkeypatch)
    monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    monkeypatch.setattr(
        gateway_module,
        "capability_catalog",
        lambda: (_ for _ in ()).throw(gateway_module.CapabilityCatalogError()),
    )
    with pytest.raises(runner.AuthoredPlanRefused) as raised:
        runner._plan_from(
            {"job_id": "j", "steps": [{"module": "robotics.move", "params": {"distance_m": 99}}]}
        )
    assert raised.value.reason_code == "capability_catalog_invalid"


def test_inline_plan_never_fetches_capabilities(monkeypatch, tmp_path):
    calls = []

    def forbidden_catalog():
        calls.append("capabilities")
        raise AssertionError("inline plans must not fetch the capability catalog")

    install_robotics_package(monkeypatch, catalog=forbidden_catalog)
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )
    assert runner._plan_from({"steps": [{"params": {"plan": PLAN}}]}) == PLAN
    assert calls == []


def test_systemd_provisions_exact_trusted_api_version():
    unit = (RUNNER_PATH.parent / "systemd" / "flyto-job-runner.service").read_text()
    assert 'pip install "flyto-modules-robotics==0.1.1"' in unit
    assert "flyto-modules-robotics==0.1.0" not in unit


def test_the_watch_window_comes_from_the_mission_not_a_constant(monkeypatch, tmp_path):
    """The gateway owns how long a mission may run; a constant here is a
    second opinion that drifts from it. The job deployed on the lab robot
    allows 600s, so a fixed 300s would call a legitimately running mission
    unknown at half time — and the cloud takes that "failed" at face value.
    """
    runner = load_runner(
        monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
    )

    assert runner._watch_seconds({"mission_timeout_seconds": 600.0}) == 620.0
    # A gateway that does not say falls back rather than waiting forever.
    assert runner._watch_seconds({}) == runner.DEFAULT_MISSION_WATCH_SECONDS
    assert (
        runner._watch_seconds({"mission_timeout_seconds": 0})
        == runner.DEFAULT_MISSION_WATCH_SECONDS
    )


# -- how the credential is kept ------------------------------------------


# -- what the runner records for someone else to read --------------------


DEVICE = {"device_id": "dev-1", "device_secret": "s-1"}

#: A value that exists only to prove it never leaves. If this string turns up in
#: an event or in a log line, something is forwarding server text verbatim.
SENTINEL_SECRET = "Bearer AKIAZZZZZZZZZZZZZZZZ.leaked-body-token"


def records(runner) -> list[dict]:
    """Every event on disk, read back through the real journal, not a fake."""
    events = runner.device_events()
    if not runner.EVENT_JOURNAL.exists():
        return []
    return [item["event"] for item in events.DeviceEventJournal(runner.EVENT_JOURNAL).read_all()]


def reason_codes(runner) -> list[str]:
    return [event["reason_code"] for event in records(runner)]


def leaking_cloud(completions, *, complete_status=200):
    """A cloud whose error bodies carry a secret, to prove none is forwarded."""

    def route(path, body, headers):
        if path.endswith("/claim"):
            return 200, {"lease_id": "lease-1"}
        if headers.get(LEASE_HEADER) != "lease-1":
            return 409, {"ok": False, "error": "Job lease is missing or invalid"}
        if complete_status != 200:
            return complete_status, {"error": SENTINEL_SECRET}
        completions.append(body)
        return 200, {"ok": True}

    return Fake({"/api/devices/jobs/": route}, pass_headers=True)


#: What a real transport failure hands over. ``URLError.reason`` is an OS error,
#: a TLS message or a socket message, and urllib has put the URL beside it — so
#: this one carries every class of thing that must not be copied out of it: a
#: credential, a URL with a token in the query string, a response body, the plan
#: and a network address.
UNREACHABLE_REASON = (
    "[Errno -2] Name or service not known while posting "
    "https://cloud.internal.example:8443/api/devices/jobs/j16/complete"
    f"?access_token=tok-abc ({SENTINEL_SECRET}) plan={PLAN['plan_id']} "
    "peer=10.77.0.9:8443 body={'error': 'nothing answered'}"
)


def sever_the_uplink(runner, monkeypatch):
    """Everything works except the completion, which never reaches Cloud.

    A DNS failure, a refused connection, a disconnect and a timeout all arrive
    as ``URLError`` rather than ``HTTPError``, so one stands in for the class.
    The claim and the mission are left alone: the point is a job that really ran
    and really cannot be reported.
    """
    severed = urllib.error.URLError(UNREACHABLE_REASON)
    real_post = runner._post

    def post(base, path, payload, headers, timeout=35.0):
        if path.endswith("/complete"):
            raise severed
        return real_post(base, path, payload, headers, timeout)

    monkeypatch.setattr(runner, "_post", post)
    return severed


#: A second sentinel, distinct from the first, so the two failures in the nested
#: case cannot be mistaken for each other. A journal write fails for its own
#: reasons — a full disk, a path that stopped being writable — and whatever it
#: says about them is no more publishable than the transport's own text.
JOURNAL_SECRET = "Bearer AKIAYYYYYYYYYYYYYYYY.journal-write-token"


class JournalRefused(RuntimeError):
    """A journal write that fails for reasons of its own, carrying a secret."""


def refuse_to_record(runner, monkeypatch, *, reason_code, error):
    """Let every event through except one, which cannot be written at all.

    Named by reason code rather than by call count: an ordinal would silently
    start refusing a different event the day one is added, and the test would
    still pass while proving something else.
    """
    real_append = runner._append_event

    def append(credentials, **fields):
        if fields.get("reason_code") == reason_code:
            raise error
        return real_append(credentials, **fields)

    monkeypatch.setattr(runner, "_append_event", append)
    return error


class TestTheRecordOfWhatTheRunnerDid:
    """The audit half, read back through the real DeviceEventJournal."""

    def test_a_succeeded_job_records_started_then_succeeded(self, monkeypatch, tmp_path, gateway):
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            runner._handle({"job_id": "j1", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            written = records(runner)
            assert [event["status"] for event in written] == ["started", "succeeded"]
            assert [event["reason_code"] for event in written] == [
                "job_execution_started",
                "job_completed",
            ]
            # Both name the job, so an upstream reader can join them.
            assert {event["run_id"] for event in written} == {"j1"}
            assert {event["resource_id"] for event in written} == {"dev-1"}
            assert written[-1]["details"]["job"]["evidence_count"] == 2
        finally:
            cloud.close()

    def test_an_accepted_completion_returns_normally(self, monkeypatch, tmp_path, gateway):
        """The refusal path used to be the only one that worked. The success
        path read a name that does not exist in the function it was moved into,
        so every job that *was* reported raised NameError afterwards — the poll
        loop backed off and re-claimed a job the cloud already considered done,
        which for a robot means running the same mission twice."""
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            body = runner._completion(status="succeeded", detail="done", pose={"x": 0.1})
            assert (
                runner._report_completion(
                    DEVICE, job_id="j14", headers={LEASE_HEADER: "lease-1"}, body=body
                )
                is None
            )
            assert completions[-1]["status"] == "success"
            # And through the whole path, not just the helper.
            runner._handle({"job_id": "j15", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            assert completions[-1]["status"] == "success"
            assert reason_codes(runner)[-1] == "job_completed"
        finally:
            cloud.close()

    def test_the_start_is_recorded_before_the_gateway_is_told_anything(
        self, monkeypatch, tmp_path, gateway
    ):
        """Not after. A robot that moved with no record of having been told to
        is the state this exists to prevent, and the ordering is the whole of
        the guarantee."""
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            observed: list[str] = []
            real_post = runner._post

            def watch(base, path, payload, headers, timeout=35.0):
                if path == "/v1/plans":
                    observed.append(f"gateway:{len(records(runner))}")
                return real_post(base, path, payload, headers, timeout)

            monkeypatch.setattr(runner, "_post", watch)
            runner._handle({"job_id": "j1", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            # The plan reached the gateway only once the start was already
            # durable: one record existed at that moment, and it was the start.
            assert observed == ["gateway:1"]
        finally:
            cloud.close()

    def test_a_failed_mission_is_recorded_as_failed_with_a_fixed_reason(
        self, monkeypatch, tmp_path
    ):
        completions: list[dict] = []
        cloud = make_cloud(completions)
        failing = Fake(
            {
                "/v1/plans": lambda p, b: (200, {"session_id": "pln-2", "status": "running"}),
                "/v1/deliveries/": lambda p, b: (
                    200,
                    {
                        "session_id": "pln-2",
                        "status": "failed",
                        "failure_reason": "obstacle_stop",
                    },
                ),
            }
        )
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=failing.url)
            runner._handle({"job_id": "j5", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            terminal = records(runner)[-1]
            assert terminal["status"] == "failed"
            assert terminal["severity"] == "error"
            assert terminal["reason_code"] == "mission_failed"
            assert terminal["message"] == "The job did not finish successfully."
            # The gateway's own words stay in the Cloud completion body. They
            # are not a reason code and they do not belong in a fleet stream.
            assert "obstacle_stop" not in json.dumps(terminal)
            assert terminal["details"]["job"]["evidence_count"] == 0
        finally:
            cloud.close()
            failing.close()

    def test_a_job_this_device_cannot_run_is_refused_and_never_reaches_the_gateway(
        self, monkeypatch, tmp_path, gateway
    ):
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            runner._handle({"job_id": "j2", "steps": [{"params": {"url": "https://x"}}]}, DEVICE)
            refusal = records(runner)[-1]
            assert refusal["status"] == "refused"
            assert refusal["reason_code"] == "job_plan_unsupported"
            assert refusal["action_codes"] == ["inspect_job_steps"]
            assert gateway.state["plans"] == []
            # It is still reported upstream, so the job does not sit pending.
            assert completions[-1]["status"] == "failed"
        finally:
            cloud.close()

    def test_refused_catalog_is_recorded_and_completed_before_any_plan_post(
        self, monkeypatch, tmp_path
    ):
        install_robotics_package(monkeypatch)
        order: list[str] = []

        def cloud_route(path, body, headers):
            if path.endswith("/claim"):
                order.append("claim")
                return 200, {"lease_id": "lease-1"}
            order.append(f"complete:{body.get('error_message')}")
            return 200, {"ok": True}

        def capabilities(path, body):
            order.append("capabilities")
            return 403, {"error": SENTINEL_SECRET}

        def plans(path, body):
            order.append("plan")
            return 200, {"session_id": "must-not-exist"}

        cloud = Fake({"/api/devices/jobs/": cloud_route}, pass_headers=True)
        gateway = Fake({"/v1/capabilities": capabilities, "/v1/plans": plans})
        try:
            monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            real_append = runner._append_event

            def append(credentials, **fields):
                order.append(f"event:{fields['reason_code']}")
                return real_append(credentials, **fields)

            monkeypatch.setattr(runner, "_append_event", append)
            runner._handle({"job_id": "canvas-refused", "steps": [TURN_STEP]}, DEVICE)

            assert order == [
                "claim",
                "capabilities",
                "event:capability_catalog_refused",
                "complete:robot capability verification failed",
            ]
            assert "/v1/plans" not in gateway.paths()
            assert reason_codes(runner) == ["capability_catalog_refused"]
            assert "job_execution_started" not in reason_codes(runner)
            assert SENTINEL_SECRET not in json.dumps(records(runner))
        finally:
            cloud.close()
            gateway.close()

    @pytest.mark.parametrize(
        ("fault_name", "reason_code"),
        [
            ("GatewayError", "capability_catalog_unavailable"),
            ("GatewayRefused", "capability_catalog_refused"),
            ("CapabilityCatalogError", "capability_catalog_invalid"),
        ],
    )
    def test_typed_catalog_faults_fail_closed(
        self, monkeypatch, tmp_path, fault_name, reason_code
    ):
        gateway_module, _ = install_robotics_package(monkeypatch)
        fault = getattr(gateway_module, fault_name)
        monkeypatch.setattr(
            gateway_module,
            "capability_catalog",
            lambda: (_ for _ in ()).throw(fault(SENTINEL_SECRET)),
        )
        completions: list[dict] = []
        cloud = make_cloud(completions)
        gateway = Fake({"/v1/plans": lambda p, b: (200, {"session_id": "never"})})
        try:
            monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            runner._handle({"job_id": f"typed-{fault_name}", "steps": [TURN_STEP]}, DEVICE)
            assert reason_codes(runner) == [reason_code]
            assert gateway.paths() == []
            assert completions == [
                {
                    "status": "failed",
                    "error_message": "robot capability verification failed",
                    "variables": {"detail": "robot capability verification failed"},
                }
            ]
            assert SENTINEL_SECRET not in json.dumps(records(runner))
        finally:
            cloud.close()
            gateway.close()

    @pytest.mark.parametrize(
        ("result", "fault", "reason_code"),
        [
            (None, None, "capability_catalog_incompatible"),
            (None, RuntimeError(SENTINEL_SECRET), "trusted_plan_construction_failed"),
        ],
    )
    def test_trusted_plan_faults_are_not_downgraded_to_unsupported(
        self, monkeypatch, tmp_path, result, fault, reason_code
    ):
        def trusted(module_id, params, *, robot_id, catalog):
            if fault is not None:
                raise fault
            return result

        install_robotics_package(monkeypatch, trusted=trusted, catalog=lambda: ("catalog",))
        completions: list[dict] = []
        cloud = make_cloud(completions)
        gateway = Fake({"/v1/plans": lambda p, b: (200, {"session_id": "never"})})
        try:
            monkeypatch.setenv("FLYTO_ROBOTICS_ROBOT_ID", "flyto-tb3-lab-001")
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            runner._handle({"job_id": "trusted-fault", "steps": [TURN_STEP]}, DEVICE)
            assert reason_codes(runner) == [reason_code]
            assert "job_plan_unsupported" not in reason_codes(runner)
            assert gateway.paths() == []
        finally:
            cloud.close()
            gateway.close()

    def test_a_claim_without_a_lease_starts_nothing_at_all(self, monkeypatch, tmp_path, gateway):
        """The completion endpoint refuses a report with no lease, so running
        the mission would move a robot that can never be reported on."""
        leaseless = Fake({"/api/devices/jobs/": lambda p, b: (200, {})})
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=leaseless.url, gateway=gateway.url)
            runner._handle({"job_id": "j7", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)

            assert gateway.state["plans"] == [], "the gateway must not have been called"
            assert [p for p in leaseless.paths() if p.endswith("/complete")] == []
            refusal = records(runner)[-1]
            assert (refusal["status"], refusal["reason_code"]) == ("refused", "job_lease_missing")
            assert refusal["action_codes"] == ["retry_job_claim", "inspect_job_lease"]
        finally:
            leaseless.close()

    def test_an_unwriteable_start_journal_stops_the_job_before_any_motion(
        self, monkeypatch, tmp_path, gateway
    ):
        """An audit record that may or may not have been written is not an audit
        record. Fail closed rather than move and hope."""
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            # A journal whose parent is a regular file: no directory can be
            # created there and no retry will change that.
            wall = tmp_path.resolve() / "wall"
            wall.write_text("not a directory")
            runner.EVENT_JOURNAL = wall / "device-events.jsonl"

            # Named exactly: the journal refuses an unusable path with its own
            # DeviceEventError, and the filesystem refuses to make a directory
            # inside a regular file with an OSError. A bare Exception here would
            # also pass if the runner raised NameError on the way to the
            # gateway, which is the opposite of the guarantee being asserted.
            events = runner.device_events()
            with pytest.raises((OSError, events.DeviceEventError)):
                runner._handle({"job_id": "j8", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)

            assert gateway.state["plans"] == [], "no plan may reach the gateway"
            assert [p for p in cloud.paths() if p.endswith("/complete")] == []
        finally:
            cloud.close()

    def test_a_refused_completion_is_recorded_and_re_raised(self, monkeypatch, tmp_path, gateway):
        """The robot moved. If the report will not go through, the device must
        still be able to say what happened."""
        refusing = leaking_cloud([], complete_status=409)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=refusing.url, gateway=gateway.url)
            with pytest.raises(urllib.error.HTTPError):
                runner._handle({"job_id": "j9", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)

            written = records(runner)
            assert [event["reason_code"] for event in written] == [
                "job_execution_started",
                "job_completed",
                "completion_report_refused",
            ]
            refusal = written[-1]
            assert refusal["status"] == "failed"
            assert refusal["details"] == {"upstream": {"http_status": 409}}
            # The terminal outcome was recorded *before* the report was sent,
            # which is what makes a refused report diagnosable at all.
            assert written[1]["status"] == "succeeded"
        finally:
            refusing.close()

    def test_a_refused_unsupported_plan_report_is_recorded_the_same_way(
        self, monkeypatch, tmp_path, gateway
    ):
        """Every completion goes through one helper. A second call site posting
        directly would be a path where a 409 produced no event at all."""
        refusing = leaking_cloud([], complete_status=409)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=refusing.url, gateway=gateway.url)
            with pytest.raises(urllib.error.HTTPError):
                runner._handle({"job_id": "j10", "steps": [{"params": {"u": "x"}}]}, DEVICE)

            assert reason_codes(runner) == ["job_plan_unsupported", "completion_report_refused"]
            assert gateway.state["plans"] == []
        finally:
            refusing.close()

    def test_no_response_body_reaches_an_event_or_the_log(
        self, monkeypatch, tmp_path, gateway, caplog
    ):
        """A server's error body is server text: it has echoed authorization
        headers and query strings before now, into a log an operator pastes
        into a ticket."""
        refusing = leaking_cloud([], complete_status=409)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=refusing.url, gateway=gateway.url)
            with caplog.at_level("DEBUG"), pytest.raises(urllib.error.HTTPError):
                runner._handle({"job_id": "j11", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)

            written = json.dumps(records(runner))
            assert SENTINEL_SECRET not in written
            assert "AKIA" not in written
            assert SENTINEL_SECRET not in caplog.text
            assert "AKIA" not in caplog.text
            # The status code still has to be sayable, or the refusal is silent.
            assert "409" in caplog.text
        finally:
            refusing.close()

    def test_an_undeliverable_completion_is_recorded_as_unreachable_and_re_raised(
        self, monkeypatch, tmp_path, gateway
    ):
        """A refusal and an unreachable Cloud are different faults with
        different first moves. Only HTTPError was recorded, so a DNS failure, a
        refused connection, a disconnect or a timeout left the terminal event on
        disk with nothing after it saying the report never arrived — and an
        offline operator could not tell "Cloud refused it" from "Cloud could not
        be reached"."""
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            severed = sever_the_uplink(runner, monkeypatch)

            with pytest.raises(urllib.error.URLError) as raised:
                runner._handle({"job_id": "j16", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            # The original, not a translation of it: the poll loop backs off on
            # exactly what urllib raised, and nothing re-runs the mission.
            assert raised.value is severed

            written = records(runner)
            assert [event["reason_code"] for event in written] == [
                "job_execution_started",
                "job_completed",
                "completion_report_unreachable",
            ]
            # The terminal outcome was durable *before* the report was
            # attempted. That ordering is what makes an undelivered report
            # explainable from this device alone.
            assert written[1]["status"] == "succeeded"

            undelivered = written[-1]
            assert undelivered["status"] == "failed"
            assert undelivered["severity"] == "error"
            assert undelivered["run_id"] == "j16"
            assert undelivered["message"] == (
                "The outcome was recorded here but the report could not be delivered."
            )
            # Retry the report, and check the link that carries it. Codes, and
            # bounded: no shell command and no endpoint.
            assert undelivered["action_codes"] == [
                "retry_completion_report",
                "inspect_device_uplink",
            ]
            # Nothing dynamic at all. A refusal has a status code worth
            # carrying; an unreachable peer offers only the exception's own
            # text, and none of that is bounded.
            assert undelivered["details"] == {}
            assert completions == [], "nothing was accepted upstream"
        finally:
            cloud.close()

    def test_no_transport_failure_text_reaches_an_event_or_the_log(
        self, monkeypatch, tmp_path, gateway, caplog
    ):
        """URLError.reason is whatever the failing transport chose to put there:
        an OS error, a TLS message, a URL, the site's own Cloud address. Copying
        it into a fleet-wide event stream is how a network identity leaves the
        robot."""
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            sever_the_uplink(runner, monkeypatch)

            with caplog.at_level("DEBUG"), pytest.raises(urllib.error.URLError):
                runner._handle({"job_id": "j17", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)

            written = json.dumps(records(runner))
            leaked = (
                SENTINEL_SECRET,
                "AKIA",
                "cloud.internal.example",
                "https://",
                "access_token=tok-abc",
                "Name or service not known",
                "10.77.0.9",
                PLAN["plan_id"],
                "127.0.0.1",
            )
            for forbidden in leaked:
                assert forbidden not in written, f"{forbidden!r} reached an event"
            # The plan id is the one thing this device may say to its own local
            # log — it already does, before the mission starts. Everything the
            # transport failure carried must not be there.
            for forbidden in (item for item in leaked if item != PLAN["plan_id"]):
                assert forbidden not in caplog.text, f"{forbidden!r} reached the log"
            # The failure still has to be sayable, or it is silent: the job and
            # the status this device tried to report, and nothing else.
            assert "j17" in caplog.text
            assert "could not be delivered" in caplog.text
        finally:
            cloud.close()

    def test_an_unrecordable_undelivered_report_still_raises_and_still_says_nothing(
        self, monkeypatch, tmp_path, gateway, caplog
    ):
        """Two failures at once: the report cannot be delivered, and the record
        of that cannot be written either.

        This is the branch where a device is least able to explain itself, and
        the one where it is most tempting to dump everything into the log. Both
        exceptions must stay unquoted — the transport's text names the site's
        Cloud address, and the journal's names whatever the filesystem was doing
        — and neither may replace the URLError the caller has to see.
        """
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            severed = sever_the_uplink(runner, monkeypatch)
            # Only the report-failure event is refused. The start and the
            # terminal outcome are written for real, through the real journal:
            # what this fixes must not cost the records that already worked.
            refuse_to_record(
                runner,
                monkeypatch,
                reason_code="completion_report_unreachable",
                error=JournalRefused(f"journal write failed: {JOURNAL_SECRET} /var/lib/flyto"),
            )

            with caplog.at_level("DEBUG"), pytest.raises(urllib.error.URLError) as raised:
                runner._handle({"job_id": "j18", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            # The transport failure, not the journal failure. Swapping them
            # would tell the poll loop the wrong thing about what went wrong,
            # and would lose the only exception the caller can act on.
            assert raised.value is severed
            assert not isinstance(raised.value, JournalRefused)

            # The two events that could be written still were, in order, and the
            # one that could not is simply absent — never a half-written stand-in.
            assert reason_codes(runner) == ["job_execution_started", "job_completed"]
            assert records(runner)[-1]["status"] == "succeeded"

            # Nothing was accepted upstream and the mission was not run twice.
            # A device that cannot record an outcome must still never repeat one.
            assert completions == []
            assert len(gateway.state["plans"]) == 1

            written = json.dumps(records(runner))
            for forbidden in (
                JOURNAL_SECRET,
                SENTINEL_SECRET,
                "AKIA",
                "cloud.internal.example",
                "https://",
                "access_token=tok-abc",
                "Name or service not known",
                "10.77.0.9",
                "/var/lib/flyto",
                "journal write failed",
            ):
                assert forbidden not in caplog.text, f"{forbidden!r} reached the log"
                assert forbidden not in written, f"{forbidden!r} reached an event"

            # Bounded and still legible: the job, what this device tried to
            # report, and the class of what stopped the record — no traceback,
            # because a traceback prints the chained URLError verbatim.
            assert "j18" in caplog.text
            assert "could not be delivered" in caplog.text
            assert "could not be recorded either" in caplog.text
            assert "JournalRefused" in caplog.text
        finally:
            cloud.close()

    def test_events_carry_no_secret_no_plan_and_no_address(self, monkeypatch, tmp_path, gateway):
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            runner._handle({"job_id": "j12", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            written = json.dumps(records(runner))
            for forbidden in ("s-1", "Bearer", "127.0.0.1", "http://", PLAN["plan_id"], "t" * 40):
                assert forbidden not in written, f"{forbidden!r} reached an event"
            for event in records(runner):
                assert event["redaction"]["raw_logs_included"] is False
                assert event["redaction"]["credentials_included"] is False
                assert event["redaction"]["personal_data_included"] is False
                assert event["sequence"] > 0
                assert event["sequence"] <= 2**53
                assert event["observed_at"].endswith("Z")
        finally:
            cloud.close()

    def test_the_journal_is_owner_only(self, monkeypatch, tmp_path, gateway):
        completions: list[dict] = []
        cloud = make_cloud(completions)
        try:
            runner = load_runner(monkeypatch, tmp_path, cloud=cloud.url, gateway=gateway.url)
            runner._handle({"job_id": "j13", "steps": [{"params": {"plan": PLAN}}]}, DEVICE)
            assert runner.EVENT_JOURNAL.stat().st_mode & 0o077 == 0
            assert runner.EVENT_JOURNAL.parent.stat().st_mode & 0o077 == 0
        finally:
            cloud.close()


class TestWhichDeviceTheEventsAreAbout:
    """There is no placeholder identity, and that is the point.

    A shipped "unidentified-device" would mean every robot in the fleet emitting
    under one resource_id: the records interleave into a stream that names no
    machine, and the first time someone needs to know which robot refused they
    cannot find out — from records that looked complete the whole time.
    """

    def test_the_paired_device_id_is_used_by_default(self, monkeypatch, tmp_path):
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        assert runner._resource_id(DEVICE) == "dev-1"

    def test_a_configured_identity_wins(self, monkeypatch, tmp_path):
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        monkeypatch.setenv("FLYTO_ROBOT_RESOURCE_ID", "ward-3-porter")
        assert runner._resource_id(DEVICE) == "ward-3-porter"

    def test_no_identity_at_all_fails_closed(self, monkeypatch, tmp_path):
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        with pytest.raises(runner.RunnerError, match="no device identity"):
            runner._resource_id({})

    def test_no_placeholder_identity_is_reachable_as_a_value(self):
        """No *runnable* string in the runner is a stand-in device name.

        Read from the AST rather than from the file's text. A raw substring
        search over the source cannot tell a value the code could return from
        prose describing why it must not exist — it matched the docstring on
        _resource_id that explains the rule, so the test failed precisely
        because the reasoning had been written down. Docstrings and comments are
        excluded here for that reason; what remains is every string literal that
        could actually reach an event.
        """
        import ast

        tree = ast.parse(RUNNER_PATH.read_text())
        docstrings = {
            id(node.body[0].value)
            for node in ast.walk(tree)
            if isinstance(
                node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)
            )
            and node.body
            and isinstance(node.body[0], ast.Expr)
            and isinstance(node.body[0].value, ast.Constant)
            and isinstance(node.body[0].value.value, str)
        }
        live = [
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and id(node) not in docstrings
        ]
        placeholders = [value for value in live if "unidentified" in value.lower()]
        assert placeholders == [], f"a stand-in identity is reachable: {placeholders}"

        # And the shape of the rule, not just the one name it would have used:
        # every way out of _resource_id is a derived identity or a raised
        # refusal. A bare string return is what a fallback looks like, whatever
        # it is spelled.
        resolver = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_resource_id"
        )
        returned = [
            node.value
            for node in ast.walk(resolver)
            if isinstance(node, ast.Return) and node.value is not None
        ]
        assert returned, "_resource_id must return something"
        assert not any(isinstance(value, ast.Constant) for value in returned)

    def test_an_id_the_contract_cannot_hold_is_linked_by_digest_not_dropped(
        self, monkeypatch, tmp_path
    ):
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        derived = runner._resource_id({"device_id": "dev 1 (lab)"})
        assert derived.startswith("device-") and " " not in derived
        assert derived == runner._resource_id({"device_id": "dev 1 (lab)"}), "must be stable"


class TestWhereTheRecordsAreKept:
    """Each installed service gets a writable owner-only journal of its own."""

    def unit(self, name: str) -> str:
        return (RUNNER_PATH.parent / "systemd" / name).read_text()

    def settings(self, text: str, key: str) -> list[str]:
        return [
            line.split("=", 1)[1].strip()
            for line in text.splitlines()
            if line.strip().startswith(f"{key}=")
        ]

    def environment(self, text: str, name: str) -> list[str]:
        return [
            value.split("=", 1)[1]
            for value in self.settings(text, "Environment")
            if value.startswith(f"{name}=")
        ]

    def test_it_follows_the_data_directory_by_default(self, monkeypatch, tmp_path):
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        assert runner.EVENT_JOURNAL == runner.DATA_DIR / runner.EVENT_JOURNAL_NAME

    def test_an_explicit_path_overrides_it(self, monkeypatch, tmp_path):
        elsewhere = tmp_path.resolve() / "elsewhere" / "events.jsonl"
        monkeypatch.setenv("FLYTO_DEVICE_EVENT_JOURNAL", str(elsewhere))
        spec = importlib.util.spec_from_file_location("flyto_job_runner_alt", RUNNER_PATH)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setenv("FLYTO_RUNNER_DATA_DIR", str(tmp_path.resolve() / "data"))
        spec.loader.exec_module(module)
        assert elsewhere == module.EVENT_JOURNAL

    def test_the_enterprise_data_directory_moves_the_journal_with_it(self, monkeypatch, tmp_path):
        """The drop-in relocates FLYTO_RUNNER_DATA_DIR to /var/lib/flyto-runner.
        A hard-coded absolute default would not follow, and an offline site
        would keep writing into a home directory it may not have."""
        monkeypatch.setenv("FLYTO_RUNNER_DATA_DIR", "/var/lib/flyto-runner")
        monkeypatch.delenv("FLYTO_DEVICE_EVENT_JOURNAL", raising=False)
        spec = importlib.util.spec_from_file_location("flyto_job_runner_ent", RUNNER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assert Path("/var/lib/flyto-runner/device-events.jsonl") == module.EVENT_JOURNAL

    def test_the_runner_unit_pins_no_lab_specific_device_identity(self):
        """A literal id in a shipped unit is one lab's TurtleBot name copied
        onto every robot that installs the file."""
        text = self.unit("flyto-job-runner.service")
        assert self.environment(text, "FLYTO_ROBOT_RESOURCE_ID") == []
        assert "flyto-tb3-lab-001" not in "".join(
            self.environment(text, "FLYTO_DEVICE_EVENT_JOURNAL")
        )

    def test_the_doctor_unit_names_its_own_journal(self):
        text = self.unit("flyto-robot-doctor.service")
        assert self.environment(text, "FLYTO_DEVICE_EVENT_JOURNAL") == [
            "/var/lib/flyto-robot/events/device-events.jsonl"
        ]
        assert "--event-journal" in text

    def test_the_two_services_never_share_one_journal(self):
        """Root writes one and the service user writes the other, so a shared
        file would have to be readable by both — and a device journal group or
        others can read is already disclosed."""
        doctor = self.environment(
            self.unit("flyto-robot-doctor.service"), "FLYTO_DEVICE_EVENT_JOURNAL"
        )
        runner_text = self.unit("flyto-job-runner.service")
        runner_data = self.environment(runner_text, "FLYTO_RUNNER_DATA_DIR")
        assert runner_data == ["/home/ubuntu/.flyto"]
        implied = f"{runner_data[0]}/device-events.jsonl"
        assert implied not in doctor
        assert doctor[0] != implied
        # And the doctor's is not inside the diagnostics directory the recovery
        # portal serves latest.json from: the journal tightens its directory to
        # 0700, which would take that read away.
        assert "/diagnostics/" not in doctor[0]


# --------------------------------------------------------------------------
# flyto-job-runner.service restart-policy contract
#
# These assert the *parsed* unit, not its text. A directive only takes effect in
# the section systemd reads it from: StartLimit* is read from [Unit] only, and
# in [Service] it is accepted and then silently ignored. So a substring check
# for "StartLimitBurst=20" passes just as happily on a unit where the limiter
# does nothing — which is exactly the bug this file previously shipped. Every
# assertion below therefore goes through _parse_unit and names section + key +
# value.
#
# configparser is deliberately not used: systemd allows a key to repeat within a
# section with cumulative meaning (several Environment= lines here), and
# configparser collapses repeats to the last one.
# --------------------------------------------------------------------------


def _parse_unit(text: str) -> dict[str, list[tuple[str, str]]]:
    """Parse systemd unit text into {section: [(key, value), ...]}.

    Repeated keys are preserved in file order. Comments are whole-line only,
    matching systemd: there is no trailing-comment syntax, and the Exec* lines
    carry shell that must survive intact. Only the first '=' splits a line, so
    an Environment=KEY=VALUE pair keeps its own '='. A trailing backslash
    continues a directive onto the next line.
    """
    sections: dict[str, list[tuple[str, str]]] = {}
    current: str | None = None
    pending = ""
    for raw in text.splitlines():
        line = raw.strip()
        if not pending and (not line or line.startswith(("#", ";"))):
            continue
        if not pending and line.startswith("[") and line.endswith("]"):
            current = line[1:-1]
            sections.setdefault(current, [])
            continue
        line = pending + line
        pending = ""
        if line.endswith("\\"):
            pending = line[:-1]
            continue
        if current is None or "=" not in line:
            continue
        key, _, value = line.partition("=")
        sections[current].append((key.strip(), value.strip()))
    return sections


def _values(unit: dict[str, list[tuple[str, str]]], section: str, key: str) -> list[str]:
    return [value for name, value in unit.get(section, []) if name == key]


def _only(unit: dict[str, list[tuple[str, str]]], section: str, key: str) -> str:
    values = _values(unit, section, key)
    assert len(values) == 1, f"expected exactly one {section}/{key}, got {values}"
    return values[0]


@pytest.fixture(scope="module")
def runner_unit() -> dict[str, list[tuple[str, str]]]:
    return _parse_unit((RUNNER_PATH.parent / "systemd" / "flyto-job-runner.service").read_text())


def test_the_unit_parser_keeps_sections_repeats_and_values_with_equals_signs():
    # Guards everything below: a parser that dropped repeats, mis-split on '='
    # or lost a section would make the contract tests pass vacuously.
    parsed = _parse_unit(
        "# comment\n"
        "[Unit]\n"
        "StartLimitBurst=20\n"
        "\n"
        "[Service]\n"
        "; also a comment\n"
        "Environment=A=1\n"
        "Environment=B=2\n"
        "ExecStart=/bin/bash -lc 'x \\\n"
        "y'\n"
    )
    assert parsed["Unit"] == [("StartLimitBurst", "20")]
    assert _values(parsed, "Service", "Environment") == ["A=1", "B=2"]
    assert _values(parsed, "Service", "ExecStart") == ["/bin/bash -lc 'x y'"]
    assert "StartLimitBurst" not in dict(parsed["Service"])
    # A commented-out directive is not a setting.
    commented = _parse_unit("[Service]\n# StartLimitBurst=20\n")
    assert _values(commented, "Service", "StartLimitBurst") == []


def test_the_start_rate_limit_is_in_the_unit_section_with_its_exact_values(runner_unit):
    # systemd reads both keys from [Unit] and nowhere else, so this placement is
    # the whole difference between a bounded restart loop and no limiter at all.
    assert _only(runner_unit, "Unit", "StartLimitIntervalSec") == "300"
    assert _only(runner_unit, "Unit", "StartLimitBurst") == "20"


def test_the_start_rate_limit_is_absent_from_the_service_section(runner_unit):
    # In [Service] both keys are accepted and ignored. That is how this unit
    # shipped with Restart=always and, in practice, no limit on it.
    service_keys = {key for key, _ in runner_unit["Service"]}
    assert "StartLimitIntervalSec" not in service_keys
    assert "StartLimitBurst" not in service_keys
    assert _values(runner_unit, "Service", "StartLimitIntervalSec") == []
    assert _values(runner_unit, "Service", "StartLimitBurst") == []


def test_the_restart_behaviour_itself_stays_in_the_service_section(runner_unit):
    # Restart=/RestartSec= are [Service] directives; moving the limiter must not
    # drag them along. always, not on-failure: the runner exits cleanly on a
    # rejected credential and on SIGTERM, and only the first should come back.
    assert _only(runner_unit, "Service", "Restart") == "always"
    assert _only(runner_unit, "Service", "RestartSec") == "10"
    unit_keys = {key for key, _ in runner_unit["Unit"]}
    assert "Restart" not in unit_keys
    assert "RestartSec" not in unit_keys


def test_the_shared_event_code_loads_from_an_absolute_script_path(monkeypatch, tmp_path):
    """The unit runs this file by absolute path with no PYTHONPATH, from a
    working directory that is not deploy/. Importing flyto_robotics by name
    would execute the package __init__, which pulls in the AI planner, the
    capability registry and the ROS adapters — the whole thing this avoids."""
    import subprocess

    script = (
        "import importlib.util, sys, json\n"
        f"spec = importlib.util.spec_from_file_location('r', {str(RUNNER_PATH)!r})\n"
        "module = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(module)\n"
        "events = module.device_events()\n"
        "print(json.dumps({\n"
        "    'contract': events.DEVICE_EVENT_CONTRACT,\n"
        "    'ros': [n for n in sys.modules if n.split('.')[0] in "
        "('rclpy', 'flyto_robotics', 'yaml', 'pydantic', 'requests')],\n"
        "}))\n"
    )
    completed = subprocess.run(
        [sys.executable, "-c", script],
        check=True,
        capture_output=True,
        text=True,
        # Neither the repository root nor deploy/, and no PYTHONPATH at all.
        cwd=str(tmp_path.resolve()),
        env={
            "PATH": os.environ.get("PATH", ""),
            "FLYTO_RUNNER_DATA_DIR": str(tmp_path.resolve() / "data"),
        },
    )
    result = json.loads(completed.stdout)
    assert result["contract"] == "flyto.device-event.v1"
    assert result["ros"] == [], f"the shared import dragged in {result['ros']}"


class TestTheCredentialOnDisk:
    """A device secret an unattended robot must read at boot.

    What is achievable here is bounded and worth stating: the robot has no TPM
    and pairs itself, so any key it can use without an operator is a key the SD
    card also holds. Encrypting against a key stored beside it would look
    stronger and be worth nothing. These tests cover what is real — that no
    other account on the machine can read it, that it is never briefly exposed,
    and that losing power does not cost the pairing.
    """

    CREDENTIAL = {"device_id": "dev-1", "device_secret": "s-1"}

    def runner(self, monkeypatch, tmp_path):
        return load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )

    def test_the_file_is_owner_only(self, monkeypatch, tmp_path):
        runner = self.runner(monkeypatch, tmp_path)
        runner._write_credentials(self.CREDENTIAL)
        assert runner.CREDENTIAL_FILE.stat().st_mode & 0o777 == 0o600

    def test_the_directory_is_owner_only(self, monkeypatch, tmp_path):
        runner = self.runner(monkeypatch, tmp_path)
        runner._write_credentials(self.CREDENTIAL)
        assert runner.DATA_DIR.stat().st_mode & 0o777 == 0o700

    def test_an_existing_loose_directory_is_tightened(self, monkeypatch, tmp_path):
        """A data dir made by hand, or by an older version, comes back 0755."""
        runner = self.runner(monkeypatch, tmp_path)
        runner.DATA_DIR.mkdir(parents=True)
        runner.DATA_DIR.chmod(0o755)
        runner._write_credentials(self.CREDENTIAL)
        assert runner.DATA_DIR.stat().st_mode & 0o777 == 0o700

    def test_it_is_never_readable_even_for_an_instant(self, monkeypatch, tmp_path):
        """The window the previous code left open.

        write_text() creates at 0666 & ~umask, then narrows. Checking the mode
        afterwards cannot see that, because afterwards is exactly when it is
        correct. So look at the file the moment before it is renamed into
        place, which is the whole of its exposed life.
        """
        runner = self.runner(monkeypatch, tmp_path)
        observed = []
        real_replace = runner.os.replace

        def watch(source, destination):
            observed.append(runner.os.stat(source).st_mode & 0o777)
            return real_replace(source, destination)

        monkeypatch.setattr(runner.os, "replace", watch)
        # Permissive umask, so any mode the code does not set explicitly shows.
        previous = runner.os.umask(0o000)
        try:
            runner._write_credentials(self.CREDENTIAL)
        finally:
            runner.os.umask(previous)

        assert observed == [0o600], "the secret existed at another mode first"

    def test_a_crash_mid_write_keeps_the_previous_credential(self, monkeypatch, tmp_path):
        """Losing the pairing to a power cut means a trip to the robot."""
        runner = self.runner(monkeypatch, tmp_path)
        runner._write_credentials(self.CREDENTIAL)

        def explode(*args, **kwargs):
            raise OSError("power cut")

        monkeypatch.setattr(runner.os, "replace", explode)
        with pytest.raises(OSError):
            runner._write_credentials({"device_id": "dev-2", "device_secret": "s-2"})

        assert runner._read_credentials() == self.CREDENTIAL

    def test_a_crash_leaves_no_secret_behind_in_a_partial_file(self, monkeypatch, tmp_path):
        runner = self.runner(monkeypatch, tmp_path)
        monkeypatch.setattr(runner.os, "replace", lambda *a, **k: (_ for _ in ()).throw(OSError()))
        with pytest.raises(OSError):
            runner._write_credentials(self.CREDENTIAL)
        leftovers = [p.name for p in runner.DATA_DIR.iterdir()]
        assert leftovers == [], f"a secret was left in {leftovers}"

    def test_a_stale_partial_does_not_block_the_next_write(self, monkeypatch, tmp_path):
        runner = self.runner(monkeypatch, tmp_path)
        runner.DATA_DIR.mkdir(parents=True)
        (runner.DATA_DIR / f"{runner.CREDENTIAL_FILE.name}.partial").write_text("{}")
        runner._write_credentials(self.CREDENTIAL)
        assert runner._read_credentials() == self.CREDENTIAL


class TestRefusingACredentialThatLeaked:
    CREDENTIAL = {"device_id": "dev-1", "device_secret": "s-1"}

    def stored(self, monkeypatch, tmp_path, mode):
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        runner._write_credentials(self.CREDENTIAL)
        runner.CREDENTIAL_FILE.chmod(mode)
        return runner

    @pytest.mark.parametrize("mode", [0o640, 0o604, 0o644, 0o666, 0o660])
    def test_a_readable_secret_is_treated_as_disclosed(self, monkeypatch, tmp_path, mode):
        """It would keep working perfectly, which is why nothing said so."""
        runner = self.stored(monkeypatch, tmp_path, mode)
        with pytest.raises(runner.RunnerError, match="already disclosed"):
            runner._read_credentials()

    def test_the_error_says_how_to_recover_and_why_re_pairing_is_safer(
        self, monkeypatch, tmp_path
    ):
        runner = self.stored(monkeypatch, tmp_path, 0o644)
        with pytest.raises(runner.RunnerError) as caught:
            runner._read_credentials()
        message = str(caught.value)
        assert "chmod 600" in message
        assert "pair again" in message

    @pytest.mark.parametrize("mode", [0o600, 0o400])
    def test_an_owner_only_secret_is_used(self, monkeypatch, tmp_path, mode):
        runner = self.stored(monkeypatch, tmp_path, mode)
        assert runner._read_credentials() == self.CREDENTIAL


class TestACredentialFromSystemd:
    """Where the host can do better than a file, it is used instead.

    LoadCredentialEncrypted= decrypts into a private tmpfs that never reaches
    persistent storage, and on a host with a TPM the ciphertext at rest is
    sealed to the hardware.
    """

    CREDENTIAL = {"device_id": "dev-tpm", "device_secret": "s-tpm"}

    def provisioned(self, monkeypatch, tmp_path):
        directory = tmp_path / "creds"
        directory.mkdir()
        runner = load_runner(
            monkeypatch, tmp_path, cloud="http://127.0.0.1:1", gateway="http://127.0.0.1:1"
        )
        (directory / runner.SYSTEMD_CREDENTIAL_NAME).write_text(json.dumps(self.CREDENTIAL))
        monkeypatch.setenv("CREDENTIALS_DIRECTORY", str(directory))
        return runner

    def test_it_is_preferred_over_the_file(self, monkeypatch, tmp_path):
        runner = self.provisioned(monkeypatch, tmp_path)
        runner._write_credentials({"device_id": "old", "device_secret": "old"})
        assert runner._read_credentials() == self.CREDENTIAL

    def test_nothing_is_written_to_disk_for_it(self, monkeypatch, tmp_path):
        runner = self.provisioned(monkeypatch, tmp_path)
        assert runner._read_credentials() == self.CREDENTIAL
        assert not runner.CREDENTIAL_FILE.exists()

    def test_the_file_is_still_used_when_systemd_supplies_nothing(self, monkeypatch, tmp_path):
        runner = self.provisioned(monkeypatch, tmp_path)
        (Path(os.environ["CREDENTIALS_DIRECTORY"]) / runner.SYSTEMD_CREDENTIAL_NAME).unlink()
        runner._write_credentials({"device_id": "dev-file", "device_secret": "s-file"})
        assert runner._read_credentials()["device_id"] == "dev-file"

    def test_a_malformed_systemd_credential_is_rejected_not_ignored(self, monkeypatch, tmp_path):
        """Falling back to the file would hide a broken provisioning."""
        runner = self.provisioned(monkeypatch, tmp_path)
        path = Path(os.environ["CREDENTIALS_DIRECTORY"]) / runner.SYSTEMD_CREDENTIAL_NAME
        path.write_text(json.dumps({"device_id": "only-half"}))
        with pytest.raises(runner.RunnerError, match="malformed"):
            runner._read_credentials()
