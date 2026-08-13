from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import textwrap
import threading
import time
import types
from enum import Enum
from pathlib import Path

import pytest

from flyto_robotics import ros2_adapter as adapter


class GraphNode:
    def __init__(self, observations):
        self.observations = iter(observations)

    def get_topic_names_and_types(self):
        value = next(self.observations)
        if isinstance(value, Exception):
            raise value
        return value


def test_adapter_is_passive_and_never_requests_a_publisher(tmp_path: Path) -> None:
    node = GraphNode([[(name, [topic_type])
                      for name, topic_type in adapter.DEFAULT_REQUIRED_TOPICS]])
    document = adapter.run_adapter(
        node, state_dir=tmp_path, required_topics=adapter.DEFAULT_REQUIRED_TOPICS,
        max_cycles=1,
    )
    assert document["state"] == "ready"
    assert not hasattr(node, "create_publisher")
    assert set(document) == {
        "schema", "service", "state", "ready", "reason", "required_topics",
        "missing_topics", "mismatched_topics", "action_code",
    }


def test_missing_topics_are_repeated_stable_unready_state(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    node = GraphNode([[], [], RuntimeError("discovery unavailable")])
    writes = []
    real_write = adapter.write_status

    def counted_write(state_dir, document):
        writes.append(document)
        return real_write(state_dir, document)

    monkeypatch.setattr(adapter, "write_status", counted_write)

    document = adapter.run_adapter(
        node, state_dir=tmp_path, required_topics=adapter.DEFAULT_REQUIRED_TOPICS,
        max_cycles=3, wait=lambda _timeout: None,
    )
    assert len(writes) == 2
    first = writes[0]
    assert first["state"] == "unready"
    assert first["missing_topics"] == [item[0] for item in adapter.DEFAULT_REQUIRED_TOPICS]
    assert document["reason"] == "graph_observation_unavailable"
    assert list(tmp_path.iterdir()) == [tmp_path / adapter.STATUS_FILE]
    assert (tmp_path / adapter.STATUS_FILE).stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize(
    "value, reason",
    [
        (None, "required_topics_wrong_type"),
        ([('scan', 'x')], "required_topic_not_absolute"),
        ([('/scan', 'x'), ('/scan', 'y')], "required_topics_duplicate"),
        ([('/scan\n', 'x')], "required_topic_control_character"),
        ([("/" + "a" * 129, 'x')], "required_topic_oversized"),
        ([('/scan',)], "required_topic_malformed"),
        ([('/bad topic', 'x')], "required_topic_malformed"),
        ([(f'/t{i}', 'x') for i in range(17)], "required_topics_count_invalid"),
        ([('/scan', 7)], "required_topic_malformed"),
    ],
)
def test_required_topic_configuration_is_strictly_bounded(value, reason: str) -> None:
    with pytest.raises(adapter.AdapterConfigurationError, match=reason):
        adapter.parse_required_topics(value)


def test_clean_stop_is_bounded_and_preserves_last_status(tmp_path: Path) -> None:
    stopped = threading.Event()
    node = GraphNode([[(name, [topic_type])
                      for name, topic_type in adapter.DEFAULT_REQUIRED_TOPICS]])

    def stop(_timeout: float) -> None:
        stopped.set()

    document = adapter.run_adapter(
        node, state_dir=tmp_path, required_topics=adapter.DEFAULT_REQUIRED_TOPICS,
        stop_event=stopped, wait=stop,
    )
    assert document["ready"] is True
    assert json.loads((tmp_path / adapter.STATUS_FILE).read_text()) == document


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_exact_cli_is_long_lived_and_stops_cleanly_on_real_signals(
    tmp_path: Path, signum: signal.Signals
) -> None:
    fake = tmp_path / "fake-runtime"
    fake.mkdir()
    package = fake / "rclpy"
    package.mkdir()
    (package / "signals.py").write_text(textwrap.dedent("""
        from enum import Enum
        class SignalHandlerOptions(Enum):
            NO = 0
    """), encoding="utf-8")
    (package / "__init__.py").write_text(textwrap.dedent("""
        import signal
        import time
        from .signals import SignalHandlerOptions
        class Node:
            def get_topic_names_and_types(self):
                return [
                    ('/battery_state', ['sensor_msgs/msg/BatteryState']),
                    ('/scan', ['sensor_msgs/msg/LaserScan']),
                    ('/odom', ['nav_msgs/msg/Odometry']),
                ]
            def destroy_node(self): pass
        def init(*, args, signal_handler_options):
            assert args is None
            assert signal_handler_options is SignalHandlerOptions.NO
        def create_node(name):
            assert name == 'flyto_ros2_readiness_adapter'
            return Node()
        def spin_once(node, *, timeout_sec): time.sleep(timeout_sec)
        def shutdown(*, uninstall_handlers):
            assert uninstall_handlers is False
    """), encoding="utf-8")
    state_dir = tmp_path / "state"
    environment = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join([str(fake), str(Path(__file__).parent.parent)]),
        "FLYTO_ROBOT_MAX_CYCLES": "1",
    }
    child = subprocess.Popen(
        [sys.executable, "-m", "flyto_robotics.ros2_adapter", "--state-dir", str(state_dir)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=environment,
    )
    status_file = state_dir / adapter.STATUS_FILE
    deadline = time.monotonic() + 5
    while not status_file.exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert status_file.exists()
    initial = status_file.read_bytes()
    time.sleep(0.1)
    assert child.poll() is None, "production environment must not bound adapter lifetime"
    child.send_signal(signum)
    try:
        stdout, stderr = child.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        child.kill()
        child.communicate()
        raise
    assert child.returncode == 0
    assert "Traceback" not in stdout + stderr
    assert "runtime_unavailable" not in stdout + stderr
    assert status_file.read_bytes() == initial
    assert json.loads(initial)["ready"] is True


def test_names_without_the_fixed_types_are_unready() -> None:
    observed = [(name, ["unexpected/msg/Type", "another/msg/Type"])
                for name, _topic_type in adapter.DEFAULT_REQUIRED_TOPICS]
    document = adapter.readiness_document(adapter.DEFAULT_REQUIRED_TOPICS, observed)
    assert document["ready"] is False
    assert document["reason"] == "required_topic_types_mismatched"
    assert document["mismatched_topics"] == [item[0] for item in adapter.DEFAULT_REQUIRED_TOPICS]
    assert "unexpected/msg/Type" not in json.dumps(document)


@pytest.mark.parametrize("observation", [None, {}, [("/scan", "not-a-type-list")], [(7, [])]])
def test_malformed_graph_is_structured_unready(observation) -> None:
    document = adapter.readiness_document(adapter.DEFAULT_REQUIRED_TOPICS, observation)
    assert document["state"] == "unready"
    assert document["reason"] == "graph_observation_unavailable"


@pytest.mark.parametrize(
    "observation",
    [
        [(f"/topic{i}", []) for i in range(adapter.MAX_GRAPH_ENTRIES + 1)],
        [("/scan", ["pkg/msg/T"] * (adapter.MAX_TYPES_PER_ENTRY + 1))],
        [("/" + "x" * adapter.MAX_TOPIC_LENGTH, [])],
        [("/scan", ["not a type"])],
        [("/scan", ["pkg/msg/" + "X" * adapter.MAX_TOPIC_LENGTH])],
    ],
)
def test_graph_projection_rejects_excess_or_malformed_fields(observation) -> None:
    document = adapter.readiness_document(adapter.DEFAULT_REQUIRED_TOPICS, observation)
    assert document["reason"] == "graph_observation_unavailable"
    assert document["ready"] is False
    assert "topic255" not in json.dumps(document)


def test_invalid_config_atomically_replaces_stale_ready(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    status_file = tmp_path / adapter.STATUS_FILE
    status_file.write_text('{"ready":true}', encoding="utf-8")
    monkeypatch.setenv(adapter.REQUIRED_TOPICS_ENV, "/scan")
    with pytest.raises(SystemExit) as stopped:
        adapter.main(["--state-dir", str(tmp_path)])
    document = json.loads(status_file.read_text())
    assert stopped.value.code == 2
    assert document["state"] == "config_invalid"
    assert document["ready"] is False
    assert document["action_code"] == "check_adapter_config"
    assert set(document) == {
        "schema", "service", "state", "ready", "reason", "action_code",
        "required_topics", "missing_topics", "mismatched_topics",
    }


def test_runtime_failure_replaces_stale_ready_without_exception_text(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    class BrokenRuntime:
        def init(self, *, args, signal_handler_options):
            raise RuntimeError("sensitive runtime detail")

    status_file = tmp_path / adapter.STATUS_FILE
    status_file.write_text('{"ready":true}', encoding="utf-8")
    monkeypatch.setitem(sys.modules, "rclpy", BrokenRuntime())
    with pytest.raises(SystemExit) as stopped:
        adapter.main(["--state-dir", str(tmp_path)])
    output = capsys.readouterr()
    document = json.loads(status_file.read_text())
    assert stopped.value.code == 1
    assert document["state"] == "runtime_unavailable"
    assert document["reason"] == "ros2_runtime_unavailable"
    assert "sensitive runtime detail" not in output.out + output.err


def test_main_restores_the_true_pre_init_handlers(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    class SignalHandlerOptions(Enum):
        NO = 0

    signals_module = types.ModuleType("rclpy.signals")
    signals_module.SignalHandlerOptions = SignalHandlerOptions
    runtime = types.ModuleType("rclpy")

    class Node:
        def get_topic_names_and_types(self):
            return []

        def destroy_node(self):
            pass

    def init(*, args, signal_handler_options):
        assert args is None
        assert signal_handler_options is SignalHandlerOptions.NO

    def spin_once(_node, *, timeout_sec):
        assert timeout_sec == 1.0
        handler = signal.getsignal(signal.SIGTERM)
        assert callable(handler)
        handler(signal.SIGTERM, None)

    def shutdown(*, uninstall_handlers):
        assert uninstall_handlers is False

    runtime.init = init
    runtime.create_node = lambda _name: Node()
    runtime.spin_once = spin_once
    runtime.shutdown = shutdown
    monkeypatch.setitem(sys.modules, "rclpy", runtime)
    monkeypatch.setitem(sys.modules, "rclpy.signals", signals_module)

    def original_int(_signum, _frame):
        pass

    def original_term(_signum, _frame):
        pass

    prior_int = signal.signal(signal.SIGINT, original_int)
    prior_term = signal.signal(signal.SIGTERM, original_term)
    try:
        adapter.main(["--state-dir", str(tmp_path)])
        assert signal.getsignal(signal.SIGINT) is original_int
        assert signal.getsignal(signal.SIGTERM) is original_term
    finally:
        signal.signal(signal.SIGINT, prior_int)
        signal.signal(signal.SIGTERM, prior_term)
