"""Passive ROS 2 graph readiness adapter for the lifecycle service.

This process only observes topic names and declared types.  It creates no publishers,
subscriptions, missions, actions, or device commands, and never reads message
samples.  Missing topics are an ordinary, stable readiness state.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import threading
from collections.abc import Callable, Sequence
from contextlib import suppress
from pathlib import Path
from typing import Any

from .fsio import atomic_write

STATUS_SCHEMA = "flyto.robotics.ros2-adapter-status.v1"
STATUS_FILE = "ros2-adapter-status.json"
REQUIRED_TOPICS_ENV = "FLYTO_ROS2_REQUIRED_TOPICS"
DEFAULT_REQUIRED_TOPICS = (
    ("/battery_state", "sensor_msgs/msg/BatteryState"),
    ("/scan", "sensor_msgs/msg/LaserScan"),
    ("/odom", "nav_msgs/msg/Odometry"),
)
MAX_REQUIRED_TOPICS = 16
MAX_TOPIC_LENGTH = 128
MAX_CONFIGURATION_LENGTH = 1024
MAX_TEST_CYCLES = 1000
MAX_GRAPH_ENTRIES = 256
MAX_TYPES_PER_ENTRY = 16
_TOPIC = re.compile(r"/(?:[A-Za-z0-9_]+)(?:/[A-Za-z0-9_]+)*\Z")
_TYPE = re.compile(r"[A-Za-z][A-Za-z0-9_]*/(?:msg|srv|action)/[A-Za-z][A-Za-z0-9_]*\Z")


class AdapterConfigurationError(ValueError):
    """The local adapter configuration is malformed or outside its bounds."""


def parse_required_topics(value: object) -> tuple[tuple[str, str], ...]:
    """Validate the fixed, closed name/type readiness configuration."""

    if not isinstance(value, (tuple, list)):
        raise AdapterConfigurationError("required_topics_wrong_type")
    if not value or len(repr(value).encode("utf-8")) > MAX_CONFIGURATION_LENGTH:
        raise AdapterConfigurationError("required_topics_size_invalid")
    topics = list(value)
    if not 1 <= len(topics) <= MAX_REQUIRED_TOPICS:
        raise AdapterConfigurationError("required_topics_count_invalid")
    if any(
        ord(character) < 0x20 or ord(character) == 0x7F
        for item in topics
        if isinstance(item, (tuple, list))
        for field in item
        if isinstance(field, str)
        for character in field
    ):
        raise AdapterConfigurationError("required_topic_control_character")
    if any(
        not isinstance(item, (tuple, list))
        or len(item) != 2
        or not all(isinstance(field, str) for field in item)
        for item in topics
    ):
        raise AdapterConfigurationError("required_topic_malformed")
    pairs = [(item[0], item[1]) for item in topics]
    if any(not name or name != name.strip() or not topic_type for name, topic_type in pairs):
        raise AdapterConfigurationError("required_topic_malformed")
    if len({name for name, _topic_type in pairs}) != len(pairs):
        raise AdapterConfigurationError("required_topics_duplicate")
    for name, topic_type in pairs:
        if len(name) > MAX_TOPIC_LENGTH or len(topic_type) > MAX_TOPIC_LENGTH:
            raise AdapterConfigurationError("required_topic_oversized")
        if not name.startswith("/"):
            raise AdapterConfigurationError("required_topic_not_absolute")
        if _TOPIC.fullmatch(name) is None:
            raise AdapterConfigurationError("required_topic_malformed")
    if tuple(pairs) != DEFAULT_REQUIRED_TOPICS:
        raise AdapterConfigurationError("required_topics_not_fixed")
    return tuple(pairs)


def required_topics_from_environ(
    environ: dict[str, str] | os._Environ[str],
) -> tuple[tuple[str, str], ...]:
    if REQUIRED_TOPICS_ENV in environ:
        raise AdapterConfigurationError("required_topics_external_override")
    return parse_required_topics(DEFAULT_REQUIRED_TOPICS)


def _status(
    *, state: str, ready: bool, reason: str, action_code: str,
    missing: Sequence[str] = (), mismatched: Sequence[str] = (),
) -> dict[str, Any]:
    return {
        "schema": STATUS_SCHEMA,
        "service": "ros2_readiness_adapter",
        "state": state,
        "ready": ready,
        "reason": reason,
        "action_code": action_code,
        "required_topics": [
            {"name": name, "type": topic_type}
            for name, topic_type in DEFAULT_REQUIRED_TOPICS
        ],
        "missing_topics": list(missing),
        "mismatched_topics": list(mismatched),
    }


def readiness_document(
    required_topics: Sequence[tuple[str, str]], observed_topics: object
) -> dict[str, Any]:
    """Project a graph observation into one fixed privacy-safe document."""

    required = parse_required_topics(required_topics)
    try:
        if (
            not isinstance(observed_topics, (tuple, list))
            or len(observed_topics) > MAX_GRAPH_ENTRIES
        ):
            raise ValueError
        validated: list[tuple[str, tuple[str, ...]]] = []
        for item in observed_topics:
            if (
                not isinstance(item, (tuple, list))
                or len(item) != 2
                or not isinstance(item[0], str)
                or not isinstance(item[1], (tuple, list))
                or len(item[1]) > MAX_TYPES_PER_ENTRY
                or not all(isinstance(topic_type, str) for topic_type in item[1])
            ):
                raise ValueError
            name = item[0]
            types = tuple(item[1])
            if (
                len(name) > MAX_TOPIC_LENGTH
                or _TOPIC.fullmatch(name) is None
                or any(
                    len(topic_type) > MAX_TOPIC_LENGTH
                    or _TYPE.fullmatch(topic_type) is None
                    for topic_type in types
                )
            ):
                raise ValueError
            validated.append((name, types))
        observed = {name: set(types) for name, types in validated}
        missing = [name for name, _topic_type in required if name not in observed]
        mismatched = [
            name for name, topic_type in required
            if name in observed and topic_type not in observed[name]
        ]
        reason = (
            "required_topics_missing" if missing
            else "required_topic_types_mismatched" if mismatched
            else "required_topics_observed"
        )
    except (TypeError, ValueError):
        missing = [name for name, _topic_type in required]
        mismatched = []
        reason = "graph_observation_unavailable"
    ready = not missing and not mismatched
    return _status(
        state="ready" if ready else "unready", ready=ready, reason=reason,
        action_code="none" if ready else "inspect_ros2_graph",
        missing=missing, mismatched=mismatched,
    )


def write_status(state_dir: Path, document: dict[str, Any]) -> Path:
    text = json.dumps(document, separators=(",", ":"), sort_keys=True) + "\n"
    if len(text.encode("utf-8")) > 4096:  # defensive invariant over fixed bounded fields
        raise AdapterConfigurationError("status_oversized")
    return atomic_write(Path(state_dir) / STATUS_FILE, text, mode=0o600)


def run_adapter(
    node: Any,
    *,
    state_dir: Path,
    required_topics: Sequence[tuple[str, str]],
    stop_event: threading.Event | None = None,
    max_cycles: int | None = None,
    wait: Callable[[float], None] | None = None,
) -> dict[str, Any]:
    """Observe until stopped; ``max_cycles`` exists solely for bounded tests."""

    required = parse_required_topics(required_topics)
    stopped = stop_event or threading.Event()
    pause = wait or stopped.wait
    last: dict[str, Any] = {}
    previous_status: dict[str, Any] | None = None
    cycles = 0
    while not stopped.is_set():
        try:
            observation = node.get_topic_names_and_types()
        except Exception:  # ROS graph discovery failures are readiness, not process failure
            observation = None
        last = readiness_document(required, observation)
        if last != previous_status:
            write_status(state_dir, last)
            print(json.dumps(last, separators=(",", ":"), sort_keys=True), flush=True)
            previous_status = last
        cycles += 1
        if max_cycles is not None and cycles >= max_cycles:
            break
        pause(1.0)
    return last


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="flyto-ros2-adapter")
    parser.add_argument("--state-dir", type=Path, required=True)
    return parser


def _publish_terminal_status(state_dir: Path, *, state: str, reason: str) -> None:
    document = _status(
        state=state, ready=False, reason=reason,
        action_code="check_adapter_config" if state == "config_invalid" else "restart_adapter",
        missing=[name for name, _topic_type in DEFAULT_REQUIRED_TOPICS],
    )
    try:
        Path(state_dir).mkdir(parents=True, exist_ok=True)
        write_status(state_dir, document)
    except OSError:
        pass
    print(json.dumps(document, separators=(",", ":"), sort_keys=True), flush=True)


def main(argv: list[str] | None = None) -> None:  # pragma: no cover - real ROS path
    args = _parser().parse_args(argv)
    try:
        required = required_topics_from_environ(os.environ)
    except AdapterConfigurationError as error:
        _publish_terminal_status(args.state_dir, state="config_invalid", reason=str(error))
        raise SystemExit(2) from None

    stopped = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopped.set()

    previous: dict[signal.Signals, Any] = {}
    rclpy = None
    node = None
    try:
        import rclpy as runtime
        from rclpy.signals import SignalHandlerOptions

        rclpy = runtime
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        previous = {
            signum: signal.signal(signum, request_stop)
            for signum in (signal.SIGINT, signal.SIGTERM)
        }
        node = rclpy.create_node("flyto_ros2_readiness_adapter")
        run_adapter(node, state_dir=args.state_dir, required_topics=required,
                    stop_event=stopped,
                    wait=lambda timeout: rclpy.spin_once(node, timeout_sec=timeout))
    except Exception:
        _publish_terminal_status(
            args.state_dir, state="runtime_unavailable", reason="ros2_runtime_unavailable"
        )
        raise SystemExit(1) from None
    finally:
        if node is not None:
            with suppress(Exception):
                node.destroy_node()
        if rclpy is not None:
            with suppress(Exception):
                rclpy.shutdown(uninstall_handlers=False)
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":  # pragma: no cover
    main()
