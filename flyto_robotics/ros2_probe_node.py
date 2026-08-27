"""Live ROS graph probe for content-addressed Flyto2 readiness evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Protocol

from .contracts import write_json_atomic
from .ros2_pairing import (
    Ros2PairingError,
    build_ros2_runtime_snapshot,
    load_ros2_adapter_manifest,
    parse_ros2_adapter_manifest,
    verify_ros2_pairing,
)

_LIFECYCLE_UNKNOWN_ATTEMPTS_BEFORE_REFRESH = 5


class Ros2GraphProbe(Protocol):
    """Minimal graph operations required by the redacted evidence builder."""

    def interface_available(
        self,
        *,
        kind: str,
        name: str,
        interface_type: str,
        timeout_seconds: float,
    ) -> bool: ...

    def lifecycle_state(
        self,
        managed_nodes: Sequence[str],
        *,
        timeout_seconds: float,
    ) -> str: ...

    def external_emergency_stop_ready(
        self,
        *,
        owner_node: str,
        service_name: str,
        timeout_seconds: float,
    ) -> bool: ...


def _query_lifecycle_states_concurrently(
    clients: Sequence[Any],
    *,
    request_factory: Callable[[], Any],
    spin_once: Callable[[float], None],
    timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> list[str]:
    """Observe all lifecycle services concurrently inside one absolute budget."""

    if (
        isinstance(timeout_seconds, bool)
        or not isinstance(timeout_seconds, (int, float))
        or not math.isfinite(float(timeout_seconds))
        or not 0.01 <= float(timeout_seconds) <= 60.0
    ):
        raise ValueError("lifecycle timeout is outside its safe range")
    if not clients:
        return []
    labels = ["unknown"] * len(clients)
    futures: list[Any | None] = [None] * len(clients)
    started_at = clock()
    if (
        isinstance(started_at, bool)
        or not isinstance(started_at, (int, float))
        or not math.isfinite(float(started_at))
    ):
        return labels
    deadline = float(started_at) + float(timeout_seconds)
    while True:
        for index, client in enumerate(clients):
            if labels[index] != "unknown":
                continue
            future = futures[index]
            if future is None:
                try:
                    if client.service_is_ready():
                        futures[index] = client.call_async(request_factory())
                except Exception:
                    # A transient client failure remains unknown and is retried.
                    futures[index] = None
                continue
            if not future.done():
                continue
            try:
                response = future.result()
                if response is not None:
                    labels[index] = str(response.current_state.label).lower()
            except Exception:
                # Retry a failed response without extending the shared deadline.
                pass
            finally:
                futures[index] = None
        if all(label != "unknown" for label in labels):
            break
        observed_at = clock()
        if (
            isinstance(observed_at, bool)
            or not isinstance(observed_at, (int, float))
            or not math.isfinite(float(observed_at))
            or float(observed_at) < float(started_at)
        ):
            break
        remaining = deadline - float(observed_at)
        if remaining <= 0:
            break
        spin_once(min(0.1, remaining))
    for future in futures:
        if future is not None and not future.done():
            future.cancel()
    return labels

def collect_ros2_runtime_snapshot(
    manifest: Mapping[str, Any],
    probe: Ros2GraphProbe,
    *,
    deployment_mode: str,
    emergency_stop_node: str,
    emergency_stop_service: str,
    timeout_seconds: float = 5.0,
    max_age_seconds: int = 30,
    observed_at: datetime | None = None,
) -> dict[str, Any]:
    """Probe live interfaces and lifecycle state, returning only redacted facts."""

    validated = parse_ros2_adapter_manifest(manifest)
    if not 0.1 <= timeout_seconds <= 30.0:
        raise Ros2PairingError("timeout_seconds must be between 0.1 and 30")
    adapter_states: list[dict[str, Any]] = []
    for adapter in validated["adapters"]:
        interface = adapter["interface"]
        available = probe.interface_available(
            kind=interface["kind"],
            name=interface["name"],
            interface_type=interface["type"],
            timeout_seconds=timeout_seconds,
        )
        lifecycle = probe.lifecycle_state(
            adapter["managed_nodes"],
            timeout_seconds=timeout_seconds,
        )
        status = "ready" if available and lifecycle == "active" else "unavailable"
        evidence = {
            "adapter_id": adapter["adapter_id"],
            "interface_available": available,
            "lifecycle_state": lifecycle,
            "managed_node_count": len(adapter["managed_nodes"]),
        }
        adapter_states.append(
            {
                "adapter_id": adapter["adapter_id"],
                "status": status,
                "interface_available": available,
                "lifecycle_state": lifecycle,
                "observation_sequence": "ros-graph:"
                + hashlib.sha256(
                    json.dumps(
                        evidence,
                        separators=(",", ":"),
                        sort_keys=True,
                    ).encode("utf-8")
                ).hexdigest(),
            }
        )
    emergency_ready = probe.external_emergency_stop_ready(
        owner_node=emergency_stop_node,
        service_name=emergency_stop_service,
        timeout_seconds=timeout_seconds,
    )
    return build_ros2_runtime_snapshot(
        validated,
        deployment_mode=deployment_mode,
        emergency_stop_ready=emergency_ready,
        adapter_states=adapter_states,
        observed_at=observed_at or datetime.now(timezone.utc),
        max_age_seconds=max_age_seconds,
    )


class RclpyGraphProbe:
    """Real graph implementation; ROS imports stay out of host-only checks."""

    def __init__(self, node: Any) -> None:
        self.node = node
        self._interface_clients: dict[tuple[str, str, str], Any] = {}
        self._action_unavailable_streaks: dict[tuple[str, str, str], int] = {}
        self._lifecycle_clients: dict[str, Any] = {}
        self._lifecycle_unknown_streaks: dict[str, int] = {}

    def _interface_client_for(
        self,
        *,
        kind: str,
        name: str,
        interface_type: str,
        factory: Callable[[], Any],
    ) -> Any:
        key = (kind, name, interface_type)
        client = self._interface_clients.get(key)
        if client is None:
            client = factory()
            self._interface_clients[key] = client
        return client

    def _lifecycle_clients_for(
        self,
        service_type: Any,
        managed_nodes: Sequence[str],
    ) -> list[Any]:
        clients = []
        for managed_node in managed_nodes:
            service_name = managed_node.rstrip("/") + "/get_state"
            client = self._lifecycle_clients.get(service_name)
            if client is None:
                client = self.node.create_client(service_type, service_name)
                self._lifecycle_clients[service_name] = client
            clients.append(client)
        return clients

    def _discard_interface_client(self, key: tuple[str, str, str]) -> None:
        """Destroy one stale discovery entity before the next bounded retry."""

        if key[0] == "action":
            self._action_unavailable_streaks.pop(key, None)
        client = self._interface_clients.pop(key, None)
        if client is None:
            return
        if key[0] == "action":
            client.destroy()
        else:
            self.node.destroy_client(client)

    def _discard_lifecycle_client(self, service_name: str) -> None:
        """Destroy one lifecycle client whose state remained unknown."""

        self._lifecycle_unknown_streaks.pop(service_name, None)
        client = self._lifecycle_clients.pop(service_name, None)
        if client is not None:
            self.node.destroy_client(client)

    def close(self) -> None:
        """Release persistent discovery clients after the bounded probe closes."""

        for key in tuple(self._interface_clients):
            self._discard_interface_client(key)
        for service_name in tuple(self._lifecycle_clients):
            self._discard_lifecycle_client(service_name)

    def interface_available(
        self,
        *,
        kind: str,
        name: str,
        interface_type: str,
        timeout_seconds: float,
    ) -> bool:
        key = (kind, name, interface_type)
        if kind == "action":
            from rclpy.action import ActionClient
            from rosidl_runtime_py.utilities import get_action

            client = self._interface_client_for(
                kind=kind,
                name=name,
                interface_type=interface_type,
                factory=lambda: ActionClient(
                    self.node,
                    get_action(interface_type),
                    name,
                ),
            )
            available = bool(client.wait_for_server(timeout_sec=timeout_seconds))
            if available:
                self._action_unavailable_streaks.pop(key, None)
            else:
                streak = self._action_unavailable_streaks.get(key, 0) + 1
                if streak >= 2:
                    self._discard_interface_client(key)
                else:
                    self._action_unavailable_streaks[key] = streak
            return available
        if kind == "service":
            from rosidl_runtime_py.utilities import get_service

            client = self._interface_client_for(
                kind=kind,
                name=name,
                interface_type=interface_type,
                factory=lambda: self.node.create_client(
                    get_service(interface_type),
                    name,
                ),
            )
            available = bool(client.wait_for_service(timeout_sec=timeout_seconds))
            if not available:
                self._discard_interface_client(key)
            return available
        return False

    def lifecycle_state(
        self,
        managed_nodes: Sequence[str],
        *,
        timeout_seconds: float,
    ) -> str:
        import rclpy
        from lifecycle_msgs.srv import GetState

        clients = self._lifecycle_clients_for(GetState, managed_nodes)
        labels = _query_lifecycle_states_concurrently(
            clients,
            request_factory=GetState.Request,
            spin_once=lambda timeout: rclpy.spin_once(
                self.node,
                timeout_sec=timeout,
            ),
            timeout_seconds=timeout_seconds,
        )
        service_names = [node.rstrip("/") + "/get_state" for node in managed_nodes]
        for service_name, label in zip(service_names, labels):
            if label == "unknown":
                streak = self._lifecycle_unknown_streaks.get(service_name, 0) + 1
                if streak >= _LIFECYCLE_UNKNOWN_ATTEMPTS_BEFORE_REFRESH:
                    self._discard_lifecycle_client(service_name)
                else:
                    self._lifecycle_unknown_streaks[service_name] = streak
            else:
                self._lifecycle_unknown_streaks.pop(service_name, None)
        if labels and all(label == "active" for label in labels):
            return "active"
        for state in ("error", "finalized", "unconfigured", "inactive"):
            if state in labels:
                return state
        return "unknown"

    def external_emergency_stop_ready(
        self,
        *,
        owner_node: str,
        service_name: str,
        timeout_seconds: float,
    ) -> bool:
        from rosidl_runtime_py.utilities import get_service

        interface_type = "std_srvs/srv/Trigger"
        key = ("service", service_name, interface_type)
        owner_name, owner_namespace = _split_node_path(owner_node)
        if (owner_namespace.rstrip("/") + "/" + owner_name).replace("//", "/") == (
            self.node.get_fully_qualified_name()
        ):
            self._discard_interface_client(key)
            return False
        try:
            services = dict(
                self.node.get_service_names_and_types_by_node(
                    owner_name,
                    owner_namespace,
                )
            )
        except Exception:
            self._discard_interface_client(key)
            return False
        if interface_type not in services.get(service_name, []):
            self._discard_interface_client(key)
            return False
        client = self._interface_client_for(
            kind="service",
            name=service_name,
            interface_type=interface_type,
            factory=lambda: self.node.create_client(
                get_service(interface_type),
                service_name,
            ),
        )
        ready = bool(client.wait_for_service(timeout_sec=timeout_seconds))
        if not ready:
            self._discard_interface_client(key)
        return ready


def _split_node_path(value: str) -> tuple[str, str]:
    text = str(value or "").strip()
    if not text.startswith("/") or text == "/" or "//" in text:
        raise Ros2PairingError("emergency_stop_node must be an absolute ROS node name")
    parts = text.rstrip("/").split("/")
    name = parts[-1]
    namespace = "/".join(parts[:-1]) or "/"
    return name, namespace


def run_probe(
    *,
    manifest_path: Path,
    output_path: Path,
    deployment_mode: str,
    emergency_stop_node: str,
    emergency_stop_service: str,
    timeout_seconds: float,
    max_age_seconds: int,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Run a one-shot real graph probe and atomically persist its evidence."""

    import rclpy
    from rclpy.node import Node

    manifest = load_ros2_adapter_manifest(manifest_path)
    rclpy.init(args=None)
    node = Node("flyto_ros2_readiness_probe")
    graph_probe = RclpyGraphProbe(node)
    try:
        runtime = collect_ros2_runtime_snapshot(
            manifest,
            graph_probe,
            deployment_mode=deployment_mode,
            emergency_stop_node=emergency_stop_node,
            emergency_stop_service=emergency_stop_service,
            timeout_seconds=timeout_seconds,
            max_age_seconds=max_age_seconds,
        )
        report = verify_ros2_pairing(manifest, runtime)
        write_json_atomic(output_path, runtime)
        return runtime, report
    finally:
        graph_probe.close()
        node.destroy_node()
        rclpy.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="flyto-ros2-readiness-probe")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--deployment-mode",
        required=True,
        choices=("simulation", "hardware"),
    )
    parser.add_argument("--emergency-stop-node", required=True)
    parser.add_argument("--emergency-stop-service", required=True)
    parser.add_argument("--timeout-seconds", type=float, default=5.0)
    parser.add_argument("--max-age-seconds", type=int, default=30)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        _, report = run_probe(
            manifest_path=args.manifest,
            output_path=args.output,
            deployment_mode=args.deployment_mode,
            emergency_stop_node=args.emergency_stop_node,
            emergency_stop_service=args.emergency_stop_service,
            timeout_seconds=args.timeout_seconds,
            max_age_seconds=args.max_age_seconds,
        )
    except (OSError, Ros2PairingError, RuntimeError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if report["passed"] is True else 5


if __name__ == "__main__":
    raise SystemExit(main())
