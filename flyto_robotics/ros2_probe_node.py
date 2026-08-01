"""Live ROS graph probe for content-addressed Flyto2 readiness evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections.abc import Mapping, Sequence
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

    def interface_available(
        self,
        *,
        kind: str,
        name: str,
        interface_type: str,
        timeout_seconds: float,
    ) -> bool:
        if kind == "action":
            from rclpy.action import ActionClient
            from rosidl_runtime_py.utilities import get_action

            client = ActionClient(self.node, get_action(interface_type), name)
            try:
                return bool(client.wait_for_server(timeout_sec=timeout_seconds))
            finally:
                client.destroy()
        if kind == "service":
            from rosidl_runtime_py.utilities import get_service

            client = self.node.create_client(get_service(interface_type), name)
            try:
                return bool(client.wait_for_service(timeout_sec=timeout_seconds))
            finally:
                self.node.destroy_client(client)
        return False

    def lifecycle_state(
        self,
        managed_nodes: Sequence[str],
        *,
        timeout_seconds: float,
    ) -> str:
        import rclpy
        from lifecycle_msgs.srv import GetState

        labels: list[str] = []
        for managed_node in managed_nodes:
            service_name = managed_node.rstrip("/") + "/get_state"
            client = self.node.create_client(GetState, service_name)
            try:
                if not client.wait_for_service(timeout_sec=timeout_seconds):
                    labels.append("unknown")
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    self.node,
                    future,
                    timeout_sec=timeout_seconds,
                )
                response = future.result()
                if response is None:
                    labels.append("unknown")
                else:
                    labels.append(str(response.current_state.label).lower())
            finally:
                self.node.destroy_client(client)
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

        owner_name, owner_namespace = _split_node_path(owner_node)
        if (owner_namespace.rstrip("/") + "/" + owner_name).replace("//", "/") == (
            self.node.get_fully_qualified_name()
        ):
            return False
        try:
            services = dict(
                self.node.get_service_names_and_types_by_node(
                    owner_name,
                    owner_namespace,
                )
            )
        except Exception:
            return False
        if "std_srvs/srv/Trigger" not in services.get(service_name, []):
            return False
        client = self.node.create_client(
            get_service("std_srvs/srv/Trigger"),
            service_name,
        )
        try:
            return bool(client.wait_for_service(timeout_sec=timeout_seconds))
        finally:
            self.node.destroy_client(client)


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
    try:
        runtime = collect_ros2_runtime_snapshot(
            manifest,
            RclpyGraphProbe(node),
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
