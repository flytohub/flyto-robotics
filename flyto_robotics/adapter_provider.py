"""Flyto2 external-adapter provider for standard ROS 2 equipment.

The provider is installed on the execution computer, never on the robot.  It
offers both Python entry points for current hosts and a tiny JSON-lines process
protocol for Flyto2 Runtime, so Cloud never imports ROS, rosbridge, or vendor
transport code.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import json
import os
import re
import socket
import sys
import threading
from collections.abc import Callable, Mapping
from datetime import datetime, timezone
from typing import Any

from . import adapter_contract as contract
from .generic_ros2_adapter import SILENT_SECONDS, GenericROS2Adapter, build

ADAPTER_ID = "ros2.generic"
PROVIDER_PROTOCOL = "flyto2.adapter-provider.v1"


def _safe_fragment(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(value or "").strip())
    return text.strip("-._")[:48] or "resource"


def _resource_identity() -> tuple[str, str]:
    domain = os.getenv("ROS_DOMAIN_ID", "0").strip() or "0"
    hostname = socket.gethostname()
    resource_id = (
        os.getenv("FLYTO_ROS2_RESOURCE_ID", "").strip()
        or f"ros2-{_safe_fragment(hostname)}-{_safe_fragment(domain)}"
    )[:128]
    resource_name = (
        os.getenv("FLYTO_ROS2_RESOURCE_NAME", "").strip()
        or f"ROS 2 robot ({hostname})"
    )[:200]
    return resource_id, resource_name


def _manifest(adapter: GenericROS2Adapter, *, resource_name: str) -> dict[str, Any]:
    declarations = tuple(adapter.describe())
    contracts: list[dict[str, Any]] = []
    for item in declarations:
        metadata = contract.capability_metadata(item.capability_id)
        contracts.append(
            {
                "capability_id": item.capability_id,
                "display_name": str(metadata.get("display_name") or item.capability_id),
                "description": str(metadata.get("description") or "")[:1000],
                "input_schema": contract.arguments_to_json_schema(item.arguments),
                "safety_class": item.safety_class,
                "required_permissions": list(item.required_permissions),
                "requires_safe_stop": item.requires_safe_stop,
            }
        )
    return {
        "contract": "flyto.resource-manifest.v1",
        "resource_id": adapter.resource_id,
        "resource_type": "robot",
        "display_name": resource_name,
        "revision": 1,
        "adapter": {
            "adapter_id": ADAPTER_ID,
            "version": "1.0.0",
            "provider": "Flyto2 Robotics",
        },
        "deployment_mode": (
            "simulation"
            if os.getenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware").strip().lower()
            == "simulation"
            else "real"
        ),
        "capability_ids": [item["capability_id"] for item in contracts],
        "capability_contracts": contracts,
        "settings": [],
        "telemetry_channels": [],
        "observed_at": datetime.now(timezone.utc).isoformat(),
        "contract_hash": "",
    }


def _discovery_enabled() -> bool:
    disabled = os.getenv("FLYTO_ROS2_AUTODISCOVER", "").strip().lower()
    if disabled in {"0", "false", "off", "no"}:
        return False
    transport = os.getenv("FLYTO_ROS2_TRANSPORT", "rclpy").strip().lower()
    if transport == "rosbridge" and not os.getenv("FLYTO_ROSBRIDGE_URL", "").strip():
        return False
    return transport in {"rclpy", "rosbridge"}


def discover_resource_manifests(*, execution_host_id: str) -> list[dict[str, Any]]:
    """Discover one reachable standard ROS 2 graph without granting authority.

    While a presence watch (``watch_resources``) holds a connection to the
    robot, the pass reads the graph over it instead of connecting again.
    """

    _ = execution_host_id
    if not _discovery_enabled():
        return []

    resource_id, resource_name = _resource_identity()
    watched = _watched_adapter(resource_id)
    if watched is not None:
        try:
            # A pass is asked for because something changed: read the graph
            # as it is now, not as the presence connection last cached it.
            watched._invalidate_discovery()
            if not watched.describe():
                return []
            return [_manifest(watched, resource_name=resource_name)]
        except Exception:
            return []
    adapter: GenericROS2Adapter | None = None
    try:
        adapter = build(resource_id)
        if not adapter.describe():
            return []
        return [_manifest(adapter, resource_name=resource_name)]
    except Exception:
        # Discovery is best effort.  An unavailable transport must not become
        # an authoritative "no equipment exists" statement.
        return []
    finally:
        if adapter is not None:
            adapter.disconnect()


# Seconds between attempts to reach a robot whose transport is not up. Nothing
# announces a remote rosbridge starting to listen, so this back-off is the
# fallback that notices it. A robot that is reachable is followed on its own
# messages and connection callbacks, and its absence is a deadline on its last
# message, not a sweep.
_REACH_BACKOFF_SECONDS = (1.0, 2.0, 5.0, 10.0)


class PresenceWatch:
    """Tells the execution host the moment its robot appears, changes or goes.

    It holds one light connection (odometry and Nav2 lifecycle events only;
    it never commands the robot) and calls ``notify(reason)`` when:

    - the transport connects or reconnects (``robot_connected``), or drops
      (``robot_disconnected``), or cannot be reached (``robot_unreachable``);
    - the robot's graph changes: its first odometry, odometry after a silence,
      a Nav2 lifecycle transition (``odometry_appeared``, ``lifecycle:...``);
    - the robot falls silent for ``SILENT_SECONDS`` over a transport that is
      still up (``robot_silent``): an rclpy node stays up when the robot
      powers off. That is a deadline re-armed from the last reading.

    The host answers each with a discovery pass, so a robot that comes up
    after the host registered is published at once, not on a heartbeat.
    """

    def __init__(
        self,
        *,
        resource_id: str,
        notify: Callable[[str], None],
        build_adapter: Callable[[str], GenericROS2Adapter] | None = None,
        wait: Callable[[threading.Event, float | None], bool] | None = None,
        start: bool = True,
    ) -> None:
        self.resource_id = resource_id
        self._notify_host = notify
        self._build = build_adapter or (lambda rid: build(rid, presence_only=True))
        self._wait = wait or (lambda event, timeout: event.wait(timeout))
        self._stop = threading.Event()
        # Set by a drop, a reading after silence, or close(): re-judge now.
        self._changed = threading.Event()
        self._adapter: GenericROS2Adapter | None = None
        self._unreachable = False
        self._silent = False
        self._thread: threading.Thread | None = None
        if start:
            self._thread = threading.Thread(
                target=self.run, name="ros2-presence-watch", daemon=True
            )
            self._thread.start()

    @property
    def adapter(self) -> GenericROS2Adapter | None:
        adapter = self._adapter
        return adapter if adapter is not None and adapter.connected else None

    def unreachable_resource_ids(self) -> set[str]:
        """This robot, when the watch has positive evidence it is gone."""
        return {self.resource_id} if (self._unreachable or self._silent) else set()

    def close(self) -> None:
        self._stop.set()
        self._changed.set()

    def _notify(self, reason: str) -> None:
        # The host's discovery is advisory: its failure never stops the watch.
        with contextlib.suppress(Exception):
            self._notify_host(reason)

    def _on_connection(self, connected: bool) -> None:
        if connected:
            return
        self._changed.set()
        if not self._stop.is_set():
            self._notify("robot_disconnected")

    def _on_graph(self, reason: str) -> None:
        if reason.endswith(("_appeared", "_returned")):
            # A reading arrived: the silence deadline is re-judged at once.
            self._changed.set()
        self._notify(reason)

    def run(self) -> None:
        attempt = 0
        try:
            while not self._stop.is_set():
                if self.adapter is None:
                    if self._reach():
                        attempt = 0
                        continue
                    delay = _REACH_BACKOFF_SECONDS[min(attempt, len(_REACH_BACKOFF_SECONDS) - 1)]
                    attempt += 1
                    if self._wait(self._stop, delay):
                        return
                    continue
                self._follow()
        finally:
            adapter, self._adapter = self._adapter, None
            if adapter is not None:
                adapter.disconnect()

    def _reach(self) -> bool:
        try:
            if self._adapter is None:
                adapter = self._build(self.resource_id)
                adapter.add_connection_listener(self._on_connection)
                adapter.add_graph_listener(self._on_graph)
                self._adapter = adapter
            else:
                self._adapter.reconnect()
        except Exception:  # noqa: BLE001 - the robot side is not up
            if not self._unreachable:
                self._unreachable = True
                self._notify("robot_unreachable")
            return False
        if self._stop.is_set() or self.adapter is None:
            return False
        self._unreachable = False
        self._notify("robot_connected")
        return True

    def _follow(self) -> None:
        """Wait on the connection until it drops, or the robot goes quiet."""
        self._changed.clear()
        adapter = self.adapter
        if adapter is None:
            return
        silent = adapter.silent_seconds()
        if silent is None:
            return
        if silent >= SILENT_SECONDS:
            if not self._silent:
                self._silent = True
                self._notify("robot_silent")
            # Nothing to time now: the next reading, a drop or close wakes it.
            self._wait(self._changed, None)
            return
        self._silent = False
        self._wait(self._changed, SILENT_SECONDS - silent)


_watches: dict[str, PresenceWatch] = {}
_watches_lock = threading.Lock()


def _watched_adapter(resource_id: str) -> GenericROS2Adapter | None:
    with _watches_lock:
        watch = _watches.get(resource_id)
    return watch.adapter if watch is not None else None


def watch_resources(
    *, execution_host_id: str, notify: Callable[[str], None]
) -> PresenceWatch | None:
    """Start following this host's robot; ``notify(reason)`` on every change.

    Returns the watch (``close()``, ``unreachable_resource_ids()``), or None
    when discovery is off for this host. Hosts find it as the ``watch``
    attribute of ``discover_resource_manifests``, so a host that predates it
    keeps discovering on its own triggers.
    """
    _ = execution_host_id
    if not _discovery_enabled():
        return None
    transport = os.getenv("FLYTO_ROS2_TRANSPORT", "rclpy").strip().lower()
    if transport == "rclpy" and importlib.util.find_spec("rclpy") is None:
        # No ROS 2 on this computer: nothing to watch, and retrying the
        # import on a back-off would only ever fail.
        return None
    resource_id, _name = _resource_identity()
    with _watches_lock:
        current = _watches.get(resource_id)
        if current is not None and not current._stop.is_set():
            # The host re-armed its discovery: changes now go to the new hook.
            current._notify_host = notify
            return current
        watch = PresenceWatch(resource_id=resource_id, notify=notify)
        _watches[resource_id] = watch
        return watch


discover_resource_manifests.watch = watch_resources  # type: ignore[attr-defined]


def build_adapter(resource_id: str) -> GenericROS2Adapter:
    """Python entry point used by hosts that load adapters in-process."""

    return build(resource_id)


def _response(request_id: Any, *, ok: bool, result: Any = None, error: str = "") -> dict[str, Any]:
    payload: dict[str, Any] = {
        "protocol": PROVIDER_PROTOCOL,
        "id": request_id,
        "ok": ok,
    }
    if ok:
        payload["result"] = result
    else:
        payload["error"] = error[:500] or "adapter provider request failed"
    return payload


def _serve(adapter_id: str, resource_id: str) -> int:
    if adapter_id != ADAPTER_ID:
        print(
            json.dumps(
                _response(None, ok=False, error=f"unsupported adapter: {adapter_id}"),
                separators=(",", ":"),
            ),
            flush=True,
        )
        return 2

    adapter = build(resource_id)
    try:
        for raw in sys.stdin:
            raw = raw.strip()
            if not raw:
                continue
            try:
                request = json.loads(raw)
                if not isinstance(request, Mapping):
                    raise ValueError("request must be an object")
                request_id = request.get("id")
                op = str(request.get("op") or "")
                if op == "invoke":
                    call = request.get("request")
                    if not isinstance(call, Mapping):
                        raise ValueError("invoke.request must be an object")
                    result = adapter.invoke(
                        contract.CallRequest(
                            call_id=str(call.get("call_id") or ""),
                            capability_id=str(call.get("capability_id") or ""),
                            arguments=dict(call.get("arguments") or {}),
                            deadline_seconds=float(call.get("deadline_seconds") or 30.0),
                        )
                    )
                    value = {
                        "call_id": result.call_id,
                        "outcome": result.outcome,
                        "evidence": dict(result.evidence),
                        "detail": result.detail,
                    }
                elif op == "cancel":
                    result = adapter.cancel(str(request.get("call_id") or ""))
                    value = {
                        "call_id": result.call_id,
                        "outcome": result.outcome,
                        "evidence": dict(result.evidence),
                        "detail": result.detail,
                    }
                elif op == "safe_stop":
                    result = adapter.safe_stop()
                    value = {
                        "call_id": result.call_id,
                        "outcome": result.outcome,
                        "evidence": dict(result.evidence),
                        "detail": result.detail,
                    }
                elif op == "observe":
                    value = adapter.observe(
                        phase=str(request.get("phase") or "preflight"),
                        execution_id=(
                            str(request["execution_id"])
                            if request.get("execution_id") is not None
                            else None
                        ),
                    )
                elif op == "describe":
                    value = _manifest(adapter, resource_name=_resource_identity()[1])
                elif op == "execution_count":
                    value = {
                        "count": adapter.execution_count(str(request.get("call_id") or ""))
                    }
                elif op == "close":
                    print(
                        json.dumps(
                            _response(
                                request_id,
                                ok=True,
                                result={"closed": True},
                            ),
                            separators=(",", ":"),
                        ),
                        flush=True,
                    )
                    return 0
                else:
                    raise ValueError(f"unsupported provider operation: {op}")
                response = _response(request_id, ok=True, result=value)
            except Exception as error:  # provider boundary must return typed failure
                response = _response(
                    locals().get("request_id"),
                    ok=False,
                    error=f"{type(error).__name__}: {error}",
                )
            print(json.dumps(response, separators=(",", ":"), ensure_ascii=True), flush=True)
    finally:
        adapter.disconnect()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="flyto2-adapter-provider-ros2-generic")
    parser.add_argument("--discover", action="store_true")
    parser.add_argument("--execution-host-id", default="")
    parser.add_argument("--adapter-id", default=ADAPTER_ID)
    parser.add_argument("--resource-id", default="")
    args = parser.parse_args(argv)

    if args.discover:
        print(
            json.dumps(
                {
                    "protocol": PROVIDER_PROTOCOL,
                    "resources": discover_resource_manifests(
                        execution_host_id=args.execution_host_id
                    ),
                },
                separators=(",", ":"),
                ensure_ascii=True,
            )
        )
        return 0

    resource_id = args.resource_id.strip() or _resource_identity()[0]
    return _serve(args.adapter_id, resource_id)


if __name__ == "__main__":
    raise SystemExit(main())
