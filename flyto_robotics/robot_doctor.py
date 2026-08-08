"""Privacy-bounded network and service diagnostics for an installed robot.

The doctor is deliberately observation-only.  It never changes Wi-Fi,
restarts a service, or handles a credential.  Its output is the existing
generic resource-telemetry envelope so Cloud and the USB recovery portal can
consume the same snapshot without learning ROS or Raspberry Pi internals.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from .resource_agent import parse_resource_telemetry

DIAGNOSTIC_CHANNEL = "system.diagnostics"
DEFAULT_OUTPUT = Path("/var/lib/flyto-robot/diagnostics/latest.json")
DEFAULT_LAST_FAILURE = Path("/var/lib/flyto-robot/diagnostics/last-failure.json")
SERVICE_NAMES = (
    "flyto-delivery.service",
    "flyto-job-runner.service",
    "turtlebot3-bringup.service",
)


@dataclass(frozen=True)
class DiagnosticObservation:
    wifi_present: bool
    wifi_operstate: str
    wifi_associated: bool
    wifi_has_address: bool
    default_route: bool
    dns_ready: bool
    cloud_reachable: bool | None
    cloud_init_status: str
    service_states: Mapping[str, str]
    usb_recovery_present: bool
    usb_recovery_has_address: bool


_ACTION_CODES = {
    "provisioning_degraded": ["inspect_cloud_init_schema", "apply_netplan"],
    "wifi_interface_missing": ["inspect_wifi_driver", "use_usb_recovery"],
    "wifi_interface_down": ["bring_wifi_interface_up", "use_usb_recovery"],
    "wifi_not_associated": ["configure_known_wifi", "apply_netplan"],
    "wifi_no_address": ["renew_wifi_address", "inspect_dhcp"],
    "default_route_missing": ["inspect_default_route", "inspect_dhcp"],
    "dns_unavailable": ["inspect_dns", "inspect_router_dns"],
    "cloud_endpoint_unconfigured": ["configure_cloud_origin"],
    "cloud_unreachable": ["inspect_firewall_or_uplink", "retry_cloud_health"],
    "robot_service_unhealthy": ["inspect_service_journal", "restart_failed_service"],
    "service_state_unknown": ["retry_service_query", "inspect_service_journal"],
    "healthy": [],
}

#: What `systemctl is-active` says when a unit is fine.
SERVICE_STATE_ACTIVE = "active"

#: What :func:`_service_state` returns when it could not find out — systemctl
#: missing, systemctl hanging past the timeout, or any reply it did not
#: recognise. It is not a state the service is in; it is the absence of an
#: answer, and it used to be filtered out alongside "active".
SERVICE_STATE_UNKNOWN = "unknown"


def classify_observation(observation: DiagnosticObservation) -> tuple[str, str]:
    """Return one stable primary reason and telemetry quality.

    A service state this tool could not read is reported as unread, never as
    well. Grouping "unknown" with "active" meant a robot whose systemd could
    not be queried at all — no systemctl on PATH, or a systemctl that hung past
    the three second timeout — came back `healthy` / `good` / `services.healthy
    true`, with no action codes. This is the diagnostic an operator consults to
    decide whether a robot is fit to run, so a reassuring answer it did not
    earn is worse here than anywhere else in the tree.
    """
    network_ready = (
        observation.default_route and observation.dns_ready and observation.cloud_reachable is True
    )
    states = list(observation.service_states.values())
    failed_services = [
        state
        for state in states
        if state not in {SERVICE_STATE_ACTIVE, SERVICE_STATE_UNKNOWN}
    ]
    unknown_services = [state for state in states if state == SERVICE_STATE_UNKNOWN]
    if network_ready:
        # A service known to be down outranks one that could not be read: the
        # first names a fix, the second only names an absence.
        if failed_services:
            return "robot_service_unhealthy", "degraded"
        if unknown_services:
            return "service_state_unknown", "degraded"
        return "healthy", "good"
    if observation.cloud_init_status in {"degraded", "error"}:
        return "provisioning_degraded", "error"
    if not observation.wifi_present:
        return "wifi_interface_missing", "error"
    if observation.wifi_operstate in {"down", "lowerlayerdown", "notpresent"}:
        return "wifi_interface_down", "error"
    if not observation.wifi_associated:
        return "wifi_not_associated", "degraded"
    if not observation.wifi_has_address:
        return "wifi_no_address", "degraded"
    if not observation.default_route:
        return "default_route_missing", "degraded"
    if not observation.dns_ready:
        return "dns_unavailable", "degraded"
    if observation.cloud_reachable is None:
        return "cloud_endpoint_unconfigured", "degraded"
    return "cloud_unreachable", "error"


def diagnostic_payload(observation: DiagnosticObservation) -> tuple[dict[str, Any], str]:
    reason, quality = classify_observation(observation)
    unhealthy_services = sorted(
        name
        for name, state in observation.service_states.items()
        if state not in {SERVICE_STATE_ACTIVE, SERVICE_STATE_UNKNOWN}
    )
    unknown_services = sorted(
        name
        for name, state in observation.service_states.items()
        if state == SERVICE_STATE_UNKNOWN
    )
    payload = {
        "primary_reason_code": reason,
        "action_codes": _ACTION_CODES[reason],
        "network": {
            "wifi_present": observation.wifi_present,
            "wifi_operstate": observation.wifi_operstate,
            "wifi_associated": observation.wifi_associated,
            "wifi_has_address": observation.wifi_has_address,
            "default_route": observation.default_route,
            "dns_ready": observation.dns_ready,
            "cloud_reachable": observation.cloud_reachable,
        },
        "provisioning": {"cloud_init_status": observation.cloud_init_status},
        "services": {
            # Healthy means every service was read and every one was active.
            # An unread service leaves this false, because the honest answer to
            # "are the services healthy" after failing to look is not "yes".
            "healthy": not unhealthy_services and not unknown_services,
            "unhealthy_service_ids": unhealthy_services,
            "unknown_service_ids": unknown_services,
        },
        "recovery": {
            "usb_present": observation.usb_recovery_present,
            "usb_address_ready": observation.usb_recovery_has_address,
            "portal_origin": "http://10.77.0.1:8770",
            "ssh_host": "10.77.0.1",
        },
        "privacy": {
            "ssid_included": False,
            "ip_addresses_included": False,
            "credentials_included": False,
            "raw_logs_included": False,
        },
    }
    return payload, quality


def build_telemetry(
    observation: DiagnosticObservation,
    *,
    resource_id: str,
    sequence: int,
    observed_at: str,
    channel_id: str = DIAGNOSTIC_CHANNEL,
) -> dict[str, Any]:
    payload, quality = diagnostic_payload(observation)
    return parse_resource_telemetry(
        {
            "contract": "flyto.resource-telemetry.v1",
            "resource_id": resource_id,
            "channel_id": channel_id,
            "sequence": sequence,
            "observed_at": observed_at,
            "quality": quality,
            "payload": payload,
        }
    )


def collect_observation(*, cloud_url: str = "") -> DiagnosticObservation:
    wifi_state_path = Path("/sys/class/net/wlan0/operstate")
    usb_state_path = Path("/sys/class/net/usb0/operstate")
    wifi_present = wifi_state_path.exists()
    wifi_operstate = _read_text(wifi_state_path, "missing")
    wpa = _key_values(_run(["wpa_cli", "-i", "wlan0", "status"])) if wifi_present else {}
    wifi_associated = wpa.get("wpa_state") == "COMPLETED"
    wifi_has_address = _interface_has_address("wlan0") if wifi_present else False
    default_route = bool(_json_command(["ip", "-j", "route", "show", "default"], []))

    parsed_cloud = urlparse(cloud_url)
    host = parsed_cloud.hostname or ""
    port = parsed_cloud.port or (443 if parsed_cloud.scheme == "https" else 80)
    dns_ready = _dns_ready(host) if host else default_route
    cloud_reachable = _tcp_ready(host, port) if host and dns_ready else None if not host else False

    cloud_init_status = _cloud_init_status()
    service_states = {name: _service_state(name) for name in SERVICE_NAMES}
    usb_present = usb_state_path.exists()
    return DiagnosticObservation(
        wifi_present=wifi_present,
        wifi_operstate=wifi_operstate,
        wifi_associated=wifi_associated,
        wifi_has_address=wifi_has_address,
        default_route=default_route,
        dns_ready=dns_ready,
        cloud_reachable=cloud_reachable,
        cloud_init_status=cloud_init_status,
        service_states=service_states,
        usb_recovery_present=usb_present,
        usb_recovery_has_address=_interface_has_address("usb0") if usb_present else False,
    )


def write_report(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.chmod(0o644)
    temporary.replace(path)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Explain why an installed robot is unreachable")
    parser.add_argument("--resource-id", required=True)
    parser.add_argument("--cloud-url", default=os.getenv("FLYTO_CLOUD_URL", ""))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--last-failure", type=Path, default=DEFAULT_LAST_FAILURE)
    args = parser.parse_args(argv)

    observed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    report = build_telemetry(
        collect_observation(cloud_url=args.cloud_url),
        resource_id=args.resource_id,
        sequence=time.time_ns(),
        observed_at=observed_at,
    )
    write_report(args.output, report)
    if report["quality"] != "good":
        failure = dict(report)
        failure["channel_id"] = "system.last_failure"
        failure.pop("payload_hash", None)
        write_report(args.last_failure, parse_resource_telemetry(failure))
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


def _run(command: Sequence[str], timeout: float = 3.0) -> str:
    try:
        completed = subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return ""
    return completed.stdout[:32_768]


def _read_text(path: Path, default: str) -> str:
    try:
        return path.read_text(encoding="utf-8").strip().lower()
    except OSError:
        return default


def _key_values(value: str) -> dict[str, str]:
    return {
        key: raw
        for line in value.splitlines()
        if "=" in line
        for key, raw in [line.split("=", 1)]
        if key in {"wpa_state"}
    }


def _json_command(command: Sequence[str], default: Any) -> Any:
    try:
        return json.loads(_run(command) or "null") or default
    except json.JSONDecodeError:
        return default


def _interface_has_address(interface: str) -> bool:
    values = _json_command(["ip", "-j", "address", "show", "dev", interface], [])
    return any(
        item.get("scope") == "global"
        for link in values
        for item in link.get("addr_info", [])
        if isinstance(item, dict)
    )


def _dns_ready(host: str) -> bool:
    try:
        socket.getaddrinfo(host, None)
    except socket.gaierror:
        return False
    return True


def _tcp_ready(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=2.0):
            return True
    except OSError:
        return False


def _cloud_init_status() -> str:
    value = _run(["cloud-init", "status", "--format", "json"])
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError:
        plain = _run(["cloud-init", "status"]).lower()
        return next(
            (state for state in ("error", "degraded", "running", "done") if state in plain),
            "unknown",
        )
    status = str(parsed.get("extended_status") or parsed.get("status") or "unknown").lower()
    return next(
        (state for state in ("error", "degraded", "running", "done") if state in status), "unknown"
    )


def _service_state(name: str) -> str:
    value = _run(["systemctl", "is-active", name]).strip().lower()
    return (
        value
        if value in {"active", "inactive", "failed", "activating", "deactivating"}
        else "unknown"
    )


if __name__ == "__main__":
    raise SystemExit(main())
