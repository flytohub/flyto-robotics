"""Privacy-bounded network and service diagnostics for an installed robot.

The doctor is deliberately observation-only.  It never changes Wi-Fi,
restarts a service, or handles a credential.  Its output is the existing
generic resource-telemetry envelope so Cloud and the USB recovery portal can
consume the same snapshot without learning ROS or Raspberry Pi internals.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import socket
import subprocess
import sys
import threading
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from .device_events import (
    DeviceEventError,
    DeviceEventJournal,
    build_device_event,
    event_sequence,
    now_observed_at,
)
from .fsio import atomic_write
from .resource_agent import parse_resource_telemetry

DIAGNOSTIC_CHANNEL = "system.diagnostics"
#: The component name this tool signs its device events with. Generic on
#: purpose: nothing upstream should have to know a robot produced them.
DIAGNOSTIC_COMPONENT = "device_diagnostics"
DEFAULT_OUTPUT = Path("/var/lib/flyto-robot/diagnostics/latest.json")
DEFAULT_LAST_FAILURE = Path("/var/lib/flyto-robot/diagnostics/last-failure.json")
DEFAULT_RECOVERY_STATE = Path("/var/lib/flyto-robot/diagnostics/recovery-state.json")

#: The event journal this tool appends a degradation to.
#
# Deliberately *not* inside the diagnostics directory beside latest.json. The
# journal owns the mode of the directory it lives in and tightens it to 0700,
# and the recovery portal serves latest.json out of that same directory under a
# different account — so sharing one directory would silently take the portal's
# read away the first time the doctor recorded anything.
DEFAULT_EVENT_JOURNAL = Path("/var/lib/flyto-robot/events/device-events.jsonl")

#: Environment name a unit uses to give this service its own journal. Each
#: installed service gets an explicit path of its own: the journal is owner-only
#: by construction, and the doctor runs as root while the job runner runs as
#: ubuntu, so one shared file would be either unwritable or loosened.
EVENT_JOURNAL_ENV = "FLYTO_DEVICE_EVENT_JOURNAL"

#: Exit status when the diagnosis was produced but could not be recorded. Not
#: 0, because something an operator relies on did not happen; not 1 or 2, which
#: argparse and a failed diagnosis already use.
EXIT_EVENT_NOT_RECORDED = 4
SERVICE_NAMES = (
    "flyto-delivery.service",
    "flyto-job-runner.service",
    "turtlebot3-bringup.service",
    # The portal is how an operator reaches a robot that has lost the network,
    # so it is the one service whose failure is hardest to notice from outside.
    # It was left off this list and spent three hours in a restart loop while
    # the doctor reported services.healthy true — a health check that does not
    # watch the recovery path will always look best exactly when it is wrong.
    "flyto-recovery-portal.service",
)
RECOVERY_STATE_CONTRACT = "flyto.service-recovery-state.v1"
RECOVERY_REASON = "managed_service_recovered"
RECOVERY_ACTIONS = ["inspect_service_recovery"]
RECOVERY_MESSAGE = "A managed service recovered after a watchdog restart."
_RECOVERY_JOURNAL_LINES = 128
_RECOVERY_THREAD_LOCK = threading.Lock()


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
    service_recoveries: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)


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
    RECOVERY_REASON: RECOVERY_ACTIONS,
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
    recoveries = list(observation.service_recoveries.values())
    known_recoveries = [item for item in recoveries if item.get("status") == "known"]
    newly_observed = sum(int(item.get("new_count", 0)) for item in known_recoveries)
    cumulative = sum(int(item.get("restart_count", 0)) for item in known_recoveries)
    watchdog_total = sum(int(item.get("watchdog_count", 0)) for item in known_recoveries)
    recovery_unknown = any(item.get("status") == "unknown" for item in recoveries)
    if reason == "healthy" and newly_observed:
        reason, quality = RECOVERY_REASON, "degraded"
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
            "service_restarts": {
                "status": "unknown" if recovery_unknown else "known",
                "current_boot_total": cumulative,
                "current_boot_watchdog_total": watchdog_total,
                "newly_observed": newly_observed,
                "current_recovery_kind": (
                    "watchdog_timeout" if any(
                        item.get("current_recovery_kind") == "watchdog_timeout"
                        for item in known_recoveries
                    ) else None
                ),
                "raw_journal_included": False,
            },
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


#: One short public sentence per reason code. Fixed text, not a formatted
#: string: an operator-facing message assembled from observed values is how a
#: hostname, an SSID or an address ends up in a fleet-wide event stream.
_REASON_MESSAGES = {
    "provisioning_degraded": "First-boot provisioning did not complete.",
    "wifi_interface_missing": "No wireless interface is present.",
    "wifi_interface_down": "The wireless interface is down.",
    "wifi_not_associated": "The wireless interface is not associated with any network.",
    "wifi_no_address": "The wireless interface has no routable address.",
    "default_route_missing": "There is no default route.",
    "dns_unavailable": "Name resolution is unavailable.",
    "cloud_endpoint_unconfigured": "No Cloud endpoint is configured on this device.",
    "cloud_unreachable": "The configured Cloud endpoint is unreachable.",
    "robot_service_unhealthy": "A managed service on this device is not running.",
    "service_state_unknown": "A managed service could not be queried.",
    RECOVERY_REASON: RECOVERY_MESSAGE,
}

#: quality -> (severity, status). "error" becomes "unavailable" rather than
#: "failed": nothing was attempted and failed, the device cannot be reached.
_QUALITY_PROJECTION = {
    "degraded": ("warning", "degraded"),
    "stale": ("warning", "degraded"),
    "error": ("error", "unavailable"),
}


def degradation_event(
    observation: DiagnosticObservation,
    *,
    resource_id: str,
    sequence: int,
    observed_at: str,
    telemetry: Mapping[str, Any] | None = None,
) -> dict[str, Any] | None:
    """Project one degraded snapshot into the generic device-event contract.

    A healthy snapshot yields ``None``.  A device that is fine has nothing to
    say, and an event stream padded with "still fine" is a stream nobody reads.
    The telemetry envelope remains the record of what was observed; this is the
    same observation restated as *what is wrong and what to do*, which is the
    only part an upstream reader can act on without learning this device.

    The linkage back to that envelope is exact rather than approximate:
    ``correlation_id`` is the telemetry's own ``payload_hash``, so an event and
    the snapshot it came from can be matched with no timestamp arithmetic and no
    trust in clocks.  ``run_id`` is empty, which this contract documents as
    meaning the observation belongs to no run — a periodic health check does
    not, and inventing one would make unrelated snapshots look like a sequence.
    """
    envelope = telemetry or build_telemetry(
        observation, resource_id=resource_id, sequence=sequence, observed_at=observed_at
    )
    quality = str(envelope["quality"])
    projection = _QUALITY_PROJECTION.get(quality)
    if projection is None:
        return None
    severity, status = projection

    payload = envelope["payload"]
    network = payload["network"]
    services = payload["services"]
    event_id = ""
    if payload["primary_reason_code"] == RECOVERY_REASON:
        occurrence = {
            name: int(item.get("watchdog_count", 0))
            for name, item in observation.service_recoveries.items()
            if item.get("status") == "known"
        }
        digest = hashlib.sha256(
            json.dumps(
                {
                    "boot_id": _current_boot_id(),
                    "occurrence": occurrence,
                    "resource_id": resource_id,
                },
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        event_id = f"evt-{digest[:32]}"
    return build_device_event(
        resource_id=resource_id,
        component=DIAGNOSTIC_COMPONENT,
        sequence=sequence,
        observed_at=observed_at,
        severity=severity,
        status=status,
        reason_code=str(payload["primary_reason_code"]),
        action_codes=list(payload["action_codes"]),
        correlation_id=str(envelope["payload_hash"]),
        run_id="",
        message=_REASON_MESSAGES.get(
            str(payload["primary_reason_code"]), "This device reported a degraded state."
        ),
        details={
            "telemetry": {
                "contract": envelope["contract"],
                "channel_id": envelope["channel_id"],
                "sequence": envelope["sequence"],
                "observed_at": envelope["observed_at"],
                "quality": quality,
                "payload_hash": envelope["payload_hash"],
            },
            # Counts and flags, never the identifiers themselves: a service name
            # is fine, but this projection exists to stay bounded and generic.
            "network": {
                "interface_present": bool(network["wifi_present"]),
                "interface_operstate": str(network["wifi_operstate"]),
                "associated": bool(network["wifi_associated"]),
                "has_routable_address": bool(network["wifi_has_address"]),
                "default_route": bool(network["default_route"]),
                "name_resolution": bool(network["dns_ready"]),
                "uplink_reachable": network["cloud_reachable"],
            },
            "services": {
                "healthy": bool(services["healthy"]),
                "unhealthy_count": len(services["unhealthy_service_ids"]),
                "unread_count": len(services["unknown_service_ids"]),
            },
            "provisioning": {"status": str(payload["provisioning"]["cloud_init_status"])},
            "recovery": {
                "offline_path_present": bool(payload["recovery"]["usb_present"]),
                "offline_path_ready": bool(payload["recovery"]["usb_address_ready"]),
                "service_restart_count": int(
                    payload["recovery"]["service_restarts"]["current_boot_total"]
                ),
                "newly_observed_restart_count": int(
                    payload["recovery"]["service_restarts"]["newly_observed"]
                ),
                "current_recovery_kind": payload["recovery"]["service_restarts"][
                    "current_recovery_kind"
                ],
            },
        },
        event_id=event_id,
    )


def _recovery_event(
    observation: DiagnosticObservation,
    *,
    resource_id: str,
    sequence: int,
    observed_at: str,
    correlation_id: str,
) -> dict[str, Any] | None:
    new_count = sum(
        int(item.get("new_count", 0))
        for item in observation.service_recoveries.values()
        if item.get("status") == "known"
    )
    if not new_count:
        return None
    occurrence = {
        name: int(item.get("watchdog_count", 0))
        for name, item in observation.service_recoveries.items()
        if item.get("status") == "known"
    }
    digest = hashlib.sha256(
        json.dumps(
            {
                "boot_id": _current_boot_id(),
                "occurrence": occurrence,
                "resource_id": resource_id,
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return build_device_event(
        resource_id=resource_id,
        component=DIAGNOSTIC_COMPONENT,
        sequence=sequence,
        observed_at=observed_at,
        severity="warning",
        status="degraded",
        reason_code=RECOVERY_REASON,
        action_codes=RECOVERY_ACTIONS,
        correlation_id=correlation_id,
        message=RECOVERY_MESSAGE,
        details={
            "recovery": {
                "newly_observed_watchdog_count": new_count,
                "current_boot_watchdog_total": sum(occurrence.values()),
                "current_recovery_kind": "watchdog_timeout",
                "raw_logs_included": False,
            }
        },
        event_id=f"evt-{digest[:32]}",
    )


def _current_boot_id() -> str:
    value = _read_text(Path("/proc/sys/kernel/random/boot_id"), "")
    return hashlib.sha256(value.encode("ascii", errors="ignore")).hexdigest() if value else ""


def _load_recovery_state(path: Path) -> tuple[dict[str, dict[str, int]], bool]:
    """Return recorded current-boot counters and whether the state is usable."""
    if not path.exists():
        return {}, True
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        if value.get("contract") != RECOVERY_STATE_CONTRACT:
            return {}, False
        stored_boot = value.get("boot_id", "")
        if not isinstance(stored_boot, str) or len(stored_boot) > 64:
            return {}, False
        current_boot = _current_boot_id()
        if stored_boot and current_boot and stored_boot != current_boot:
            return {}, True
        restart_counts = value.get("restart_counts")
        watchdog_counts = value.get("watchdog_counts")
        if not isinstance(restart_counts, dict) or not isinstance(watchdog_counts, dict):
            return {}, False
        if any(
            name not in SERVICE_NAMES
            or isinstance(count, bool)
            or not isinstance(count, int)
            or count < 0
            or count > 1_000_000
            for counters in (restart_counts, watchdog_counts)
            for name, count in counters.items()
        ):
            return {}, False
        return {
            name: {
                "restart_count": restart_counts.get(name, 0),
                "watchdog_count": watchdog_counts.get(name, 0),
            }
            for name in SERVICE_NAMES
        }, True
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, AttributeError):
        return {}, False


def _systemd_restart_count(name: str) -> int | None:
    value = _run(
        ["systemctl", "show", name, "--property=NRestarts", "--value"], timeout=3.0
    ).strip()
    try:
        count = int(value)
    except ValueError:
        return None
    return count if 0 <= count <= 1_000_000 else None


def _watchdog_count_this_boot(name: str) -> int | None:
    """Count exact structured current-boot watchdog results without returning logs."""
    value = _run(
        [
            "journalctl",
            "--boot=0",
            "--output=json",
            "--no-pager",
            f"--lines={_RECOVERY_JOURNAL_LINES}",
            f"UNIT={name}",
        ],
        timeout=3.0,
    )
    if not value:
        return None
    count = 0
    for line in value.splitlines()[:_RECOVERY_JOURNAL_LINES]:
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            return None
        if not isinstance(record, dict):
            return None
        unit = record.get("UNIT")
        result = record.get("UNIT_RESULT")
        if unit is not None and not isinstance(unit, str):
            return None
        if result is not None and not isinstance(result, str):
            return None
        if unit == name and result == "watchdog":
            count += 1
    return count


def collect_service_recoveries(path: Path) -> dict[str, dict[str, Any]]:
    previous, state_valid = _load_recovery_state(path)
    if not state_valid:
        return {
            name: {
                "status": "unknown",
                "restart_count": 0,
                "watchdog_count": 0,
                "new_count": 0,
                "current_recovery_kind": None,
            }
            for name in SERVICE_NAMES
        }
    result: dict[str, dict[str, Any]] = {}
    for name in SERVICE_NAMES:
        count = _systemd_restart_count(name)
        if count is None:
            result[name] = {
                "status": "unknown",
                "restart_count": 0,
                "watchdog_count": 0,
                "new_count": 0,
                "current_recovery_kind": None,
            }
            continue
        old = previous.get(name, {})
        watchdog_count = _watchdog_count_this_boot(name)
        if watchdog_count is None:
            result[name] = {
                "status": "unknown",
                "restart_count": count,
                "watchdog_count": 0,
                "new_count": 0,
                "current_recovery_kind": None,
            }
            continue
        old_watchdog_count = old.get("watchdog_count", 0)
        new_count = (
            watchdog_count - old_watchdog_count
            if watchdog_count >= old_watchdog_count
            else watchdog_count
        )
        result[name] = {
            "status": "known",
            "restart_count": count,
            "watchdog_count": watchdog_count,
            "new_count": new_count,
            "current_recovery_kind": "watchdog_timeout" if new_count else None,
        }
    return result


def _commit_recovery_state(
    path: Path, recoveries: Mapping[str, Mapping[str, Any]]
) -> None:
    if not recoveries:
        return
    previous, state_valid = _load_recovery_state(path)
    if not state_valid:
        return
    restart_counts = {
        name: int(item.get("restart_count", 0)) for name, item in previous.items()
    }
    restart_counts.update({
        name: int(item["restart_count"])
        for name, item in recoveries.items()
        if name in SERVICE_NAMES and item.get("status") == "known"
    })
    watchdog_counts = {
        name: int(item.get("watchdog_count", 0)) for name, item in previous.items()
    }
    watchdog_counts.update({
        name: int(item["watchdog_count"])
        for name, item in recoveries.items()
        if name in SERVICE_NAMES and item.get("status") == "known"
    })
    if not restart_counts and any(item.get("status") == "unknown" for item in recoveries.values()):
        return
    atomic_write(
        path,
        json.dumps(
            {
                "contract": RECOVERY_STATE_CONTRACT,
                "boot_id": _current_boot_id(),
                "restart_counts": restart_counts,
                "watchdog_counts": watchdog_counts,
            },
            sort_keys=True,
        )
        + "\n",
        0o600,
    )


def collect_observation(
    *, cloud_url: str = "", recovery_state_path: Path = DEFAULT_RECOVERY_STATE
) -> DiagnosticObservation:
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
    service_recoveries = collect_service_recoveries(recovery_state_path)
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
        service_recoveries=service_recoveries,
    )


def write_report(path: Path, value: Mapping[str, Any]) -> None:
    atomic_write(path, json.dumps(value, ensure_ascii=False, sort_keys=True) + "\n", 0o644)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Explain why an installed robot is unreachable")
    parser.add_argument("--resource-id", required=True)
    parser.add_argument("--cloud-url", default=os.getenv("FLYTO_CLOUD_URL", ""))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--last-failure", type=Path, default=DEFAULT_LAST_FAILURE)
    parser.add_argument("--recovery-state", type=Path, default=DEFAULT_RECOVERY_STATE)
    parser.add_argument(
        "--event-journal",
        type=Path,
        default=Path(os.getenv(EVENT_JOURNAL_ENV, "") or DEFAULT_EVENT_JOURNAL),
        help="owner-only append-only device event journal for degraded snapshots",
    )
    args = parser.parse_args(argv)

    # One instant for both stamps. Taking the timestamp and the ordering key
    # from two separate clock reads lets an event claim a microsecond it was not
    # observed in, and the two are what a reader uses to line snapshots up.
    moment = datetime.now(timezone.utc)
    observed_at = now_observed_at(moment)
    sequence = event_sequence(moment)

    observation = collect_observation(
        cloud_url=args.cloud_url, recovery_state_path=args.recovery_state
    )
    report = build_telemetry(
        observation,
        resource_id=args.resource_id,
        sequence=sequence,
        observed_at=observed_at,
    )
    write_report(args.output, report)
    if report["quality"] != "good":
        failure = dict(report)
        failure["channel_id"] = "system.last_failure"
        failure.pop("payload_hash", None)
        write_report(args.last_failure, parse_resource_telemetry(failure))
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))

    # Recorded last, on purpose. The diagnosis an operator reads is on disk and
    # on stdout before this runs, so a journal that cannot be written costs the
    # event and nothing else — and it is still said out loud rather than
    # swallowed, because a device that has quietly stopped reporting degradation
    # is indistinguishable from a device that is well.
    result = _record_degradation(
        observation,
        report,
        resource_id=args.resource_id,
        sequence=sequence,
        observed_at=observed_at,
        journal_path=args.event_journal,
    )
    if result == 0:
        _commit_recovery_state(args.recovery_state, observation.service_recoveries)
    return result


def _record_degradation(
    observation: DiagnosticObservation,
    report: Mapping[str, Any],
    *,
    resource_id: str,
    sequence: int,
    observed_at: str,
    journal_path: Path,
) -> int:
    """Record the primary degradation and any independent recovery transition."""
    primary_event = degradation_event(
        observation,
        resource_id=resource_id,
        sequence=sequence,
        observed_at=observed_at,
        telemetry=report,
    )
    recovery_event = _recovery_event(
        observation,
        resource_id=resource_id,
        sequence=sequence,
        observed_at=observed_at,
        correlation_id=str(report["payload_hash"]),
    )
    events = [event for event in (primary_event, recovery_event) if event is not None]
    events = list({event["event_id"]: event for event in events}.values())
    if not events:
        return 0
    _RECOVERY_THREAD_LOCK.acquire()
    try:
        journal_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        directory_fd = os.open(
            journal_path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        lock_fd = os.open(
            f".{journal_path.name}.recovery.lock",
            os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW,
            0o600,
            dir_fd=directory_fd,
        )
        os.close(directory_fd)
        fcntl.flock(lock_fd, fcntl.LOCK_EX)
        journal = DeviceEventJournal(journal_path)
        existing_ids = (
            {record["event"]["event_id"] for record in journal.read_all()}
            if journal_path.exists()
            else set()
        )
        for event in events:
            if event["reason_code"] == RECOVERY_REASON and event["event_id"] in existing_ids:
                continue
            journal.append(event)
    except (DeviceEventError, OSError) as exc:
        # The refusal text is this repository's own, not a raw log or a
        # credential, and it goes to stderr for an operator rather than into an
        # event: an event about a journal that will not accept events has
        # nowhere to be written.
        print(
            f"robot-doctor: the degradation event could not be recorded in "
            f"{journal_path}: {exc}",
            file=sys.stderr,
        )
        return EXIT_EVENT_NOT_RECORDED
    finally:
        if "lock_fd" in locals():
            os.close(lock_fd)
        _RECOVERY_THREAD_LOCK.release()
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
