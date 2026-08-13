"""Read-only local portal for the latest robot diagnostic snapshot."""

from __future__ import annotations

import argparse
import contextlib
import html
import json
import socket
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .resource_agent import ResourceContractError, parse_resource_telemetry
from .robot_doctor import DEFAULT_OUTPUT

_REASONS = {
    "healthy": "Robot, network, and Cloud path are healthy.",
    "provisioning_degraded": "Initial provisioning is degraded; inspect cloud-init before Wi-Fi.",
    "wifi_interface_missing": "The Wi-Fi interface is missing from the operating system.",
    "wifi_interface_down": "The Wi-Fi interface exists but is down.",
    "wifi_not_associated": "Wi-Fi is not associated with any configured network.",
    "wifi_no_address": "Wi-Fi associated, but DHCP did not provide an address.",
    "default_route_missing": "The robot has an address but no default route.",
    "dns_unavailable": "The default route works, but DNS resolution does not.",
    "cloud_endpoint_unconfigured": "No Cloud origin is configured on the robot.",
    "cloud_unreachable": "The network is up, but the configured Cloud origin is unreachable.",
    "robot_service_unhealthy": (
        "The network is healthy, but a required robot service is not active."
    ),
    "managed_service_recovered": (
        "A managed service is active now after a watchdog recovery."
    ),
}


def load_report(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.stat().st_size > 128 * 1024:
        raise ValueError("diagnostic report is missing or too large")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return parse_resource_telemetry(value)
    except (UnicodeDecodeError, json.JSONDecodeError, ResourceContractError) as exc:
        raise ValueError("diagnostic report is invalid") from exc


#: The robot-doctor timer runs every 60s (OnUnitActiveSec in
#: deploy/systemd/flyto-robot-doctor.timer), so a live snapshot is at most a
#: minute old. Five missed runs is no longer a hiccup; it means the writer
#: stopped, which is exactly the failure the portal exists to survive.
STALE_AFTER_SECONDS = 300.0


def _snapshot_age(observed_at: Any, now: datetime) -> float | None:
    """Seconds since the snapshot was written, or None if that is unknowable."""
    if not isinstance(observed_at, str) or not observed_at:
        return None
    try:
        written = datetime.fromisoformat(observed_at.replace("Z", "+00:00"))
    except ValueError:
        return None
    if written.tzinfo is None:
        written = written.replace(tzinfo=timezone.utc)
    return (now - written).total_seconds()


def report_view(
    report: Mapping[str, Any],
    *,
    now: datetime | None = None,
    stale_after_seconds: float = STALE_AFTER_SECONDS,
) -> dict[str, Any]:
    """Render one snapshot, saying plainly whether it is still current.

    The age was previously left for the reader to work out from a timestamp at
    the foot of the page, while the headline said GOOD / healthy / no action
    required. A frozen latest.json therefore looked exactly like a live one —
    and a frozen latest.json is the normal consequence of the diagnostic writer
    dying, which is precisely when someone opens this page.
    """
    payload = report.get("payload", {})
    reason = str(payload.get("primary_reason_code", "diagnostic_pending"))
    summary = _REASONS.get(reason, "Diagnostic snapshot is not ready.")
    action_codes = list(payload.get("action_codes", []))
    quality = report.get("quality")
    recovery = payload.get("recovery", {})
    service_restarts = recovery.get("service_restarts", {})
    watchdog_total = service_restarts.get("current_boot_watchdog_total", 0)
    if reason == "healthy" and isinstance(watchdog_total, int) and watchdog_total > 0:
        summary = "Services are active now; watchdog recovery history exists earlier this boot."

    age = _snapshot_age(report.get("observed_at"), now or datetime.now(timezone.utc))
    # An unreadable or absent timestamp is not evidence of freshness. Treat not
    # knowing the age the same as knowing it is too old.
    stale = age is None or age > stale_after_seconds
    if stale:
        quality = "stale"
        summary = (
            f"This snapshot is {age / 60:.0f} minutes old and may no longer "
            "describe the robot. Everything below predates it."
            if age is not None
            else "This snapshot carries no readable timestamp, so it cannot be "
            "shown to be current. Everything below may be out of date."
        )
        action_codes = ["inspect_diagnostic_timer", *action_codes]

    return {
        "observed_at": report.get("observed_at"),
        "age_seconds": age,
        "stale": stale,
        "quality": quality,
        "reason_code": reason,
        "summary": summary,
        "action_codes": action_codes,
        "network": payload.get("network", {}),
        "services": payload.get("services", {}),
        "recovery": recovery,
    }


def render_html(view: Mapping[str, Any]) -> str:
    reason = html.escape(str(view["reason_code"]))
    summary = html.escape(str(view["summary"]))
    quality = html.escape(str(view["quality"]))
    observed = html.escape(str(view["observed_at"]))
    actions = "".join(
        f"<li><code>{html.escape(str(item))}</code></li>" for item in view["action_codes"]
    )
    # A stale reading must not be able to look like a live one at a glance. The
    # timestamp at the foot of the page was the only signal, and nobody reads a
    # timestamp when the headline already says the robot is fine.
    banner = (
        '<p class="stale">This reading is not current.</p>' if view.get("stale") else ""
    )
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Flyto Robot Recovery</title><style>
body{{font:16px system-ui;max-width:760px;margin:3rem auto;padding:0 1rem;
background:#0b1020;color:#edf2ff}}
main{{background:#151d34;border:1px solid #314166;border-radius:18px;padding:1.5rem}}
code{{color:#83d6ff}} .quality{{text-transform:uppercase;color:#8ef0b1}}
.stale{{background:#4a2020;border:1px solid #a24b4b;border-radius:10px;
padding:.75rem 1rem;color:#ffd9d9;font-weight:600;margin:0 0 1rem}}
</style></head><body><main><h1>Flyto Robot Recovery</h1>
{banner}<p class="quality">{quality}</p><h2>{reason}</h2><p>{summary}</p>
<h3>Recommended actions</h3><ul>{actions or "<li>No action required.</li>"}</ul>
<p>Observed: {observed}</p><p><a href="/v1/diagnostics">Machine-readable JSON</a></p>
</main></body></html>"""


def handler_for(report_path: Path):
    class RecoveryHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            if self.path == "/health":
                self._send(200, b'{"status":"ok"}\n', "application/json")
                return
            try:
                view = report_view(load_report(report_path))
            except ValueError as exc:
                view = {
                    "observed_at": None,
                    "age_seconds": None,
                    # No readable report at all is the least current state there
                    # is; say so with the same field the age check uses, so a
                    # consumer has one thing to look at rather than two.
                    "stale": True,
                    "quality": "stale",
                    "reason_code": "diagnostic_pending",
                    "summary": str(exc),
                    "action_codes": ["wait_for_first_snapshot"],
                }
            if self.path == "/v1/diagnostics":
                self._send(
                    200,
                    (json.dumps(view, sort_keys=True) + "\n").encode(),
                    "application/json",
                )
            elif self.path == "/":
                self._send(200, render_html(view).encode(), "text/html; charset=utf-8")
            else:
                self._send(404, b'{"detail":"not found"}\n', "application/json")

        def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            self._send(405, b'{"detail":"read only"}\n', "application/json")

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy", "default-src 'none'; style-src 'unsafe-inline'"
            )
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args: Any) -> None:
            return

    return RecoveryHandler


class FreeBindHTTPServer(ThreadingHTTPServer):
    """An HTTP server that can listen on an address that does not exist yet.

    The recovery portal binds the USB gadget address, 10.77.0.1. That address
    only exists while a cable is plugged in, so with no cable the bind fails
    with EADDRNOTAVAIL and the service dies. systemd restarts it, it dies
    again, and the unit sits in auto-restart reporting ``activating`` — which
    reads as "coming up" rather than "has failed 760 times in three hours".

    That is what happened: the portal an operator reaches for when the robot is
    unreachable was itself dead, silently, and the health check did not watch
    it. Binding 0.0.0.0 instead would have fixed the crash by publishing the
    diagnostics on Wi-Fi, which is worse than the disease.

    ``IP_FREEBIND`` is the option for exactly this: hold the port for an
    address the host does not have yet, and start answering the moment it
    appears. Linux only; elsewhere this degrades to an ordinary bind.
    """

    #: Not exposed by Python's socket module on every platform or version.
    _IP_FREEBIND = getattr(socket, "IP_FREEBIND", 15)

    def server_bind(self) -> None:
        # Not Linux, or a kernel without it: bind normally, and let a real
        # failure surface rather than pretending it was asked for.
        with contextlib.suppress(OSError):
            self.socket.setsockopt(socket.IPPROTO_IP, self._IP_FREEBIND, 1)
        super().server_bind()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Serve read-only robot diagnostics over USB")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--report", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if not (1 <= args.port <= 65535):
        parser.error("port must be between 1 and 65535")
    server = FreeBindHTTPServer((args.host, args.port), handler_for(args.report))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
