"""Read-only local portal for the latest robot diagnostic snapshot."""

from __future__ import annotations

import argparse
import html
import json
from collections.abc import Mapping, Sequence
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
}


def load_report(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.stat().st_size > 128 * 1024:
        raise ValueError("diagnostic report is missing or too large")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return parse_resource_telemetry(value)
    except (UnicodeDecodeError, json.JSONDecodeError, ResourceContractError) as exc:
        raise ValueError("diagnostic report is invalid") from exc


def report_view(report: Mapping[str, Any]) -> dict[str, Any]:
    payload = report.get("payload", {})
    reason = str(payload.get("primary_reason_code", "diagnostic_pending"))
    return {
        "observed_at": report.get("observed_at"),
        "quality": report.get("quality"),
        "reason_code": reason,
        "summary": _REASONS.get(reason, "Diagnostic snapshot is not ready."),
        "action_codes": payload.get("action_codes", []),
        "network": payload.get("network", {}),
        "services": payload.get("services", {}),
        "recovery": payload.get("recovery", {}),
    }


def render_html(view: Mapping[str, Any]) -> str:
    reason = html.escape(str(view["reason_code"]))
    summary = html.escape(str(view["summary"]))
    quality = html.escape(str(view["quality"]))
    observed = html.escape(str(view["observed_at"]))
    actions = "".join(
        f"<li><code>{html.escape(str(item))}</code></li>" for item in view["action_codes"]
    )
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Flyto Robot Recovery</title><style>
body{{font:16px system-ui;max-width:760px;margin:3rem auto;padding:0 1rem;
background:#0b1020;color:#edf2ff}}
main{{background:#151d34;border:1px solid #314166;border-radius:18px;padding:1.5rem}}
code{{color:#83d6ff}} .quality{{text-transform:uppercase;color:#8ef0b1}}
</style></head><body><main><h1>Flyto Robot Recovery</h1>
<p class="quality">{quality}</p><h2>{reason}</h2><p>{summary}</p>
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


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Serve read-only robot diagnostics over USB")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--report", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if not (1 <= args.port <= 65535):
        parser.error("port must be between 1 and 65535")
    server = ThreadingHTTPServer((args.host, args.port), handler_for(args.report))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
