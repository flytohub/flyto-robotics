"""SIMULATION cargo bridge: publish the simulated tray's cargo state to ROS 2.

The live Gazebo/Nav2 twin has no cargo sensor. This process holds the
symbolic cargo state of the simulated tray (sim/fetch_return/cargo_tray.py)
and publishes it over rosbridge as std_msgs/msg/Bool on
``$FLYTO_ROS2_CARGO_TOPIC`` (default ``/flyto/cargo_present``), latched and
re-sent at about 2 Hz, so the robot's adapter reads ``sensing.cargo_present``
from a topic exactly as it would from a real tray sensor.

Inputs:
- robot pose and speed from ``/odom`` on the same rosbridge;
- the fixed arm's place, from the MuJoCo arm host's simulation control
  (``GET /sim/truth``, rising edge of ``inside``) and/or a loopback HTTP
  control endpoint on this bridge (``POST /sim/cargo/place``,
  ``POST /sim/cargo/unload``, ``GET /sim/cargo``) for glue that has no truth
  port.

The value is simulation state, not a measurement. It is labelled so in every
HTTP answer and in the README; the ROS message itself is a bare Bool by the
shared contract.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

if __package__ in ("", None):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from flyto_robotics.cargo_sensor import configured_topic
from sim.fetch_return.arm_place_signal import ArmTruthWatcher, http_fetch, truth_url
from sim.fetch_return.cargo_tray import CargoTray, Station
from sim.fetch_return.rosbridge_link import BOOL_TYPE, ODOMETRY_TYPE, RosbridgeLink

DEFAULT_CONTROL_PORT = 7110

# configured_topic is the adapter's one definition (flyto_robotics.cargo_sensor),
# so the twin publishes exactly where the adapter reads.


def configured_odom_topic() -> str:
    return os.getenv("FLYTO_ROS2_ODOM_TOPIC", "").strip() or "/odom"


class CargoBridge:
    """Wires odometry and the arm's place signal into the tray and publishes it."""

    def __init__(
        self,
        *,
        link: RosbridgeLink,
        tray: CargoTray,
        topic: str,
        arm: ArmTruthWatcher | None = None,
        odom_topic: str = "/odom",
    ) -> None:
        self.link, self.tray, self.topic, self.arm = link, tray, topic, arm
        self.odom_topic = odom_topic
        self.last_error: str | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self.link.subscribe(self.odom_topic, ODOMETRY_TYPE, self.tray.observe_odom)
        self.link.advertise(self.topic, BOOL_TYPE, latch=True)
        self.link.connect()

    def tick(self) -> bool:
        """One cycle: read the arm, then publish the tray. Returns the published value."""
        if self.arm is not None and self.arm.poll():
            self.tray.report_place(source="arm_sim_truth")
        value = self.tray.cargo_present
        try:
            if not self.link.connected:
                self.link.connect()
            self.link.publish(self.topic, {"data": value})
            self.last_error = None
        except Exception as error:  # rosbridge down: retry next tick
            self.last_error = f"{type(error).__name__}: {error}"[:200]
        return value

    def run(self, *, rate_hz: float = 2.0) -> None:
        period = 1.0 / rate_hz
        while not self._stop.is_set():
            self.tick()
            self._stop.wait(period)

    def stop(self) -> None:
        self._stop.set()
        self.link.close()

    def state(self) -> dict[str, Any]:
        return {
            **self.tray.snapshot(),
            "topic": self.topic,
            "message_type": BOOL_TYPE,
            "rosbridge_connected": self.link.connected,
            "rosbridge_error": self.last_error,
            "arm_signal_error": None if self.arm is None else self.arm.last_error,
        }


def serve_control(bridge: CargoBridge, *, host: str, port: int) -> ThreadingHTTPServer:
    """Loopback-only HTTP control for arm-sim glue. Simulation harness only."""
    if not ipaddress.ip_address(host).is_loopback:
        raise ValueError("the simulation cargo control endpoint binds loopback only")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: Any) -> None:
            pass

        def _send(self, code: int, body: dict[str, Any]) -> None:
            raw = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self) -> None:
            if self.path == "/sim/cargo":
                return self._send(200, bridge.state())
            self._send(404, {"error": "not_found"})

        def do_POST(self) -> None:
            self.rfile.read(int(self.headers.get("content-length") or 0))
            if self.path == "/sim/cargo/place":
                event = bridge.tray.report_place(source="http_control")
            elif self.path == "/sim/cargo/unload":
                event = bridge.tray.unload(source="http_control")
            else:
                return self._send(404, {"error": "not_found"})
            self._send(200, {"event": event, "state": bridge.state()})

    return ThreadingHTTPServer((host, port), Handler)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--station-x", type=float, required=True, help="odom frame, metres")
    parser.add_argument("--station-y", type=float, required=True, help="odom frame, metres")
    parser.add_argument("--station-tolerance-m", type=float, default=0.10)
    parser.add_argument(
        "--rosbridge-url",
        default=os.getenv("FLYTO_ROSBRIDGE_URL", "").strip() or "ws://127.0.0.1:19090",
    )
    parser.add_argument("--topic", default=configured_topic())
    parser.add_argument(
        "--arm-source",
        choices=("sim-truth", "http-only"),
        default="sim-truth",
        help="sim-truth polls the arm host's GET /sim/truth; http-only waits for POSTs",
    )
    parser.add_argument(
        "--control-port",
        type=int,
        default=int(os.getenv("FLYTO_SIM_CARGO_BRIDGE_PORT", str(DEFAULT_CONTROL_PORT))),
    )
    parser.add_argument("--rate-hz", type=float, default=2.0)
    args = parser.parse_args(argv)

    tray = CargoTray(Station(args.station_x, args.station_y, args.station_tolerance_m))
    arm = ArmTruthWatcher(fetch=http_fetch(truth_url())) if args.arm_source == "sim-truth" else None
    bridge = CargoBridge(
        link=RosbridgeLink(args.rosbridge_url),
        tray=tray,
        topic=args.topic,
        arm=arm,
        odom_topic=configured_odom_topic(),
    )
    server = serve_control(bridge, host="127.0.0.1", port=args.control_port)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(
        f"[cargo-bridge SIMULATION] {args.topic} (std_msgs/msg/Bool, latched) via "
        f"{args.rosbridge_url}; station={tray.station.as_dict()}; arm={args.arm_source}; "
        f"control=http://127.0.0.1:{args.control_port}/sim/cargo",
        flush=True,
    )
    try:
        bridge.start()
    except Exception as error:
        print(f"[cargo-bridge SIMULATION] rosbridge not reachable yet: {error}", flush=True)
    worker = threading.Thread(target=bridge.run, kwargs={"rate_hz": args.rate_hz}, daemon=True)
    worker.start()
    last: tuple[Any, ...] = ()
    try:
        while True:
            state = bridge.state()
            now = (
                state["cargo_present"],
                state["rosbridge_connected"],
                len(state["events"]),
            )
            if now != last:
                event = state["events"][-1] if state["events"] else None
                print(
                    f"[cargo-bridge SIMULATION] {json.dumps({**state, 'events': event})}",
                    flush=True,
                )
                last = now
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    finally:
        bridge.stop()
        server.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
