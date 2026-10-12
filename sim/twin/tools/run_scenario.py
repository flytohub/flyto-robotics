"""Twin only: run one `scenarios/twin/*.json` scenario through the adapter and judge it.

    python3 sim/twin/tools/run_scenario.py SCENARIO.json [--url ws://127.0.0.1:19090]
        [--output verdict.json]

Started by `scripts/run_twin_scenario.sh`, which brings the twin up in the
scenario's world and start pose first. The call goes through the formal
Generic ROS 2 Adapter over rosbridge, exactly as a host sends it. Every action
goal and velocity command the adapter sends is recorded on the way out; after
the call returns, odometry is sampled for the scenario's quiet window. The
verdict comes from `scenario_check.evaluate`; the exit status is 0 only when
every check passed.

Simulation only, always: FLYTO_ROS2_DEPLOYMENT_MODE is forced to simulation,
so a graph without /clock (the physical robot) refuses the motion. The floor
override FLYTO_ROS2_MIN_CLEARANCE_M is removed, so the adapter applies its
0.35 m default and the verdict checks that it did.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

os.environ["FLYTO_ROS2_DEPLOYMENT_MODE"] = "simulation"
os.environ.pop("FLYTO_ROS2_MIN_CLEARANCE_M", None)

TOOLS = Path(__file__).resolve().parent
REPOSITORY = TOOLS.parents[2]
sys.path.insert(0, str(REPOSITORY))
sys.path.insert(0, str(TOOLS))
from scenario_check import evaluate, load_scenario  # noqa: E402

from flyto_robotics.adapter_contract import CallRequest  # noqa: E402
from flyto_robotics.generic_ros2_adapter import (  # noqa: E402
    GenericROS2Adapter,
    RosbridgeROS2Backend,
)

READY_SECONDS = 180.0


def _nonzero_velocity(message: Any) -> bool:
    """Whether a Twist or TwistStamped message commands any motion."""

    if not isinstance(message, dict):
        return False
    twist = message.get("twist") if isinstance(message.get("twist"), dict) else message
    for part in ("linear", "angular"):
        vector = twist.get(part) or {}
        for axis in ("x", "y", "z"):
            value = vector.get(axis)
            if isinstance(value, (int, float)) and math.isfinite(value) and abs(value) > 1e-6:
                return True
    return False


class Recorder:
    """Wraps the backend's send so every outgoing goal and command is recorded."""

    def __init__(self, backend: RosbridgeROS2Backend) -> None:
        self._lock = threading.Lock()
        self.result_returned = False
        self.goals: list[dict[str, Any]] = []
        self.motion_commands_after_result: list[dict[str, Any]] = []
        self._send = backend._send
        backend._send = self.send

    def send(self, message: dict[str, Any]) -> None:
        with self._lock:
            if message.get("op") == "send_action_goal":
                self.goals.append(
                    {
                        "action": message.get("action"),
                        "action_type": message.get("action_type"),
                        "after_result": self.result_returned,
                    }
                )
            elif (
                message.get("op") == "publish"
                and self.result_returned
                and _nonzero_velocity(message.get("msg"))
            ):
                self.motion_commands_after_result.append(
                    {"topic": message.get("topic"), "msg": message.get("msg")}
                )
        self._send(message)


def _connect(adapter: GenericROS2Adapter) -> None:
    """Connect, retrying while rosbridge starts (`reconnect` never raises)."""

    deadline = time.monotonic() + READY_SECONDS
    while True:
        status = adapter.reconnect()
        if status.get("rosbridge_ok"):
            return
        if time.monotonic() > deadline:
            raise RuntimeError(f"rosbridge did not accept a connection: {status}")
        time.sleep(3.0)


def _wait_ready(adapter: GenericROS2Adapter, capability_id: str) -> dict[str, Any]:
    deadline = time.monotonic() + READY_SECONDS
    observation: dict[str, Any] = {}
    capabilities: list[str] = []
    while time.monotonic() < deadline:
        observation = dict(adapter.backend.observation())
        capabilities = sorted(item.capability_id for item in adapter.describe())
        pose = observation.get("pose") or {}
        scan = observation.get("range") or {}
        if (
            capability_id in capabilities
            and pose.get("x") is not None
            and scan.get("minimum_range_m") is not None
        ):
            return {"capabilities": capabilities, "pose": pose, "range_m": scan["minimum_range_m"]}
        if capability_id not in capabilities:
            # Discovery is cached until a graph event; re-read the graph.
            _connect(adapter)
        time.sleep(2.0)
    raise RuntimeError(
        f"twin not ready for {capability_id} within {READY_SECONDS:.0f} s; "
        f"capabilities={capabilities} observation keys={sorted(observation)}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--url", default="ws://127.0.0.1:19090")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    scenario = load_scenario(args.scenario, twin_root=TOOLS.parent)
    call = scenario["call"]
    expected = scenario["expectations"]

    backend = RosbridgeROS2Backend(url=args.url)
    recorder = Recorder(backend)
    adapter = GenericROS2Adapter(backend=backend, resource_id="twin-scenario")
    _connect(adapter)
    report: dict[str, Any] = {"scenario_id": scenario["scenario_id"]}
    try:
        report["ready"] = _wait_ready(adapter, call["capability_id"])
        print("ready:", json.dumps(report["ready"]), flush=True)
        started = time.monotonic()
        result = adapter.invoke(
            CallRequest(
                call_id=f"twin-scenario-{int(time.time())}",
                capability_id=call["capability_id"],
                arguments=dict(call["arguments"]),
                deadline_seconds=float(call["deadline_seconds"]),
            )
        )
        recorder.result_returned = True
        report["call_seconds"] = round(time.monotonic() - started, 3)
        print("result:", result.outcome, "|", (result.detail or "")[:300], flush=True)
        poses: list[dict[str, Any]] = []
        quiet_until = time.monotonic() + float(expected["quiet_after_seconds"])
        while time.monotonic() < quiet_until:
            pose = (adapter.backend.observation().get("pose") or {})
            poses.append({key: pose.get(key) for key in ("x", "y", "yaw")})
            time.sleep(0.5)
        observed = {
            "outcome": result.outcome,
            "detail": result.detail,
            "evidence": result.evidence or {},
            "goals": recorder.goals,
            "motion_commands_after_result": recorder.motion_commands_after_result,
            "poses_after_result": poses,
        }
        verdict = evaluate(scenario, observed)
        report.update({"observed": observed, "verdict": verdict})
    finally:
        try:
            adapter.safe_stop()
        except Exception as error:  # noqa: BLE001
            print("safe_stop:", error, flush=True)
        adapter.disconnect()
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(report, indent=2, sort_keys=True, default=str) + "\n",
                encoding="utf-8",
            )
    print(json.dumps(report["verdict"], indent=2, sort_keys=True), flush=True)
    return 0 if report["verdict"]["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
