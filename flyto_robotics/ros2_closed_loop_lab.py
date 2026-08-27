"""One-shot live Nav2/Gazebo closed-loop evidence runner."""

from __future__ import annotations

import json
import math
import sys
import time
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .contracts import write_json_atomic
from .resource_binding import load_resource_plan
from .ros2_action_executor import (
    execute_rclpy_navigation,
    prepare_authorized_navigation,
)
from .ros2_execution import authorize_ros2_execution
from .ros2_execution_evidence import (
    build_ros2_execution_evidence,
    evaluate_closed_loop_evidence,
)
from .ros2_pairing import load_ros2_adapter_manifest, verify_ros2_pairing
from .ros2_probe_node import RclpyGraphProbe, collect_ros2_runtime_snapshot

LIFECYCLE_SHUTDOWN_TIMEOUT_SECONDS = 15.0
LIFECYCLE_SHUTDOWN_READY_TIMEOUT_SECONDS = 5.0
LIFECYCLE_MANAGER_SERVICE_ATTEMPTS = 3
LIFECYCLE_SERVICE_ATTEMPT_TIMEOUT_SECONDS = 2.0
LIFECYCLE_PREPARATION_PARTICIPANT_ATTEMPT_SECONDS = 12.0
LIFECYCLE_RESUME_TIMEOUT_SECONDS = 15.0
LIFECYCLE_MANAGER_SERVICES = (
    "/lifecycle_manager_navigation/manage_nodes",
    "/map_lifecycle_manager/manage_nodes",
)
NAVIGATION_MANAGER_SERVICE = LIFECYCLE_MANAGER_SERVICES[0]
NAVIGATION_LIFECYCLE_NODES = (
    "/controller_server",
    "/smoother_server",
    "/planner_server",
    "/behavior_server",
    "/bt_navigator",
)
# Stable values from lifecycle_msgs keep the recovery state machine testable
# on development hosts that do not have a ROS installation sourced.
LIFECYCLE_STATE_UNCONFIGURED = 1
LIFECYCLE_STATE_INACTIVE = 2
LIFECYCLE_STATE_ACTIVE = 3
LIFECYCLE_TRANSITION_CONFIGURE = 1
LIFECYCLE_TRANSITION_CLEANUP = 2
LIFECYCLE_TRANSITION_ACTIVATE = 3
LIFECYCLE_TRANSITION_DEACTIVATE = 4
LIFECYCLE_MANAGER_RESUME = 2


def _load_json(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _spin_discovery_once(node: Any, timeout_seconds: float) -> None:
    import rclpy

    rclpy.spin_once(node, timeout_sec=timeout_seconds)


def _fresh_service_call(
    node: Any,
    service_type: Any,
    service_name: str,
    request: Any,
    *,
    deadline: float,
    clock: Callable[[], float] = time.monotonic,
) -> Any:
    """Call a ROS service with a fresh client per bounded transient attempt."""

    last_error = "service unavailable"
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            break
        attempt_deadline = min(
            deadline,
            clock() + LIFECYCLE_SERVICE_ATTEMPT_TIMEOUT_SECONDS,
        )
        client = node.create_client(service_type, service_name)
        future = None
        try:
            wait_timeout = max(0.0, attempt_deadline - clock())
            if not client.wait_for_service(timeout_sec=wait_timeout):
                last_error = "service unavailable"
                continue
            try:
                future = client.call_async(request)
            except RuntimeError as exc:
                last_error = str(exc)
                continue
            while not future.done():
                remaining = attempt_deadline - clock()
                if remaining <= 0:
                    future.cancel()
                    last_error = "response timed out"
                    break
                _spin_discovery_once(node, min(0.1, remaining))
            if future.done():
                try:
                    response = future.result()
                except RuntimeError as exc:
                    last_error = str(exc)
                    continue
                if response is not None:
                    return response
                last_error = "response missing"
        finally:
            node.destroy_client(client)
    raise RuntimeError(f"lifecycle service failed: {service_name}: {last_error}")


def _read_lifecycle_state(
    node: Any,
    node_name: str,
    *,
    deadline: float,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    from lifecycle_msgs.srv import GetState

    response = _fresh_service_call(
        node,
        GetState,
        node_name.rstrip("/") + "/get_state",
        GetState.Request(),
        deadline=deadline,
        clock=clock,
    )
    return int(response.current_state.id)


def _change_lifecycle_state(
    node: Any,
    node_name: str,
    transition_id: int,
    target_state_id: int,
    *,
    deadline: float,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState

    request = ChangeState.Request()
    request.transition = Transition(id=transition_id)
    response = _fresh_service_call(
        node,
        ChangeState,
        node_name.rstrip("/") + "/change_state",
        request,
        deadline=deadline,
        clock=clock,
    )
    # A lost response can report false after the server already transitioned,
    # so actual observed state remains the authority.
    observed = _read_lifecycle_state(
        node,
        node_name,
        deadline=deadline,
        clock=clock,
    )
    if response.success is not True and observed != target_state_id:
        raise RuntimeError(f"lifecycle transition was rejected: {node_name}")
    if observed != target_state_id:
        raise RuntimeError(f"lifecycle transition did not converge: {node_name}")


def _normalize_navigation_nodes_inactive(
    read_state: Callable[[str], int],
    change_state: Callable[[str, int, int], None],
) -> None:
    """Normalize a partially completed Nav2 startup without masking state."""

    stable_states = {
        LIFECYCLE_STATE_UNCONFIGURED,
        LIFECYCLE_STATE_INACTIVE,
        LIFECYCLE_STATE_ACTIVE,
    }
    for node_name in NAVIGATION_LIFECYCLE_NODES:
        observed = read_state(node_name)
        if observed not in stable_states:
            raise RuntimeError(f"navigation lifecycle state is unknown: {node_name}")
        if observed == LIFECYCLE_STATE_UNCONFIGURED:
            change_state(
                node_name,
                LIFECYCLE_TRANSITION_CONFIGURE,
                LIFECYCLE_STATE_INACTIVE,
            )
        elif observed == LIFECYCLE_STATE_ACTIVE:
            # STARTUP creates bonds during activation. If any node is already
            # active, the manager may own partial bond state that direct
            # transitions cannot safely clear.
            raise RuntimeError(
                f"navigation lifecycle recovery found partial activation: {node_name}"
            )
        elif observed != LIFECYCLE_STATE_INACTIVE:
            raise RuntimeError(f"navigation lifecycle normalization failed: {node_name}")


def _request_lifecycle_manager_command(
    node: Any,
    client: Any,
    command: int,
    *,
    timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> bool:
    from nav2_msgs.srv import ManageLifecycleNodes

    request = ManageLifecycleNodes.Request()
    request.command = command
    deadline = clock() + timeout_seconds
    last_error = "service unavailable"
    for attempt in range(LIFECYCLE_MANAGER_SERVICE_ATTEMPTS):
        remaining = deadline - clock()
        if remaining <= 0:
            break
        command_client = (
            client
            if attempt == 0
            else node.create_client(ManageLifecycleNodes, NAVIGATION_MANAGER_SERVICE)
        )
        owned_client = command_client is not client
        try:
            if not command_client.wait_for_service(timeout_sec=remaining):
                last_error = "service unavailable"
                continue
            try:
                future = command_client.call_async(request)
            except RuntimeError as exc:
                # A locally rejected send is known not to have reached the
                # manager, so recreating this client cannot duplicate a state
                # transition. Once a future exists, ambiguity fails closed.
                last_error = str(exc)
                continue
            while not future.done():
                remaining = deadline - clock()
                if remaining <= 0:
                    future.cancel()
                    raise RuntimeError("lifecycle manager request timed out")
                _spin_discovery_once(node, min(0.1, remaining))
            response = future.result()
            return bool(response is not None and response.success is True)
        finally:
            if owned_client:
                node.destroy_client(command_client)
    raise RuntimeError(f"lifecycle manager request failed: {last_error}")


def _start_navigation_lifecycle(
    node: Any,
    manager_client: Any,
    *,
    deadline: float,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Configure safely, then let the official manager activate and bond."""

    if deadline - clock() <= 0:
        raise RuntimeError("navigation lifecycle startup budget expired")
    _prepare_navigation_lifecycle(
        node,
        deadline=deadline,
        clock=clock,
    )
    remaining = deadline - clock()
    if remaining <= 0:
        raise RuntimeError("navigation lifecycle recovery exhausted its readiness budget")
    if _request_lifecycle_manager_command(
        node,
        manager_client,
        LIFECYCLE_MANAGER_RESUME,
        timeout_seconds=min(LIFECYCLE_RESUME_TIMEOUT_SECONDS, remaining),
        clock=clock,
    ):
        return
    # A failed RESUME can leave manager-owned bonds for nodes that activated
    # before the transient. Directly retrying would preserve those stale bond
    # objects, so activation failure remains a hard fail-closed boundary.
    raise RuntimeError("navigation lifecycle recovery resume was rejected")


def _prepare_navigation_lifecycle(
    node: Any,
    *,
    deadline: float,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Normalize every navigation node before the manager creates clients."""

    if deadline - clock() <= 0:
        raise RuntimeError("navigation lifecycle preparation budget expired")
    _normalize_navigation_nodes_inactive(
        lambda node_name: _read_lifecycle_state(
            node,
            node_name,
            deadline=deadline,
            clock=clock,
        ),
        lambda node_name, transition_id, target_state_id: _change_lifecycle_state(
            node,
            node_name,
            transition_id,
            target_state_id,
            deadline=deadline,
            clock=clock,
        ),
    )


def run_lifecycle_preparation(
    node: Any,
    *,
    timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> dict[str, Any]:
    """Run the one-shot pre-manager lifecycle phase inside one wall-clock budget."""

    timeout_seconds = float(timeout_seconds)
    if not 5.0 <= timeout_seconds <= 60.0:
        raise ValueError("lifecycle preparation timeout must be between 5 and 60")
    started_at = clock()
    _prepare_navigation_lifecycle(
        node,
        deadline=started_at + timeout_seconds,
        clock=clock,
    )
    return {
        "prepared": True,
        "navigation_nodes": list(NAVIGATION_LIFECYCLE_NODES),
        "timeout_seconds": timeout_seconds,
    }


def run_lifecycle_preparation_with_participant_retries(
    node: Any,
    *,
    timeout_seconds: float,
    reset_participant: Callable[[Any], Any],
    clock: Callable[[], float] = time.monotonic,
) -> dict[str, Any]:
    """Retry stale DDS participants inside one unchanged preparation budget."""

    timeout_seconds = float(timeout_seconds)
    if not 5.0 <= timeout_seconds <= 60.0:
        raise ValueError("lifecycle preparation timeout must be between 5 and 60")
    deadline = clock() + timeout_seconds
    max_attempts = max(
        1,
        math.ceil(timeout_seconds / LIFECYCLE_PREPARATION_PARTICIPANT_ATTEMPT_SECONDS),
    )
    current_node = node
    last_error: RuntimeError | None = None
    for attempt in range(1, max_attempts + 1):
        remaining = deadline - clock()
        if remaining <= 0:
            break
        attempt_deadline = min(
            deadline,
            clock() + LIFECYCLE_PREPARATION_PARTICIPANT_ATTEMPT_SECONDS,
        )
        try:
            _prepare_navigation_lifecycle(
                current_node,
                deadline=attempt_deadline,
                clock=clock,
            )
        except RuntimeError as exc:
            last_error = exc
            if attempt >= max_attempts or deadline - clock() <= 0:
                raise
            current_node = reset_participant(current_node)
            continue
        return {
            "prepared": True,
            "navigation_nodes": list(NAVIGATION_LIFECYCLE_NODES),
            "timeout_seconds": timeout_seconds,
            "participant_attempts": attempt,
        }
    if last_error is not None:
        raise last_error
    raise RuntimeError("navigation lifecycle preparation budget expired")


def _await_lifecycle_shutdown(
    client: Any,
    request: Any,
    *,
    service_name: str,
    spin_once: Callable[[float], None],
    timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Require one lifecycle manager shutdown inside one absolute budget."""

    deadline = clock() + timeout_seconds
    remaining = deadline - clock()
    if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
        raise RuntimeError(f"lifecycle shutdown service is unavailable: {service_name}")
    future = client.call_async(request)
    while not future.done():
        remaining = deadline - clock()
        if remaining <= 0:
            future.cancel()
            raise RuntimeError(f"lifecycle shutdown timed out: {service_name}")
        spin_once(min(0.1, remaining))
    response = future.result()
    if response is None or response.success is not True:
        raise RuntimeError(f"lifecycle shutdown was rejected: {service_name}")


def _create_lifecycle_shutdown_clients(node: Any) -> list[tuple[str, Any]]:
    from nav2_msgs.srv import ManageLifecycleNodes

    return [
        (service_name, node.create_client(ManageLifecycleNodes, service_name))
        for service_name in LIFECYCLE_MANAGER_SERVICES
    ]


def _close_lifecycle_shutdown_clients(
    node: Any,
    clients: list[tuple[str, Any]],
) -> None:
    for _service_name, client in clients:
        node.destroy_client(client)


def _prepare_lifecycle_shutdown_clients(
    clients: list[tuple[str, Any]],
) -> None:
    """Fail before motion when the ordered-shutdown control path is absent."""

    for service_name, client in clients:
        if not client.wait_for_service(timeout_sec=LIFECYCLE_SHUTDOWN_READY_TIMEOUT_SECONDS):
            raise RuntimeError(f"lifecycle shutdown service is unavailable: {service_name}")


def _shutdown_lifecycle_manager(
    node: Any,
    service_name: str,
    client: Any,
    *,
    timeout_seconds: float,
) -> None:
    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes

    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.SHUTDOWN
    _await_lifecycle_shutdown(
        client,
        request,
        service_name=service_name,
        spin_once=lambda timeout: rclpy.spin_once(node, timeout_sec=timeout),
        timeout_seconds=timeout_seconds,
    )


def _shutdown_lifecycle_managers(
    node: Any,
    clients: list[tuple[str, Any]],
) -> None:
    """Use the graph-connected lab participant for deterministic teardown."""

    for service_name, client in clients:
        _shutdown_lifecycle_manager(
            node,
            service_name,
            client,
            timeout_seconds=LIFECYCLE_SHUTDOWN_TIMEOUT_SECONDS,
        )


def _collect_ready_runtime_snapshot(
    node: Any,
    manifest: dict[str, Any],
    graph_probe: RclpyGraphProbe,
    *,
    emergency_stop_node: str,
    emergency_stop_service: str,
    discovery_timeout_seconds: float,
) -> tuple[dict[str, Any], datetime]:
    """Retry transient discovery probes within one absolute fail-closed budget."""

    deadline = time.monotonic() + discovery_timeout_seconds
    last_runtime: dict[str, Any] | None = None
    last_observed_at: datetime | None = None
    while (remaining := deadline - time.monotonic()) >= 0.3:
        # One snapshot has three bounded phases: interface, lifecycle, E-stop.
        # The division keeps even the last attempt inside the absolute budget.
        attempt_timeout_seconds = max(0.1, min(2.0, remaining / 3.0))
        observed_at = datetime.now(timezone.utc)
        runtime = collect_ros2_runtime_snapshot(
            manifest,
            graph_probe,
            deployment_mode="simulation",
            emergency_stop_node=emergency_stop_node,
            emergency_stop_service=emergency_stop_service,
            timeout_seconds=attempt_timeout_seconds,
            max_age_seconds=300,
            observed_at=observed_at,
        )
        last_runtime = runtime
        last_observed_at = observed_at
        if (
            verify_ros2_pairing(
                manifest,
                runtime,
                observed_at=observed_at,
            )["passed"]
            is True
        ):
            return runtime, observed_at
        remaining = deadline - time.monotonic()
        if remaining >= 0.1:
            _spin_discovery_once(node, min(0.1, remaining))
    if last_runtime is None or last_observed_at is None:
        raise RuntimeError("ROS 2 discovery budget expired before the first probe")
    return last_runtime, last_observed_at


def _run_lab(
    node: Any,
    shutdown_clients: list[tuple[str, Any]],
) -> dict[str, Any]:
    """Probe, authorize, execute, and attest one real navigation scenario."""

    for name, default in (
        ("manifest_file", ""),
        ("resource_plan_file", ""),
        ("semantic_map_file", ""),
        ("semantic_map_id", ""),
        ("scenario", "success"),
        ("output_file", "results/nav2-closed-loop.json"),
        ("odometry_topic", "/flyto/odom"),
        ("lidar_topic", "/flyto/scan"),
        ("safety_state_topic", "/safety/emergency_stop_state"),
        ("safety_reason_topic", "/safety/stop_reason"),
        ("fault_state_topic", "/fault_injection/state"),
        ("execution_state_topic", "/flyto/navigation_execution_active"),
        ("emergency_stop_node", "/safety/emergency_supervisor"),
        ("emergency_stop_service", "/safety/emergency_stop"),
        ("goal_frame", "map"),
    ):
        node.declare_parameter(name, default)
    node.declare_parameter("cancel_after_displacement_m", 0.25)
    node.declare_parameter("discovery_timeout_seconds", 60.0)
    node.declare_parameter("sensor_timeout_seconds", 0.55)
    values = {
        name: node.get_parameter(name).value
        for name in (
            "manifest_file",
            "resource_plan_file",
            "semantic_map_file",
            "semantic_map_id",
            "scenario",
            "output_file",
            "odometry_topic",
            "lidar_topic",
            "safety_state_topic",
            "safety_reason_topic",
            "fault_state_topic",
            "execution_state_topic",
            "emergency_stop_node",
            "emergency_stop_service",
            "goal_frame",
            "cancel_after_displacement_m",
            "discovery_timeout_seconds",
            "sensor_timeout_seconds",
        )
    }
    if not values["manifest_file"] or not values["resource_plan_file"]:
        raise ValueError("manifest_file and resource_plan_file are required")
    if not values["semantic_map_file"] or not values["semantic_map_id"]:
        raise ValueError("semantic_map_file and semantic_map_id are required")
    manifest = load_ros2_adapter_manifest(str(values["manifest_file"]))
    semantic_map = _load_json(str(values["semantic_map_file"]))
    if semantic_map.get("map_id") != values["semantic_map_id"]:
        raise ValueError("semantic map identity does not match")
    graph_probe = RclpyGraphProbe(node)
    discovery_timeout_seconds = float(values["discovery_timeout_seconds"])
    if not 5.0 <= discovery_timeout_seconds <= 60.0:
        raise ValueError("discovery_timeout_seconds must be between 5 and 60")
    sensor_timeout_seconds = float(values["sensor_timeout_seconds"])
    if not 0.45 <= sensor_timeout_seconds <= 0.60:
        raise ValueError("sensor_timeout_seconds must be between 0.45 and 0.60")
    if shutdown_clients:
        readiness_deadline = time.monotonic() + discovery_timeout_seconds
        manager_clients = dict(shutdown_clients)
        navigation_manager_client = manager_clients.get(NAVIGATION_MANAGER_SERVICE)
        if navigation_manager_client is None:
            raise RuntimeError("navigation lifecycle manager client is missing")
        _start_navigation_lifecycle(
            node,
            navigation_manager_client,
            deadline=readiness_deadline,
        )
        remaining_readiness_seconds = readiness_deadline - time.monotonic()
        if remaining_readiness_seconds <= 0:
            raise RuntimeError("ROS 2 readiness budget expired during lifecycle startup")
        try:
            runtime, observed_at = _collect_ready_runtime_snapshot(
                node,
                manifest,
                graph_probe,
                emergency_stop_node=str(values["emergency_stop_node"]),
                emergency_stop_service=str(values["emergency_stop_service"]),
                discovery_timeout_seconds=remaining_readiness_seconds,
            )
        finally:
            graph_probe.close()
        _prepare_lifecycle_shutdown_clients(shutdown_clients)
    else:
        # Pure-Python contract tests deliberately provide no ROS client API.
        # Keep that import/test seam while requiring lifecycle control for
        # every real rclpy node before any authority or motion is issued.
        observed_at = datetime.now(timezone.utc)
        runtime = collect_ros2_runtime_snapshot(
            manifest,
            graph_probe,
            deployment_mode="simulation",
            emergency_stop_node=str(values["emergency_stop_node"]),
            emergency_stop_service=str(values["emergency_stop_service"]),
            timeout_seconds=discovery_timeout_seconds,
            max_age_seconds=300,
            observed_at=observed_at,
        )
    grant = authorize_ros2_execution(
        resource_plan=load_resource_plan(str(values["resource_plan_file"])),
        manifest=manifest,
        runtime=runtime,
        workflow_id="hospital_delivery.v1",
        resource_id=manifest["robot_id"],
        capability_id="robotics.motion.navigate@1",
        target_space_id="gazebo-nav2-lab",
        observed_at=observed_at,
    )
    scenario = str(values["scenario"])
    location_id = (
        "hospital.route.blue_end" if scenario == "success" else ("hospital.route.yellow_end")
    )
    prepared = prepare_authorized_navigation(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id=location_id,
        frame_id=str(values["goal_frame"]),
        observed_at=observed_at,
    )
    outcome = execute_rclpy_navigation(
        node,
        prepared,
        odometry_topic=str(values["odometry_topic"]),
        lidar_topic=str(values["lidar_topic"]),
        safety_state_topic=str(values["safety_state_topic"]),
        safety_reason_topic=str(values["safety_reason_topic"]),
        fault_state_topic=str(values["fault_state_topic"]),
        execution_state_topic=str(values["execution_state_topic"]),
        emergency_stop_service=str(values["emergency_stop_service"]),
        scenario=scenario,
        cancel_after_displacement_m=float(values["cancel_after_displacement_m"]),
        sensor_timeout_seconds=sensor_timeout_seconds,
    )
    evidence = build_ros2_execution_evidence(
        grant,
        prepared,
        outcome,
        scenario=scenario,
    )
    verdict = evaluate_closed_loop_evidence(evidence, expected_scenario=scenario)
    write_json_atomic(str(values["output_file"]), evidence)
    if shutdown_clients:
        _shutdown_lifecycle_managers(node, shutdown_clients)
    return {
        "evidence": evidence,
        "verdict": verdict,
        # This verdict covers the ROS action and safety controls only.  It is
        # never evidence that the Cloud-owned physical mission is complete.
        "task_completion_eligible": False,
    }


def run_lab(
    node: Any,
    *,
    shutdown_clients: list[tuple[str, Any]] | None = None,
) -> dict[str, Any]:
    """Create shutdown endpoints before discovery, then close them exactly once."""

    has_ros_client_api = callable(getattr(node, "create_client", None))
    owned_clients = shutdown_clients is None and has_ros_client_api
    clients = (
        _create_lifecycle_shutdown_clients(node)
        if owned_clients
        else ([] if shutdown_clients is None else shutdown_clients)
    )
    try:
        return _run_lab(node, clients)
    finally:
        if owned_clients:
            _close_lifecycle_shutdown_clients(node, clients)


def main() -> None:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("flyto_nav2_closed_loop_lab")
    node_holder = [node]
    exit_code = 1
    try:
        node.declare_parameter("prepare_lifecycle_only", False)
        prepare_lifecycle_only = node.get_parameter("prepare_lifecycle_only").value
        if not isinstance(prepare_lifecycle_only, bool):
            raise ValueError("prepare_lifecycle_only must be boolean")
        if prepare_lifecycle_only:
            node.declare_parameter("discovery_timeout_seconds", 60.0)
            timeout_seconds = float(node.get_parameter("discovery_timeout_seconds").value)

            def reset_preparation_participant(current_node: Any) -> Any:
                current_node.destroy_node()
                rclpy.shutdown()
                rclpy.init()
                fresh_node = Node("flyto_nav2_closed_loop_lab")
                node_holder[0] = fresh_node
                return fresh_node

            report = run_lifecycle_preparation_with_participant_retries(
                node,
                timeout_seconds=timeout_seconds,
                reset_participant=reset_preparation_participant,
            )
            exit_code = 0
        else:
            report = run_lab(node)
            exit_code = 0 if report["verdict"]["passed"] else 2
        print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
    except Exception as exc:
        print(f"closed-loop lab failed: {str(exc)[:500]}", file=sys.stderr, flush=True)
    finally:
        node_holder[0].destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
