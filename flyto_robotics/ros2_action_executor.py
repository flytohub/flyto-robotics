"""Grant-bound, semantic-only ROS 2 NavigateToPose execution."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from .contracts import StationPose
from .ros2_execution import resolve_ros2_execution_target
from .semantic_map import parse_semantic_location_map

FAULT_SCENARIOS = {
    "lidar_dropout": "lidar_stale",
    "odometry_freeze": "odometry_stale",
    "nav2_lifecycle_failure": "command_stale",
}
SAFETY_STOP_REASONS = {"emergency_stop", *FAULT_SCENARIOS.values()}


class Ros2ActionExecutionError(RuntimeError):
    """Raised when an authorized action cannot complete safely."""


@dataclass(frozen=True)
class PreparedNavigation:
    """Private action target and trusted semantic pose resolved at dispatch time."""

    target: Any
    location_id: str
    map_id: str
    frame_id: str
    pose: StationPose


@dataclass(frozen=True)
class NavigationOutcome:
    """Transport-neutral terminal facts used to build redacted evidence."""

    status: str
    result_code: str
    goal_accepted: bool
    feedback_count: int
    initial_pose: StationPose
    final_pose: StationPose
    displacement_m: float
    goal_error_m: float
    post_stop_drift_m: float
    duration_seconds: float
    cancel_requested: bool
    cancel_reason: str | None
    safety_stop_observed: bool
    safety_stop_reason: str | None
    fault_injection_observed: bool
    safety_stop_latency_ms: float | None
    event_codes: tuple[str, ...]


def prepare_authorized_navigation(
    *,
    grant: dict[str, Any],
    manifest: dict[str, Any],
    runtime: dict[str, Any],
    semantic_map: dict[str, Any],
    location_id: str,
    frame_id: str,
    observed_at: datetime | None = None,
) -> PreparedNavigation:
    """Resolve a semantic location only after the exact execution grant validates."""

    target = resolve_ros2_execution_target(
        grant,
        manifest,
        runtime,
        observed_at=observed_at,
    )
    if target.interface_kind != "action":
        raise Ros2ActionExecutionError("authorized adapter is not an action")
    if target.interface_type != "nav2_msgs/action/NavigateToPose":
        raise Ros2ActionExecutionError("authorized action type is unsupported")
    if target.capability_id not in {
        "robotics.motion.navigate@1",
        "robotics.motion.navigate_to_location@1",
    }:
        raise Ros2ActionExecutionError("authorized capability is not semantic navigation")
    if frame_id not in {"map", "odom"}:
        raise Ros2ActionExecutionError("goal frame must be map or odom")
    trusted_map = parse_semantic_location_map(semantic_map)
    location = trusted_map.resolve(location_id)
    return PreparedNavigation(
        target=target,
        location_id=location.location_id,
        map_id=trusted_map.map_id,
        frame_id=frame_id,
        pose=location.pose,
    )


class NavigationExecutionMonitor:
    """Deterministic fail-closed state machine shared by fake and real transports."""

    def __init__(
        self,
        prepared: PreparedNavigation,
        initial_pose: StationPose,
        *,
        started_at: datetime,
    ) -> None:
        self.prepared = prepared
        self.initial_pose = _pose(initial_pose, "initial_pose")
        self.started_at = _utc(started_at)
        self.state = "prepared"
        self.feedback_count = 0
        self.cancel_reason: str | None = None
        self.safety_stop_reason: str | None = None
        self.fault_injection_observed = False
        self.safety_stop_latency_ms: float | None = None
        self._pending_feedback: list[float] = []
        self.event_codes: list[str] = ["authority_validated", "server_available"]

    def begin_goal_submission(self) -> None:
        if self.state != "prepared":
            raise Ros2ActionExecutionError("goal submission is out of order")
        self.state = "awaiting_acceptance"

    def accept_goal(self) -> None:
        if self.state not in {"prepared", "awaiting_acceptance"}:
            raise Ros2ActionExecutionError("goal acceptance is out of order")
        self.state = "executing"
        self.event_codes.append("goal_accepted")
        pending = tuple(self._pending_feedback)
        self._pending_feedback.clear()
        for distance_remaining in pending:
            self._record_feedback(distance_remaining)

    def reject_goal(self) -> None:
        if self.state not in {"prepared", "awaiting_acceptance"}:
            raise Ros2ActionExecutionError("goal rejection is out of order")
        self.state = "rejected"
        self._pending_feedback.clear()
        self.event_codes.append("goal_rejected")

    def feedback(self, distance_remaining: float) -> None:
        parsed = _finite(distance_remaining, "distance_remaining", minimum=0.0)
        if self.state == "awaiting_acceptance":
            if len(self._pending_feedback) >= 256:
                raise Ros2ActionExecutionError("pre-accept feedback limit exceeded")
            self._pending_feedback.append(parsed)
            return
        if self.state not in {"executing", "canceling"}:
            raise Ros2ActionExecutionError("feedback arrived outside execution")
        self._record_feedback(parsed)

    def _record_feedback(self, distance_remaining: float) -> None:
        _finite(distance_remaining, "distance_remaining", minimum=0.0)
        self.feedback_count += 1
        if self.feedback_count == 1:
            self.event_codes.append("feedback_observed")

    def request_cancel(
        self,
        reason: str,
        *,
        fault_injection_observed: bool = False,
        safety_stop_latency_ms: float | None = None,
    ) -> None:
        if self.state != "executing":
            raise Ros2ActionExecutionError("cancel requires an executing goal")
        if reason not in {
            "operator_cancel",
            "emergency_stop",
            "timeout",
            *FAULT_SCENARIOS.values(),
        }:
            raise Ros2ActionExecutionError("cancel reason is unsupported")
        self.cancel_reason = reason
        if reason in SAFETY_STOP_REASONS:
            self._record_safety_stop(
                reason,
                fault_injection_observed=fault_injection_observed,
                safety_stop_latency_ms=safety_stop_latency_ms,
            )
        self.state = "canceling"
        self.event_codes.append(reason + "_requested")

    def observe_safety_stop(
        self,
        reason: str,
        *,
        fault_injection_observed: bool,
        safety_stop_latency_ms: float | None,
    ) -> None:
        """Record an independent stop even if Nav2 already aborted the action."""

        if self.state not in {"executing", "canceling"}:
            raise Ros2ActionExecutionError("safety stop arrived outside execution")
        self._record_safety_stop(
            reason,
            fault_injection_observed=fault_injection_observed,
            safety_stop_latency_ms=safety_stop_latency_ms,
        )

    def _record_safety_stop(
        self,
        reason: str,
        *,
        fault_injection_observed: bool,
        safety_stop_latency_ms: float | None,
    ) -> None:
        if reason not in SAFETY_STOP_REASONS:
            raise Ros2ActionExecutionError("safety stop reason is unsupported")
        if safety_stop_latency_ms is not None:
            _finite(
                safety_stop_latency_ms,
                "safety_stop_latency_ms",
                minimum=0.0,
                maximum=10_000.0,
            )
        self.safety_stop_reason = reason
        self.fault_injection_observed = bool(fault_injection_observed)
        self.safety_stop_latency_ms = safety_stop_latency_ms
        if "safety_stop_observed" not in self.event_codes:
            self.event_codes.append("safety_stop_observed")

    def finish(
        self,
        result_code: str,
        final_pose: StationPose,
        *,
        settled_pose: StationPose | None = None,
        finished_at: datetime,
    ) -> NavigationOutcome:
        terminal_pose = _pose(final_pose, "final_pose")
        settled = _pose(settled_pose or terminal_pose, "settled_pose")
        finished = _utc(finished_at)
        duration = (finished - self.started_at).total_seconds()
        if not 0.0 <= duration <= 3600.0:
            raise Ros2ActionExecutionError("execution duration is invalid")
        if result_code == "rejected":
            if self.state != "rejected":
                raise Ros2ActionExecutionError("rejected result requires rejected goal")
            status = "rejected"
            accepted = False
        else:
            if self.state not in {"executing", "canceling"}:
                raise Ros2ActionExecutionError("terminal result is out of order")
            accepted = True
            if result_code == "succeeded":
                if self.state != "executing" or self.feedback_count < 1:
                    raise Ros2ActionExecutionError(
                        "success requires an uncanceled goal with feedback"
                    )
                status = "succeeded"
            elif result_code == "canceled" and self.cancel_reason is not None:
                if self.cancel_reason in SAFETY_STOP_REASONS:
                    status = "safety_stopped"
                else:
                    status = {
                        "operator_cancel": "canceled",
                        "timeout": "timed_out",
                    }[self.cancel_reason]
            elif result_code == "aborted":
                status = (
                    "safety_stopped"
                    if self.safety_stop_reason is not None
                    else "aborted"
                )
            else:
                raise Ros2ActionExecutionError("terminal action result is inconsistent")
        self.event_codes.append("execution_" + status)
        self.event_codes.append("post_stop_observed")
        displacement = math.hypot(
            settled.x - self.initial_pose.x,
            settled.y - self.initial_pose.y,
        )
        goal_error = math.hypot(
            settled.x - self.prepared.pose.x,
            settled.y - self.prepared.pose.y,
        )
        post_stop_drift = math.hypot(
            settled.x - terminal_pose.x,
            settled.y - terminal_pose.y,
        )
        return NavigationOutcome(
            status=status,
            result_code=result_code,
            goal_accepted=accepted,
            feedback_count=self.feedback_count,
            initial_pose=self.initial_pose,
            final_pose=settled,
            displacement_m=round(displacement, 6),
            goal_error_m=round(goal_error, 6),
            post_stop_drift_m=round(post_stop_drift, 6),
            duration_seconds=round(duration, 6),
            cancel_requested=self.cancel_reason is not None,
            cancel_reason=self.cancel_reason,
            safety_stop_observed=self.safety_stop_reason is not None,
            safety_stop_reason=self.safety_stop_reason,
            fault_injection_observed=self.fault_injection_observed,
            safety_stop_latency_ms=(
                round(self.safety_stop_latency_ms, 3)
                if self.safety_stop_latency_ms is not None
                else None
            ),
            event_codes=tuple(self.event_codes),
        )


def execute_rclpy_navigation(
    node: Any,
    prepared: PreparedNavigation,
    *,
    odometry_topic: str,
    safety_state_topic: str,
    safety_reason_topic: str,
    fault_state_topic: str,
    execution_state_topic: str,
    emergency_stop_service: str,
    scenario: str,
    cancel_after_displacement_m: float = 0.25,
) -> NavigationOutcome:
    """Execute one real ROS action while monitoring odometry and independent stop."""

    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    if scenario not in {
        "success",
        "cancel",
        "emergency_stop",
        *FAULT_SCENARIOS,
    }:
        raise Ros2ActionExecutionError("scenario is unsupported")
    _finite(
        cancel_after_displacement_m,
        "cancel_after_displacement_m",
        minimum=0.05,
        maximum=10.0,
    )
    latest: dict[str, Any] = {
        "pose": None,
        "safety": None,
        "safety_reason": None,
        "safety_seen_at": None,
        "fault": None,
        "fault_seen_at": None,
    }

    def on_odometry(message: Any) -> None:
        latest["pose"] = _station_from_odometry(message)

    def on_safety(message: Any) -> None:
        latest["safety"] = bool(message.data)
        if message.data and latest["safety_seen_at"] is None:
            latest["safety_seen_at"] = time.monotonic()

    def on_safety_reason(message: Any) -> None:
        reason = str(message.data)
        latest["safety_reason"] = None if reason == "reset" else reason

    def on_fault(message: Any) -> None:
        state = str(message.data)
        expected = f"{scenario}:active"
        if state == expected and latest["fault_seen_at"] is None:
            latest["fault"] = scenario
            latest["fault_seen_at"] = time.monotonic()

    odom_sub = node.create_subscription(Odometry, odometry_topic, on_odometry, 20)
    safety_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    safety_sub = node.create_subscription(
        Bool,
        safety_state_topic,
        on_safety,
        safety_qos,
    )
    reason_sub = node.create_subscription(
        String,
        safety_reason_topic,
        on_safety_reason,
        safety_qos,
    )
    fault_sub = node.create_subscription(
        String,
        fault_state_topic,
        on_fault,
        safety_qos,
    )
    execution_publisher = node.create_publisher(
        Bool,
        execution_state_topic,
        safety_qos,
    )

    def publish_execution_state(active: bool) -> None:
        message = Bool()
        message.data = active
        execution_publisher.publish(message)

    publish_execution_state(False)
    action_client = ActionClient(
        node,
        NavigateToPose,
        prepared.target.interface_name,
    )
    stop_client = node.create_client(Trigger, emergency_stop_service)
    try:
        startup_deadline = time.monotonic() + 30.0
        while (
            latest["pose"] is None or latest["safety"] is None
        ) and time.monotonic() < startup_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest["pose"] is None:
            raise Ros2ActionExecutionError("fresh odometry was not observed")
        if latest["safety"] is not False:
            raise Ros2ActionExecutionError("emergency stop is not reset")
        if not action_client.wait_for_server(timeout_sec=15.0):
            raise Ros2ActionExecutionError("authorized action server is unavailable")
        if not stop_client.wait_for_service(timeout_sec=5.0):
            raise Ros2ActionExecutionError("independent emergency stop is unavailable")

        initial_pose = latest["pose"]
        monitor = NavigationExecutionMonitor(
            prepared,
            initial_pose,
            started_at=datetime.now(timezone.utc),
        )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = prepared.frame_id
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = prepared.pose.x
        goal.pose.pose.position.y = prepared.pose.y
        goal.pose.pose.orientation.z = math.sin(prepared.pose.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(prepared.pose.yaw / 2.0)

        def on_feedback(message: Any) -> None:
            monitor.feedback(float(message.feedback.distance_remaining))

        monitor.begin_goal_submission()
        send_future = action_client.send_goal_async(
            goal,
            feedback_callback=on_feedback,
        )
        _spin_future(node, send_future, timeout_seconds=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            monitor.reject_goal()
            return monitor.finish(
                "rejected",
                latest["pose"],
                finished_at=datetime.now(timezone.utc),
            )
        monitor.accept_goal()
        publish_execution_state(True)
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + prepared.target.timeout_seconds
        cancel_future: Any | None = None
        stop_future: Any | None = None
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            pose = latest["pose"]
            displacement = math.hypot(
                pose.x - initial_pose.x,
                pose.y - initial_pose.y,
            )
            if (
                latest["safety"] is True
                and latest["safety_reason"] is not None
                and monitor.state == "executing"
            ):
                latency = _stop_latency_ms(latest)
                monitor.request_cancel(
                    latest["safety_reason"],
                    fault_injection_observed=latest["fault"] == scenario,
                    safety_stop_latency_ms=latency,
                )
                cancel_future = goal_handle.cancel_goal_async()
            elif (
                scenario == "cancel"
                and displacement >= cancel_after_displacement_m
                and monitor.state == "executing"
            ):
                monitor.request_cancel("operator_cancel")
                cancel_future = goal_handle.cancel_goal_async()
            elif (
                scenario == "emergency_stop"
                and displacement >= cancel_after_displacement_m
                and stop_future is None
            ):
                stop_future = stop_client.call_async(Trigger.Request())
            elif time.monotonic() >= deadline and monitor.state == "executing":
                monitor.request_cancel("timeout")
                cancel_future = goal_handle.cancel_goal_async()
            if monitor.state == "canceling" and time.monotonic() > deadline + 10.0:
                raise Ros2ActionExecutionError("action did not acknowledge cancellation")
        if cancel_future is not None and not cancel_future.done():
            _spin_future(node, cancel_future, timeout_seconds=5.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise Ros2ActionExecutionError("action returned no result")
        result_code = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }.get(wrapped.status, "aborted")
        if scenario not in FAULT_SCENARIOS:
            publish_execution_state(False)
        terminal_pose = latest["pose"]
        settle_deadline = time.monotonic() + 0.75
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if (
            latest["safety"] is True
            and latest["safety_reason"] is not None
            and monitor.safety_stop_reason is None
        ):
            monitor.observe_safety_stop(
                latest["safety_reason"],
                fault_injection_observed=latest["fault"] == scenario,
                safety_stop_latency_ms=_stop_latency_ms(latest),
            )
        return monitor.finish(
            result_code,
            terminal_pose,
            settled_pose=latest["pose"],
            finished_at=datetime.now(timezone.utc),
        )
    finally:
        publish_execution_state(False)
        action_client.destroy()
        node.destroy_client(stop_client)
        node.destroy_subscription(odom_sub)
        node.destroy_subscription(safety_sub)
        node.destroy_subscription(reason_sub)
        node.destroy_subscription(fault_sub)
        node.destroy_publisher(execution_publisher)


def _stop_latency_ms(latest: dict[str, Any]) -> float | None:
    fault_at = latest.get("fault_seen_at")
    stopped_at = latest.get("safety_seen_at")
    if fault_at is None or stopped_at is None:
        return None
    return max(0.0, (float(stopped_at) - float(fault_at)) * 1000.0)


def _spin_future(node: Any, future: Any, *, timeout_seconds: float) -> None:
    import rclpy

    deadline = time.monotonic() + timeout_seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise Ros2ActionExecutionError("ROS operation timed out")


def _station_from_odometry(message: Any) -> StationPose:
    orientation = message.pose.pose.orientation
    siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return StationPose(
        station_id="robot.current",
        x=float(message.pose.pose.position.x),
        y=float(message.pose.pose.position.y),
        yaw=math.atan2(siny, cosy),
    )


def _pose(value: StationPose, label: str) -> StationPose:
    if not isinstance(value, StationPose):
        raise Ros2ActionExecutionError(f"{label} must be a StationPose")
    _finite(value.x, f"{label}.x", minimum=-1000.0, maximum=1000.0)
    _finite(value.y, f"{label}.y", minimum=-1000.0, maximum=1000.0)
    _finite(value.yaw, f"{label}.yaw", minimum=-math.pi, maximum=math.pi)
    return value


def _finite(
    value: Any,
    label: str,
    *,
    minimum: float,
    maximum: float = 1000.0,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Ros2ActionExecutionError(f"{label} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise Ros2ActionExecutionError(f"{label} is outside its safe range")
    return parsed


def _utc(value: datetime) -> datetime:
    if value.tzinfo is None:
        raise Ros2ActionExecutionError("timestamps must include a UTC offset")
    return value.astimezone(timezone.utc)
