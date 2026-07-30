from __future__ import annotations

import math

import pytest

from flyto_robotics.ai_planner import PlanValidationError
from flyto_robotics.cli import PROJECT_ROOT
from flyto_robotics.contracts import load_job
from flyto_robotics.input_runtime import (
    INPUT_EVENT_CONTRACT_VERSION,
    InputEvent,
    InputPhase,
    InputValidationError,
    ShortcutBinding,
    ShortcutRuntime,
    ValidatedWorkflowCatalog,
    parse_input_event,
)
from flyto_robotics.mission import Pose2D
from flyto_robotics.workflow import MissionState

EXAMPLE_JOB = PROJECT_ROOT / "examples/jobs/pharmacy-to-ward.json"


def relative_plan(*, distance_m: float = 0.3) -> dict[str, object]:
    return {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": "shortcut.forward.30cm.v1",
        "robot_id": "flyto-rover-sim-001",
        "goal": "前進三十公分後安全停止",
        "generated_by": {
            "kind": "human",
            "provider": "flyto-cloud",
            "model": "workflow-card",
        },
        "steps": [
            {
                "step_id": "move.forward",
                "capability": "move_relative",
                "arguments": {"distance_m": distance_m, "speed": 0.12},
                "timeout_seconds": 5.0,
                "on_failure": "abort",
            },
            {
                "step_id": "stop.finish",
                "capability": "safe_stop",
                "arguments": {"seconds": 0.0},
                "timeout_seconds": 1.0,
                "on_failure": "abort",
            },
        ],
    }


def event(
    phase: InputPhase,
    sequence: int,
    *,
    event_id: str | None = None,
    source_id: str = "keyboard.main",
    control_id: str = "ArrowUp",
    session_id: str = "session.1",
) -> InputEvent:
    return InputEvent(
        event_id=event_id or f"event.{sequence}",
        source_id=source_id,
        control_id=control_id,
        session_id=session_id,
        phase=phase,
        sequence=sequence,
    )


def runtime(*, timeout: float = 0.5) -> ShortcutRuntime:
    job = load_job(EXAMPLE_JOB)
    catalog = ValidatedWorkflowCatalog.from_plan_payloads([relative_plan()])
    return ShortcutRuntime(
        job,
        catalog=catalog,
        bindings=(
            ShortcutBinding(
                binding_id="binding.forward",
                source_id="keyboard.main",
                control_id="ArrowUp",
                workflow_id="shortcut.forward.30cm.v1",
                deadman_timeout_seconds=timeout,
            ),
        ),
    )


def test_input_event_parser_is_strict_and_versioned() -> None:
    parsed = parse_input_event(
        {
            "contract_version": INPUT_EVENT_CONTRACT_VERSION,
            "event_id": "event.1",
            "source_id": "keyboard.main",
            "control_id": "ArrowUp",
            "session_id": "session.1",
            "phase": "press",
            "sequence": 1,
        }
    )

    assert parsed.phase == InputPhase.PRESS
    with pytest.raises(InputValidationError, match="unsupported fields"):
        parse_input_event(
            {
                "contract_version": INPUT_EVENT_CONTRACT_VERSION,
                "event_id": "event.2",
                "source_id": "keyboard.main",
                "control_id": "ArrowUp",
                "session_id": "session.1",
                "phase": "press",
                "sequence": 2,
                "linear_x": 1.0,
            }
        )


def test_press_starts_only_registered_workflow_then_controller_moves() -> None:
    active = runtime()

    action = active.handle_event(event(InputPhase.PRESS, 1), now=0.0)
    command = active.tick(
        Pose2D(0.0, 0.0, 0.0),
        minimum_range=math.inf,
        now=0.01,
    )

    assert action.kind == "start_workflow"
    assert action.workflow is not None
    assert action.workflow.workflow_id == "shortcut.forward.30cm.v1"
    assert command.linear_x > 0.0
    assert active.controller is not None
    assert active.controller.state == MissionState.MOVING_RELATIVE


@pytest.mark.parametrize(
    ("phase", "reason"),
    (
        (InputPhase.RELEASE, "input_released"),
        (InputPhase.DISCONNECT, "input_disconnected"),
    ),
)
def test_release_or_disconnect_forces_zero_velocity(
    phase: InputPhase,
    reason: str,
) -> None:
    active = runtime()
    active.handle_event(event(InputPhase.PRESS, 1), now=0.0)
    assert (
        active.tick(
            Pose2D(0.0, 0.0, 0.0),
            minimum_range=math.inf,
            now=0.01,
        ).linear_x
        > 0.0
    )

    action = active.handle_event(event(phase, 2), now=0.1)
    stopped = active.tick(
        Pose2D(0.01, 0.0, 0.0),
        minimum_range=math.inf,
        now=0.11,
    )

    assert action.kind == "safe_stop"
    assert action.reason == reason
    assert stopped.linear_x == 0.0
    assert stopped.angular_z == 0.0
    assert stopped.state == MissionState.CANCELLED
    assert active.events[-1].reason == reason
    assert active.controller is not None
    assert active.controller.events[-1].detail == reason


def test_deadman_timeout_stops_before_next_control_update() -> None:
    active = runtime(timeout=0.2)
    active.handle_event(event(InputPhase.PRESS, 1), now=0.0)
    moving = active.tick(
        Pose2D(0.0, 0.0, 0.0),
        minimum_range=math.inf,
        now=0.05,
    )
    stopped = active.tick(
        Pose2D(0.02, 0.0, 0.0),
        minimum_range=math.inf,
        now=0.21,
    )

    assert moving.linear_x > 0.0
    assert stopped.linear_x == 0.0
    assert stopped.state == MissionState.CANCELLED
    assert active.events[-1].kind == "safe_stop"
    assert active.events[-1].reason == "input_timeout"


def test_deadman_can_stop_without_a_sensor_pose() -> None:
    active = runtime(timeout=0.2)
    active.handle_event(event(InputPhase.PRESS, 1), now=0.0)

    action = active.poll(now=0.21)

    assert action is not None
    assert action.kind == "safe_stop"
    assert action.reason == "input_timeout"
    assert active.controller is not None
    assert active.controller.state == MissionState.CANCELLED


def test_heartbeat_extends_deadman_window_and_replay_does_not() -> None:
    active = runtime(timeout=0.2)
    active.handle_event(event(InputPhase.PRESS, 1), now=0.0)
    heartbeat = active.handle_event(event(InputPhase.HEARTBEAT, 2), now=0.15)
    replay = active.handle_event(
        event(InputPhase.HEARTBEAT, 3, event_id="event.2"),
        now=0.25,
    )
    moving = active.tick(
        Pose2D(0.02, 0.0, 0.0),
        minimum_range=math.inf,
        now=0.3,
    )

    assert heartbeat.kind == "keepalive"
    assert replay.kind == "ignored"
    assert replay.reason == "event_replay"
    assert moving.linear_x > 0.0


def test_catalog_rejects_unvalidated_or_wrong_robot_workflows() -> None:
    unsafe = relative_plan()
    unsafe["steps"] = unsafe["steps"][:-1]  # type: ignore[index]
    with pytest.raises(PlanValidationError, match="must end with safe_stop"):
        ValidatedWorkflowCatalog.from_plan_payloads([unsafe])

    catalog = ValidatedWorkflowCatalog.from_plan_payloads([relative_plan()])
    with pytest.raises(InputValidationError, match="not registered for robot"):
        catalog.resolve(
            "shortcut.forward.30cm.v1",
            robot_id="another-robot",
        )


def test_shortcut_relative_motion_soak_completes_30_of_30() -> None:
    completed = 0
    obstacle_stops = 0

    for run_index in range(30):
        active = runtime(timeout=0.5)
        active.handle_event(
            event(
                InputPhase.PRESS,
                1,
                event_id=f"run.{run_index}.press",
                session_id=f"session.{run_index}",
            ),
            now=0.0,
        )
        moving = active.tick(
            Pose2D(0.0, 0.0, 0.0),
            minimum_range=math.inf,
            now=0.01,
        )
        assert moving.linear_x > 0.0
        active.handle_event(
            event(
                InputPhase.HEARTBEAT,
                2,
                event_id=f"run.{run_index}.heartbeat",
                session_id=f"session.{run_index}",
            ),
            now=0.1,
        )

        if run_index % 5 == 0:
            blocked = active.tick(
                Pose2D(0.05, 0.0, 0.0),
                minimum_range=0.2,
                now=0.15,
            )
            assert blocked.linear_x == 0.0
            assert blocked.reason == "obstacle_stop"
            obstacle_stops += 1
            resumed = active.tick(
                Pose2D(0.05, 0.0, 0.0),
                minimum_range=math.inf,
                now=0.2,
            )
            assert resumed.linear_x > 0.0

        reached = active.tick(
            Pose2D(0.3, 0.0, 0.0),
            minimum_range=math.inf,
            now=0.25,
        )
        terminal = active.tick(
            Pose2D(0.3, 0.0, 0.0),
            minimum_range=math.inf,
            now=0.26,
        )
        assert reached.linear_x == 0.0
        assert terminal.linear_x == 0.0
        assert terminal.state == MissionState.COMPLETED
        completed += 1

    assert completed == 30
    assert obstacle_stops == 6
