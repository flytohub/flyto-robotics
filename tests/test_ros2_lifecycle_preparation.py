from __future__ import annotations

import pytest

from flyto_robotics import ros2_closed_loop_lab as lab


def test_navigation_lifecycle_normalizes_partial_startup_to_inactive() -> None:
    states = {
        node_name: lab.LIFECYCLE_STATE_INACTIVE
        for node_name in lab.NAVIGATION_LIFECYCLE_NODES
    }
    states["/smoother_server"] = lab.LIFECYCLE_STATE_UNCONFIGURED
    changes: list[tuple[str, int, int]] = []

    def change(node_name: str, transition: int, target: int) -> None:
        changes.append((node_name, transition, target))
        states[node_name] = target

    lab._normalize_navigation_nodes_inactive(states.__getitem__, change)

    assert changes == [
        (
            "/smoother_server",
            lab.LIFECYCLE_TRANSITION_CONFIGURE,
            lab.LIFECYCLE_STATE_INACTIVE,
        ),
    ]
    assert set(states.values()) == {lab.LIFECYCLE_STATE_INACTIVE}


def test_navigation_lifecycle_recovery_rejects_partial_activation() -> None:
    states = {
        node_name: lab.LIFECYCLE_STATE_INACTIVE
        for node_name in lab.NAVIGATION_LIFECYCLE_NODES
    }
    states["/planner_server"] = lab.LIFECYCLE_STATE_ACTIVE

    with pytest.raises(RuntimeError, match="partial activation"):
        lab._normalize_navigation_nodes_inactive(
            states.__getitem__,
            lambda *_args: None,
        )


def test_navigation_lifecycle_configures_directly_then_resumes_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands: list[int] = []
    normalizations: list[str] = []
    monkeypatch.setattr(
        lab,
        "_request_lifecycle_manager_command",
        lambda *_args, **_kwargs: (
            commands.append(_args[2]),
            True,
        )[1],
    )
    monkeypatch.setattr(
        lab,
        "_normalize_navigation_nodes_inactive",
        lambda *_args: normalizations.append("normalized"),
    )

    lab._start_navigation_lifecycle(
        object(),
        object(),
        deadline=60.0,
        clock=lambda: 0.0,
    )

    assert commands == [
        lab.LIFECYCLE_MANAGER_RESUME,
    ]
    assert normalizations == ["normalized"]


def test_navigation_lifecycle_fails_closed_after_bounded_recovery(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands: list[int] = []
    monkeypatch.setattr(
        lab,
        "_request_lifecycle_manager_command",
        lambda *_args, **_kwargs: commands.append(_args[2]) or False,
    )
    monkeypatch.setattr(
        lab,
        "_normalize_navigation_nodes_inactive",
        lambda *_args: None,
    )

    with pytest.raises(RuntimeError, match="resume was rejected"):
        lab._start_navigation_lifecycle(
            object(),
            object(),
            deadline=60.0,
            clock=lambda: 0.0,
        )

    assert commands == [
        lab.LIFECYCLE_MANAGER_RESUME,
    ]


def test_navigation_lifecycle_rejects_expired_absolute_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requested: list[bool] = []
    monkeypatch.setattr(
        lab,
        "_request_lifecycle_manager_command",
        lambda *_args, **_kwargs: requested.append(True) or True,
    )

    with pytest.raises(RuntimeError, match="startup budget expired"):
        lab._start_navigation_lifecycle(
            object(),
            object(),
            deadline=10.0,
            clock=lambda: 10.0,
        )

    assert requested == []


def test_lifecycle_preparation_uses_one_bounded_absolute_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[object, float]] = []
    node = object()
    monkeypatch.setattr(
        lab,
        "_prepare_navigation_lifecycle",
        lambda observed_node, *, deadline, clock: calls.append(
            (observed_node, deadline)
        )
        if clock() == 10.0
        else None,
    )

    report = lab.run_lifecycle_preparation(
        node,
        timeout_seconds=60.0,
        clock=lambda: 10.0,
    )

    assert calls == [(node, 70.0)]
    assert report == {
        "prepared": True,
        "navigation_nodes": list(lab.NAVIGATION_LIFECYCLE_NODES),
        "timeout_seconds": 60.0,
    }


def test_lifecycle_preparation_fails_closed_without_success_report(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        lab,
        "_prepare_navigation_lifecycle",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("planner lifecycle service unavailable")
        ),
    )

    with pytest.raises(RuntimeError, match="planner lifecycle service unavailable"):
        lab.run_lifecycle_preparation(
            object(),
            timeout_seconds=60.0,
            clock=lambda: 10.0,
        )


def test_lifecycle_preparation_participant_retry_succeeds_without_reset(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[object, float]] = []
    resets: list[object] = []
    node = object()
    monkeypatch.setattr(
        lab,
        "_prepare_navigation_lifecycle",
        lambda observed_node, *, deadline, clock: calls.append(
            (observed_node, deadline)
        ),
    )

    report = lab.run_lifecycle_preparation_with_participant_retries(
        node,
        timeout_seconds=60.0,
        reset_participant=lambda stale_node: resets.append(stale_node) or object(),
        clock=lambda: 10.0,
    )

    assert calls == [(node, 22.0)]
    assert resets == []
    assert report["participant_attempts"] == 1
    assert report["timeout_seconds"] == 60.0


def test_lifecycle_preparation_recovers_with_fresh_participant(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [0.0]
    first_node = object()
    second_node = object()
    calls: list[tuple[object, float]] = []
    resets: list[object] = []

    def prepare(observed_node: object, *, deadline: float, clock: object) -> None:
        calls.append((observed_node, deadline))
        if observed_node is first_node:
            now[0] = deadline
            raise RuntimeError("participant graph missed planner service")

    monkeypatch.setattr(lab, "_prepare_navigation_lifecycle", prepare)

    report = lab.run_lifecycle_preparation_with_participant_retries(
        first_node,
        timeout_seconds=60.0,
        reset_participant=lambda stale_node: resets.append(stale_node) or second_node,
        clock=lambda: now[0],
    )

    assert calls == [(first_node, 12.0), (second_node, 24.0)]
    assert resets == [first_node]
    assert report["participant_attempts"] == 2


def test_lifecycle_preparation_participants_exhaust_absolute_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = [0.0]
    nodes = [object() for _ in range(5)]
    calls: list[tuple[object, float]] = []
    resets: list[object] = []

    def prepare(observed_node: object, *, deadline: float, clock: object) -> None:
        calls.append((observed_node, deadline))
        now[0] = deadline
        raise RuntimeError(f"participant attempt {len(calls)} unavailable")

    def reset(stale_node: object) -> object:
        resets.append(stale_node)
        return nodes[len(resets)]

    monkeypatch.setattr(lab, "_prepare_navigation_lifecycle", prepare)

    with pytest.raises(RuntimeError, match="participant attempt 5 unavailable"):
        lab.run_lifecycle_preparation_with_participant_retries(
            nodes[0],
            timeout_seconds=60.0,
            reset_participant=reset,
            clock=lambda: now[0],
        )

    assert calls == [
        (nodes[0], 12.0),
        (nodes[1], 24.0),
        (nodes[2], 36.0),
        (nodes[3], 48.0),
        (nodes[4], 60.0),
    ]
    assert resets == nodes[:4]
    assert now[0] == pytest.approx(60.0)


@pytest.mark.parametrize("timeout_seconds", [4.99, 60.01, float("nan")])
def test_lifecycle_preparation_rejects_unbounded_timeout(
    timeout_seconds: float,
) -> None:
    with pytest.raises(ValueError, match="timeout must be between 5 and 60"):
        lab.run_lifecycle_preparation(
            object(),
            timeout_seconds=timeout_seconds,
        )


