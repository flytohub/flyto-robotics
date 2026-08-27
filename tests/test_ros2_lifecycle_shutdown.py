from __future__ import annotations

from types import SimpleNamespace

import pytest

from flyto_robotics import ros2_closed_loop_lab as lab


def test_fresh_lifecycle_client_is_recreated_after_send_failure() -> None:
    response = SimpleNamespace(success=True)

    class Future:
        def done(self) -> bool:
            return True

        def result(self) -> object:
            return response

    class Client:
        def __init__(self, fail: bool) -> None:
            self.fail = fail

        def wait_for_service(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == pytest.approx(2.0)
            return True

        def call_async(self, _request: object) -> Future:
            if self.fail:
                raise RuntimeError("async_send_request failed")
            return Future()

    class Node:
        def __init__(self) -> None:
            self.clients = [Client(True), Client(False)]
            self.destroyed: list[Client] = []

        def create_client(self, _service_type: object, _name: str) -> Client:
            return self.clients.pop(0)

        def destroy_client(self, client: Client) -> None:
            self.destroyed.append(client)

    node = Node()

    observed = lab._fresh_service_call(
        node,
        object(),
        "/node/get_state",
        object(),
        deadline=10.0,
        clock=lambda: 0.0,
    )

    assert observed is response
    assert len(node.destroyed) == 2


def test_fresh_lifecycle_client_uses_fourth_attempt_inside_absolute_budget() -> None:
    response = SimpleNamespace(success=True)
    now = [0.0]

    class Future:
        def done(self) -> bool:
            return True

        def result(self) -> object:
            return response

    class Client:
        def __init__(self, available: bool) -> None:
            self.available = available

        def wait_for_service(self, *, timeout_sec: float) -> bool:
            if self.available:
                return True
            now[0] += timeout_sec
            return False

        def call_async(self, _request: object) -> Future:
            return Future()

    class Node:
        def __init__(self) -> None:
            self.created = 0
            self.destroyed = 0

        def create_client(self, _service_type: object, _name: str) -> Client:
            self.created += 1
            return Client(available=self.created == 4)

        def destroy_client(self, _client: Client) -> None:
            self.destroyed += 1

    node = Node()

    observed = lab._fresh_service_call(
        node,
        object(),
        "/planner_server/change_state",
        object(),
        deadline=10.0,
        clock=lambda: now[0],
    )

    assert observed is response
    assert node.created == 4
    assert node.destroyed == 4
    assert now[0] == pytest.approx(6.0)


def test_fresh_lifecycle_client_exhausts_one_absolute_budget() -> None:
    now = [0.0]
    waits: list[float] = []

    class Client:
        def wait_for_service(self, *, timeout_sec: float) -> bool:
            waits.append(timeout_sec)
            now[0] += timeout_sec
            return False

    class Node:
        def __init__(self) -> None:
            self.created = 0
            self.destroyed = 0

        def create_client(self, _service_type: object, _name: str) -> Client:
            self.created += 1
            return Client()

        def destroy_client(self, _client: Client) -> None:
            self.destroyed += 1

    node = Node()

    with pytest.raises(RuntimeError, match="service unavailable"):
        lab._fresh_service_call(
            node,
            object(),
            "/planner_server/change_state",
            object(),
            deadline=7.0,
            clock=lambda: now[0],
        )

    assert waits == pytest.approx([2.0, 2.0, 2.0, 1.0])
    assert node.created == 4
    assert node.destroyed == 4
    assert now[0] == pytest.approx(7.0)


def test_lifecycle_managers_shutdown_in_dependency_order(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node = object()
    clients = [
        ("/lifecycle_manager_navigation/manage_nodes", object()),
        ("/map_lifecycle_manager/manage_nodes", object()),
    ]
    calls: list[tuple[object, str, object, float]] = []
    monkeypatch.setattr(
        lab,
        "_shutdown_lifecycle_manager",
        lambda observed_node, service_name, client, *, timeout_seconds: calls.append(
            (observed_node, service_name, client, timeout_seconds)
        ),
    )

    lab._shutdown_lifecycle_managers(node, clients)

    assert calls == [
        (
            node,
            "/lifecycle_manager_navigation/manage_nodes",
            clients[0][1],
            lab.LIFECYCLE_SHUTDOWN_TIMEOUT_SECONDS,
        ),
        (
            node,
            "/map_lifecycle_manager/manage_nodes",
            clients[1][1],
            lab.LIFECYCLE_SHUTDOWN_TIMEOUT_SECONDS,
        ),
    ]


def test_run_lab_creates_persistent_shutdown_clients_before_work_and_closes_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node = SimpleNamespace(create_client=lambda *_args, **_kwargs: None)
    clients = [("/manager/manage_nodes", object())]
    sequence: list[str] = []
    monkeypatch.setattr(
        lab,
        "_create_lifecycle_shutdown_clients",
        lambda observed_node: (
            sequence.append("clients_created"),
            clients,
        )[1]
        if observed_node is node
        else [],
    )
    monkeypatch.setattr(
        lab,
        "_run_lab",
        lambda observed_node, observed_clients: (
            sequence.append("lab_executed"),
            {"clients": observed_clients, "node": observed_node},
        )[1],
    )
    monkeypatch.setattr(
        lab,
        "_close_lifecycle_shutdown_clients",
        lambda observed_node, observed_clients: sequence.append("clients_closed")
        if observed_node is node and observed_clients is clients
        else None,
    )

    report = lab.run_lab(node)

    assert report == {"clients": clients, "node": node}
    assert sequence == ["clients_created", "lab_executed", "clients_closed"]


def test_shutdown_control_path_must_be_ready_before_motion() -> None:
    class Client:
        def wait_for_service(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == lab.LIFECYCLE_SHUTDOWN_READY_TIMEOUT_SECONDS
            return False

    with pytest.raises(RuntimeError, match="lifecycle shutdown service is unavailable"):
        lab._prepare_lifecycle_shutdown_clients(
            [("/manager/manage_nodes", Client())]
        )


def test_lifecycle_shutdown_uses_one_absolute_budget() -> None:
    now = [0.0]

    class Future:
        canceled = False

        def done(self) -> bool:
            return False

        def cancel(self) -> None:
            self.canceled = True

    future = Future()

    class Client:
        wait_timeout = 0.0

        def wait_for_service(self, *, timeout_sec: float) -> bool:
            self.wait_timeout = timeout_sec
            return True

        def call_async(self, _request: object) -> Future:
            return future

    client = Client()

    with pytest.raises(RuntimeError, match="lifecycle shutdown timed out"):
        lab._await_lifecycle_shutdown(
            client,
            object(),
            service_name="/manager/manage_nodes",
            spin_once=lambda timeout: now.__setitem__(0, now[0] + timeout),
            timeout_seconds=0.25,
            clock=lambda: now[0],
        )

    assert client.wait_timeout == pytest.approx(0.25)
    assert now[0] == pytest.approx(0.25)
    assert future.canceled is True


@pytest.mark.parametrize("response", [None, SimpleNamespace(success=False)])
def test_lifecycle_shutdown_rejects_missing_or_negative_response(response: object) -> None:
    class Future:
        def done(self) -> bool:
            return True

        def result(self) -> object:
            return response

    class Client:
        def wait_for_service(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == pytest.approx(1.0)
            return True

        def call_async(self, _request: object) -> Future:
            return Future()

    with pytest.raises(RuntimeError, match="lifecycle shutdown was rejected"):
        lab._await_lifecycle_shutdown(
            Client(),
            object(),
            service_name="/manager/manage_nodes",
            spin_once=lambda _timeout: None,
            timeout_seconds=1.0,
            clock=lambda: 0.0,
        )


