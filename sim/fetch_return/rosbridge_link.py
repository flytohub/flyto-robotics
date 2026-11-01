"""Minimal rosbridge v2 websocket link for SIMULATION side processes.

Advertise and publish one topic, subscribe to others, and restore both after
a reconnect. Nothing here moves a robot: the motion adapter has its own,
supervised rosbridge backend in flyto_robotics.generic_ros2_adapter.
"""

from __future__ import annotations

import contextlib
import inspect
import json
import threading
from typing import Any, Callable

BOOL_TYPE = "std_msgs/msg/Bool"
ODOMETRY_TYPE = "nav_msgs/msg/Odometry"


def _websocket(url: str) -> Any:
    try:
        from websockets.sync.client import connect
    except ImportError as error:  # pragma: no cover - install-time error
        raise RuntimeError("websockets>=12 is required for the rosbridge link") from error
    options: dict[str, Any] = {"open_timeout": 3, "close_timeout": 1, "max_size": None}
    # websockets 17.1 deprecates a bare connect() unless legacy=True; older
    # releases (>=12) do not know the keyword. The link owns the lifetime.
    if "legacy" in inspect.signature(connect).parameters:
        options["legacy"] = True
    return connect(url, **options)


class RosbridgeLink:
    def __init__(
        self,
        url: str,
        *,
        connection_factory: Callable[[str], Any] = _websocket,
    ) -> None:
        self.url = url
        self._factory = connection_factory
        self._lock = threading.Lock()
        self._ws: Any | None = None
        self._reader: threading.Thread | None = None
        self._advertised: dict[str, dict[str, Any]] = {}
        self._subscriptions: dict[str, tuple[dict[str, Any], Callable[[Any], None]]] = {}

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._ws is not None

    def _send_on(self, ws: Any, payload: dict[str, Any]) -> None:
        ws.send(json.dumps(payload, separators=(",", ":"), allow_nan=False))

    def connect(self) -> None:
        """(Re)connect, then restore every advertisement and subscription."""
        self._drop()
        ws = self._factory(self.url)
        with self._lock:
            self._ws = ws
            for request, _ in self._subscriptions.values():
                self._send_on(ws, request)
            for request in self._advertised.values():
                self._send_on(ws, request)
        self._reader = threading.Thread(
            target=self._read, args=(ws,), name="rosbridge-link-reader", daemon=True
        )
        self._reader.start()

    def advertise(self, topic: str, message_type: str, *, latch: bool = False) -> None:
        # rosbridge on ROS 2 maps latch to a transient-local publisher, so a
        # subscriber that joins later still receives the last value.
        request = {"op": "advertise", "topic": topic, "type": message_type, "latch": latch}
        with self._lock:
            self._advertised[topic] = request
            if self._ws is not None:
                self._send_on(self._ws, request)

    def subscribe(
        self,
        topic: str,
        message_type: str,
        handler: Callable[[Any], None],
        *,
        throttle_rate_ms: int = 100,
    ) -> None:
        request = {
            "op": "subscribe",
            "topic": topic,
            "type": message_type,
            "throttle_rate": throttle_rate_ms,
            "queue_length": 1,
        }
        with self._lock:
            self._subscriptions[topic] = (request, handler)
            if self._ws is not None:
                self._send_on(self._ws, request)

    def publish(self, topic: str, message: dict[str, Any]) -> None:
        with self._lock:
            if self._ws is None:
                raise ConnectionError("rosbridge link disconnected")
            ws = self._ws
            try:
                self._send_on(ws, {"op": "publish", "topic": topic, "msg": message})
            except Exception as error:
                self._ws = None
                raise ConnectionError(f"rosbridge publish failed: {error}") from error

    def _read(self, ws: Any) -> None:
        while True:
            with self._lock:
                if ws is not self._ws:
                    return
            try:
                raw = ws.recv(timeout=0.5)
            except TimeoutError:
                continue
            except Exception:
                with self._lock:
                    if ws is self._ws:
                        self._ws = None
                return
            try:
                payload = json.loads(raw)
            except (TypeError, ValueError):
                continue
            if not isinstance(payload, dict) or payload.get("op") != "publish":
                continue
            with self._lock:
                entry = self._subscriptions.get(payload.get("topic"))
            if entry is not None:
                entry[1](payload.get("msg"))

    def _drop(self) -> None:
        with self._lock:
            ws, self._ws = self._ws, None
        if ws is not None:
            with contextlib.suppress(Exception):
                ws.close()

    def close(self) -> None:
        self._drop()
        reader = self._reader
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=2)
