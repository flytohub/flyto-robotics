"""Thin optional ROS 2 adapter and loopback-only HTTP server."""

from __future__ import annotations

import argparse
import json
import os
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .camera_observation import CameraConfigurationError, CameraObservation
from .camera_sources import AvfoundationRuntime, CameraSettings, probe_source


def _camera_settings() -> CameraSettings:
    return CameraSettings.from_environ(os.environ)


def _settings() -> tuple[str, int, str, str, float]:
    """Compatibility projection retained for the original ROS gateway tests."""

    settings = _camera_settings()
    return settings.bind, settings.port, settings.topic, settings.zone, settings.freshness_seconds


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="flyto-camera-gateway")
    parser.add_argument(
        "--check-settings",
        action="store_true",
        help="validate local camera settings without importing ROS 2",
    )
    parser.add_argument(
        "--probe-once", action="store_true",
        help="perform one bounded source probe without starting the gateway",
    )
    return parser


def _emit(result: dict) -> None:
    print(json.dumps(result, separators=(",", ":"), sort_keys=True))


class _AvfoundationGatewayRuntime:
    """Own the AVFoundation source and HTTP server as one stoppable unit."""

    def __init__(self, server, source, thread, *, stop_event=None):
        self.server = server
        self.source = source
        self.thread = thread
        self.stop_event = threading.Event() if stop_event is None else stop_event
        self._stop_lock = threading.Lock()
        self._stopped = False
        self._server_started = False

    def _request_stop(self, _signum=None, _frame=None) -> None:
        self.stop_event.set()

    def start(self) -> None:
        self.thread.start()
        self._server_started = True
        self.source.start()

    def stop(self) -> None:
        """Stop accepting requests, then stop the producer and reap the server."""

        self.stop_event.set()
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True
            errors = []
            if self._server_started:
                try:
                    self.server.shutdown()
                except Exception as exc:
                    errors.append(exc)
            try:
                self.source.stop()
            except Exception as exc:
                errors.append(exc)
            try:
                self.server.server_close()
            except Exception as exc:
                errors.append(exc)
            if self._server_started:
                try:
                    self.thread.join(timeout=2.0)
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise errors[0]

    def run(self) -> None:
        previous_handlers = {}
        error = None
        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous_handlers[signum] = signal.signal(signum, self._request_stop)
            try:
                self.start()
                self.stop_event.wait()
            except KeyboardInterrupt:
                self._request_stop()
        except Exception as exc:
            error = exc
        finally:
            try:
                self.stop()
            except Exception as exc:
                if error is None:
                    error = exc
            for signum, handler in previous_handlers.items():
                try:
                    signal.signal(signum, handler)
                except Exception as exc:
                    if error is None:
                        error = exc
        if error is not None:
            raise error


def main(argv: list[str] | None = None) -> None:  # pragma: no cover - ROS runtime path
    try:
        args = _parser().parse_args(argv)
        settings = _camera_settings()
        observation = CameraObservation(
            settings.zone, settings.freshness_seconds, provider=settings.provider,
            source_id=settings.source_id,
        )
        if args.check_settings:
            _emit({"action_code": "none", "ok": True,
                   "reason": "camera_settings_valid", "usable": False})
            return
        if args.probe_once:
            result = probe_source(settings)
            _emit(result)
            if not result["ok"]:
                raise SystemExit(1)
            return
    except (CameraConfigurationError, ValueError, OSError) as exc:
        reason = str(exc)
        if not reason.startswith("camera_"):
            reason = "camera_settings_invalid"
        _emit({"action_code": "none", "ok": False, "reason": reason, "usable": False})
        raise SystemExit(2) from None

    bind, port, topic = settings.bind, settings.port, settings.topic

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def _serve(self):
            length = self.headers.get("Content-Length", "0")
            try:
                size = int(length)
            except ValueError:
                size = 1025
            result = observation.handle(self.command, self.path, request_size=size)
            self.send_response(result.status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(result.body)))
            if result.allow:
                self.send_header("Allow", result.allow)
            self.end_headers()
            self.wfile.write(result.body)

        do_GET = do_POST = do_PUT = do_DELETE = do_PATCH = _serve
        do_HEAD = do_OPTIONS = do_CONNECT = do_TRACE = _serve

        def log_message(self, _format, *_args):
            return

    server = ThreadingHTTPServer((bind, port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    if settings.provider == "avfoundation":
        source = AvfoundationRuntime(settings, observation)
        _AvfoundationGatewayRuntime(server, source, thread).run()
        return

    thread.start()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class CameraNode(Node):
        def __init__(self):
            super().__init__("flyto_camera_observation")
            self.create_subscription(Image, topic, self.receive, qos_profile_sensor_data)

        def receive(self, message):
            observation.accept_image(encoding=message.encoding, width=message.width,
                                     height=message.height, step=message.step, data=message.data)

    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
