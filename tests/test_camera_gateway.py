import signal
import subprocess
import sys
import textwrap
import time

import pytest

from flyto_robotics import camera_gateway


class FakeServer:
    def __init__(self, events, *, shutdown_error=None):
        self.events = events
        self.shutdown_error = shutdown_error

    def shutdown(self):
        self.events.append("server.shutdown")
        if self.shutdown_error:
            raise self.shutdown_error

    def server_close(self):
        self.events.append("server.close")


class FakeSource:
    def __init__(self, events, *, start_error=None):
        self.events = events
        self.start_error = start_error

    def start(self):
        self.events.append("source.start")
        if self.start_error:
            raise self.start_error

    def stop(self):
        self.events.append("source.stop")


class FakeThread:
    def __init__(self, events, *, start_error=None):
        self.events = events
        self.start_error = start_error

    def start(self):
        self.events.append("thread.start")
        if self.start_error:
            raise self.start_error

    def join(self, timeout=None):
        self.events.append(("thread.join", timeout))


class ControlledEvent:
    def __init__(self, action=None):
        self.action = action
        self.was_set = False

    def set(self):
        self.was_set = True

    def wait(self):
        if self.action:
            self.action()


def runtime(events, *, event=None, source=None, thread=None, server=None):
    return camera_gateway._AvfoundationGatewayRuntime(
        server or FakeServer(events), source or FakeSource(events),
        thread or FakeThread(events), stop_event=event or ControlledEvent(),
    )


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_operator_signals_cleanly_stop_and_restore_handlers(monkeypatch, signum, capsys):
    events = []
    handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    installed = {}

    def fake_signal(number, handler):
        previous = handlers[number]
        handlers[number] = handler
        installed.setdefault(number, handler)
        return previous

    event = ControlledEvent(lambda: installed[signum](signum, None))
    monkeypatch.setattr(camera_gateway.signal, "signal", fake_signal)

    runtime(events, event=event).run()

    assert event.was_set
    assert events == [
        "thread.start", "source.start", "server.shutdown", "source.stop",
        "server.close", ("thread.join", 2.0),
    ]
    assert handlers[signal.SIGINT] is not installed[signal.SIGINT]
    assert handlers[signal.SIGTERM] is not installed[signal.SIGTERM]
    assert capsys.readouterr().err == ""


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_real_child_process_operator_signal_exits_cleanly_and_bounded(signum):
    program = textwrap.dedent(
        """
        import threading

        from flyto_robotics.camera_gateway import _AvfoundationGatewayRuntime

        class Server:
            def shutdown(self):
                pass

            def server_close(self):
                pass

        class Source:
            def start(self):
                print("READY", flush=True)

            def stop(self):
                pass

        class Thread:
            def start(self):
                pass

            def join(self, timeout=None):
                pass

        _AvfoundationGatewayRuntime(
            Server(), Source(), Thread(), stop_event=threading.Event()
        ).run()
        """
    )
    child = subprocess.Popen(
        [sys.executable, "-c", program],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert child.stdout.readline() == "READY\n"
        started = time.monotonic()
        child.send_signal(signum)
        stdout, stderr = child.communicate(timeout=5.0)
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate()

    assert time.monotonic() - started < 5.0
    assert child.returncode == 0
    assert stdout == ""
    assert stderr == ""


def test_keyboard_interrupt_is_an_explicit_clean_exit(monkeypatch, capsys):
    events = []
    event = ControlledEvent(lambda: (_ for _ in ()).throw(KeyboardInterrupt()))
    handlers = {signal.SIGINT: object(), signal.SIGTERM: object()}
    monkeypatch.setattr(
        camera_gateway.signal, "signal",
        lambda number, handler: handlers.update({number: handler}) or handlers[number],
    )

    runtime(events, event=event).run()

    assert event.was_set
    assert "source.stop" in events
    assert capsys.readouterr().err == ""


def test_normal_stop_is_ordered_bounded_and_idempotent():
    events = []
    owner = runtime(events)
    owner.start()

    owner.stop()
    owner.stop()

    assert events == [
        "thread.start", "source.start", "server.shutdown", "source.stop",
        "server.close", ("thread.join", 2.0),
    ]


def test_partially_started_thread_is_closed_without_shutdown_or_join():
    events = []
    owner = runtime(events, thread=FakeThread(events, start_error=RuntimeError("start")))

    with pytest.raises(RuntimeError, match="start"):
        owner.run()

    assert events == ["thread.start", "source.stop", "server.close"]


def test_source_start_failure_cleans_up_and_propagates():
    events = []
    owner = runtime(events, source=FakeSource(events, start_error=RuntimeError("boom")))

    with pytest.raises(RuntimeError, match="boom"):
        owner.run()

    assert events == [
        "thread.start", "source.start", "server.shutdown", "source.stop",
        "server.close", ("thread.join", 2.0),
    ]


def test_cleanup_continues_but_unexpected_error_still_fails():
    events = []
    owner = runtime(events, server=FakeServer(events, shutdown_error=RuntimeError("bad stop")))
    owner.start()

    with pytest.raises(RuntimeError, match="bad stop"):
        owner.stop()

    assert "source.stop" in events
    assert "server.close" in events
    assert ("thread.join", 2.0) in events


def test_handler_restoration_continues_and_preserves_lifecycle_error(monkeypatch):
    events = []
    originals = {signal.SIGINT: object(), signal.SIGTERM: object()}
    handlers = dict(originals)

    def fake_signal(number, handler):
        if handler is originals[number]:
            events.append(("restore", number))
            if number == signal.SIGINT:
                raise RuntimeError("restore failed")
        previous = handlers[number]
        handlers[number] = handler
        return previous

    monkeypatch.setattr(camera_gateway.signal, "signal", fake_signal)
    event = ControlledEvent(lambda: (_ for _ in ()).throw(RuntimeError("lifecycle failed")))

    with pytest.raises(RuntimeError, match="lifecycle failed"):
        runtime(events, event=event).run()

    assert ("restore", signal.SIGINT) in events
    assert ("restore", signal.SIGTERM) in events


def test_avfoundation_main_clean_stop_returns_success_semantics(monkeypatch):
    class Settings:
        bind = "127.0.0.1"
        port = 19000
        topic = "/camera/image_raw"
        zone = "ward-a"
        freshness_seconds = 2.0
        provider = "avfoundation"
        source_id = "camera-0"

    class Server:
        daemon_threads = False

        def __init__(self, address, _handler):
            assert address == ("127.0.0.1", 19000)

        def serve_forever(self):
            raise AssertionError("fake thread must not execute")

    class Owner:
        def __init__(self, _server, _source, _thread):
            pass

        def run(self):
            return None

    monkeypatch.setattr(camera_gateway, "_camera_settings", lambda: Settings())
    monkeypatch.setattr(camera_gateway, "ThreadingHTTPServer", Server)
    monkeypatch.setattr(camera_gateway, "AvfoundationRuntime", lambda *_args: object())
    monkeypatch.setattr(camera_gateway, "_AvfoundationGatewayRuntime", Owner)

    assert camera_gateway.main([]) is None
