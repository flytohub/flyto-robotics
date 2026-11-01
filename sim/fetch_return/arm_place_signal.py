"""SIMULATION place signal from the MuJoCo arm host's simulation control.

The flyto-cloud e2e host (scripts/e2e_closed_loop/e2e_host.py) already serves
``GET /sim/truth`` on ``127.0.0.1:$E2E_SIM_CONTROL_PORT`` (default 7102). Its
``inside`` field is MuJoCo ground truth: the block rests in ``bin-blue``, the
arm world's proxy for the docked robot's tray. A place is the moment that
truth turns from False to True.

Reading it needs no change to Cloud. Only a rising edge counts: a block that
is already inside when the watcher starts is never reported, because nothing
says the robot was parked when it got there. An unreachable host or a reading
without a boolean ``inside`` is no signal, and it leaves the last known value
alone so one dropped poll can neither invent nor hide a place.
"""

from __future__ import annotations

import json
import os
import urllib.request
from typing import Any, Callable

DEFAULT_CONTROL_PORT = 7102


def truth_url() -> str:
    port = int(os.getenv("E2E_SIM_CONTROL_PORT", str(DEFAULT_CONTROL_PORT)))
    return f"http://127.0.0.1:{port}/sim/truth"


def http_fetch(url: str, timeout_s: float = 1.0) -> Callable[[], Any]:
    def fetch() -> Any:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            return json.loads(response.read())

    return fetch


class ArmTruthWatcher:
    """Turns polled arm ground truth into place edges."""

    def __init__(self, *, fetch: Callable[[], Any]) -> None:
        self._fetch = fetch
        self._inside: bool | None = None
        self.last_error: str | None = None

    def poll(self) -> bool:
        """True exactly once per transition of the block into the tray proxy."""
        try:
            reading = self._fetch()
        except Exception as error:  # an unreachable host is no signal
            self.last_error = f"{type(error).__name__}: {error}"[:200]
            return False
        inside = reading.get("inside") if isinstance(reading, dict) else None
        if not isinstance(inside, bool):
            self.last_error = "sim_truth_without_boolean_inside"
            return False
        self.last_error = None
        previous, self._inside = self._inside, inside
        return previous is False and inside is True
