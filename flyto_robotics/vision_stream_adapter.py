#!/usr/bin/env python3
"""A conformance adapter over a vision gateway that serves stream addresses.

The camera end of the reference design. `flyto-modules-vision` reads
observations and deliberately never opens a device or decodes a pixel; this
does neither either. It asks a gateway which address serves a given resource
and hands that back, so the whole path from camera to browser has exactly one
component that touches media, and it is not any of ours.

## Why this is not written for USB

A USB camera is what exists today. A camera on the robot, a screen capture, and
whatever comes after are the same shape: something continuous that a media
server already knows how to ingest. Writing this against ``/dev/video0`` would
mean rewriting it three times.

So the gateway is asked one question — *what address serves this resource* —
and the answer is a URL. Whether the far end is a webcam, a robot publishing
RTSP, or a headless browser is a line in the media server's config and nothing
this file, Cloud, or the room ever learns.

**Radar is not this.** LiDAR and range readings are numbers, and a number that
has been through a video codec is a picture of a number. They stay on
``sensing.range`` and the ``ranges`` surface, which is why the vocabulary keeps
``vision.stream`` and ``sensing.range`` as two capabilities rather than one
"live" one.

## Which gateway, and where

Two exist, and they serve the same contract at different paths.

Today it is flyto-cloud's own local backend — the host with the camera attached
— at ``/api/spaces/zone-camera/streams``, which is the default here because it
is the one that exists. `flyto-modules-vision`'s architecture names the same
seam and the same succession: *"a vision gateway — today flyto-cloud's desktop
backend … later the robot's own."*

Later it is the robot's, which serves ``/v1/streams``. Point this at it with
``FLYTO_VISION_STREAM_PATH`` and nothing else changes, because what moved is an
address and not a contract.

## What the gateway must serve

::

    GET <the path above>
    {
      "contract_version": "flyto.vision.stream-catalog.v1",
      "streams": [
        {
          "resource_id": "cam-z3",
          "zone_id": "zone-03",
          "protocol": "whep",
          "url": "https://media.local:8889/zone3/whep",
          "label": "Zone 3 overhead",
          "ttl_seconds": 120,
          "measured_latency_ms": 210
        }
      ]
    }

That is the entire contract. No frame, no still, no thumbnail — the gateway is
answering *where*, and anything it returned that was not a URL would be the
thing this design replaced.

Run it with::

    FLYTO_VISION_GATEWAY_URL=http://127.0.0.1:8000 \\
    python scripts/run_adapter_conformance.py \\
        scripts.adapters.vision_stream_adapter:build
"""

from __future__ import annotations

import json
import os
import urllib.request
from collections.abc import Mapping, Sequence
from typing import Any

from . import adapter_contract as decl
from . import stream_contract as st
from .adapter_contract import (
    OUTCOME_REFUSED,
    CallRequest,
    CallResult,
)

CATALOG_CONTRACT = "flyto.vision.stream-catalog.v1"

DEFAULT_GATEWAY_URL = "http://127.0.0.1:9000"
GATEWAY_URL_ENV = "FLYTO_VISION_GATEWAY_URL"

# Where on that gateway the catalog is. Defaulted to the route that exists
# rather than the one a robot will serve later: pointing at a path nothing
# answers on produces a silent empty catalog, which reads as "this Space has no
# cameras" and sends somebody looking for a hardware fault.
CATALOG_PATH_ENV = "FLYTO_VISION_STREAM_PATH"
DEFAULT_CATALOG_PATH = "/api/spaces/zone-camera/streams"

MAX_RESPONSE_BYTES = 256 * 1024
MAX_STREAMS = 64

# What the gateway may say, mapped onto what a browser opens. Declared rather
# than passed through, so a gateway inventing a protocol name reaches a refusal
# here instead of a black rectangle in the room.
GATEWAY_TO_PROTOCOL: Mapping[str, str] = {
    "whep": st.PROTOCOL_WHEP,
    "webrtc": st.PROTOCOL_WEBRTC,
    "mjpeg": st.PROTOCOL_MJPEG,
    "hls": st.PROTOCOL_HLS,
}

assert set(GATEWAY_TO_PROTOCOL.values()) <= st.PROTOCOLS, (
    "every gateway protocol must land on one the reference vocabulary admits"
)


def gateway_url() -> str:
    return (os.environ.get(GATEWAY_URL_ENV) or DEFAULT_GATEWAY_URL).rstrip("/")


def catalog_path() -> str:
    path = (os.environ.get(CATALOG_PATH_ENV) or DEFAULT_CATALOG_PATH).strip()
    return path if path.startswith("/") else f"/{path}"


def _fetch(path: str, *, opener=urllib.request.urlopen) -> Any:
    """One bounded GET. Nothing here posts, and nothing here follows a redirect
    into somewhere the operator did not configure."""
    request = urllib.request.Request(f"{gateway_url()}{path}", method="GET")
    with opener(request, timeout=10) as response:
        raw = response.read(MAX_RESPONSE_BYTES + 1)
    if len(raw) > MAX_RESPONSE_BYTES:
        raise ValueError("stream catalog exceeded the response ceiling")
    return json.loads(raw.decode("utf-8"))


class VisionStreamAdapter:
    """The conformance methods over a gateway that answers *where*."""

    def __init__(self, *, fetch=None):
        self._fetch = fetch or _fetch
        self.unmapped: list[tuple[str, str]] = []
        self._catalog: list[Mapping[str, Any]] = []

    # -- describe ------------------------------------------------------------

    def describe(self) -> Sequence[decl.CapabilityDeclaration]:
        """Every camera the gateway serves, as declarations nobody approved yet.

        ``vision.stream`` is read-only in the shipped vocabulary, so nothing
        here needs a safe stop or a permission — but it still needs approving,
        because who may watch a room is a decision and not a property of the
        camera being plugged in.
        """
        payload = self._fetch(catalog_path())
        if not isinstance(payload, dict):
            raise ValueError("stream catalog was not an object")
        if payload.get("contract_version") != CATALOG_CONTRACT:
            raise ValueError(
                f"stream catalog is {payload.get('contract_version')!r}, "
                f"not {CATALOG_CONTRACT}"
            )
        rows = list(payload.get("streams") or ())[:MAX_STREAMS]
        self._catalog = rows
        out: list[decl.CapabilityDeclaration] = []
        for row in rows:
            resource_id = str(row.get("resource_id") or "").strip()
            if not resource_id:
                self.unmapped.append(("", "a stream with no resource id"))
                continue
            protocol = str(row.get("protocol") or "")
            if protocol not in GATEWAY_TO_PROTOCOL:
                self.unmapped.append((resource_id, f"protocol {protocol!r} unknown"))
                continue
            try:
                out.append(
                    decl.declare(
                        capability_id=st.STREAM_CAPABILITY,
                        resource_id=resource_id,
                        # A media server is not this platform and not a robot.
                        executor_kind=decl.EXECUTOR_EXTERNAL_API,
                        source=decl.SOURCE_DEVICE,
                        runtime_name=protocol,
                    )
                )
            except decl.DeclarationRefused as error:
                self.unmapped.append((resource_id, str(error)))
        return out

    # -- the fifth method, required only of something that streams -----------

    def stream_reference(self, resource_id: str) -> st.StreamReference | None:
        """Where to watch this resource, if the gateway serves it.

        Minted through :func:`streams.mint`, not assembled here, so a gateway
        offering a plaintext address across a network or a reference with no
        expiry is refused by the same code that refuses it everywhere else. An
        adapter that built its own reference could quietly skip that.
        """
        row = next(
            (
                item
                for item in self._catalog
                if str(item.get("resource_id") or "") == resource_id
            ),
            None,
        )
        if row is None:
            return None
        declaration = decl.approve(
            decl.declare(
                capability_id=st.STREAM_CAPABILITY,
                resource_id=resource_id,
                executor_kind=decl.EXECUTOR_EXTERNAL_API,
                source=decl.SOURCE_DEVICE,
            ),
            # A bench run stands in for the operator. In the room this comes
            # from the approval queue and never from the adapter.
            actor="conformance-bench",
        )
        ttl = int(row.get("ttl_seconds") or st.DEFAULT_TTL_SECONDS)
        measured = row.get("measured_latency_ms")
        return st.mint(
            declaration,
            protocol=GATEWAY_TO_PROTOCOL[str(row.get("protocol"))],
            url=str(row.get("url") or ""),
            # The gateway states when its address stops working. Computing one
            # here would be this process inventing an authority it does not have.
            expires_at=str(row.get("expires_at") or "gateway-stated"),
            zone_id=str(row.get("zone_id") or ""),
            label=str(row.get("label") or ""),
            audio=bool(row.get("audio", False)),
            measured_latency_ms=None if measured is None else int(measured),
            ttl_seconds=ttl,
        )

    # -- the other three -----------------------------------------------------

    def invoke(self, request: CallRequest) -> CallResult:
        """Watching is not a call.

        ``vision.stream`` has no invocation: an operator opens an address and
        closes it. Reporting a completed call here would put an entry in a
        task's evidence for something that produced nothing, which is the
        ``action.execution`` mistake in a new place.
        """
        if request.capability_id != st.STREAM_CAPABILITY:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=f"{request.capability_id} is not one this gateway serves",
            )
        return CallResult(
            request.call_id,
            OUTCOME_REFUSED,
            detail="vision.stream is opened through stream_reference, not dispatched",
        )

    def cancel(self, call_id: str) -> CallResult:
        """Nothing is in flight to withdraw. Closing the tab is the cancel."""
        return CallResult(
            call_id, OUTCOME_REFUSED, detail="a stream is watched, not dispatched"
        )

    def safe_stop(self) -> CallResult:
        """A camera has nothing to stop.

        Reported as refused rather than completed, because answering "stopped"
        about a device that never moves would be a stop that proved nothing —
        and the check only runs at all when something declared needs one.
        """
        return CallResult(
            "safe-stop", OUTCOME_REFUSED, detail="a camera has nothing to bring to rest"
        )


def build() -> VisionStreamAdapter:
    return VisionStreamAdapter()


if __name__ == "__main__":
    adapter = build()
    declarations = adapter.describe()
    print(f"gateway: {gateway_url()}")
    for item in declarations:
        reference = adapter.stream_reference(item.resource_id)
        where = f"{reference.protocol} {reference.url}" if reference else "no address"
        delay = f"{reference.latency_ms}ms" if reference else "-"
        print(f"  {item.resource_id:<14} {item.approval_status:<11} {delay:<8} {where}")
    for identifier, why in adapter.unmapped:
        print(f"  UNMAPPED {identifier or '(no id)'}: {why}")
    print(f"\n{len(declarations)} camera(s), {len(adapter.unmapped)} unusable")
    print("None are watchable until somebody approves them.")
