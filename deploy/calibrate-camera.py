#!/usr/bin/env python3
"""Calibrate the robot's camera without a screen attached to it.

`camera_calibration`'s `cameracalibrator` opens an OpenCV window, and this Pi
is headless. The usual answer is X11 forwarding, which means installing XQuartz
on the operator's Mac and logging out and back in. This does the same job with
what the robot already has — OpenCV 4.6 — and uses the MJPEG stream as the
viewfinder: point a browser at
`http://127.0.0.1:8080/stream?topic=/camera/image_raw` through the tunnel and
you can see exactly what the camera sees while this tells you what it still
needs.

## Why this is needed at all

`/camera/camera_info` currently publishes `K = [0,0,0,0,0,0,0,0,0]`. AprilTag
detection can find a tag with no intrinsics, but `mission_gateway`'s
`CalibrationMarker` requires `x`, `y` and `yaw` — a full pose — and a pose
solved against a zero intrinsics matrix is not a weak answer, it is a
meaningless one that `_finite()` will happily accept. `image_proc` cannot
rectify at all: `apriltag_node` subscribes correctly today and receives zero
images because `/image_rect` never appears.

## What makes a calibration good rather than merely finished

Twenty photographs of a board held flat in the middle of the frame will produce
a confident, wrong answer: the solver has nothing to separate focal length from
distance, or principal point from board offset. What separates them is
*variety* — the board near and far, at each edge and corner, and tilted. So
this refuses a view that does not add coverage the collected set lacks, and
says which kind is missing. It is the same rule `cameracalibrator` applies, and
the reason it shows those four progress bars.
"""

from __future__ import annotations

import argparse
import contextlib
import sys
from pathlib import Path

DEFAULT_OUTPUT = Path("/home/ubuntu/.flyto/camera_info/tb3-front.yaml")
DEFAULT_TOPIC = "/camera/image_raw"
DEFAULT_NAME = "tb3-front"

# 9x6 interior corners on a 10x7 board of 20 mm squares, which is what
# deploy/checkerboard-9x6-20mm.svg prints. Interior corners, not squares: a
# 10x7 board has 9x6 points where four squares meet, and giving the square
# count here is the single most common way to get "no board found" forever.
DEFAULT_COLS, DEFAULT_ROWS = 9, 6
DEFAULT_SQUARE_MM = 20.0

# A view has to differ from every kept view in at least one of these by this
# much before it earns a slot. Values are fractions of the frame, or of the
# parameter's own plausible range for skew.
MIN_DISTANCE = {"x": 0.13, "y": 0.13, "size": 0.12, "skew": 0.14}
TARGET_VIEWS = 24
# Below this the solve is not trustworthy no matter what the reprojection error
# says, because a small set can fit itself well and generalise to nothing.
MIN_VIEWS = 12


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="calibrate-camera",
        description="Collect checkerboard views from a ROS image topic and solve intrinsics.",
    )
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--camera-name", default=DEFAULT_NAME)
    parser.add_argument("--cols", type=int, default=DEFAULT_COLS,
                        help="interior corners across (not squares)")
    parser.add_argument("--rows", type=int, default=DEFAULT_ROWS,
                        help="interior corners down (not squares)")
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_MM)
    parser.add_argument("--views", type=int, default=TARGET_VIEWS)
    return parser


class Collector:
    """Keep only views that add coverage, and be able to say what is missing."""

    def __init__(self, board, square_m, target):
        import numpy as np

        self.board = board
        self.target = target
        self.kept = []          # (corners, feature-tuple)
        self.size = None
        cols, rows = board
        grid = np.zeros((cols * rows, 3), np.float32)
        grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
        self.object_points = grid

    @staticmethod
    def _features(corners, width, height):
        """Where the board is, how big, and how far from square-on.

        Skew is measured from the two edges of the outer quadrilateral rather
        than by solving a pose: a board tilted away has one edge visibly shorter
        than its opposite, and that ratio is enough to tell "held flat" from
        "held at an angle" without needing the intrinsics this is trying to find.
        """
        import numpy as np

        pts = corners.reshape(-1, 2)
        x = float(pts[:, 0].mean() / width)
        y = float(pts[:, 1].mean() / height)
        span_x = float(pts[:, 0].max() - pts[:, 0].min())
        span_y = float(pts[:, 1].max() - pts[:, 1].min())
        size = float(np.sqrt(span_x * span_y) / np.sqrt(width * height))
        up = float(np.linalg.norm(pts[0] - pts[-1]))
        cross = float(np.linalg.norm(pts[1] - pts[-2])) or 1.0
        skew = min(1.0, abs(1.0 - up / cross))
        return x, y, size, skew

    def offer(self, corners, width, height):
        """Return (accepted, why-not)."""
        feature = self._features(corners, width, height)
        self.size = (width, height)
        for _, kept in self.kept:
            close = all(
                abs(feature[i] - kept[i]) < MIN_DISTANCE[key]
                for i, key in enumerate(("x", "y", "size", "skew"))
            )
            if close:
                return False, "too like a view already kept"
        self.kept.append((corners, feature))
        return True, ""

    def missing(self):
        """What variety the set still lacks, named the way a person can act on."""
        if not self.kept:
            return "any view at all"
        import numpy as np

        got = np.array([f for _, f in self.kept])
        gaps = []
        if got[:, 0].min() > 0.34:
            gaps.append("the left edge")
        if got[:, 0].max() < 0.66:
            gaps.append("the right edge")
        if got[:, 1].min() > 0.34:
            gaps.append("the top")
        if got[:, 1].max() < 0.66:
            gaps.append("the bottom")
        if got[:, 2].min() > 0.35:
            gaps.append("further away (smaller in frame)")
        if got[:, 2].max() < 0.55:
            gaps.append("closer (filling more of the frame)")
        if got[:, 3].max() < 0.22:
            gaps.append("tilted rather than flat-on")
        return ", ".join(gaps) if gaps else "nothing — coverage looks good"


def write_yaml(path: Path, name, size, matrix, distortion, projection) -> None:
    """The format `camera_calibration_parsers` reads back.

    Written by hand rather than with a YAML library because the robot's runtime
    carries no yaml dependency for this and the shape is fixed.
    """
    width, height = size

    def row(values):
        return ", ".join(f"{v:.10g}" for v in values)

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"image_width: {width}\n"
        f"image_height: {height}\n"
        f"camera_name: {name}\n"
        "camera_matrix:\n  rows: 3\n  cols: 3\n"
        f"  data: [{row(matrix.flatten())}]\n"
        "distortion_model: plumb_bob\n"
        "distortion_coefficients:\n  rows: 1\n  cols: 5\n"
        f"  data: [{row(distortion.flatten()[:5])}]\n"
        "rectification_matrix:\n  rows: 3\n  cols: 3\n"
        "  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
        "projection_matrix:\n  rows: 3\n  cols: 4\n"
        f"  data: [{row(projection.flatten())}]\n",
        encoding="utf-8",
    )


def main(argv=None) -> int:
    args = _parser().parse_args(argv)

    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        print(f"missing dependency: {exc}", file=sys.stderr)
        print("run this on the robot, after `source /opt/ros/jazzy/setup.bash`",
              file=sys.stderr)
        return 2

    board = (args.cols, args.rows)
    collector = Collector(board, args.square_mm / 1000.0, args.views)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    refine = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print(f"Looking for a {args.cols} x {args.rows} interior-corner board "
          f"({args.square_mm:.0f} mm squares) on {args.topic}.")
    print("Watch what the camera sees at "
          "http://127.0.0.1:8080/stream?topic=/camera/image_raw")
    print("Move the board around the frame. Ctrl-C to solve with what you have.\n")

    class Calibrator(Node):
        def __init__(self):
            super().__init__("flyto_camera_calibration")
            self.create_subscription(Image, args.topic, self.on_image,
                                     qos_profile_sensor_data)
            self.seen = 0
            self.found = 0

        def on_image(self, message):
            if message.encoding not in {"rgb8", "bgr8"}:
                return
            self.seen += 1
            # Every frame is a full corner search, which a Pi cannot do at
            # 30 Hz. Sampling keeps the node responsive; the board is moved by
            # hand, so consecutive frames carry no new information anyway.
            if self.seen % 6:
                return
            frame = np.frombuffer(message.data, np.uint8).reshape(
                message.height, message.width, 3)
            grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY
                                if message.encoding == "rgb8" else cv2.COLOR_BGR2GRAY)
            ok, corners = cv2.findChessboardCorners(grey, board, flags)
            if not ok:
                return
            self.found += 1
            corners = cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), refine)
            accepted, why = collector.offer(corners, message.width, message.height)
            kept = len(collector.kept)
            if accepted:
                print(f"  kept {kept}/{args.views}   still need: {collector.missing()}")
            elif self.found % 12 == 0:
                print(f"  ({kept}/{args.views}) {why} — {collector.missing()}")

    rclpy.init()
    node = Calibrator()
    try:
        while rclpy.ok() and len(collector.kept) < args.views:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print("\ninterrupted — solving with what was collected")
    finally:
        # Both guarded, because the path this script tells the operator to take
        # is the one that breaks them. Ctrl-C reaches rclpy's own signal
        # handler first and shuts the context down, so an unconditional
        # `shutdown()` here raises `rcl_shutdown already called` -- a traceback
        # instead of the solve, on the exact keystroke the banner above asks
        # for. `timeout` sending SIGTERM does the same.
        with contextlib.suppress(Exception):
            node.destroy_node()
        if rclpy.ok():
            with contextlib.suppress(Exception):
                rclpy.shutdown()

    kept = len(collector.kept)
    if kept < MIN_VIEWS:
        print(f"\nOnly {kept} usable views. Below {MIN_VIEWS} a solve fits itself "
              f"and generalises to nothing, so nothing was written.", file=sys.stderr)
        return 1

    print(f"\nSolving from {kept} views…")
    size = collector.size
    error, matrix, distortion, _, _ = cv2.calibrateCamera(
        [collector.object_points] * kept,
        [c for c, _ in collector.kept],
        size, None, None,
    )
    projection, _ = cv2.getOptimalNewCameraMatrix(matrix, distortion, size, 0.0, size)
    projection = np.hstack([projection, np.zeros((3, 1))])

    print(f"  reprojection error: {error:.4f} px")
    if error > 1.0:
        print("  that is high. Re-run with more variety, or check the board is "
              "flat and the printed square really measures "
              f"{args.square_mm:.0f} mm.")
    print(f"  fx {matrix[0, 0]:.1f}  fy {matrix[1, 1]:.1f}  "
          f"cx {matrix[0, 2]:.1f}  cy {matrix[1, 2]:.1f}")

    write_yaml(args.output, args.camera_name, size, matrix, distortion, projection)
    print(f"\nWrote {args.output}")
    print("Point the driver at it and restart:")
    print(f"  -p camera_info_url:=file://{args.output}")
    print("  sudo systemctl restart flyto-camera-v4l2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
