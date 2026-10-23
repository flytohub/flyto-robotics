#!/usr/bin/env python3
"""Render the README's demo GIF from the deterministic dry-run.

It runs the same loop as ``flyto_robotics.cli dry_run`` (the real
``MissionController`` against planar kinematics, with the same two-second
obstruction) and draws each tick from above. Nothing is staged: the path, the
stop and the arrival are what the controller did. Pillow is needed only here:

    python -m pip install pillow
    python scripts/render_readme_demo.py docs/assets/demo-dry-run.gif
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from flyto_robotics.contracts import load_job  # noqa: E402
from flyto_robotics.mission import MissionController, Pose2D, normalize_angle  # noqa: E402

JOB = ROOT / "examples/jobs/pharmacy-to-ward.json"
WIDTH, HEIGHT = 760, 440
# The floor in metres (x0, y0, x1, y1). It is drawn turned a quarter: the
# pharmacy-to-ward axis (y) runs left to right, x runs up.
FLOOR = (-0.4, -2.6, 3.5, 2.6)
BACKGROUND = (13, 17, 23)
GRID = (33, 38, 45)
TRAIL = (110, 93, 252)
ROBOT = (237, 242, 247)
STATION = (63, 185, 80)
STOP = (248, 81, 73)
TEXT = (201, 209, 217)
MUTED = (125, 133, 144)
EVERY = 4  # draw every fourth 50 ms tick: about four times real time
FRAME_MS = 50
HOLD_MS = 1800

LABELS = {
    "accepted": "Goal accepted",
    "navigating_to_pickup": "Driving to the pharmacy",
    "waiting_for_pickup": "At the pharmacy: loading",
    "navigating_to_dropoff": "Driving to Ward A",
    "waiting_for_dropoff": "At Ward A: unloading",
    "completed": "Delivered: stopped within 0.2 m of Ward A",
}


def font(size: int) -> ImageFont.ImageFont:
    for name in (
        "DejaVuSans.ttf",
        "/System/Library/Fonts/SFNS.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
    ):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def simulate() -> list[tuple[float, Pose2D, str, bool]]:
    """Every tick of the dry-run: time, pose, mission state, obstructed."""
    job = load_job(JOB)
    controller = MissionController(job)
    pose, now, step = Pose2D(0.0, 0.0, 0.0), 0.0, 0.05
    ticks = []
    for _ in range(math.ceil((job.safety.mission_timeout_seconds + 1.0) / step)):
        obstructed = 2.0 <= now < 4.0
        command = controller.tick(pose, minimum_range=0.25 if obstructed else math.inf, now=now)
        ticks.append((now, pose, command.state.value, obstructed))
        pose = Pose2D(
            x=pose.x + command.linear_x * math.cos(pose.yaw) * step,
            y=pose.y + command.linear_x * math.sin(pose.yaw) * step,
            yaw=normalize_angle(pose.yaw + command.angular_z * step),
        )
        now += step
        if controller.terminal:
            ticks.append((now, pose, controller.state.value, False))
            break
    return ticks


def to_px(x: float, y: float) -> tuple[float, float]:
    x0, y0, x1, y1 = FLOOR
    scale = min((WIDTH - 80) / (y1 - y0), (HEIGHT - 130) / (x1 - x0))
    left = (WIDTH - (y1 - y0) * scale) / 2
    return left + (y - y0) * scale, 75 + (x1 - x) * scale


def frame(ticks, index: int, fonts) -> Image.Image:
    now, pose, state, obstructed = ticks[index]
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    x0, y0, x1, y1 = FLOOR
    for gx in range(math.ceil(x0), math.floor(x1) + 1):
        draw.line([to_px(gx, y0), to_px(gx, y1)], fill=GRID)
    for gy in range(math.ceil(y0), math.floor(y1) + 1):
        draw.line([to_px(x0, gy), to_px(x1, gy)], fill=GRID)
    for name, (sx, sy) in (("Pharmacy", (3.0, -2.0)), ("Ward A", (3.0, 2.0))):
        cx, cy = to_px(sx, sy)
        draw.rounded_rectangle(
            [cx - 16, cy - 16, cx + 16, cy + 16], radius=5, outline=STATION, width=3
        )
        draw.text(
            (cx - draw.textlength(name, font=fonts["small"]) / 2, cy - 42),
            name,
            fill=STATION,
            font=fonts["small"],
        )
    trail = [to_px(p.x, p.y) for _, p, _, _ in ticks[: index + 1]]
    if len(trail) > 1:
        draw.line(trail, fill=TRAIL, width=4, joint="curve")
    rx, ry = to_px(pose.x, pose.y)
    if obstructed:
        ox, oy = to_px(pose.x + 0.35 * math.cos(pose.yaw), pose.y + 0.35 * math.sin(pose.yaw))
        draw.rectangle([ox - 9, oy - 9, ox + 9, oy + 9], fill=STOP)
        draw.ellipse([rx - 30, ry - 30, rx + 30, ry + 30], outline=STOP, width=2)
    draw.ellipse([rx - 13, ry - 13, rx + 13, ry + 13], fill=ROBOT)
    hx, hy = to_px(pose.x + 0.32 * math.cos(pose.yaw), pose.y + 0.32 * math.sin(pose.yaw))
    draw.line([(rx, ry), (hx, hy)], fill=BACKGROUND, width=4)
    status = (
        "Obstacle ahead: stopped, waiting for a clear path"
        if obstructed
        else LABELS.get(state, state)
    )
    draw.text(
        (20, 18),
        "“Take the sealed bin from the pharmacy to Ward A.”",
        fill=TEXT,
        font=fonts["title"],
    )
    draw.text((20, HEIGHT - 34), status, fill=STOP if obstructed else TEXT, font=fonts["body"])
    clock = f"t = {now:4.1f} s"
    draw.text(
        (WIDTH - 20 - draw.textlength(clock, font=fonts["small"]), HEIGHT - 32),
        clock,
        fill=MUTED,
        font=fonts["small"],
    )
    tag = "Simulated: python -m flyto_robotics.cli dry-run"
    draw.text(
        (WIDTH - 20 - draw.textlength(tag, font=fonts["small"]), 22),
        tag,
        fill=MUTED,
        font=fonts["small"],
    )
    return image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    ticks = simulate()
    if ticks[-1][2] != "completed":
        raise SystemExit(f"the dry-run did not complete: {ticks[-1][2]}")
    fonts = {"title": font(19), "body": font(18), "small": font(14)}
    indices = list(range(0, len(ticks), EVERY))
    if indices[-1] != len(ticks) - 1:
        indices.append(len(ticks) - 1)
    frames = [frame(ticks, i, fonts) for i in indices]
    durations = [FRAME_MS] * (len(frames) - 1) + [HOLD_MS]
    palette = [f.convert("P", palette=Image.Palette.ADAPTIVE, colors=32) for f in frames]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    palette[0].save(
        args.output,
        save_all=True,
        append_images=palette[1:],
        duration=durations,
        loop=0,
        optimize=True,
    )
    print(f"{args.output}: {len(frames)} frames, {args.output.stat().st_size // 1024} KiB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
