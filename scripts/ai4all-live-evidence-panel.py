#!/usr/bin/env python3
"""Render a reloadable evidence panel from one planning and Gazebo run."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    return value if isinstance(value, dict) else None


def _write_text_atomic(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(value, encoding="utf-8")
    os.replace(temporary, path)


def _write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    _write_text_atomic(
        path,
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
    )


def _write_panel_image_atomic(
    path: Path,
    panel: str,
    font_path: Path,
    font_index: int,
) -> None:
    """Render the panel through Pillow so Traditional Chinese stays intact."""
    from PIL import Image, ImageDraw, ImageFont

    path.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGB", (520, 1020), color="#0B1420")
    font = ImageFont.truetype(str(font_path), size=22, index=font_index)
    draw = ImageDraw.Draw(image)
    draw.multiline_text(
        (24, 24),
        panel,
        fill="white",
        font=font,
        spacing=10,
    )
    temporary = path.with_name(f".{path.name}.tmp")
    image.save(temporary, format="PNG")
    os.replace(temporary, path)


def _planning_facts(planning: dict[str, Any]) -> dict[str, Any]:
    rounds = planning.get("rounds", [])
    usable_rounds = [item for item in rounds if isinstance(item, dict)]
    first = usable_rounds[0] if usable_rounds else {}
    final = usable_rounds[-1] if usable_rounds else {}

    def response(round_data: dict[str, Any]) -> dict[str, Any]:
        value = round_data.get("response", {})
        return value if isinstance(value, dict) else {}

    def attestation(round_data: dict[str, Any]) -> dict[str, Any]:
        value = response(round_data).get("attestation", {})
        return value if isinstance(value, dict) else {}

    def route(round_data: dict[str, Any]) -> str:
        return str(attestation(round_data).get("selected_route_id", "unknown"))

    def candidates(round_data: dict[str, Any]) -> int | None:
        value = round_data.get("route_evaluation", {})
        if not isinstance(value, dict):
            return None
        count = value.get("candidate_count")
        return int(count) if isinstance(count, (int, float)) else None

    resource_change = planning.get("resource_change", {})
    if not isinstance(resource_change, dict):
        resource_change = {}
    before = resource_change.get("before", {})
    after = resource_change.get("after", {})
    before_healthy = before.get("healthy") if isinstance(before, dict) else None
    after_healthy = after.get("healthy") if isinstance(after, dict) else None
    final_attestation = attestation(final)
    return {
        "planning_mode": str(planning.get("planning_mode", "unknown")),
        "model": str(final_attestation.get("model", "unknown")),
        "initial_route": route(first),
        "selected_route": route(final),
        "initial_candidate_count": candidates(first),
        "selected_candidate_count": candidates(final),
        "resource_id": str(resource_change.get("resource_id", "unknown")),
        "resource_before_healthy": before_healthy,
        "resource_after_healthy": after_healthy,
        "plan_sha256": str(final_attestation.get("plan_sha256", "unknown")),
    }


def _first_action(actions: list[dict[str, Any]], kind: str) -> dict[str, Any] | None:
    return next((item for item in actions if item.get("kind") == kind), None)


def _fault_action(
    actions: list[dict[str, Any]], detail: str
) -> dict[str, Any] | None:
    return next(
        (
            item
            for item in actions
            if item.get("kind") == "fault_injection"
            and item.get("detail") == detail
            and item.get("success") is True
        ),
        None,
    )


def _mark(done: bool, failed: bool = False) -> str:
    if failed:
        return "✕"
    return "✓" if done else "·"


def _route_label(route_id: object) -> str:
    replacements = {"yellow": "黃", "orange": "橘", "purple": "紫", "green": "綠"}
    return "—".join(replacements.get(part, part) for part in str(route_id).split("-"))


def _render_panel(
    planning: dict[str, Any], driver: dict[str, Any] | None
) -> tuple[str, dict[str, Any]]:
    facts = _planning_facts(planning)
    raw_actions = driver.get("actions", []) if driver else []
    actions = [item for item in raw_actions if isinstance(item, dict)]
    trolley_entered = _fault_action(actions, "obstacle_enter")
    stopped = _first_action(actions, "safety_stop_observed")
    trolley_exited = _fault_action(actions, "obstacle_exit")
    resumed = _first_action(actions, "motion_resumed_observed")
    item_rejected = _first_action(actions, "item_rejected")
    item_verified = _first_action(actions, "item_verified")
    recipient_rejected = _first_action(actions, "recipient_rejected")
    recipient_verified = _first_action(actions, "recipient_verified")
    unlocked = _first_action(actions, "container_unlocked")
    completed = _first_action(actions, "handoff_completed")

    stop_range = stopped.get("minimum_range") if stopped else None
    stop_limit = stopped.get("configured_stop_distance") if stopped else None
    stop_command = stopped.get("latest_command_velocity", {}) if stopped else {}
    if not isinstance(stop_command, dict):
        stop_command = {}
    stop_detail = ""
    if isinstance(stop_range, (int, float)) and isinstance(stop_limit, (int, float)):
        stop_detail = f"  LiDAR {stop_range:.3f} < {stop_limit:.2f} m"
    command_detail = ""
    if stop_command:
        command_detail = (
            f"  cmd v={float(stop_command.get('linear_x', 0.0)):.3f} "
            f"w={float(stop_command.get('angular_z', 0.0)):.3f}"
        )

    lines = [
        "Flyto2｜同步證據",
        "━━━━━━━━━━━━━━━━━━",
        "規劃證據｜執行前已產生",
        f"模型  {facts['model']}",
        f"模式  {facts['planning_mode']}",
        (
            "Camera B  健康 → 故障"
            if facts["resource_before_healthy"] is True
            and facts["resource_after_healthy"] is False
            else f"資源  {facts['resource_id']}"
        ),
        f"路線  {_route_label(facts['initial_route'])} → {_route_label(facts['selected_route'])}",
        (
            f"候選  {facts['initial_candidate_count']} → {facts['selected_candidate_count']}"
            if facts["initial_candidate_count"] is not None
            else "候選  未提供"
        ),
        "",
        "Gazebo 證據｜同回合即時",
        f"[{_mark(trolley_entered is not None)}] 醫療推車進入路線",
        f"[{_mark(stopped is not None)}] 安全停止",
    ]
    if stop_detail:
        lines.append(stop_detail)
    if command_detail:
        lines.append(command_detail)
    lines.extend(
        [
            f"[{_mark(trolley_exited is not None and resumed is not None)}] 推車離開，恢復行駛",
            f"[{_mark(item_rejected is not None, item_rejected is not None)}] B13 拒絕｜保持上鎖",
            f"[{_mark(item_verified is not None)}] A12 藥袋通過",
            (
                f"[{_mark(recipient_rejected is not None, recipient_rejected is not None)}] "
                "病人 13 拒絕｜保持上鎖"
            ),
            f"[{_mark(recipient_verified is not None)}] 病人 12 通過",
            f"[{_mark(unlocked is not None)}] 兩項正確才解鎖",
            f"[{_mark(completed is not None)}] 送藥流程完成",
            "",
            "來源｜planning-session.json",
            "      driver-manifest.json",
            "面板與 Gazebo 同一段錄影",
        ]
    )
    state = {
        "contract_version": "flyto.robotics.live-evidence-panel.v1",
        "planning": facts,
        "latest_driver_sequence": max(
            (int(item.get("sequence", 0)) for item in actions), default=0
        ),
        "status": {
            "trolley_entered": trolley_entered is not None,
            "safety_stop_observed": stopped is not None,
            "trolley_exited": trolley_exited is not None,
            "motion_resumed_observed": resumed is not None,
            "item_rejected": item_rejected is not None,
            "item_verified": item_verified is not None,
            "recipient_rejected": recipient_rejected is not None,
            "recipient_verified": recipient_verified is not None,
            "container_unlocked": unlocked is not None,
            "handoff_completed": completed is not None,
        },
        "stop_evidence": {
            "minimum_range": stop_range,
            "configured_stop_distance": stop_limit,
            "command_velocity": stop_command or None,
        },
    }
    return "\n".join(lines) + "\n", state


def _append_new_actions(
    path: Path,
    actions: list[dict[str, Any]],
    seen_sequences: set[int],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        for action in actions:
            sequence = action.get("sequence")
            if not isinstance(sequence, int) or sequence in seen_sequences:
                continue
            record = {
                "contract_version": "flyto.robotics.live-panel-event.v1",
                "observed_at_epoch": round(time.time(), 6),
                "source": "images/driver-manifest.json",
                "source_sequence": sequence,
                "kind": action.get("kind"),
                "at_seconds": action.get("at_seconds"),
                "detail": action.get("detail"),
                "minimum_range": action.get("minimum_range"),
                "latest_command_velocity": action.get("latest_command_velocity"),
                "container_locked": action.get("container_locked"),
            }
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            stream.flush()
            seen_sequences.add(sequence)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planning-session", type=Path, required=True)
    parser.add_argument("--driver-manifest", type=Path, required=True)
    parser.add_argument("--panel-text", type=Path, required=True)
    parser.add_argument("--panel-image", type=Path, required=True)
    parser.add_argument("--font-file", type=Path, required=True)
    parser.add_argument("--font-index", type=int, default=3)
    parser.add_argument("--state-output", type=Path, required=True)
    parser.add_argument("--events-output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--poll-seconds", type=float, default=0.1)
    args = parser.parse_args()
    if not 0.05 <= args.poll_seconds <= 2.0:
        parser.error("--poll-seconds must be between 0.05 and 2.0")
    planning = _read_json(args.planning_session)
    if planning is None:
        parser.error("planning session must be valid JSON")
    if not args.font_file.is_file():
        parser.error("font file must exist")

    seen_sequences: set[int] = set()
    previous_panel = ""
    while True:
        driver = _read_json(args.driver_manifest)
        actions = driver.get("actions", []) if driver else []
        usable_actions = [item for item in actions if isinstance(item, dict)]
        _append_new_actions(args.events_output, usable_actions, seen_sequences)
        panel, state = _render_panel(planning, driver)
        if panel != previous_panel:
            state["updated_at_epoch"] = round(time.time(), 6)
            _write_text_atomic(args.panel_text, panel)
            _write_panel_image_atomic(
                args.panel_image,
                panel,
                args.font_file,
                args.font_index,
            )
            _write_json_atomic(args.state_output, state)
            previous_panel = panel
        if not args.ready_file.exists():
            _write_text_atomic(args.ready_file, "ready\n")
        if args.stop_file.exists():
            break
        time.sleep(args.poll_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
