#!/bin/bash
# Bring the robot's loopback services onto this Mac's loopback.
#
# Everything the robot serves binds to 127.0.0.1 on the robot, and that is not
# a precaution that can be relaxed for convenience:
#
#   * the delivery gateway is the process that drives a physical robot;
#   * flyto_robotics.camera_observation.validate_bind() refuses outright to
#     bind anything that is not a literal IPv4 loopback address;
#   * flyto-cloud's services/space_tasks/streams.py refuses to mint a plaintext
#     stream reference whose host is not 127.0.0.1, ::1 or localhost, so an
#     http:// camera served across the venue wi-fi would not merely be unsafe,
#     it would not appear in the room at all.
#
# So the tunnel is the transport, not a workaround for one. Forwarding to the
# same port numbers locally is what lets flyto-cloud's configuration name
# 127.0.0.1 and be telling the truth.
#
# Usage:  ./robot-tunnel.sh [up|down|status]      (default: up)
set -euo pipefail

ROBOT="${FLYTO_ROBOT_HOST:-ubuntu@flyto-robot.local}"

# port:name — kept in one list so `status` cannot drift from what `up` forwards.
PORTS=(
  "8766:delivery gateway (AI Space 智慧交付)"
  "9000:camera observation (evidence)"
  "8080:camera MJPEG stream (watch)"
)

# A tag this script can find its own tunnel by. ssh does not offer a handle, so
# the control socket is the handle: one path, so a second `up` reuses the first
# connection instead of silently racing it onto the same local ports.
CONTROL="${TMPDIR:-/tmp}/flyto-robot-tunnel.sock"

port_of() { printf '%s' "${1%%:*}"; }
name_of() { printf '%s' "${1#*:}"; }

is_up() { ssh -S "$CONTROL" -O check "$ROBOT" >/dev/null 2>&1; }

up() {
  if is_up; then
    echo "tunnel 已經在跑（control: $CONTROL）"
    status
    return 0
  fi

  # Fail loudly rather than half-forwarding: ExitOnForwardFailure means a port
  # already taken on this Mac stops the whole thing, instead of leaving one
  # service silently unreachable and the room showing a frozen picture.
  local args=(-f -N -M -S "$CONTROL"
              -o ExitOnForwardFailure=yes
              -o ServerAliveInterval=15
              -o ServerAliveCountMax=3)
  local spec
  for spec in "${PORTS[@]}"; do
    local p; p="$(port_of "$spec")"
    args+=(-L "${p}:127.0.0.1:${p}")
  done

  echo "開 tunnel 到 $ROBOT ..."
  if ! ssh "${args[@]}" "$ROBOT"; then
    echo "失敗。常見原因：本機 port 已被占用，或機器人沒開機。" >&2
    echo "檢查占用：lsof -nP -iTCP:8766 -iTCP:9000 -iTCP:8080 -sTCP:LISTEN" >&2
    return 1
  fi
  status
}

down() {
  if ! is_up; then echo "tunnel 沒在跑"; return 0; fi
  ssh -S "$CONTROL" -O exit "$ROBOT" >/dev/null 2>&1 || true
  echo "tunnel 已關閉"
}

status() {
  if is_up; then
    echo "tunnel: 連線中 → $ROBOT"
  else
    echo "tunnel: 未連線"
    return 0
  fi
  local spec
  for spec in "${PORTS[@]}"; do
    local p n
    p="$(port_of "$spec")"; n="$(name_of "$spec")"
    # A forwarded port that accepts a TCP connection only proves ssh is
    # listening here, never that anything answers on the robot — so this says
    # "forwarded", and the service checks below say whether it replies.
    if nc -z 127.0.0.1 "$p" >/dev/null 2>&1; then
      printf '  %-5s ✓ forwarded  — %s\n' "$p" "$n"
    else
      printf '  %-5s ✗ 不通       — %s\n' "$p" "$n"
    fi
  done
}

case "${1:-up}" in
  up)     up ;;
  down)   down ;;
  status) status ;;
  *) echo "用法: $0 [up|down|status]" >&2; exit 2 ;;
esac
