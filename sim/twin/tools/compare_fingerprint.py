#!/usr/bin/env python3
"""Compare the twin's ROS interface with the robot's recorded fingerprint.

    docker exec -i flyto-turtlebot3-twin bash -lc \
      'source /opt/ros/jazzy/setup.bash; python3 - ; echo "#ACTIONS"; ros2 action list -t' \
      < tools/fingerprint.py > /tmp/twin.txt
    python3 tools/compare_fingerprint.py config/real/interface-fingerprint.json /tmp/twin.txt

Exits non-zero when anything differs beyond the differences listed in
EXPECTED, which README.md "Different from the robot" explains.
"""

from __future__ import annotations

import json
import sys

# Differences the twin keeps on purpose, or cannot remove.
EXPECTED = {
    # The simulator's time source, and the adapter's simulation marker.
    ("topic only on twin", "/clock"),
    # Advertised like the robot, but the twin produces no frames on them.
    ("rate", "/camera/image_raw/theora"),
    ("rate", "/camera/image_raw/zstd"),
    ("sample", "/camera/image_raw/theora"),
    ("sample", "/camera/image_raw/zstd"),
}
# Depend on the world, the map built so far, or recent activity.
IGNORED_TOPICS = {"/map_metadata"}
IGNORED_PREFIXES = ("/global_costmap/", "/local_costmap/", "/map")
IGNORED_SUFFIXES = ("/_action/status", "/_action/feedback")
TRANSIENT_NODES = ("transform_listener_impl", "launch_ros_", "_ros2cli_daemon", "ro_fingerprint")


def load(path: str) -> dict:
    with open(path) as handle:
        text = handle.read()
    if "#ACTIONS" in text:
        body, actions = text.split("#ACTIONS", 1)
        data = json.loads(body)
        data["actions"] = sorted(line.strip() for line in actions.splitlines() if line.strip())
    else:
        data = json.loads(text)
    return data


def ignored(topic: str) -> bool:
    return (
        topic in IGNORED_TOPICS
        or topic.startswith(IGNORED_PREFIXES)
        or topic.endswith(IGNORED_SUFFIXES)
    )


def compare(robot: dict, twin: dict) -> list[tuple[str, str, str]]:
    found: list[tuple[str, str, str]] = []
    rt, tt = robot["topics"], twin["topics"]
    for topic in sorted(set(rt) - set(tt)):
        found.append(("topic only on robot", topic, rt[topic]["type"]))
    for topic in sorted(set(tt) - set(rt)):
        found.append(("topic only on twin", topic, tt[topic]["type"]))
    for topic in sorted(set(rt) & set(tt)):
        if ignored(topic):
            continue
        a, b = rt[topic], tt[topic]
        if a["type"] != b["type"]:
            found.append(("type", topic, f"{a['type']} != {b['type']}"))
        qa, qb = a.get("qos") or {}, b.get("qos") or {}
        for key in ("reliability", "durability", "nodes"):
            if qa.get(key) != qb.get(key):
                found.append(("qos", topic, f"{key}: {qa.get(key)} != {qb.get(key)}"))
        sa, sb = a.get("sample", {}), b.get("sample", {})
        for key in sorted(set(sa) | set(sb)):
            va, vb = sa.get(key), sb.get(key)
            if key == "voltage":
                continue
            if key == "n_ranges" and {va, vb} <= {399, 400, 401}:
                continue
            if isinstance(va, float) and isinstance(vb, float) and abs(va - vb) < 1e-4:
                continue
            if va != vb:
                found.append(("sample", topic, f"{key}: {va} != {vb}"))
        if (a["hz"] > 0) != (b["hz"] > 0):
            found.append(("rate", topic, f"{a['hz']} Hz != {b['hz']} Hz"))

    def frames(data: dict) -> set[str]:
        topics = data["topics"]
        return set(topics.get("/tf", {}).get("pairs", [])) | set(
            topics.get("/tf_static", {}).get("pairs", [])
        )

    for pair in sorted(frames(robot) ^ frames(twin)):
        side = "robot" if pair in frames(robot) else "twin"
        found.append(("transform only on " + side, pair, ""))
    for action in sorted(set(robot["actions"]) ^ set(twin["actions"])):
        found.append(("action differs", action, ""))

    def nodes(data: dict) -> set[str]:
        return {n for n in data["nodes"] if not any(t in n for t in TRANSIENT_NODES)}

    for node in sorted(nodes(robot) ^ nodes(twin)):
        found.append(("node only on " + ("robot" if node in nodes(robot) else "twin"), node, ""))
    for service in sorted(set(robot["services"]) ^ set(twin["services"])):
        side = "robot" if service in robot["services"] else "twin"
        found.append(("service only on " + side, service, ""))
    return found


def main() -> int:
    robot, twin = load(sys.argv[1]), load(sys.argv[2])
    unexpected = 0
    for kind, name, detail in compare(robot, twin):
        expected = (kind, name) in EXPECTED
        unexpected += not expected
        print(f"{'expected  ' if expected else 'DIFFERENT '} {kind:22s} {name} {detail}")
    print(f"{unexpected} unexpected difference(s)")
    return 1 if unexpected else 0


if __name__ == "__main__":
    sys.exit(main())
