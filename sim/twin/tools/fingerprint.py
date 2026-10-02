"""Read-only ROS 2 interface fingerprint: subscribes only, publishes nothing.

Run it inside the ROS environment of the robot or the twin and keep stdout:

    ssh ubuntu@flyto-robot.local 'source /opt/ros/jazzy/setup.bash; \
      ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 -; \
      echo "#ACTIONS"; ros2 action list -t' < tools/fingerprint.py > robot.txt

It records every topic with its type, QoS, publishing nodes, rate over six
seconds and a few header fields, plus nodes and services. Subscribing to every
topic at once loads the robot, so rates read lower than unloaded.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

SKIP = {"/rosout", "/parameter_events"}
PARAMETER_SERVICES = (
    "/get_parameters",
    "/set_parameters",
    "/list_parameters",
    "/describe_parameters",
    "/get_parameter_types",
    "/set_parameters_atomically",
    "/get_type_description",
)
FIELDS = (
    "child_frame_id",
    "encoding",
    "width",
    "height",
    "range_min",
    "range_max",
    "angle_min",
    "angle_max",
    "angle_increment",
    "distortion_model",
    "format",
)
WINDOW_S = 6.0


def qos_of(node, topic):
    publishers = node.get_publishers_info_by_topic(topic)
    if not publishers:
        return None
    profile = publishers[0].qos_profile
    return {
        "publishers": len(publishers),
        "reliability": profile.reliability.name,
        "durability": profile.durability.name,
        "nodes": sorted(
            {p.node_namespace.rstrip("/") + "/" + p.node_name for p in publishers}
        ),
    }


def summarize(message):
    summary = {}
    header = getattr(message, "header", None)
    if header is not None:
        summary["frame_id"] = header.frame_id
    for field in FIELDS:
        if hasattr(message, field):
            value = getattr(message, field)
            summary[field] = round(value, 5) if isinstance(value, float) else value
    if hasattr(message, "ranges"):
        summary["n_ranges"] = len(message.ranges)
    if hasattr(message, "k"):
        summary["k_nonzero"] = any(abs(x) > 0 for x in message.k)
    info = getattr(message, "info", None)
    if info is not None and hasattr(info, "resolution"):
        summary["map"] = [info.width, info.height, round(info.resolution, 3)]
    if hasattr(message, "transforms"):
        summary["pairs"] = sorted(
            {f"{t.header.frame_id}->{t.child_frame_id}" for t in message.transforms}
        )
    if type(message).__name__ == "JointState":
        summary["names"] = list(message.name)
    if type(message).__name__ == "BatteryState":
        summary["voltage"] = round(message.voltage, 2)
    return summary


def main():
    rclpy.init()
    node = Node("ro_fingerprint")
    time.sleep(2.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    topics = {name: types[0] for name, types in node.get_topic_names_and_types()}
    info = {}
    subscriptions = []
    for topic, type_name in sorted(topics.items()):
        if topic in SKIP:
            continue
        qos = qos_of(node, topic)
        info[topic] = {"type": type_name, "qos": qos, "count": 0}
        if qos is None:
            continue
        try:
            message_type = get_message(type_name)
        except Exception as error:  # an interface package missing on this host
            info[topic]["error"] = str(error)
            continue
        profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT
            if qos["reliability"] == "BEST_EFFORT"
            else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
            if qos["durability"] == "TRANSIENT_LOCAL"
            else DurabilityPolicy.VOLATILE,
        )

        def callback(message, topic=topic):
            entry = info[topic]
            entry["count"] += 1
            summary = summarize(message)
            if "pairs" in summary:
                entry.setdefault("pairs", set()).update(summary.pop("pairs"))
            entry["sample"] = summary

        subscriptions.append(node.create_subscription(message_type, topic, callback, profile))

    started = time.time()
    while time.time() - started < WINDOW_S:
        rclpy.spin_once(node, timeout_sec=0.02)
    for entry in info.values():
        entry["hz"] = round(entry.pop("count") / WINDOW_S, 1)
        if "pairs" in entry:
            entry["pairs"] = sorted(entry["pairs"])
    result = {
        "nodes": sorted(
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in node.get_node_names_and_namespaces()
            if name != "ro_fingerprint"
        ),
        "topics": info,
        "services": sorted(
            name
            for name, _ in node.get_service_names_and_types()
            if not name.endswith(PARAMETER_SERVICES)
        ),
    }
    json.dump(result, sys.stdout, indent=1, default=str)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
