"""What a full rosbridge connection reads continuously, and at what rate.

On 2026-10-12 rosbridge on burger-01 used 75 % of a core (load ~10 on a 4-core
Pi 4B) and the braking guard saw scans up to 0.965 s late while the sensor kept
10 Hz. Measured read-only through the robot's tunnel the same evening, one
adapter connection carried 884 KiB/s, 756 KiB/s of it a raw camera frame a
second that only an unread observation digest used. These tests pin the
connection to what its readers need (``ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS``).
"""

from __future__ import annotations

import pytest

from flyto_robotics.generic_ros2_adapter import (
    DEFAULT_INTERFACES,
    ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS,
    RosbridgeROS2Backend,
)
from tests.test_presence_on_state import RecordingSocket, no_sleep  # noqa: F401 - fixture

#: Measured on 2026-10-12, read-only, KiB/s per topic on one connection as
#: 0.8.1 subscribed them (odometry at the robot's ~18 Hz, the others as sent).
MEASURED_KIB_S = {
    "/camera/image_raw": 756.1,
    "/scan": 98.0,
    "/odom": 13.2,
    "/tf": 10.0,
    "/camera/camera_info": 6.2,
    "/battery_state": 0.4,
}


def subscribed():
    sockets: list[RecordingSocket] = []

    def connect(_url):
        sockets.append(RecordingSocket({}))
        return sockets[-1]

    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=connect)
    requests = {item["topic"]: item for item in sockets[-1].subscribed}
    backend.disconnect()
    return requests


@pytest.mark.usefixtures("no_sleep")
def test_a_full_connection_reads_no_camera_continuously():
    requests = subscribed()
    assert "/camera/image_raw" not in requests
    assert "/camera/camera_info" not in requests
    assert "/camera/image_raw/compressed" not in requests
    # The one frame a capture needs comes from the compressed topic.
    assert DEFAULT_INTERFACES["vision.observe"][1] == "/camera/image_raw/compressed"


@pytest.mark.usefixtures("no_sleep")
def test_the_guard_reads_every_scan_newest_first():
    scan = subscribed()["/scan"]
    assert scan["throttle_rate"] == 0
    assert scan["queue_length"] == 1


@pytest.mark.usefixtures("no_sleep")
def test_odometry_is_read_at_the_scan_rate_newest_first():
    odom = subscribed()["/odom"]
    assert 0 < odom["throttle_rate"] <= 100  # at least 10 Hz
    assert odom["queue_length"] == 1


def test_every_continuous_subscription_names_its_reader():
    for row in ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS:
        assert row.reader.strip(), row.default_topic


@pytest.mark.usefixtures("no_sleep")
def test_the_connection_carries_a_seventh_of_its_measured_bytes():
    requests = subscribed()
    before = sum(MEASURED_KIB_S.values())
    after = 0.0
    for topic, rate in MEASURED_KIB_S.items():
        if topic not in requests:
            continue
        throttle = requests[topic].get("throttle_rate", 0)
        # 0.8.1 read odometry unthrottled at ~18 Hz; a throttle caps it.
        scale = min(1.0, (1000.0 / throttle) / 18.1) if topic == "/odom" and throttle else 1.0
        after += rate * scale
    assert before > 880.0
    assert after < before / 7.0
