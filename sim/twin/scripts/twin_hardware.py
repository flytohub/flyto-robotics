#!/usr/bin/env python3
"""Stand-in for the TurtleBot3's hardware drivers, fed by Gazebo.

On flyto-robot.local four driver nodes sit between the hardware and ROS:

  turtlebot3_node        OpenCR: /imu, /joint_states, /battery_state,
                         /sensor_state, /magnetic_field, /cmd_vel, services
  diff_drive_controller  /odom and the odom -> base_footprint transform
  lidar_node             LDS-03 (coin_d4 driver): /scan
  /camera/v4l2_camera    USB webcam: /camera/image_raw and transports

This process runs nodes with the same names, topics, message fields, QoS,
rates and parameters, reading Gazebo's own transport (`twin/*` topics in the
model) instead of hardware. Nothing here is a Flyto2 component; it only makes
the simulated robot present the robot's ROS interface. Every value below that
is not read from Gazebo was measured on the robot on 2026-10-02 and is
recorded in config/real/.
"""

from __future__ import annotations

import math
import random
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, TwistStamped
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.imu_pb2 import IMU as GzImu
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.model_pb2 import Model as GzModel
from gz.msgs10.odometry_pb2 import Odometry as GzOdometry
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    CompressedImage,
    Image,
    Imu,
    JointState,
    LaserScan,
    MagneticField,
)
from sensor_msgs.srv import SetCameraInfo
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from theora_image_transport.msg import Packet
from turtlebot3_msgs.msg import SensorState
from turtlebot3_msgs.srv import Sound

try:
    import cv2
except ImportError:  # JPEG transport is then unavailable, raw still works.
    cv2 = None

RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
BEST_EFFORT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
)

# OpenCR firmware (turtlebot3 burger): velocity limits and the motor watchdog
# that stops the wheels when no command arrives for 500 ms.
MAX_LINEAR_MPS = 0.22
MAX_ANGULAR_RPS = 2.84
CMD_VEL_TIMEOUT_S = 0.5
# Dynamixel XL430 encoder resolution used by turtlebot3_node.
TICKS_PER_RAD = 4096 / (2 * math.pi)
# Battery: the robot reports percentage = (V - 10.5) / 1.8 * 100 (fits all
# three readings taken on 2026-10-02) and design_capacity 1.8. The twin holds
# a charged pack.
TWIN_BATTERY_V = 12.0


class SimClock:
    """Latest Gazebo simulation time, shared by every emulator node."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sec = 0
        self._nsec = 0

    def update(self, sec: int, nsec: int) -> None:
        with self._lock:
            self._sec, self._nsec = sec, nsec

    def now(self) -> Time:
        with self._lock:
            return Time(sec=self._sec, nanosec=self._nsec)

    def seconds(self) -> float:
        with self._lock:
            return self._sec + self._nsec * 1e-9


def while_running(callback):
    """Wrap a Gazebo callback so it is a no-op once rclpy has shut down."""

    def guarded(msg):
        if rclpy.ok():
            callback(msg)

    return guarded


def stamp_of(gz_msg, clock: SimClock) -> Time:
    header = getattr(gz_msg, "header", None)
    if header is not None and header.HasField("stamp"):
        return Time(sec=header.stamp.sec, nanosec=header.stamp.nsec)
    return clock.now()


class Turtlebot3Node(Node):
    """OpenCR driver: IMU, wheel joints, battery, sensor state, cmd_vel."""

    def __init__(self, gz: GzNode, clock: SimClock, odom: DiffDriveController) -> None:
        super().__init__("turtlebot3_node", automatically_declare_parameters_from_overrides=True)
        self._sim = clock
        self._odom = odom
        self._lock = threading.Lock()
        self._torque = True
        self._last_cmd = 0.0
        self._moving = False
        self._joints: dict[str, tuple[float, float]] = {}

        self._imu = self.create_publisher(Imu, "imu", RELIABLE)
        self._joint_pub = self.create_publisher(JointState, "joint_states", RELIABLE)
        self._battery = self.create_publisher(BatteryState, "battery_state", RELIABLE)
        self._sensor = self.create_publisher(SensorState, "sensor_state", RELIABLE)
        self._mag = self.create_publisher(MagneticField, "magnetic_field", RELIABLE)
        self._clock_pub = self.create_publisher(Clock, "/clock", RELIABLE)
        self.create_subscription(TwistStamped, "cmd_vel", self._on_cmd_vel, RELIABLE)
        self.create_service(SetBool, "motor_power", self._on_motor_power)
        self.create_service(Trigger, "reset", self._on_reset)
        self.create_service(Trigger, "reset_odometry", self._on_reset_odometry)
        self.create_service(Sound, "sound", self._on_sound)

        self._cmd = gz.advertise("/twin/cmd_vel", GzTwist)
        gz.subscribe(GzImu, "/twin/imu", while_running(self._on_gz_imu))
        gz.subscribe(GzModel, "/twin/joint_states", while_running(self._on_gz_joints))
        gz.subscribe(GzClock, "/clock", while_running(self._on_gz_clock))
        self._last_clock_pub = 0.0
        # The robot publishes these at ~20 Hz from one control loop.
        self.create_timer(0.05, self._publish_state)
        self.create_timer(0.05, self._watchdog)

    # Gazebo -> ROS -----------------------------------------------------
    def _on_gz_clock(self, msg: GzClock) -> None:
        self._sim.update(msg.sim.sec, msg.sim.nsec)
        now = msg.sim.sec + msg.sim.nsec * 1e-9
        # Gazebo ticks at 1 kHz; 250 Hz is plenty for use_sim_time consumers.
        if now - self._last_clock_pub >= 0.004 or now < self._last_clock_pub:
            self._last_clock_pub = now
            self._clock_pub.publish(Clock(clock=Time(sec=msg.sim.sec, nanosec=msg.sim.nsec)))

    def _on_gz_imu(self, msg: GzImu) -> None:
        out = Imu()
        out.header.stamp = stamp_of(msg, self._sim)
        out.header.frame_id = "imu_link"
        o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        out.orientation.x, out.orientation.y = o.x, o.y
        out.orientation.z, out.orientation.w = o.z, o.w
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = w.x, w.y, w.z
        out.linear_acceleration.x = a.x
        out.linear_acceleration.y = a.y
        out.linear_acceleration.z = a.z
        # The robot leaves every covariance at zero.
        self._imu.publish(out)

    def _on_gz_joints(self, msg: GzModel) -> None:
        with self._lock:
            for joint in msg.joint:
                self._joints[joint.name] = (joint.axis1.position, joint.axis1.velocity)

    def _publish_state(self) -> None:
        stamp = self._sim.now()
        with self._lock:
            left = self._joints.get("wheel_left_joint", (0.0, 0.0))
            right = self._joints.get("wheel_right_joint", (0.0, 0.0))
            torque = self._torque
        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = "base_link"
        joints.name = ["wheel_left_joint", "wheel_right_joint"]
        joints.position = [left[0], right[0]]
        joints.velocity = [left[1], right[1]]
        self._joint_pub.publish(joints)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = TWIN_BATTERY_V
        battery.design_capacity = 1.8
        battery.percentage = (TWIN_BATTERY_V - 10.5) / 1.8 * 100.0
        battery.present = True
        self._battery.publish(battery)

        sensor = SensorState()
        sensor.header.stamp = stamp
        sensor.torque = torque
        sensor.left_encoder = int(round(left[0] * TICKS_PER_RAD))
        sensor.right_encoder = int(round(right[0] * TICKS_PER_RAD))
        sensor.battery = TWIN_BATTERY_V
        self._sensor.publish(sensor)

        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = "imu_link"
        self._mag.publish(mag)

    # ROS -> Gazebo -----------------------------------------------------
    def _send(self, linear: float, angular: float) -> None:
        msg = GzTwist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd.publish(msg)

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        with self._lock:
            if not self._torque:
                return
            self._last_cmd = time.monotonic()
            self._moving = True
        linear = max(-MAX_LINEAR_MPS, min(MAX_LINEAR_MPS, msg.twist.linear.x))
        angular = max(-MAX_ANGULAR_RPS, min(MAX_ANGULAR_RPS, msg.twist.angular.z))
        self._send(linear, angular)

    def _watchdog(self) -> None:
        with self._lock:
            expired = self._moving and time.monotonic() - self._last_cmd > CMD_VEL_TIMEOUT_S
            if expired:
                self._moving = False
        if expired:
            self._send(0.0, 0.0)

    # Services ----------------------------------------------------------
    def _on_motor_power(self, request: SetBool.Request, response: SetBool.Response):
        with self._lock:
            self._torque = bool(request.data)
            self._moving = False
        if not request.data:
            self._send(0.0, 0.0)
        response.success = True
        return response

    def _on_reset(self, _request, response: Trigger.Response):
        self._odom.reset()
        response.success = True
        return response

    def _on_reset_odometry(self, _request, response: Trigger.Response):
        self._odom.reset()
        response.success = True
        return response

    def _on_sound(self, _request, response):
        response.success = True
        return response


class DiffDriveController(Node):
    """Wheel odometry and the odom -> base_footprint transform."""

    def __init__(self, gz: GzNode, clock: SimClock) -> None:
        super().__init__(
            "diff_drive_controller", automatically_declare_parameters_from_overrides=True
        )
        self._sim = clock
        self._lock = threading.Lock()
        self._origin = (0.0, 0.0, 0.0)  # x, y, yaw subtracted after a reset
        self._latest = (0.0, 0.0, 0.0)
        self._odom = self.create_publisher(Odometry, "odom", RELIABLE)
        self._tf = TransformBroadcaster(self)
        gz.subscribe(GzOdometry, "/twin/odom", while_running(self._on_gz_odom))

    def reset(self) -> None:
        with self._lock:
            self._origin = self._latest

    def _on_gz_odom(self, msg: GzOdometry) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._latest = (p.x, p.y, yaw)
            ox, oy, oyaw = self._origin
        dx, dy = p.x - ox, p.y - oy
        x = math.cos(-oyaw) * dx - math.sin(-oyaw) * dy
        y = math.sin(-oyaw) * dx + math.cos(-oyaw) * dy
        rel_yaw = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
        qz, qw = math.sin(rel_yaw / 2), math.cos(rel_yaw / 2)
        stamp = stamp_of(msg, self._sim)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = msg.twist.linear.x
        odom.twist.twist.angular.z = msg.twist.angular.z
        self._odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self._tf.sendTransform(tf)


class Lds03Node(Node):
    """LDS-03 through the coin_d4 driver, as measured on the robot.

    Reproduced: 399/400/401 readings per turn over [0, 2*pi] with
    angle_increment = 2*pi / (n + 1); range_min 0.1, range_max 100;
    no-return readings reported as 0.0 (about 15 per turn, in clusters) with
    intensity 66-97; about 0.9 NaN per turn; valid intensity 219-247; static
    noise median 3.4 mm, 90th percentile 17 mm; BEST_EFFORT; ~10 Hz.
    """

    # Beyond this the twin reports no return. The robot never returned more
    # than 4.73 m indoors; 12 m is the LDS-03 class figure, not a measurement.
    MAX_RETURN_M = 12.0
    TIME_INCREMENT = 0.00025026750518009067
    SCAN_TIME = 0.10010699927806854

    def __init__(self, gz: GzNode, clock: SimClock) -> None:
        super().__init__("lidar_node", automatically_declare_parameters_from_overrides=True)
        self._sim = clock
        self._rng = random.Random()
        self._pub = self.create_publisher(LaserScan, "scan", BEST_EFFORT)
        gz.subscribe(GzLaserScan, "/twin/scan", while_running(self._on_gz_scan))

    def _noise(self) -> float:
        sigma = 0.0034 if self._rng.random() < 0.85 else 0.017
        return self._rng.gauss(0.0, sigma)

    def _on_gz_scan(self, msg: GzLaserScan) -> None:
        source = list(msg.ranges)
        if not source:
            return
        bins = len(source)
        n = self._rng.choices((399, 400, 401), weights=(1, 2, 1))[0]
        increment = 2 * math.pi / (n + 1)
        ranges: list[float] = []
        intensities: list[float] = []
        for i in range(n):
            r = source[int(round(i * increment / (2 * math.pi) * bins)) % bins]
            if math.isfinite(r) and 0.1 <= r <= self.MAX_RETURN_M:
                ranges.append(max(0.1, r + self._noise()))
                intensities.append(float(min(247, max(0, round(self._rng.gauss(222, 6))))))
            else:
                ranges.append(0.0)
                intensities.append(float(self._rng.randint(66, 97)))
        # Dropout clusters (dark or glancing surfaces on the real sensor).
        dropped = 0
        target = max(0, int(self._rng.gauss(13, 3)))
        while dropped < target:
            start = self._rng.randrange(n)
            for j in range(self._rng.randint(2, 6)):
                k = (start + j) % n
                if ranges[k] != 0.0:
                    ranges[k] = 0.0
                    intensities[k] = float(self._rng.randint(66, 97))
                    dropped += 1
        nan_count = 1 if self._rng.random() < 0.87 else 0
        for _ in range(nan_count):
            ranges[self._rng.randrange(n)] = float("nan")

        out = LaserScan()
        out.header.stamp = stamp_of(msg, self._sim)
        out.header.frame_id = "base_scan"
        out.angle_min = 0.0
        out.angle_max = 2 * math.pi
        out.angle_increment = increment
        out.time_increment = self.TIME_INCREMENT
        out.scan_time = self.SCAN_TIME
        out.range_min = 0.1
        out.range_max = 100.0
        out.ranges = ranges
        out.intensities = intensities
        self._pub.publish(out)


class V4l2CameraNode(Node):
    """USB webcam through v4l2_camera: 640x480 yuv422_yuy2 at 30 Hz.

    The robot's camera is uncalibrated, so camera_info carries all-zero K, R,
    P and an empty distortion model; the twin reports the same until a real
    calibration exists. Transports: raw and JPEG are produced; compressedDepth,
    theora and zstd advertise like the robot but publish nothing.
    """

    def __init__(self, gz: GzNode, clock: SimClock) -> None:
        super().__init__(
            "v4l2_camera", namespace="camera", automatically_declare_parameters_from_overrides=True
        )
        self._sim = clock
        self._lock = threading.Lock()
        self._frame: bytes | None = None
        self._info = CameraInfo(height=480, width=640)
        self._info.header.frame_id = "camera"
        self._raw = self.create_publisher(Image, "image_raw", RELIABLE)
        self._info_pub = self.create_publisher(CameraInfo, "camera_info", RELIABLE)
        self._jpeg = self.create_publisher(CompressedImage, "image_raw/compressed", RELIABLE)
        self.create_publisher(CompressedImage, "image_raw/compressedDepth", RELIABLE)
        self.create_publisher(Packet, "image_raw/theora", RELIABLE)
        self.create_publisher(CompressedImage, "image_raw/zstd", RELIABLE)
        self.create_service(SetCameraInfo, "~/set_camera_info", self._on_set_camera_info)
        gz.subscribe(GzImage, "/twin/camera", while_running(self._on_gz_image))
        # Publish at the camera's 30 Hz even when software rendering is slower:
        # consumers check rate and freshness, which a stalled renderer must not
        # silently change.
        self.create_timer(1.0 / 30.0, self._publish)

    @staticmethod
    def _rgb_to_yuyv(rgb: np.ndarray) -> bytes:
        rgb = rgb.astype(np.float32)
        r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
        y = 0.299 * r + 0.587 * g + 0.114 * b
        u = -0.169 * r - 0.331 * g + 0.5 * b + 128
        v = 0.5 * r - 0.419 * g - 0.081 * b + 128
        u = (u[:, 0::2] + u[:, 1::2]) / 2
        v = (v[:, 0::2] + v[:, 1::2]) / 2
        out = np.empty((rgb.shape[0], rgb.shape[1] * 2), dtype=np.uint8)
        out[:, 0::4] = np.clip(y[:, 0::2], 0, 255)
        out[:, 1::4] = np.clip(u, 0, 255)
        out[:, 2::4] = np.clip(y[:, 1::2], 0, 255)
        out[:, 3::4] = np.clip(v, 0, 255)
        return out.tobytes()

    def _on_gz_image(self, msg: GzImage) -> None:
        if msg.width != 640 or msg.height != 480:
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(480, 640, 3)
        with self._lock:
            self._frame = self._rgb_to_yuyv(rgb)
            self._rgb = rgb

    def _publish(self) -> None:
        with self._lock:
            frame = self._frame
            rgb = getattr(self, "_rgb", None)
            info = self._info
        if frame is None:
            return
        stamp = self._sim.now()
        image = Image(height=480, width=640, encoding="yuv422_yuy2", is_bigendian=0, step=1280)
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        image.data = frame
        self._raw.publish(image)
        info.header.stamp = stamp
        self._info_pub.publish(info)
        if cv2 is not None and rgb is not None and self._jpeg.get_subscription_count() > 0:
            ok, jpeg = cv2.imencode(".jpg", rgb[..., ::-1])
            if ok:
                compressed = CompressedImage(format="yuv422_yuy2; jpeg compressed bgr8")
                compressed.header = image.header
                compressed.data = jpeg.tobytes()
                self._jpeg.publish(compressed)

    def _on_set_camera_info(self, request, response):
        info = request.camera_info
        info.header.frame_id = "camera"
        with self._lock:
            self._info = info
        response.success = True
        response.status_message = ""
        return response


def main() -> None:
    # The robot's parameter dumps arrive as --ros-args --params-file ... and
    # are applied to each node by its fully qualified name.
    rclpy.init(args=sys.argv)
    gz = GzNode()
    clock = SimClock()
    odom = DiffDriveController(gz, clock)
    nodes = [
        Turtlebot3Node(gz, clock, odom),
        odom,
        Lds03Node(gz, clock),
        V4l2CameraNode(gz, clock),
    ]
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
