#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

BINS = 400
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / BINS
ANGLE_MAX = ANGLE_MIN + (BINS - 1) * ANGLE_INCREMENT

class Stabilizer(Node):
    def __init__(self):
        super().__init__("laser_scan_stabilizer")
        self.pub = self.create_publisher(LaserScan, "/scan_stable", qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)
    def cb(self, src):
        dst = LaserScan()
        dst.header = src.header
        dst.angle_min = ANGLE_MIN
        dst.angle_max = ANGLE_MAX
        dst.angle_increment = ANGLE_INCREMENT
        dst.scan_time = src.scan_time
        dst.time_increment = (src.scan_time / BINS) if src.scan_time > 0.0 else 0.0
        dst.range_min = src.range_min
        dst.range_max = src.range_max
        dst.ranges = [math.inf] * BINS
        use_intensity = len(src.intensities) == len(src.ranges) and len(src.intensities) > 0
        if use_intensity:
            dst.intensities = [0.0] * BINS
        if not src.ranges or not math.isfinite(src.angle_increment) or src.angle_increment <= 0.0:
            self.pub.publish(dst); return
        for i, value in enumerate(src.ranges):
            angle = src.angle_min + i * src.angle_increment
            wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
            index = int(round((wrapped - ANGLE_MIN) / ANGLE_INCREMENT)) % BINS
            current = dst.ranges[index]
            if math.isfinite(value) and value > 0.0:
                if not math.isfinite(current) or value < current:
                    dst.ranges[index] = float(value)
                    if use_intensity:
                        dst.intensities[index] = float(src.intensities[i])
            elif not math.isfinite(current):
                dst.ranges[index] = float(value)
        self.pub.publish(dst)

def main():
    rclpy.init(); node = Stabilizer()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
