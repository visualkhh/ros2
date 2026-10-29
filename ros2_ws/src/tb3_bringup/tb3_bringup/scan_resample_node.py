"""Resample LaserScan to a fixed number of beams.

LDS-03 (coin_d4_driver) emits a slightly different beam count every revolution (395..404).
The SLAM front-end (slam_toolbox when this was written) registers the sensor with the first scan's count and drops every later scan whose count
differs ("LaserRangeScan contains 403 range readings, expected 404") -> no map. This node bins each
scan into `beams` fixed angles over 360 deg (nearest beam per bin, closest range wins).

Empty bin = 0.0 (below range_min = invalid), NOT inf: on this robot the rear ~135 deg is blocked by the
robot body (lidar on the 1st floor front) and returns nothing. inf would mean "nothing up to max range",
so SLAM/costmap would ray-trace free space through the robot body behind it.

  sub scan_in (LaserScan)  ->  pub scan_out (LaserScan, fixed beams)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanResample(Node):
    def __init__(self):
        super().__init__('scan_resample')
        self.beams = int(self.declare_parameter('beams', 400).value)   # LDS-03: 0.9 deg
        self.inc = 2.0 * math.pi / self.beams
        self.pub = self.create_publisher(LaserScan, 'scan_out', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_in', self.cb, qos_profile_sensor_data)

    def cb(self, msg: LaserScan) -> None:
        out = [0.0] * self.beams   # 0 < range_min -> ignored by SLAM / costmap
        a = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                i = int(round((a + math.pi) / self.inc)) % self.beams
                if out[i] == 0.0 or r < out[i]:
                    out[i] = r
            a += msg.angle_increment
        res = LaserScan()
        res.header = msg.header
        res.angle_min = -math.pi
        res.angle_increment = self.inc
        res.angle_max = -math.pi + self.inc * (self.beams - 1)
        res.time_increment = msg.scan_time / self.beams if msg.scan_time > 0 else 0.0
        res.scan_time = msg.scan_time
        res.range_min = msg.range_min
        res.range_max = msg.range_max
        res.ranges = out
        self.pub.publish(res)


def main(args=None):
    rclpy.init(args=args)
    node = ScanResample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
