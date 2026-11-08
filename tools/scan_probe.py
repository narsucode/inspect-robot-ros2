#!/usr/bin/env python3
"""Summarise /scan by 30-degree sectors: nearest object in each sector.

Laser frame convention (REP-103): 0 deg = front, 90 deg = left,
180 deg = behind, 270 deg = right.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class P(Node):
    def __init__(self):
        super().__init__("scan_probe")
        self.msg = None
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.msg = m


def main():
    rclpy.init()
    n = P()
    end = 0
    while n.msg is None and end < 150 and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        end += 1
    if n.msg is None:
        print("no /scan")
        return 1
    m = n.msg
    ang = [m.angle_min + i * m.angle_increment for i in range(len(m.ranges))]
    print("frame=%s  n=%d  range=[%.2f, %.2f]" % (m.header.frame_id, len(m.ranges), m.range_min, m.range_max))
    for k in range(12):
        lo, hi = k * 30, k * 30 + 30
        best = (9.9, 0.0)
        for a, r in zip(ang, m.ranges):
            d = math.degrees(a) % 360.0
            if lo <= d < hi and math.isfinite(r) and r > 0.02 and r < best[0]:
                best = (r, d)
        label = {0: "front", 3: "left", 6: "back", 9: "right"}.get(k, "")
        print("%3d-%3d deg %-5s min=%5.2f m @ %5.1f deg" % (lo, hi, label, best[0], best[1]))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
