#!/usr/bin/env python3
"""Fit one live /scan against the saved map and print its pose."""
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/home/ros/narsu/ros2/tools")
from scan_map_check import load_map, global_search, extract, ScanMapChecker  # noqa: E402


def main():
    dist, occupied, free, res, ox, oy = load_map(
        "/home/ros/narsu/ros2/patrol_ws/src/youdaobot_navigation/maps/room.yaml")
    rclpy.init()
    n = ScanMapChecker("/scan")
    try:
        if not n.wait_scan(15.0):
            print("ERROR: no /scan")
            return 1
        pts = extract(n.scan)
        cost, yaw, x, y = global_search(pts, dist, free, res, ox, oy, mirror=False)
        print("pose x=%.3f y=%.3f yaw=%.4f cost=%.3f n=%d" % (x, y, yaw, cost, len(pts)))
        return 0
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
