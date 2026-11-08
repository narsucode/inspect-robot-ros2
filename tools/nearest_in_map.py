#!/usr/bin/env python3
"""Print the position of the nearest lidar object in a fixed world frame.

Default frame is odom (works without SLAM/AMCL). Put one box near the robot,
run this, rotate the robot, run it again:
  - same coordinates -> the scan is correctly fixed to the world
  - coordinates move -> scan/TF inconsistency
"""
import argparse
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

import sys
sys.path.insert(0, "/home/ros/narsu/ros2/tools")
from scan_map_check import extract, ScanMapChecker, quat_to_rot  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="odom")
    args = ap.parse_args()

    rclpy.init()
    n = ScanMapChecker("/scan")
    try:
        if not n.wait_scan(15.0):
            print("no /scan")
            return 1
        pts = extract(n.scan)
        if len(pts) == 0:
            print("empty scan")
            return 1
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = r >= 0.15
        if keep.sum() == 0:
            print("all points too close")
            return 1
        pts, r = pts[keep], r[keep]
        k = int(np.argmin(r))

        tf = None
        for target in (Time.from_msg(n.scan.header.stamp), Time()):
            for _ in range(30):
                try:
                    tf = n.tf_buffer.lookup_transform(args.frame, n.scan.header.frame_id, target,
                                                      timeout=Duration(seconds=0.2))
                    break
                except Exception:
                    rclpy.spin_once(n, timeout_sec=0.1)
            if tf is not None:
                break
        if tf is None:
            print("no %s -> %s TF" % (args.frame, n.scan.header.frame_id))
            return 1

        R = quat_to_rot(tf.transform.rotation)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        p = np.array([pts[k, 0], pts[k, 1], 0.0])
        m = R @ p + t
        print("nearest: range=%.2f m  laser_angle=%.1f deg  %s=(%.3f, %.3f)" % (
            r[k], math.degrees(math.atan2(pts[k, 1], pts[k, 0])), args.frame, m[0], m[1]))
        return 0
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
