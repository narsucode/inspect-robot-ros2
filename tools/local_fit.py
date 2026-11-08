#!/usr/bin/env python3
"""Check whether a good scan fit exists near the current AMCL pose."""
import math
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/home/ros/narsu/ros2/tools")
from scan_map_check import load_map, extract, ScanMapChecker  # noqa: E402


def main():
    dist, occ, free, res, ox, oy = load_map(
        "/home/ros/narsu/ros2/patrol_ws/src/youdaobot_navigation/maps/room.yaml")
    rclpy.init()
    n = ScanMapChecker("/scan")
    if not n.wait_scan(15.0):
        print("missing scan")
        return 1
    pts = extract(n.scan)
    tf = n.lookup(n.scan.header.stamp)
    if tf is None:
        print("missing map->base TF")
        return 1
    t, q = tf.transform.translation, tf.transform.rotation
    yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    x0, y0 = t.x, t.y
    rows, cols = dist.shape
    best = (9.9, 0, 0, 0)
    for dx in np.arange(-0.4, 0.401, 0.05):
        for dy in np.arange(-0.4, 0.401, 0.05):
            for dyaw in np.arange(-0.35, 0.351, 0.02):
                yaw = yaw0 + dyaw
                R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                p = pts @ R.T + np.array([x0 + dx, y0 + dy])
                ci = ((p[:, 0] - ox) / res).astype(int)
                ri = (rows - 1 - (p[:, 1] - oy) / res).astype(int)
                ok = (ci >= 0) & (ci < cols) & (ri >= 0) & (ri < rows)
                if ok.sum() < 30:
                    continue
                c = float(dist[ri[ok], ci[ok]].mean())
                if c < best[0]:
                    best = (c, x0 + dx, y0 + dy, yaw)
    print("amcl pose: x=%.3f y=%.3f yaw=%.1fdeg" % (x0, y0, math.degrees(yaw0)))
    print("best fit : x=%.3f y=%.3f yaw=%.1fdeg cost=%.3f" % (best[1], best[2], math.degrees(best[3]), best[0]))
    print("shift    : dx=%+.3f dy=%+.3f dyaw=%+.1fdeg" % (best[1] - x0, best[2] - y0, math.degrees(best[3] - yaw0)))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
