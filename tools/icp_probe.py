#!/usr/bin/env python3
"""Ground-truth check: compare wheel odometry against scan-to-scan ICP.

1. take a scan while stopped (A)
2. drive straight slowly, stop
3. take a scan again (B)
4. align A onto B with 2D ICP -> the real motion
5. compare with the /odom delta over the same two moments
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scipy.spatial import cKDTree


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def scan_points(msg, max_range=3.0):
    a = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
    r = np.asarray(msg.ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r > 0.10) & (r < min(max_range, msg.range_max))
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def icp(src, dst, iters=40):
    R = np.eye(2)
    t = np.zeros(2)
    tree = cKDTree(dst)
    for _ in range(iters):
        p = src @ R.T + t
        d, idx = tree.query(p)
        keep = d < 0.25
        if keep.sum() < 10:
            break
        A, B = p[keep], dst[idx[keep]]
        ca, cb = A.mean(axis=0), B.mean(axis=0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[-1] *= -1
            Rn = Vt.T @ U.T
        tn = cb - Rn @ ca
        R = Rn @ R
        t = Rn @ t + tn
        if np.linalg.norm(tn) < 1e-4 and abs(math.atan2(Rn[1, 0], Rn[0, 0])) < 1e-4:
            break
    return R, t


class P(Node):
    def __init__(self):
        super().__init__("icp_probe")
        self.scan = None
        self.odom = None
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 20)

    def cb_scan(self, m):
        self.scan = m

    def cb_odom(self, m):
        self.odom = m

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.03)

    def drive(self, vx, s, mode="fwd"):
        c = Twist()
        if mode == "rot":
            c.angular.z = vx
        else:
            c.linear.x = vx
        end = time.time() + s
        while time.time() < end and rclpy.ok():
            self.pub.publish(c)
            rclpy.spin_once(self, timeout_sec=0.03)
        self.pub.publish(Twist())
        self.spin_for(1.0)


def pose_of(o):
    p = o.pose.pose
    return np.array([p.position.x, p.position.y]), yaw_of(p.orientation)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "fwd"
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else (0.8 if mode == "rot" else 0.1)
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else (1.2 if mode == "rot" else 2.0)
    rclpy.init()
    n = P()
    n.spin_for(2.0)
    if n.scan is None or n.odom is None:
        print("ERROR: missing /scan or /odom")
        return 1
    a = scan_points(n.scan)
    ta = n.scan.header.stamp
    pa, ya = pose_of(n.odom)

    n.drive(speed, secs, mode)
    b = scan_points(n.scan)
    pb, yb = pose_of(n.odom)

    R, t = icp(a, b)
    icp_yaw = math.atan2(R[1, 0], R[0, 0])
    odom_dyaw = yb - ya
    odom_d = pb - pa
    print("points: A=%d B=%d" % (len(a), len(b)))
    print("odom : dx=%+.3f dy=%+.3f dyaw=%+.1f deg" % (odom_d[0], odom_d[1], math.degrees(odom_dyaw)))
    print("icp  : dx=%+.3f dy=%+.3f dyaw=%+.1f deg" % (t[0], t[1], math.degrees(icp_yaw)))
    print("error: dist=%.3f m, yaw=%.1f deg" % (math.hypot(*(odom_d - t)), math.degrees(odom_dyaw - icp_yaw)))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
