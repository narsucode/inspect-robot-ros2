#!/usr/bin/env python3
"""Drive forward for a few seconds and print /odom samples."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class P(Node):
    def __init__(self):
        super().__init__("drive_probe")
        self.odom = None
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.cb, 20)

    def cb(self, m):
        self.odom = m


def main():
    vx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    rclpy.init()
    n = P()
    t0 = time.time()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(n, timeout_sec=0.05)
    x0 = n.odom.pose.pose.position.x if n.odom else 0.0
    y0 = n.odom.pose.pose.position.y if n.odom else 0.0
    th0 = yaw_of(n.odom.pose.pose.orientation) if n.odom else 0.0

    cmd = Twist()
    cmd.linear.x = vx
    end = time.time() + secs
    last = 0.0
    while time.time() < end and rclpy.ok():
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last > 0.5:
            last = time.time()
            if n.odom:
                p = n.odom.pose.pose
                print("t=%.1f x=%+.3f y=%+.3f yaw=%+.1fdeg vx=%+.3f vth=%+.3f" % (
                    time.time() - t0, p.position.x, p.position.y,
                    math.degrees(yaw_of(p.orientation)),
                    n.odom.twist.twist.linear.x, n.odom.twist.twist.angular.z))
    n.pub.publish(Twist())
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    p = n.odom.pose.pose if n.odom else None
    if p:
        print("DELTA dx=%+.3f dy=%+.3f dyaw=%+.1fdeg" % (
            p.position.x - x0, p.position.y - y0,
            math.degrees(yaw_of(p.orientation) - th0)))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
