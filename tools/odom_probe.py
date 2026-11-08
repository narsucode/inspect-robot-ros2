#!/usr/bin/env python3
"""Drive straight briefly and report how /odom's yaw behaves.

If yaw changes while linear.x is constant and angular.z is zero, the wheel
odometry is producing rotation from straight motion - which makes the scan
appear to swivel in RViz.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("odom_probe")
        self.odom = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.cb, 20)

    def cb(self, msg):
        self.odom = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, vx, seconds):
        t = Twist()
        t.linear.x = vx
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        self.spin_for(0.5)


def main():
    rclpy.init()
    n = Probe()
    n.spin_for(2.0)
    if n.odom is None:
        print("ERROR: no /odom")
        return 1

    o = n.odom
    print("idle: x=%.3f y=%.3f yaw=%.3f" % (o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o.pose.pose.orientation)))
    n.spin_for(3.0)
    o = n.odom
    print("after 3s idle: x=%.3f y=%.3f yaw=%.3f" % (o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o.pose.pose.orientation)))

    start = n.odom
    x0, y0, th0 = start.pose.pose.position.x, start.pose.pose.position.y, yaw_of(start.pose.pose.orientation)
    print("driving straight 0.05 m/s for 2s ...")
    n.drive(0.05, 2.0)
    o = n.odom
    x1, y1, th1 = o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o.pose.pose.orientation)
    print("delta: dx=%.3f dy=%.3f dyaw=%.1f deg (%.3f rad)" % (x1 - x0, y1 - y0, math.degrees(th1 - th0), th1 - th0))
    print("odom twist: vx=%.3f vth=%.3f" % (o.twist.twist.linear.x, o.twist.twist.angular.z))
    gain = (th1 - th0) / max(abs(x1 - x0), 1e-6)
    print("yaw per metre: %.1f deg/m" % math.degrees(gain))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
