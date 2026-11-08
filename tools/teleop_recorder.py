#!/usr/bin/env python3
"""Log /cmd_vel and /odom to a file so a manual teleop run can be analysed."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class R(Node):
    def __init__(self, out):
        super().__init__("teleop_recorder")
        self.out = out
        self.create_subscription(Odometry, "/odom", self.cb_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd, 20)
        self.last_cmd = (0.0, 0.0)
        self.t0 = time.time()

    def cb_cmd(self, m):
        self.last_cmd = (m.linear.x, m.angular.z)

    def cb_odom(self, m):
        p = m.pose.pose
        self.out.write("%.2f odom %.3f %.3f %.1f cmd %.3f %.3f\n" % (
            time.time() - self.t0, p.position.x, p.position.y,
            math.degrees(yaw_of(p.orientation)), self.last_cmd[0], self.last_cmd[1]))

    def tick(self):
        self.out.write("%.2f tick %.3f %.3f\n" % (time.time() - self.t0, self.last_cmd[0], self.last_cmd[1]))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/teleop_log.txt"
    rclpy.init()
    f = open(path, "w")
    n = R(f)
    n.get_logger().info("recording to %s for %.0fs" % (path, dur))
    end = time.time() + dur
    last_tick = 0.0
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - last_tick > 1.0:
            last_tick = time.time()
            n.tick()
            f.flush()
    f.write("END\n")
    f.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
