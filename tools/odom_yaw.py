#!/usr/bin/env python3
"""Print the odometry yaw live, for a 90-degree rotation calibration."""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class P(Node):
    def __init__(self):
        super().__init__("odom_yaw")
        self.create_subscription(Odometry, "/odom", self.cb, 20)
        self.last = None

    def cb(self, m):
        y = math.degrees(yaw_of(m.pose.pose.orientation))
        p = m.pose.pose.position
        if self.last is not None and abs(y - self.last) < 0.05:
            return
        self.last = y
        print("yaw=%+7.2f deg   x=%+.3f y=%+.3f" % (y, p.x, p.y))


def main():
    rclpy.init()
    n = P()
    print("Ctrl+C 退出")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
