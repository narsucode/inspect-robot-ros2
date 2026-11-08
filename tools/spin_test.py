#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""原地旋转自检：发几档角速度，比较实际转角与理论值。

用法（拔掉 USB、板子连上 agent 后）：
    python3 ~/narsu/ros2/tools/spin_test.py

判读：
    比值 ≥ 0.8  → 低速转向正常，nav2 的 U 形有希望过
    比值 0.3~0.8 → 有改善但仍偏弱
    比值 < 0.3 或者 0 → 死区没解决，需要继续加大最小 PWM
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class T(Node):
    def __init__(self):
        super().__init__('spin_test')
        self.odom = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, m):
        self.odom = m

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.degrees(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))


def delta(a, b):
    d = b - a
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def main():
    rclpy.init()
    n = T()
    n.spin_for(1.5)
    if n.odom is None:
        print('没有 /odom：确认 bringup 在跑、板子已连上 agent')
        return 1

    print('角度指令 → 实际转角（每档 2 秒）')
    for w in (0.15, 0.3, 0.6, 1.0):
        y0 = n.yaw()
        t = Twist()
        t.angular.z = w
        end = time.time() + 2.0
        while time.time() < end and rclpy.ok():
            n.pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.02)
        n.pub.publish(Twist())
        n.spin_for(1.0)
        got = delta(y0, n.yaw())
        theo = math.degrees(w * 2.0)
        print('  %.2f rad/s -> %6.1f 度 (理论 %5.1f)  比值 %.2f'
              % (w, got, theo, got / theo))

    print('\n如果比值都接近 1，说明死区补偿生效；可以用 nav2 再跑 U 形。')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
