#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固定速度直行测试：发 cmd_vel 跑 N 秒，打印里程计偏差。

用法：
    python3 ~/narsu/ros2/tools/straight_test.py 0.06 3
    python3 ~/narsu/ros2/tools/straight_test.py 0.08 3
    python3 ~/narsu/ros2/tools/straight_test.py 0.10 3

参数：
    速度 m/s（默认 0.08）
    时长 秒（默认 3）

输出：
    dx / dy / dyaw —— 里程计自己认为的位移和转角。
    真值要用卷尺量：车头起点对准地上的线，跑完看横向偏了多少厘米。

注意：
    运行时人在旁边，随时准备按板子 RST 或断电；别对着墙。
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__('straight_test')
        self.odom = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 20)

    def cb(self, m):
        self.odom = m

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.08
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    rclpy.init()
    n = T()
    n.spin_for(1.0)
    if n.odom is None:
        print('没有 /odom，先起 bringup')
        return 1

    p0 = n.odom.pose.pose
    x0, y0 = p0.position.x, p0.position.y
    th0 = yaw_of(p0.orientation)

    print('速度 %.2f m/s，跑 %.1f 秒，理论位移 %.2f m' % (v, secs, v * secs))
    cmd = Twist()
    cmd.linear.x = v
    end = time.time() + secs
    while time.time() < end and rclpy.ok():
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.02)

    n.pub.publish(Twist())
    n.spin_for(0.8)

    p1 = n.odom.pose.pose
    dx = p1.position.x - x0
    dy = p1.position.y - y0
    dth = yaw_of(p1.orientation) - th0
    while dth > math.pi:
        dth -= 2 * math.pi
    while dth < -math.pi:
        dth += 2 * math.pi

    dist = math.hypot(dx, dy)
    fwd = dx * math.cos(th0) + dy * math.sin(th0)
    lat = -dx * math.sin(th0) + dy * math.cos(th0)
    print('里程计: 前进 %.3f m，横向 %+.3f m，转角 %+.1f 度' % (
        fwd, lat, math.degrees(dth)))
    if abs(fwd) > 0.05:
        print('每米横漂: %.1f cm/m' % (100.0 * lat / abs(fwd)))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
