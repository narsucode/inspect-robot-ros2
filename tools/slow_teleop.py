#!/usr/bin/env python3
"""
低速遥控（建图专用）

为什么不用 teleop_twist_keyboard：
  它的 q/z 会同时缩放线速度和角速度，按几次 z 之后角速度也小到
  转不动；而且默认 0.5 m/s，建图时太快会把墙扫歪。

用法：
  python3 ~/narsu/ros2/tools/slow_teleop.py
  python3 ~/narsu/ros2/tools/slow_teleop.py --lin 0.08 --ang 0.35

按键（不用回车，按一下就动，松开不停直到按别的）：
  w / s      前进 / 后退
  a / d      左转 / 右转
  空格       停
  + / -      线速度 ±0.02
  > / <      角速度 ±0.05
  q          退出（退出前自动发一次停车）
"""

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SlowTeleop(Node):
    def __init__(self, lin, ang):
        super().__init__('slow_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = lin
        self.ang = ang
        self.cur_lin = 0.0
        self.cur_ang = 0.0

    def send(self, lin, ang):
        self.cur_lin = lin
        self.cur_ang = ang

    def tick(self):
        """持续重发，底盘一般有 0.5 秒超时，不重发会自己停。"""
        t = Twist()
        t.linear.x = self.cur_lin
        t.angular.z = self.cur_ang
        self.pub.publish(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lin', type=float, default=0.12, help='线速度 m/s')
    ap.add_argument('--ang', type=float, default=0.45, help='角速度 rad/s')
    args = ap.parse_args()

    rclpy.init()
    node = SlowTeleop(args.lin, args.ang)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    def banner():
        print(f'\r线速度 {node.lin:.2f} m/s   角速度 {node.ang:.2f} rad/s   '
              f'当前 lin={node.cur_lin:+.2f} ang={node.cur_ang:+.2f}   ',
              end='', flush=True)

    print('w/s 前后   a/d 转向   空格停   +/- 线速度   >/< 角速度   q 退出')
    last = 0.0
    try:
        tty.setcbreak(fd)
        while True:
            now = time.time()
            if now - last > 0.1:
                node.tick()
                last = now
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                rclpy.spin_once(node, timeout_sec=0)
                continue
            k = sys.stdin.read(1)
            if k == 'w':
                node.send(node.lin, 0.0)
            elif k == 's':
                node.send(-node.lin, 0.0)
            elif k == 'a':
                node.send(0.0, node.ang)
            elif k == 'd':
                node.send(0.0, -node.ang)
            elif k == ' ':
                node.send(0.0, 0.0)
            elif k == '+':
                node.lin = round(node.lin + 0.02, 2)
            elif k == '-':
                node.lin = max(0.02, round(node.lin - 0.02, 2))
            elif k == '>':
                node.ang = round(node.ang + 0.05, 2)
            elif k == '<':
                node.ang = max(0.05, round(node.ang - 0.05, 2))
            elif k in ('q', 'Q', '\x03'):
                break
            banner()
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.send(0.0, 0.0)
        for _ in range(5):
            node.tick()
            time.sleep(0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print('\n已停车')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
