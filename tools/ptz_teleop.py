#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ptz_teleop —— 键盘遥控云台，用来验证云台线路和方向。

按键（2026-11-02 实测映射：pan 越小越往右，tilt 越小越往下）：
    a / d    云台左 / 右
    w / s    云台抬 / 低
    1 2 3    步长 1° / 5° / 10°
    r        回中（0, 0）
    q        退出

为什么不只发一次：下位机是"收到就动一次"的绝对值范式，UDP/TF 丢包就不动了，
所以这里按 8 Hz 持续补发当前角度，屏幕上的角度就是实际下发的值。

用法：
    python3 ~/narsu/ros2/tools/ptz_teleop.py
    python3 ~/narsu/ros2/tools/ptz_teleop.py --step 10 --pan-limit 90 --tilt-limit 30
"""
import argparse
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node


class PtzTeleop(Node):
    def __init__(self, topic):
        super().__init__('ptz_teleop')
        self.pub = self.create_publisher(Vector3, topic, 10)
        self.pan = 0.0
        self.tilt = 0.0

    def send(self):
        m = Vector3()
        m.x = float(self.pan)
        m.y = float(self.tilt)
        self.pub.publish(m)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0.12)
        if r:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/cmd_ptz_angle')
    ap.add_argument('--step', type=float, default=5.0)
    ap.add_argument('--pan-limit', type=float, default=90.0)
    ap.add_argument('--tilt-limit', type=float, default=35.0)
    args = ap.parse_args()

    rclpy.init()
    node = PtzTeleop(args.topic)
    step = args.step

    print('云台键盘遥控  话题=%s  步长=%.0f°' % (args.topic, step))
    print('a/d=左右  w/s=上下  1/2/3=步长  r=回中  q=退出')
    print('pan=%+.0f  tilt=%+.0f' % (node.pan, node.tilt))

    last = time.time()
    try:
        while True:
            key = get_key()
            changed = False
            if key == 'a':
                node.pan = min(args.pan_limit, node.pan + step)
                changed = True
            elif key == 'd':
                node.pan = max(-args.pan_limit, node.pan - step)
                changed = True
            elif key == 'w':
                node.tilt = min(args.tilt_limit, node.tilt + step)
                changed = True
            elif key == 's':
                node.tilt = max(-args.tilt_limit, node.tilt - step)
                changed = True
            elif key in ('1', '2', '3'):
                step = {'1': 1.0, '2': 5.0, '3': 10.0}[key]
                print('\n步长改为 %.0f°' % step)
            elif key == 'r':
                node.pan = 0.0
                node.tilt = 0.0
                changed = True
            elif key == 'q':
                break

            # 8 Hz 补发，保证下位机不丢当前目标
            now = time.time()
            if changed or now - last > 0.12:
                node.send()
                last = now
            if changed:
                print('pan=%+.0f  tilt=%+.0f' % (node.pan, node.tilt))
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n退出，云台停在 pan=%+.0f tilt=%+.0f' % (node.pan, node.tilt))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
