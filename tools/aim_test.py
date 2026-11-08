#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单次测试云台扫视对准：发一个点位名，看 aim_marker 能不能找到二维码。

用法：
    python3 ~/narsu/ros2/tools/aim_test.py A
    python3 ~/narsu/ros2/tools/aim_test.py A --timeout 120

前提：aim_marker 节点在跑（patrol_gui.launch.py 会自动起它）。
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__('aim_probe')
        self.res = None
        self.txt = ''
        self.create_subscription(Vector3, '/aim_result', self._on_res, 10)
        self.create_subscription(String, '/aim_text', self._on_txt, 10)
        self.pub = self.create_publisher(String, '/aim_request', 10)

    def _on_res(self, msg):
        self.res = (float(msg.x), float(msg.y), float(msg.z) > 0.5)

    def _on_txt(self, msg):
        self.txt = msg.data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('point', help='点位名，如 A')
    ap.add_argument('--timeout', type=float, default=90.0)
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    time.sleep(0.5)
    node.pub.publish(String(data=args.point))
    print('已发送对准请求: %s，等待结果（最长 %.0f 秒）...' % (args.point, args.timeout))

    t0 = time.time()
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.res is not None:
            pan, tilt, ok = node.res
            print('\n结果: %s' % ('成功' if ok else '失败'))
            print('  pan  = %.1f' % pan)
            print('  tilt = %.1f' % tilt)
            print('  说明 = %s' % node.txt)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0 if ok else 1)
    print('\n超时，没收到 /aim_result。检查 aim_marker 是否在跑：ros2 node list | grep aim')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(2)


if __name__ == '__main__':
    main()
