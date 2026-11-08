#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""led_monitor —— 灯色监视器：灯变红/黄/绿时，终端实时报出判定。

用途：演示「直接识别灯色」这一段——不扫视、不找码，只盯着灯看。
配合 traffic_light_node（弹窗里能看到框）使用。

用法：
    python3 ~/narsu/ros2/tools/led_monitor.py
    python3 ~/narsu/ros2/tools/led_monitor.py --quiet   # 只在变化时打印

输出示例：
    17:56:03  RED      故障   面积 78px   （从 OFF 变化）
    17:56:08  YELLOW   警告   面积 76px
    17:56:13  GREEN    正常   面积 80px
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LEVEL = {'GREEN': '正常', 'YELLOW': '警告', 'RED': '故障',
         'OFF': '熄灭', 'UNKNOWN': '未识别', 'MULTI': '多灯同亮', 'DISABLED': '未启用'}


class Monitor(Node):
    def __init__(self):
        super().__init__('led_monitor')
        self.last = None
        self.last_t = 0.0
        self.create_subscription(String, '/vision/light_state', self._on, 10)

    def _on(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        state = d.get('state', '?')
        area = 0
        lamps = d.get('lamps') or []
        if lamps:
            area = int(lamps[0].get('area', 0))
        code = '/'.join(d.get('code') or [])
        self.report(state, area, code)

    def report(self, state, area, code):
        now = time.time()
        if state == self.last:
            return
        self.last = state
        self.last_t = now
        lvl = LEVEL.get(state, '未知')
        extra = '  码=%s' % code if code else ''
        print('%s  %-8s %-5s  面积 %3dpx%s'
              % (time.strftime('%H:%M:%S'), state, lvl, area, extra), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true', help='（保留参数）')
    args = ap.parse_args()

    rclpy.init()
    n = Monitor()
    print('灯色监视中…  (Ctrl+C 退出)')
    print('把手上的灯依次换成 红 / 黄 / 绿，屏幕会实时报出判定\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        print('\n结束，最后状态：%s' % (n.last or '-'))
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
