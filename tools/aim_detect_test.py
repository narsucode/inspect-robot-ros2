#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aim_detect_test —— 不装车，单点验证整套「对准 + 识别」流程。

流程（按顺序，每步都有打印）：
    1. 云台复位到 0°/0°
    2. 发对准请求 → 云台扫视找 ArUco（先左后右，再由中间向外）
    3. 对准成功 → 连续采 3.5 秒识别结果，投票定颜色
    4. 和 patrol.yaml 里的 expect 比对，给出结论

前提：另开两个终端跑
    ros2 run roscar_patrol aim_marker
    ros2 run roscar_vision traffic_light_node --ros-args -p show:=true

用法：
    python3 ~/narsu/ros2/tools/aim_detect_test.py A
    python3 ~/narsu/ros2/tools/aim_detect_test.py A --dwell 5
"""
import argparse
import collections
import json
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import String

CFG = os.path.expanduser('~/narsu/ros2/patrol_ws/src/roscar_patrol/config/patrol.yaml')


class Probe(Node):
    def __init__(self):
        super().__init__('aim_detect_probe')
        self.aim = None
        self.aim_txt = ''
        self.light = None
        self.create_subscription(Vector3, '/aim_result', self._on_aim, 10)
        self.create_subscription(String, '/aim_text', self._on_txt, 10)
        self.create_subscription(String, '/vision/light_state', self._on_light, 10)
        self.ptz = self.create_publisher(Vector3, '/cmd_ptz_angle', 10)
        self.req = self.create_publisher(String, '/aim_request', 10)

    def _on_aim(self, m):
        self.aim = (float(m.x), float(m.y), float(m.z) > 0.5)

    def _on_txt(self, m):
        self.aim_txt = m.data

    def _on_light(self, m):
        try:
            self.light = json.loads(m.data)
        except Exception:
            pass

    def reset_ptz(self):
        m = Vector3()
        for _ in range(4):
            self.ptz.publish(m)
            time.sleep(0.12)


def spin_for(node, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('point', nargs='?', default='A', help='点位名，默认 A')
    ap.add_argument('--dwell', type=float, default=3.5, help='识别采样时长（秒）')
    ap.add_argument('--timeout', type=float, default=90.0, help='等对准的超时')
    args = ap.parse_args()

    expect = None
    try:
        with open(CFG, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        expect = (cfg.get('points') or {}).get(args.point, {}).get('expect')
    except Exception:
        pass

    rclpy.init()
    n = Probe()
    spin_for(n, 1.0)

    print('==== 1. 云台复位 ====')
    n.reset_ptz()
    spin_for(n, 1.0)
    print('已发 0°/0°，云台应回到正中\n')

    print('==== 2. 扫视找码（点位 %s）====' % args.point)
    n.aim = None
    n.req.publish(String(data=args.point))
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.15)
        if n.aim is not None:
            break
    aim_ok = False
    if n.aim is None:
        print('超时：没收到 /aim_result，检查 aim_marker 是否在跑')
    else:
        pan, tilt, aim_ok = n.aim
        print('对准%s  pan=%+.1f  tilt=%+.1f  %s  (用时 %.1f s)'
              % ('成功' if aim_ok else '失败', pan, tilt, n.aim_txt, time.time() - t0))
    if not aim_ok:
        print('\n对准没成功，后面的识别结果不可信，先修对准')
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print('\n==== 3. 识别灯色（采样 %.1f 秒）====' % args.dwell)
    votes = collections.Counter()
    samples = 0
    t0 = time.time()
    while time.time() - t0 < args.dwell:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.light is not None:
            votes[n.light.get('state', '?')] += 1
            samples += 1
            n.light = None
    if not votes:
        print('没收到 /vision/light_state，检查 traffic_light 节点是否在跑')
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    top, cnt = votes.most_common(1)[0]
    print('采样 %d 帧，分布: %s' % (samples, dict(votes)))
    print('投票结果: %s  (%d/%d = %.0f%%)' % (top, cnt, samples, 100.0 * cnt / samples))

    print('\n==== 4. 结论 ====')
    verdict = 'UNKNOWN'
    if top in ('RED', 'YELLOW', 'GREEN'):
        if expect:
            verdict = 'PASS' if top == expect else 'MISMATCH'
        else:
            verdict = 'PASS(未配 expect)'
    elif top == 'OFF':
        verdict = 'OFF（灯没亮或没对准）'
    elif top == 'MULTI':
        verdict = 'MULTI（同时看到多种颜色）'

    print('识别=%s  期望=%s  结论=%s' % (top, expect, verdict))
    if verdict == 'MISMATCH':
        print('  颜色和配置不符：要么灯装错，要么 patrol.yaml 的 expect 要改')

    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if verdict.startswith('PASS') else 2)


if __name__ == '__main__':
    main()
