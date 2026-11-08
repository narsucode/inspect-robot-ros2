#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""point_test —— 不导航，单独验证某个点位的「转云台 + 识别灯色」。

逻辑和巡检里完全一致：
    1. 读 patrol.yaml 里该点的 pan/tilt
    2. 云台转过去
    3. 采 2.5 秒，投票
    4. 打印判色结果，并存一张图

用法：
    python3 ~/narsu/ros2/tools/point_test.py A
    python3 ~/narsu/ros2/tools/point_test.py B --dwell 4
    python3 ~/narsu/ros2/tools/point_test.py C --pan 15 --tilt -5   # 临时试角度

前提：traffic_light 节点在跑（用 point_test.launch.py 会自动起）。
"""
import argparse
import collections
import json
import os
import time
import urllib.request

import rclpy
import yaml
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import String

CFG = os.path.expanduser('~/narsu/ros2/patrol_ws/src/roscar_patrol/config/patrol.yaml')
CAM = 'http://10.142.138.251/capture'
SHOTS = os.path.expanduser('~/patrol_shots')


class Probe(Node):
    def __init__(self):
        super().__init__('point_test')
        self.state = None
        self.light = None
        self.create_subscription(String, '/vision/light_state', self._on_light, 10)
        self.ptz = self.create_publisher(Vector3, '/cmd_ptz_angle', 10)

    def _on_light(self, msg):
        try:
            self.light = json.loads(msg.data)
        except Exception:
            pass

    def send_ptz(self, pan, tilt, hold=1.0):
        m = Vector3()
        m.x = float(pan)
        m.y = float(tilt)
        t0 = time.time()
        while time.time() - t0 < hold:
            self.ptz.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('point', nargs='?', default='A')
    ap.add_argument('--dwell', type=float, default=2.5)
    ap.add_argument('--pan', default='')
    ap.add_argument('--tilt', default='')
    args = ap.parse_args()

    with open(CFG, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    pt = (cfg.get('points') or {}).get(args.point, {}) or {}
    pan = float(args.pan) if str(args.pan).strip() else float(pt.get('pan', 0.0))
    tilt = float(args.tilt) if str(args.tilt).strip() else float(pt.get('tilt', 0.0))

    rclpy.init()
    n = Probe()
    print('点位 %s：云台转到 pan=%.1f tilt=%.1f' % (args.point, pan, tilt))
    rclpy.spin_once(n, timeout_sec=0.5)
    n.send_ptz(pan, tilt, hold=1.2)

    print('等云台停稳 0.5 秒…')
    t0 = time.time()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(n, timeout_sec=0.05)

    print('开始识别灯色（%.1f 秒）…' % args.dwell)
    votes = collections.Counter()
    t0 = time.time()
    frames = 0
    while time.time() - t0 < args.dwell:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.light is not None:
            votes[n.light.get('state', '?')] += 1
            frames += 1
            n.light = None

    if not votes:
        print('  没收到 /vision/light_state —— traffic_light 没在跑？')
        n.destroy_node()
        rclpy.shutdown()
        return

    top, cnt = votes.most_common(1)[0]
    print('  采样 %d 帧：%s' % (frames, dict(votes)))
    print('  判定结果：%s（%d/%d = %.0f%%）' % (top, cnt, frames, 100.0 * cnt / frames))

    os.makedirs(SHOTS, exist_ok=True)
    path = os.path.join(SHOTS, 'test_%s_%s.jpg' % (args.point, time.strftime('%H%M%S')))
    try:
        urllib.request.urlretrieve(CAM, path)
        print('  截图：%s' % path)
    except Exception as e:
        print('  截图失败：%s' % e)

    level = {'GREEN': '正常', 'YELLOW': '警告', 'RED': '故障',
             'OFF': '熄灭', 'UNKNOWN': '未识别', 'MULTI': '多灯同亮'}.get(top, '未知')
    print('  语义等级：%s' % level)
    if top == 'OFF':
        print('  提示：画面里没有任何亮斑 —— 角度没对着灯，或灯没亮')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
