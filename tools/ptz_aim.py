#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""点一下机柜 → 自动算出该点位的云台 pan/tilt，写进 patrol.yaml。

用法：
    python3 ~/narsu/ros2/tools/ptz_aim.py --cam-h 0.25 --light-h 0.20
    python3 ~/narsu/ros2/tools/ptz_aim.py --flip-pan     # 方向反了再加这个

前提：起过 map_only.launch.py（有 RViz 和 /clicked_point）。
操作：RViz 顶部换 "Publish Point" 工具，点机柜上的指示灯，终端里说这是哪个点位。
      Ctrl+C 退出，结果已即时写盘。
"""
import argparse
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

CFG = os.path.expanduser('~/narsu/ros2/patrol_ws/src/roscar_patrol/config')
WP = os.path.join(CFG, 'waypoints.yaml')
PT = os.path.join(CFG, 'patrol.yaml')

HEADER = """# 巡检点位配置
#
# order: 巡检顺序，必须是 waypoints.yaml 里存在的点位名
# points: 每个点位的云台预设角度（度）和期望状态（可选，用于自动核对）
#         pan  水平角度，正值向右（以固件实际方向为准，反了就填负数）
#         tilt 俯仰角度，正值向下
#         expect 期望识别到的状态：RED / YELLOW / GREEN，不写就只记录不核对
"""


def load():
    with open(WP, encoding='utf-8') as f:
        wps = yaml.safe_load(f)['waypoints']
    try:
        with open(PT, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        cfg = {}
    cfg.setdefault('order', list(wps))
    cfg.setdefault('points', {})
    return wps, cfg


def save(cfg):
    body = yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False)
    with open(PT, 'w', encoding='utf-8') as f:
        f.write(HEADER + '\n' + body)


class Clicker(Node):
    def __init__(self):
        super().__init__('ptz_aim')
        self.pts = []
        self.create_subscription(PointStamped, '/clicked_point', self.cb, 10)

    def cb(self, msg):
        self.pts.append((msg.point.x, msg.point.y))


def norm180(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam-h', type=float, default=0.25, help='摄像头离地高度 m')
    ap.add_argument('--light-h', type=float, default=0.20, help='指示灯离地高度 m')
    ap.add_argument('--flip-pan', action='store_true', help='云台反着转就加这个')
    args = ap.parse_args()

    wps, cfg = load()
    pts_cfg = cfg['points']
    print('已有点位:', list(wps))
    print('当前 pan/tilt:',
          {k: (v.get('pan'), v.get('tilt')) for k, v in pts_cfg.items()})
    print('用 RViz 的 "Publish Point" 点机柜指示灯；Ctrl+C 退出。')

    rclpy.init()
    node = Clicker()
    last = None
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            while node.pts:
                x, y = node.pts.pop(0)
                print('\n点击: x=%.3f y=%.3f' % (x, y))
                name = input('属于哪个点位 %s [%s]: ' % (list(wps), last or '')).strip().upper()
                if not name:
                    name = last
                if name not in wps:
                    print('  跳过：%s 不在 waypoints.yaml' % name)
                    continue
                last = name
                wx, wy, wyaw = wps[name]
                dist = math.hypot(x - wx, y - wy)
                brg = math.degrees(math.atan2(y - wy, x - wx))
                delta = norm180(brg - math.degrees(wyaw))
                # ROS 里逆时针为正（目标在左手边）；云台 pan 正值向右，所以取反。
                pan = delta if args.flip_pan else -delta
                pan = max(-90.0, min(90.0, pan))
                tilt = math.degrees(math.atan2(args.cam_h - args.light_h, dist))
                pts_cfg.setdefault(name, {})
                pts_cfg[name]['pan'] = round(pan, 1)
                pts_cfg[name]['tilt'] = round(tilt, 1)
                save(cfg)
                print('  %s: 距 %s 点 %.2f m，机柜在该点 %+.0f° 方向 → pan %.1f°  tilt %.1f°'
                      % (name, name, dist, delta, pan, tilt))
    except KeyboardInterrupt:
        print('\n已退出，patrol.yaml 已更新')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
