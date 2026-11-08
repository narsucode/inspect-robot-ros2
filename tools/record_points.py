#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在 RViz 地图上标点，写入 waypoints.yaml。

两种模式：

  1) 点击模式（默认）—— 用 Publish Point，只有位置，朝向自动“朝外”
       python3 record_clicked.py

  2) 目标模式 —— 用 2D Goal Pose，位置 + 朝向一次给全
       python3 record_clicked.py --mode goal

     注意：目标模式必须在“没有 nav2”的情况下用，否则 RViz 一发
     /goal_pose，车就真的开过去了。配合 map_only.launch.py 使用。

通用操作：
  - 点完地图，终端输入点位名（回车 = 自动 A/B/C…）；
  - 输入 q 退出并保存；
  - 自动检查该点在地图上是“可走 / 墙里 / 图外”。
"""

import argparse
import math
import os
import queue
import select
import sys
import threading

import cv2
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node

OUT_DEFAULT = os.path.expanduser(
    '~/narsu/ros2/patrol_ws/src/roscar_patrol/config/waypoints.yaml')
MAP_DIRS = ('roscar_navigation', 'youdaobot_navigation')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PickNode(Node):
    def __init__(self, mode):
        super().__init__('record_points')
        self.q = queue.Queue()
        if mode == 'goal':
            self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        else:
            self.create_subscription(PointStamped, '/clicked_point', self.cb_point, 10)

    def cb_point(self, msg):
        p = msg.point
        self.q.put((p.x, p.y, None, msg.header.frame_id))

    def cb_goal(self, msg):
        p = msg.pose.position
        self.q.put((p.x, p.y, yaw_of(msg.pose.orientation), msg.header.frame_id))


def load_map_checker():
    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in MAP_DIRS:
            try:
                d = get_package_share_directory(pkg)
            except Exception:
                continue
            for name in ('room_20261101', 'room_new', 'room'):
                yp = os.path.join(d, 'maps', name + '.yaml')
                if os.path.exists(yp):
                    meta = yaml.safe_load(open(yp, encoding='utf-8')) or {}
                    ip = meta.get('image')
                    if not os.path.isabs(ip):
                        ip = os.path.join(os.path.dirname(yp), ip)
                    img = cv2.imread(ip, cv2.IMREAD_GRAYSCALE)
                    if img is not None:
                        return img, meta
    except Exception:
        pass
    return None, None


def check_point(img, meta, x, y):
    if img is None:
        return '未知（没读到地图文件）'
    h, w = img.shape
    res = meta['resolution']
    ox, oy, _ = meta['origin']
    c = int((x - ox) / res)
    r = h - 1 - int((y - oy) / res)
    if not (0 <= r < h and 0 <= c < w):
        return '图外'
    v = int(img[r, c])
    if v > 191:
        return '可走'
    if v < 89:
        return '墙里'
    return '未知区'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['click', 'goal'], default='click')
    ap.add_argument('--out', default=OUT_DEFAULT)
    args = ap.parse_args()

    data = {'waypoints': {}}
    if os.path.exists(args.out):
        with open(args.out, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {'waypoints': {}}
        data.setdefault('waypoints', {})

    img, meta = load_map_checker()
    center = None
    if meta is not None and img is not None:
        h, w = img.shape
        center = (meta['origin'][0] + w * meta['resolution'] / 2.0,
                  meta['origin'][1] + h * meta['resolution'] / 2.0)

    rclpy.init()
    node = PickNode(args.mode)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    used = list(data['waypoints'].keys())
    letters = [c for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' if c not in used]
    print('已有点位:', used or '无')
    if args.mode == 'goal':
        print('用 RViz 的 “2D Goal Pose” 点并拖动（拖动方向 = 朝向）；输入 q 退出。')
        print('提醒：此时不能有 nav2 在跑，否则车会真的开过去。')
    else:
        print('用 RViz 的 “Publish Point” 点地图；输入 q 退出。')

    try:
        while True:
            x = None
            while x is None:
                r, _, _ = select.select([sys.stdin], [], [], 0.3)
                if r:
                    line = sys.stdin.readline().strip()
                    if line.lower() == 'q':
                        raise KeyboardInterrupt
                    if line:
                        print('（先用鼠标在地图上点一下）')
                try:
                    x, y, yaw, frame = node.q.get(timeout=0.1)
                except queue.Empty:
                    continue

            info = check_point(img, meta, x, y)
            print('\n点击: x=%.3f  y=%.3f  frame=%s  -> %s' % (x, y, frame, info))

            default = letters[0] if letters else ''
            name = input('点位名 [%s]: ' % default).strip()
            if name.lower() == 'q':
                break
            if not name:
                if not default:
                    print('没有可用字母了，请输入名字')
                    continue
                name = default
            if len(name) == 1:
                name = name.upper()

            if yaw is None:
                if center is not None:
                    auto = math.degrees(math.atan2(y - center[1], x - center[0]))
                else:
                    auto = 0.0
                s = input('朝向(度) [回车=朝外 %.0f]: ' % auto).strip()
                s = s.replace('°', '').replace('度', '').strip()
                if s:
                    try:
                        yaw = math.radians(float(s))
                    except ValueError:
                        print('看不懂角度，用自动值')
                        yaw = math.radians(auto)
                else:
                    yaw = math.radians(auto)
            else:
                print('朝向来自拖动方向: %.0f°' % math.degrees(yaw))

            data['waypoints'][name] = {
                'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
            if name in letters:
                letters.remove(name)
            print('已记录 %s: (%.3f, %.3f) yaw=%.3f' % (name, x, y, yaw))
    except KeyboardInterrupt:
        pass

    with open(args.out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print('\n已写入: %s' % args.out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
