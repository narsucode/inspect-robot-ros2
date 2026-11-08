#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在 RViz 地图上点选点位，直接写进 waypoints.yaml（不用开车过去）。

用法：
    1. 起 nav2 + RViz，确认地图显示正常；
    2. 运行本脚本：
         python3 ~/narsu/ros2/tools/record_clicked.py
    3. RViz 工具栏选 “Publish Point”，在地图上点一下；
    4. 终端会提示输入点位名和朝向；
    5. 重复 A/B/C，最后输入 q 保存。

注意：
    不要用 “2D Goal Pose” 工具，那个会把目标发给 nav2，车会动。
    用 “Publish Point”（一个带箭头的十字图标或 “Publish Point” 字样）。

朝向怎么给：
    输入角度（度），0 = 朝 map 的 +x 方向，90 = 朝 +y 方向。
    不确定就回车用 0，之后在 yaml 里改。
"""

import math
import os
import queue
import select
import sys
import threading

import cv2
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

OUT_DEFAULT = os.path.expanduser(
    '~/narsu/ros2/patrol_ws/src/roscar_patrol/config/waypoints.yaml')
MAP_DIRS = ('roscar_navigation', 'youdaobot_navigation')


class ClickNode(Node):
    def __init__(self):
        super().__init__('record_clicked')
        self.q = queue.Queue()
        self.create_subscription(PointStamped, '/clicked_point', self.cb, 10)

    def cb(self, msg):
        p = msg.point
        self.q.put((p.x, p.y, msg.header.frame_id))


def load_map_checker():
    """读地图，用来判断点是否落在墙里。"""
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
                    if img is None:
                        continue
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
    out = sys.argv[1] if len(sys.argv) > 1 else OUT_DEFAULT

    data = {'waypoints': {}}
    if os.path.exists(out):
        with open(out, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {'waypoints': {}}
        data.setdefault('waypoints', {})

    img, meta = load_map_checker()
    # 场地中心：没给朝向时，默认“从中心朝外”，也就是面向那一侧的机柜
    center = None
    if meta is not None and img is not None:
        h, w = img.shape
        center = (meta['origin'][0] + w * meta['resolution'] / 2.0,
                  meta['origin'][1] + h * meta['resolution'] / 2.0)

    rclpy.init()
    node = ClickNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    used = list(data['waypoints'].keys())
    letters = [c for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' if c not in used]
    print('已有点位:', used or '无')
    print('RViz 里用 “Publish Point” 点地图；输入 q 退出并保存。')

    try:
        while True:
            # 等点击的同时盯着键盘：随时可以输入 q 退出，不必先点地图
            x = None
            while x is None:
                r, _, _ = select.select([sys.stdin], [], [], 0.3)
                if r:
                    line = sys.stdin.readline().strip()
                    if line.lower() == 'q':
                        raise KeyboardInterrupt
                    if line:
                        print('（请先用 Publish Point 点地图，再输入点位名）')
                try:
                    x, y, frame = node.q.get(timeout=0.1)
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
            # 单字母统一大写，避免 a / A 混成两个点
            if len(name) == 1:
                name = name.upper()

            if center is not None:
                auto = math.degrees(math.atan2(y - center[1], x - center[0]))
            else:
                auto = 0.0
            yaw_s = input('朝向(度) [回车=朝外 %.0f]: ' % auto).strip()
            # 允许输入 "90"、"90°"、"90度"
            yaw_s = yaw_s.replace('°', '').replace('度', '').strip()
            if yaw_s:
                try:
                    yaw = math.radians(float(yaw_s))
                except ValueError:
                    print('角度看不懂，先用自动值 %.0f' % auto)
                    yaw = math.radians(auto)
            else:
                yaw = math.radians(auto)

            data['waypoints'][name] = {
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw': round(yaw, 3),
            }
            if name in letters:
                letters.remove(name)
            print('已记录 %s: (%.3f, %.3f) yaw=%.3f' % (name, x, y, yaw))
    except KeyboardInterrupt:
        pass

    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print('\n已写入: %s' % out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
