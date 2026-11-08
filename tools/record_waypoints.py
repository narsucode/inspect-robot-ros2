#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""边开车边记录巡检点位。

用法：
    ros2 run 不行，直接：
    python3 ~/narsu/ros2/tools/record_waypoints.py

操作：
    1. 先把车开到第一个点位，停稳；
    2. 终端里输入点位名（直接回车 = 自动用 A、B、C…）；
    3. 脚本立刻读取当前 map 位姿并记下来；
    4. 重复，直到输入 q 退出；
    5. 退出时自动写回 waypoints.yaml（保留原有的其他点位）。

说明：
    位姿来源优先 /amcl_pose（需要 nav2 的 AMCL 在跑、已做 2D Pose 定位）；
    如果收不到，就退回 TF: map -> base_footprint。
"""

import math
import os
import sys
import threading

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

OUT_DEFAULT = os.path.expanduser(
    '~/narsu/ros2/patrol_ws/src/roscar_patrol/config/waypoints.yaml')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PoseNode(Node):
    def __init__(self):
        super().__init__('record_waypoints')
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def _on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def current(self):
        """优先 AMCL，其次 TF map->base_footprint。"""
        if self.pose is not None:
            return self.pose
        try:
            tr = self.buf.lookup_transform('map', 'base_footprint',
                                           rclpy.time.Time())
            t = tr.transform.translation
            return (t.x, t.y, yaw_of(tr.transform.rotation))
        except Exception:
            return None


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUT_DEFAULT

    rclpy.init()
    node = PoseNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # 读旧文件，保留未改动的点位
    data = {'waypoints': {}}
    if os.path.exists(out):
        try:
            with open(out, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {'waypoints': {}}
            data.setdefault('waypoints', {})
        except Exception as exc:
            print('读取旧文件失败，将新建:', exc)

    used = set(data['waypoints'].keys())
    letters = [c for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' if c not in used]

    print('当前已有点位:', sorted(used) or '无')
    print('输入点位名后回车记录当前位姿；直接回车=自动取 %s；输入 q 退出。'
          % (letters[0] if letters else '新名字'))

    try:
        while True:
            default = letters[0] if letters else ''
            name = input('\n点位名 [%s]: ' % default).strip()
            if name.lower() == 'q':
                break
            if not name:
                if not default:
                    print('没有可用的自动字母了，请手动输入名字')
                    continue
                name = default

            pose = node.current()
            if pose is None:
                print('还没收到位姿：确认 nav2/AMCL 在跑，且点过 2D Pose')
                continue

            data['waypoints'][name] = {
                'x': round(pose[0], 3),
                'y': round(pose[1], 3),
                'yaw': round(pose[2], 3),
            }
            if name in letters:
                letters.remove(name)
            print('已记录 %s: x=%.3f y=%.3f yaw=%.3f' % (name, pose[0], pose[1], pose[2]))
    except KeyboardInterrupt:
        pass

    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print('\n已写入: %s' % out)
    print('别忘了同步 patrol.yaml 的 order 和 points（pan/tilt/expect）')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
