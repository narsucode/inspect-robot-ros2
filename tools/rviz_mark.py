#!/usr/bin/env python3
"""
RViz 直接标点（不开车）

用法：
  1. 终端 1：ros2 launch roscar_navigation map_only.launch.py
  2. 终端 2：python3 ~/narsu/ros2/tools/rviz_mark.py
  3. 按提示输入点位名（A），回车
  4. 在 RViz 工具栏点 "2D Goal Pose"，在 A 柜位置按住往机柜方向拖一下松开
     —— 起点是位置，箭头方向是车头朝向
  5. 脚本记下 A，提示输入下一个；B、C、O 同理
  6. 全部标完输入 q 退出，再 --write 写入

  查看：python3 ~/narsu/ros2/tools/rviz_mark.py --show
  写入：python3 ~/narsu/ros2/tools/rviz_mark.py --write
"""

import json
import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

WAYPOINTS = os.path.expanduser(
    '~/narsu/ros2/patrol_ws/src/roscar_patrol/config/waypoints.yaml')
STASH = os.path.expanduser('~/.rviz_points.json')


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ClickCatcher(Node):
    def __init__(self):
        super().__init__('rviz_mark')
        self.pose = None
        self.create_subscription(PoseStamped, '/goal_pose', self.cb, 10)

    def cb(self, msg):
        p = msg.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))


def load():
    if os.path.exists(STASH):
        with open(STASH) as f:
            return json.load(f)
    return {}


def save(d):
    with open(STASH, 'w') as f:
        json.dump(d, f, indent=2, ensure_ascii=False)


def show(d):
    if not d:
        print('还没有点位')
        return
    print('\n已记录：')
    for k in sorted(d):
        v = d[k]
        print(f'  {k}:  x={v["x"]:+.3f}  y={v["y"]:+.3f}  yaw={v["yaw"]:+.3f}'
              f'  ({math.degrees(v["yaw"]):+.1f}°)')


def write_yaml(d):
    order = [k for k in ('A', 'B', 'C') if k in d]
    lines = ['# ?????RViz ???\n', 'waypoints:\n']
    for k in ('A', 'B', 'C', 'O'):
        if k not in d:
            continue
        v = d[k]
        lines.append(f'  {k}:\n')
        lines.append(f'    x: {v["x"]:.3f}\n')
        lines.append(f'    y: {v["y"]:.3f}\n')
        lines.append(f'    yaw: {v["yaw"]:.3f}\n')
    with open(WAYPOINTS, 'w') as f:
        f.writelines(lines)
    print(f'??? {WAYPOINTS}')
    print(f'?? {order}')


def main():
    args = sys.argv[1:]
    if args and args[0] == '--show':
        show(load()); return
    if args and args[0] == '--write':
        write_yaml(load()); return

    rclpy.init()
    node = ClickCatcher()
    d = load()

    print('等待 /goal_pose ...')
    print('RViz 工具栏要选 "2D Goal Pose"（不是 2D Pose Estimate）')
    print('输入点位名回车，然后在图上拖一下；输入 q 退出\n')

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    while True:
        try:
            name = input('点位名 (A/B/C/O，q 退出): ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if name.lower() in ('q', 'quit', 'exit', ''):
            break
        name = name.upper()

        node.pose = None
        print(f'  在图上的 {name} 位置按住拖出朝向，松开 ...')
        while node.pose is None:
            threading.Event().wait(0.1)

        x, y, yaw = node.pose
        d[name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        save(d)
        print(f'  ✓ {name}:  x={x:+.3f}  y={y:+.3f}  '
              f'yaw={yaw:+.3f} ({math.degrees(yaw):+.1f}°)\n')

    node.destroy_node()
    rclpy.shutdown()
    show(d)
    print('\n确认无误后执行：')
    print('  python3 ~/narsu/ros2/tools/rviz_mark.py --write')


if __name__ == '__main__':
    main()
