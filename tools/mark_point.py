#!/usr/bin/env python3
"""
点位标定工具

用法：
  1. 起 bringup + navigation（RViz 给好初始位姿）
  2. 手动遥控车到目标位置，车头朝机柜
  3. 另开终端：
       python3 ~/narsu/ros2/tools/mark_point.py A
     看到 "已记录 A" 就成了，继续开去下一个点
  4. 全标完：
       python3 ~/narsu/ros2/tools/mark_point.py --show
     确认无误后：
       python3 ~/narsu/ros2/tools/mark_point.py --write

为什么必须在车内看 /amcl_pose：
  里程计和地图的坐标系是 map，只有 AMCL 收敛后的位姿才是
  "地图上的真实位置"。拿 RViz 手点只能得 x/y，得不出朝向。
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

WAYPOINTS = os.path.expanduser(
    '~/narsu/ros2/patrol_ws/src/roscar_patrol/config/waypoints.yaml')
STASH = os.path.expanduser('~/.patrol_points.json')


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MarkPoint(Node):
    def __init__(self):
        super().__init__('mark_point')
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb, 10)

    def cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))


def load_stash():
    import json
    if os.path.exists(STASH):
        with open(STASH) as f:
            return json.load(f)
    return {}


def save_stash(d):
    import json
    with open(STASH, 'w') as f:
        json.dump(d, f, indent=2, ensure_ascii=False)


def show(d):
    if not d:
        print('还没有记录任何点位')
        return
    print('\n已记录：')
    for k in sorted(d):
        v = d[k]
        print(f'  {k}:  x={v["x"]:+.3f}  y={v["y"]:+.3f}  yaw={v["yaw"]:+.3f}'
              f'  ({math.degrees(v["yaw"]):+.1f}°)')


def write_yaml(d):
    order = [k for k in ('A', 'B', 'C') if k in d]
    home = 'O' if 'O' in d else (order[-1] if order else 'O')
    lines = []
    lines.append('# 巡检点位（新地图 room_20261107）\n')
    lines.append('frame_id: map\n\n')
    for k in sorted(d):
        v = d[k]
        lines.append(f'{k}:\n')
        lines.append(f'  x: {v["x"]:.3f}\n')
        lines.append(f'  y: {v["y"]:.3f}\n')
        lines.append(f'  yaw: {v["yaw"]:.3f}\n')
    lines.append('\norder: [' + ', '.join(order) + ']\n')
    lines.append(f'home_point: {home}\n')
    with open(WAYPOINTS, 'w') as f:
        f.writelines(lines)
    print(f'已写入 {WAYPOINTS}')
    print(f'巡检顺序：{order}   返航点：{home}')


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return

    if args[0] == '--show':
        show(load_stash())
        return
    if args[0] == '--write':
        write_yaml(load_stash())
        return

    name = args[0]
    rclpy.init()
    node = MarkPoint()

    print(f'等待 /amcl_pose（{name}）...')
    deadline = time.time() + 10.0
    while time.time() < deadline and node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.pose is None:
        print('收不到 /amcl_pose')
        print('检查：1) navigation 起了吗  2) RViz 里点过 2D Pose Estimate 吗')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 多采几帧取平均，避免单帧抖动
    samples = [node.pose]
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is not None:
            samples.append(node.pose)

    n = len(samples)
    x = sum(s[0] for s in samples) / n
    y = sum(s[1] for s in samples) / n
    # 角度取平均要处理绕圈
    sy = sum(math.sin(s[2]) for s in samples)
    cy = sum(math.cos(s[2]) for s in samples)
    yaw = math.atan2(sy, cy)

    node.destroy_node()
    rclpy.shutdown()

    d = load_stash()
    d[name] = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
    save_stash(d)

    print(f'\n已记录 {name}:  x={x:+.3f}  y={y:+.3f}  yaw={yaw:+.3f}  '
          f'({math.degrees(yaw):+.1f}°  {n} 帧平均)')
    show(d)


if __name__ == '__main__':
    main()
