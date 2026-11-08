#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""save_angle —— 把某个点位的云台角度写进 patrol.yaml。

用法：
    python3 ~/narsu/ros2/tools/save_angle.py B 12 -5
    python3 ~/narsu/ros2/tools/save_angle.py C -20 -8
    python3 ~/narsu/ros2/tools/save_angle.py            # 只看当前所有角度

说明：
    pan  正值 = 向左，负值 = 向右
    tilt 正值 = 向上，负值 = 向下
"""
import os
import sys

import yaml

CFG = os.path.expanduser('~/narsu/ros2/patrol_ws/src/roscar_patrol/config')
PT = os.path.join(CFG, 'patrol.yaml')

HEADER = """# 巡检点位配置
#
# order: 巡检顺序，必须是 waypoints.yaml 里存在的点位名
# points: 每个点位的云台预设角度（度）和期望状态（可选，用于自动核对）
#         pan  水平角度，正值向左（pan 减小 = 向右）
#         tilt 俯仰角度，正值向上（tilt 减小 = 向下）
#         expect 期望识别到的状态：RED / YELLOW / GREEN，不写就只记录不核对
"""


def main():
    with open(PT, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    pts = cfg.setdefault('points', {})

    if len(sys.argv) < 2:
        print('当前角度：')
        for k in sorted(pts):
            v = pts[k]
            print('  %-3s pan=%6.1f  tilt=%6.1f  inspect=%s'
                  % (k, float(v.get('pan', 0)), float(v.get('tilt', 0)),
                     v.get('inspect', True)))
        print('\n用法: save_angle.py B <pan> <tilt>')
        return

    if len(sys.argv) < 4:
        print('用法: save_angle.py <点位> <pan> <tilt>')
        sys.exit(1)

    name = sys.argv[1].upper()
    pan = float(sys.argv[2])
    tilt = float(sys.argv[3])

    if name not in pts:
        print('点位 %s 不存在，现有：%s' % (name, ', '.join(sorted(pts))))
        sys.exit(1)

    pts[name]['pan'] = pan
    pts[name]['tilt'] = tilt

    body = yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False)
    with open(PT, 'w', encoding='utf-8') as f:
        f.write(HEADER + '\n' + body)

    print('已写入 %s: pan=%.1f tilt=%.1f' % (name, pan, tilt))
    for k in sorted(pts):
        v = pts[k]
        print('  %-3s pan=%6.1f  tilt=%6.1f' % (k, float(v.get('pan', 0)), float(v.get('tilt', 0))))


if __name__ == '__main__':
    main()
