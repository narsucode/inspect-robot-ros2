#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ptz_hold —— 云台驱动：固定角度 / 左右扫视 / 空转。

三种模式：
    固定：ptz_hold.py 3.3 0.1
    扫视：ptz_hold.py --sweep 35 --speed 20        # ±35° 来回摆，20°/s
    空转：ptz_hold.py                              # 什么都不做

为什么按频率重复发：
    下位机是「收到就动」的一次性范式，UDP 丢一包云台就不动了。
"""
import argparse
import time


def idle():
    print('未指定角度，云台保持不动')
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pan', nargs='?', default='')
    ap.add_argument('tilt', nargs='?', default='')
    ap.add_argument('--sweep', type=float, default=0.0, help='扫视幅度（度），0=不扫视')
    ap.add_argument('--speed', type=float, default=20.0, help='扫视角速度 度/秒')
    ap.add_argument('--center', type=float, default=0.0, help='扫视中心角（度）')
    ap.add_argument('--tilt-angle', type=float, default=0.0, help='扫视时的俯仰角')
    args = ap.parse_args()

    sweep = args.sweep > 0.5
    if not sweep and (not str(args.pan).strip() or not str(args.tilt).strip()):
        idle()
        return

    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.node import Node

    rclpy.init()
    n = Node('ptz_hold')
    pub = n.create_publisher(Vector3, '/cmd_ptz_angle', 10)

    if sweep:
        center = args.center
        amp = args.sweep
        tilt = args.tilt_angle
        step = max(0.5, args.speed * 0.05)      # 每 50 ms 走多少度
        pan = center + amp
        direction = -1
        print('云台扫视：中心 %.0f°，幅度 ±%.0f°，速度 %.0f°/s，tilt=%.0f'
              % (center, amp, args.speed, tilt))
    else:
        pan = float(args.pan)
        tilt = float(args.tilt)
        print('云台固定：pan=%.1f tilt=%.1f' % (pan, tilt))

    # ??? 0/0?????????????????????
    m0 = Vector3()
    t_c = time.time()
    while time.time() - t_c < 0.8:
        pub.publish(m0)
        rclpy.spin_once(n, timeout_sec=0.05)
    if sweep:
        pan = center + amp
    print('??? 0/0?????')

    m = Vector3()
    try:
        while rclpy.ok():
            if sweep:
                pan += direction * step
                if pan <= center - amp:
                    pan = center - amp
                    direction = 1
                elif pan >= center + amp:
                    pan = center + amp
                    direction = -1
            m.x = float(pan)
            m.y = float(tilt)
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        print('\n云台停止')
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
