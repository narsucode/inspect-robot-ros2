#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""led_aim_demo —— 单窗口一体化演示：云台扫视找码 → 对准 → 识别灯色。

为什么单进程单窗口：
    之前用三个节点各开一个 cv2 窗口，GTK 要求主线程跑 GUI，
    在 ROS 定时器线程里 imshow 会把整个进程卡死，窗口也要等到
    第一帧才出现，看起来就是"很慢"和"卡死"。
    这里所有事情都在一个 main 循环里做：读帧 → 识别 → 画 → 显示 → 发云台指令。

云台不是一格一格跳：
    每帧只走 pan_speed/tilt_speed 度以内的距离，慢慢挪到目标角，
    到齐了才算这一格到站，再开始停稳计时。想更慢就调小 --pan-speed。

前提：micro-ROS agent 在跑（云台指令要发给板子）。
这个脚本自己占摄像头流，别同时开 traffic_light_node。

用法：
    python3 ~/narsu/ros2/tools/led_aim_demo.py A
    python3 ~/narsu/ros2/tools/led_aim_demo.py A --pan-speed 15 --tilt-speed 12
    python3 ~/narsu/ros2/tools/led_aim_demo.py A --expect RED

按键：q 退出   r 重跑一轮   s 存当前画面
"""
import argparse
import collections
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

URL = 'http://10.192.225.251:81/stream'
ARUCO_MAP = {0: 'A', 1: 'B', 2: 'C'}


class Ptz:
    def __init__(self):
        self.node = Node('led_aim_demo')
        self.pub = self.node.create_publisher(Vector3, '/cmd_ptz_angle', 10)

    def send(self, pan, tilt):
        m = Vector3()
        m.x = float(pan)
        m.y = float(tilt)
        self.pub.publish(m)


def make_detector():
    ad = cv2.aruco
    d = ad.Dictionary_get(ad.DICT_4X4_50) if hasattr(ad, 'Dictionary_get') \
        else ad.getPredefinedDictionary(ad.DICT_4X4_50)
    params = ad.DetectorParameters_create() if hasattr(ad, 'DetectorParameters_create') \
        else ad.DetectorParameters()
    det = None if hasattr(ad, 'detectMarkers') else ad.ArucoDetector(d, params)
    return ad, d, params, det


def detect_marker(frame, ad, d, params, det):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if det is not None:
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = ad.detectMarkers(gray, d, parameters=params)
    if ids is None or len(ids) == 0:
        return None
    pts = corners[0].reshape(-1, 2)
    h, w = gray.shape
    return {'code': ARUCO_MAP.get(int(ids.ravel()[0]), 'id%d' % int(ids.ravel()[0])),
            'u': float(pts[:, 0].mean()) / w, 'v': float(pts[:, 1].mean()) / h,
            'pts': pts}


def detect_led(frame, thresh_v=210, s_min=60, min_area=10):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    _, m = cv2.threshold(hsv[:, :, 2], thresh_v, 255, cv2.THRESH_BINARY)
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, 1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, 2)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:5]:
        area = cv2.contourArea(c)
        if area < min_area or area > frame.shape[0] * frame.shape[1] * 0.25:
            continue
        sel = cv2.drawContours(np.zeros(m.shape, np.uint8), [c], -1, 1, -1) > 0
        hp = hsv[sel]
        keep = hp[(hp[:, 1] >= s_min) & (hp[:, 2] >= thresh_v)]
        if len(keep) < 20 or float(hp[:, 1].mean()) < 25:
            continue
        hh = keep[:, 0].astype(np.int32)
        n = float(len(hh))
        cnt = {'RED': np.count_nonzero((hh <= 8) | (hh >= 172)) / n,
               'YELLOW': np.count_nonzero((hh >= 12) & (hh <= 38)) / n,
               'GREEN': np.count_nonzero((hh >= 40) & (hh <= 90)) / n}
        best = max(cnt, key=cnt.get)
        if cnt[best] < 0.25:
            continue
        x, y, w, h = cv2.boundingRect(c)
        out.append({'color': best, 'conf': cnt[best], 'box': (x, y, w, h),
                    'area': area})
    return out


class Demo:
    def __init__(self, args):
        self.args = args
        self.ptz = Ptz()
        self.ad, self.d, self.params, self.det = make_detector()
        self.cap = cv2.VideoCapture(args.url)
        if not self.cap.isOpened():
            raise SystemExit('打不开视频流: %s' % args.url)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # 扫视序列：先右后左，再由中间向外
        seq = [(-args.sweep, 0.0), (args.sweep, 0.0)]
        grid = []
        for tilt in (10, 0, -10, -20):
            for pan in range(int(args.pan_min), int(args.pan_max) + 1, int(args.pan_step)):
                grid.append((float(pan), float(tilt)))
        grid.sort(key=lambda q: abs(q[0]) / args.pan_step + 1.3 * abs(q[1]) / 10.0)
        for p in grid:
            if p not in seq:
                seq.append(p)
        self.seq = seq

        # cmd = 已经下发的角度（云台实际朝向的估计）；tgt = 想去的角度
        self.cmd = [0.0, 0.0]
        self.tgt = [0.0, 0.0]
        self.phase = 'RESET'
        self.t0 = time.time()
        self.idx = 0
        self.armed = False
        self.refine_n = 0
        self.bad = 0
        self.votes = collections.Counter()
        self.verdict = ''
        self.fps = 0.0
        self.gain = args.brightness
        self._t = time.time()

    def set_phase(self, name):
        self.phase = name
        self.t0 = time.time()
        self.armed = False

    def move(self, dt):
        """按速度限幅往目标角走。返回 True 表示已经到齐。"""
        sp = self.args.pan_speed * dt
        st = self.args.tilt_speed * dt
        dp = self.tgt[0] - self.cmd[0]
        dtl = self.tgt[1] - self.cmd[1]
        np_ = self.cmd[0] + max(-sp, min(sp, dp))
        nt = self.cmd[1] + max(-st, min(st, dtl))
        if abs(np_ - self.cmd[0]) > 1e-4 or abs(nt - self.cmd[1]) > 1e-4:
            self.cmd = [np_, nt]
            self.ptz.send(np_, nt)
        return abs(self.tgt[0] - self.cmd[0]) < 0.4 and abs(self.tgt[1] - self.cmd[1]) < 0.4

    def step(self, frame, t, dt):
        a = self.args
        arrived = self.move(dt)

        if self.phase == 'RESET':
            self.tgt = [0.0, 0.0]
            if arrived and t - self.t0 > 0.6:
                self.set_phase('SWEEP')
            return

        if self.phase == 'SWEEP':
            if self.idx >= len(self.seq):
                self.verdict = 'FAIL: 扫完没找到码'
                self.set_phase('RESULT')
                return
            pan, tilt = self.seq[self.idx]
            if self.tgt[0] != pan or self.tgt[1] != tilt:
                self.tgt = [pan, tilt]
                self.armed = False
                self.t0 = t
                return
            if not arrived:
                return
            if not self.armed:
                self.armed = True
                self.t0 = t
                return
            if t - self.t0 < a.settle:
                return
            hit = detect_marker(frame, self.ad, self.d, self.params, self.det)
            if hit and hit['code'].upper() == a.point.upper():
                self.refine_n = 0
                self.bad = 0
                self.set_phase('REFINE')
                return
            self.idx += 1
            self.armed = False
            self.t0 = t
            return

        if self.phase == 'REFINE':
            hit = detect_marker(frame, self.ad, self.d, self.params, self.det)
            if not hit:
                self.bad += 1
                if self.bad > 5:
                    self.verdict = 'FAIL: 对准时丢了码'
                    self.set_phase('RESULT')
                return
            self.bad = 0
            du = hit['u'] - a.target_u
            dv = hit['v'] - a.target_v
            if abs(du) <= a.tol and abs(dv) <= a.tol:
                self.set_phase('DETECT')
                return
            if not self.armed:
                self.armed = True
                self.t0 = t
                return
            if t - self.t0 < a.settle:
                return
            self.tgt = [max(-75.0, min(75.0, self.cmd[0] + a.pan_sign * du * 65.0 * a.refine_gain)),
                        max(-35.0, min(35.0, self.cmd[1] + a.tilt_sign * dv * 45.0 * a.refine_gain))]
            self.refine_n += 1
            self.armed = False
            self.t0 = t
            if self.refine_n > 6:
                self.set_phase('DETECT')
            return

        if self.phase == 'DETECT':
            lamps = detect_led(frame)
            self.votes[lamps[0]['color'] if lamps else 'OFF'] += 1
            if t - self.t0 >= a.dwell:
                top = self.votes.most_common(1)[0][0] if self.votes else 'OFF'
                self.verdict = '识别=%s' % top
                if a.expect:
                    self.verdict += '  期望=%s  %s' % (
                        a.expect, 'PASS' if top == a.expect else 'MISMATCH')
                self.set_phase('RESULT')
            return

    def draw(self, frame, t):
        a = self.args
        # 只提亮“看”的画面，不改变送入检测的原始帧，避免影响判色阈值
        vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
        h, w = vis.shape[:2]
        cv2.drawMarker(vis, (int(a.target_u * w), int(a.target_v * h)),
                       (0, 255, 0), cv2.MARKER_CROSS, 26, 2)

        hit = detect_marker(frame, self.ad, self.d, self.params, self.det)
        if hit:
            cv2.polylines(vis, [hit['pts'].astype(int).reshape(-1, 1, 2)], True,
                          (0, 0, 255), 2)
            cv2.putText(vis, '%s u=%.2f v=%.2f' % (hit['code'], hit['u'], hit['v']),
                        (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        for l in detect_led(frame):
            x, y, bw, bh = l['box']
            col = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0),
                   'YELLOW': (0, 255, 255)}.get(l['color'], (200, 200, 200))
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), col, 2)
            cv2.putText(vis, '%s %.2f' % (l['color'], l['conf']), (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        cv2.putText(vis, '%s  pan=%+.1f tilt=%+.1f  %d/%d  FPS=%.1f'
                    % (self.phase, self.cmd[0], self.cmd[1],
                       min(self.idx + 1, len(self.seq)), len(self.seq), self.fps),
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        if self.verdict:
            cv2.putText(vis, self.verdict, (8, 76), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
        cv2.putText(vis, 'q=quit r=restart s=save  +/-=brightness %.1fx' % self.gain,
                    (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        return vis

    def run(self):
        print('阶段：RESET → SWEEP（先右后左）→ REFINE → DETECT → RESULT')
        print('云台速度：pan %.0f°/s  tilt %.0f°/s'
              % (self.args.pan_speed, self.args.tilt_speed))
        while True:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            t = time.time()
            dt = t - self._t
            self._t = t
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 / dt

            self.step(frame, t, dt)
            vis = self.draw(frame, t)
            cv2.imshow('LED aim demo', vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('r'):
                self.idx = 0
                self.votes.clear()
                self.verdict = ''
                self.refine_n = 0
                self.bad = 0
                self.set_phase('RESET')
            if key == ord('s'):
                p = '/home/ros/led_tests/demo_%s.png' % time.strftime('%H%M%S')
                cv2.imwrite(p, vis)
                print('已保存: %s' % p)
            if key in (ord('+'), ord('=')):
                self.gain = min(3.0, self.gain + 0.1)
            if key in (ord('-'), ord('_')):
                self.gain = max(0.5, self.gain - 0.1)
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('point', nargs='?', default='A')
    ap.add_argument('--url', default=URL)
    ap.add_argument('--expect', default='')
    ap.add_argument('--dwell', type=float, default=3.0)
    ap.add_argument('--settle', type=float, default=0.5)
    ap.add_argument('--sweep', type=float, default=25.0)
    ap.add_argument('--pan-min', type=float, default=-45.0)
    ap.add_argument('--pan-max', type=float, default=45.0)
    ap.add_argument('--pan-step', type=float, default=15.0)
    ap.add_argument('--pan-speed', type=float, default=25.0, help='水平转速 度/秒')
    ap.add_argument('--tilt-speed', type=float, default=20.0, help='俯仰转速 度/秒')
    ap.add_argument('--target-u', type=float, default=0.5)
    ap.add_argument('--target-v', type=float, default=0.38)
    ap.add_argument('--tol', type=float, default=0.08)
    ap.add_argument('--refine-gain', type=float, default=0.5, help='对准闭环增益')
    ap.add_argument('--brightness', type=float, default=1.6, help='显示亮度倍数（不影响检测）')
    ap.add_argument('--pan-sign', type=float, default=-1.0)
    ap.add_argument('--tilt-sign', type=float, default=-1.0)

    args = ap.parse_args()
    rclpy.init()
    demo = Demo(args)
    try:
        demo.run()
    except KeyboardInterrupt:
        pass
    finally:
        demo.cap.release()
        cv2.destroyAllWindows()
        demo.ptz.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
