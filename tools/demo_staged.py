#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo_staged —— 答辩演示：云台巡视 → 发现码 → 停下居中 → 识别灯色。

单进程单窗口：自己拉流、自己发云台指令（/cmd_ptz_angle），不依赖 ROS 图像话题。

三个阶段（画面标题会变）：
    STAGE 1 SEARCH   云台慢速左右巡视，画面上只有绿框（码）和绿十字，没有灯框
    STAGE 1 ALIGN    发现码后停下，取新帧、逐步把码挪到十字位置
    STAGE 2 DETECT   对准完成后，才出现灯框与颜色判定

用法：
    python3 ~/narsu/ros2/tools/demo_staged.py
    python3 ~/narsu/ros2/tools/demo_staged.py --sweep 30 --speed 6 --gain 2.2
    python3 ~/narsu/ros2/tools/demo_staged.py --tilt-angle -6

按键：q 退出   r 重来一轮   s 存图
"""
import argparse
import collections
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

URL = 'http://10.142.138.251:81/stream'
ARUCO_MAP = {0: 'A', 1: 'B', 2: 'C'}
LEVEL = {'GREEN': '正常', 'YELLOW': '警告', 'RED': '故障', 'OFF': '熄灭'}
LED_BGR = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'YELLOW': (0, 255, 255)}


def make_detector():
    ad = cv2.aruco
    d = ad.Dictionary_get(ad.DICT_4X4_50) if hasattr(ad, 'Dictionary_get') \
        else ad.getPredefinedDictionary(ad.DICT_4X4_50)
    p = ad.DetectorParameters_create() if hasattr(ad, 'DetectorParameters_create') \
        else ad.DetectorParameters()
    try:
        p.adaptiveThreshConstant = 10
        p.minMarkerPerimeterRate = 0.02
        p.polygonalApproxAccuracyRate = 0.06
    except Exception:
        pass
    det = None if hasattr(ad, 'detectMarkers') else ad.ArucoDetector(d, p)
    return ad, d, p, det


def find_marker(gray, ad, d, p, det):
    if det is not None:
        c, ids, _ = det.detectMarkers(gray)
    else:
        c, ids, _ = ad.detectMarkers(gray, d, parameters=p)
    if ids is None or len(ids) == 0:
        return None
    pts = c[0].reshape(-1, 2)
    h, w = gray.shape
    return {'id': int(ids.ravel()[0]),
            'name': ARUCO_MAP.get(int(ids.ravel()[0]), 'id%d' % int(ids.ravel()[0])),
            'u': float(pts[:, 0].mean()) / w,
            'v': float(pts[:, 1].mean()) / h,
            'pts': pts}


def find_led(frame, thresh_v=200, s_min=60, min_area=8):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    _, m = cv2.threshold(hsv[:, :, 2], thresh_v, 255, cv2.THRESH_BINARY)
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, 1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, 2)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:4]:
        area = cv2.contourArea(c)
        if area < min_area or area > frame.shape[0] * frame.shape[1] * 0.25:
            continue
        sel = cv2.drawContours(np.zeros(m.shape, np.uint8), [c], -1, 1, -1) > 0
        hp = hsv[sel]
        keep = hp[(hp[:, 1] >= s_min) & (hp[:, 2] >= thresh_v)]
        if len(keep) < 8 or float(hp[:, 1].mean()) < 15:
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
        out.append({'color': best, 'conf': cnt[best], 'box': (x, y, w, h)})
    return out


class Demo:
    SEARCH = 'SEARCH'
    ALIGN = 'ALIGN'
    DETECT = 'DETECT'

    def __init__(self, args):
        self.args = args
        self.node = Node('demo_staged')
        self.pub = self.node.create_publisher(Vector3, '/cmd_ptz_angle', 10)
        self.ad, self.d, self.p, self.det = make_detector()
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        self.cap = cv2.VideoCapture(args.url)
        if not self.cap.isOpened():
            raise SystemExit('打不开视频流: %s' % args.url)

        self.pan = -float(args.sweep)      # ??????
        self.tilt = float(args.tilt_angle)
        self.dir = 1                       # ?? ? ??
        self.stage = Demo.SEARCH
        self.align_n = 0
        self.votes = collections.Counter()
        self.t_detect = 0.0
        self.verdict = ''
        self.fps = 0.0
        self._t = time.time()
        self.hit = None

    # ---------- 基础动作 ----------
    def ptz(self, pan, tilt, hold=0.3):
        m = Vector3()
        m.x = float(pan)
        m.y = float(tilt)
        t0 = time.time()
        while time.time() - t0 < hold:
            self.pub.publish(m)
            rclpy.spin_once(self.node, timeout_sec=0.04)
        self.pan, self.tilt = float(pan), float(tilt)

    def fresh(self, n=3):
        """丢掉缓冲旧帧，取最新一帧（云台刚转完时尤其重要）。"""
        frame = None
        for _ in range(max(1, n)):
            ok, f = self.cap.read()
            if ok and f is not None:
                frame = f
        return frame

    def detect_code(self, frame):
        gray = self.clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        return find_marker(gray, self.ad, self.d, self.p, self.det)

    def reset(self):
        self.stage = Demo.SEARCH
        self.votes.clear()
        self.verdict = ''
        self.align_n = 0
        print('--- 重来一轮 ---')

    # ---------- 状态机 ----------
    def step(self, frame, dt):
        a = self.args

        if self.stage == Demo.SEARCH:
            self.hit = self.detect_code(frame)
            if self.hit:
                self.ptz(self.pan, self.tilt, hold=0.5)   # 先停稳
                self.stage = Demo.ALIGN
                self.align_n = 0
                print('[发现靶标] 码=%s  pan=%.1f tilt=%.1f → 停止巡视，开始居中'
                      % (self.hit['name'], self.pan, self.tilt))
                return
            # 慢速巡视
            step = a.speed * min(dt, 0.1)      # ???? dt ?????????
            self.pan += self.dir * step
            if self.pan <= -a.sweep:
                self.pan = -a.sweep
                self.dir = 1
            elif self.pan >= a.sweep:
                self.pan = a.sweep
                self.dir = -1
            self.ptz(self.pan, a.tilt_angle, hold=0.05)   # ???????????
            return

        if self.stage == Demo.ALIGN:
            f2 = self.fresh(4)
            if f2 is None:
                return
            hit = self.detect_code(f2)
            self.hit = hit
            if not hit:
                self.align_n += 1
                if self.align_n > 6:
                    print('[放弃居中] 丢失靶标，回到巡视')
                    self.stage = Demo.SEARCH
                return

            du = hit['u'] - a.target_u
            dv = hit['v'] - a.target_v
            self.align_n += 1
            print('  居中 %d: du=%+.2f dv=%+.2f  pan=%.1f tilt=%.1f'
                  % (self.align_n, du, dv, self.pan, self.tilt))
            if abs(du) <= a.tol and abs(dv) <= a.tol:
                print('[对准完成] 码=%s  pan=%.1f tilt=%.1f' % (hit['name'], self.pan, self.tilt))
                self.stage = Demo.DETECT
                self.t_detect = time.time()
                return
            if self.align_n > 8:
                print('[对准结束] 未完全居中，直接进入识别')
                self.stage = Demo.DETECT
                self.t_detect = time.time()
                return

            self.pan = max(-80.0, min(80.0, self.pan - du * 65.0 * 0.7))
            self.tilt = max(-35.0, min(35.0, self.tilt - dv * 45.0 * 0.7))
            self.ptz(self.pan, self.tilt, hold=0.5)
            return

        # DETECT
        lamps = find_led(frame)
        if lamps:
            self.votes[lamps[0]['color']] += 1
        else:
            self.votes['OFF'] += 1
        if time.time() - self.t_detect > a.dwell:
            top = self.votes.most_common(1)[0][0]
            self.verdict = '%s（%s）' % (top, LEVEL.get(top, '未知'))
            print('[识别完成] %s   分布=%s' % (self.verdict, dict(self.votes)))

    # ---------- 画面 ----------
    def draw(self, frame):
        a = self.args
        vis = np.clip(frame.astype(np.float32) * a.gain, 0, 255).astype(np.uint8)
        h, w = vis.shape[:2]

        if self.stage in (Demo.SEARCH, Demo.ALIGN):
            cv2.drawMarker(vis, (int(a.target_u * w), int(a.target_v * h)),
                           (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
            if self.hit:
                cv2.polylines(vis, [self.hit['pts'].astype(int).reshape(-1, 1, 2)],
                              True, (0, 255, 0), 2)
                cv2.putText(vis, self.hit['name'],
                            (int(self.hit['u'] * w) + 8, int(self.hit['v'] * h)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(vis, 'STAGE 1  %s   码=%s'
                        % ('SEARCHING' if self.stage == Demo.SEARCH else 'ALIGNING',
                           self.hit['name'] if self.hit else '-'),
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            for lp in find_led(frame):
                x, y, bw, bh = lp['box']
                col = LED_BGR.get(lp['color'], (200, 200, 200))
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), col, 2)
                cv2.putText(vis, '%s %.2f' % (lp['color'], lp['conf']),
                            (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            cv2.putText(vis, 'STAGE 2  DETECT   %s' % (self.verdict or 'sampling...'),
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(vis, 'pan=%+.1f tilt=%+.1f  fps=%.1f   q=quit r=restart s=save'
                    % (self.pan, self.tilt, self.fps),
                    (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        return vis

    def run(self):
        print('STAGE1 巡视找码 → 停 → 居中 → STAGE2 判灯')
        self.ptz(0.0, 0.0, hold=1.0)   # start from center
        print('gimbal centered, start scanning')
        while True:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            now = time.time()
            dt = now - self._t
            self._t = now
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 / dt

            self.step(frame, dt)
            cv2.imshow('demo_staged', self.draw(frame))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('r'):
                self.reset()
            if key == ord('s'):
                p = '/home/ros/patrol_shots/demo_%s.png' % time.strftime('%H%M%S')
                cv2.imwrite(p, self.draw(frame))
                print('已保存:', p)
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default=URL)
    ap.add_argument('--sweep', type=float, default=35.0, help='巡视幅度（度）')
    ap.add_argument('--speed', type=float, default=8.0, help='巡视速度 度/秒')
    ap.add_argument('--tilt-angle', type=float, default=-4.0, help='巡视时的俯仰角')
    ap.add_argument('--gain', type=float, default=2.0, help='显示亮度倍数')
    ap.add_argument('--dwell', type=float, default=3.0, help='阶段二采样秒数')
    ap.add_argument('--target-u', type=float, default=0.5)
    ap.add_argument('--target-v', type=float, default=0.42)
    ap.add_argument('--tol', type=float, default=0.07)
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
        demo.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
