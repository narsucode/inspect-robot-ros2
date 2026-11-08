#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aruco_live —— ArUco 检测单独测试（实时视频窗口，边看边调）。

用途：
    不跑巡检、不动云台，只用来看“这块码到底能不能被认出来”。
    窗口里同时显示原始画面和增强后的灰度画面，一眼能看出是不是太暗。

用法：
    python3 ~/narsu/ros2/tools/aruco_live.py
    python3 ~/narsu/ros2/tools/aruco_live.py --url http://10.142.138.251:81/stream
    python3 ~/narsu/ros2/tools/aruco_live.py --no-enhance      # 关掉增强对比一下

按键：
    q / ESC   退出
    e         开/关 CLAHE 增强
    c / C     CLAHE 强度 -1 / +1（1~8）
    p         开/关 ArUco 参数“宽松模式”
    s         存当前原图到 ~/led_tests/（发给我看）
"""
import argparse
import os
import time

import cv2
import numpy as np

ARUCO_MAP = {0: 'A', 1: 'B', 2: 'C'}
OUT_DIR = os.path.expanduser('~/led_tests')


def make_detector(strict):
    ad = cv2.aruco
    d = ad.Dictionary_get(ad.DICT_4X4_50) if hasattr(ad, 'Dictionary_get') \
        else ad.getPredefinedDictionary(ad.DICT_4X4_50)
    params = ad.DetectorParameters_create() if hasattr(ad, 'DetectorParameters_create') \
        else ad.DetectorParameters()
    if not strict:
        try:
            params.adaptiveThreshConstant = 12
            params.minMarkerPerimeterRate = 0.015
            params.maxMarkerPerimeterRate = 6.0
            params.polygonalApproxAccuracyRate = 0.08
            params.errorCorrectionRate = 0.9
        except Exception:
            pass
    det = None if hasattr(ad, 'detectMarkers') else ad.ArucoDetector(d, params)
    return ad, d, params, det


def detect(frame, ad, d, params, det, enhance, clahe):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    g = clahe.apply(gray) if enhance else gray
    if det is not None:
        corners, ids, _ = det.detectMarkers(g)
    else:
        corners, ids, _ = ad.detectMarkers(g, d, parameters=params)
    return g, corners, ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://10.142.138.251:81/stream')
    ap.add_argument('--no-enhance', action='store_true')
    ap.add_argument('--clip', type=float, default=3.0)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        raise SystemExit('打不开视频流: %s' % args.url)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    enhance = not args.no_enhance
    clip = args.clip
    strict = True
    ad, d, params, det = make_detector(strict)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
    os.makedirs(OUT_DIR, exist_ok=True)

    t_prev = time.time()
    fps = 0.0
    last_log = 0.0
    print('已连上。e=增强开关  c/C=增强强度  p=参数宽松  s=存图  q=退出')

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        g, corners, ids = detect(frame, ad, d, params, det, enhance, clahe)

        # 左：原图（画框），右：增强灰度（检测实际用的画面）
        left = frame.copy()
        right = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        for img in (left, right):
            if ids is not None and len(ids) > 0:
                ad.drawDetectedMarkers(img, corners, ids)
                for i, c in enumerate(corners):
                    pts = c.reshape(-1, 2)
                    cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
                    name = ARUCO_MAP.get(int(ids.ravel()[i]), 'id%d' % int(ids.ravel()[i]))
                    cv2.circle(img, (cx, cy), 4, (0, 255, 0), -1)
                    cv2.putText(img, name, (cx + 6, cy), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)

        vis = np.hstack([left, right])
        h, w = vis.shape[:2]

        mean_g = float(g.mean())
        n = 0 if ids is None else len(ids)
        now = time.time()
        dt, t_prev = now - t_prev, now
        if dt > 0:
            fps = 0.9 * fps + 0.1 / dt

        cv2.putText(vis, 'enhance=%s clip=%.1f  mean=%.0f  found=%d  FPS=%.1f'
                    % ('ON' if enhance else 'OFF', clip, mean_g, n, fps),
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if n else (0, 0, 255), 2)
        cv2.putText(vis, 'q=quit e=enhance c/C=clip p=params s=save',
                    (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(vis, 'ORIGINAL', (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 0), 1)
        cv2.putText(vis, 'DETECT INPUT', (w // 2 + 8, 46), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 0), 1)
        cv2.imshow('ArUco test', vis)

        if now - last_log >= 1.0:
            last_log = now
            if n:
                print('检测到 %d 个码: %s   mean=%.0f' %
                      (n, [ARUCO_MAP.get(int(i), i) for i in ids.ravel()], mean_g))
            else:
                print('没检测到码   mean=%.0f  enhance=%s clip=%.1f' % (mean_g, enhance, clip))

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('e'):
            enhance = not enhance
            print('增强 ->', 'ON' if enhance else 'OFF')
        elif key == ord('c'):
            clip = max(1.0, clip - 1.0)
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
            print('clip ->', clip)
        elif key == ord('C'):
            clip = min(8.0, clip + 1.0)
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
            print('clip ->', clip)
        elif key == ord('p'):
            strict = not strict
            ad, d, params, det = make_detector(strict)
            print('参数 ->', '严格' if strict else '宽松')
        elif key == ord('s'):
            p1 = os.path.join(OUT_DIR, 'aruco_%s.jpg' % time.strftime('%H%M%S'))
            cv2.imwrite(p1, frame)
            cv2.imwrite(p1.replace('.jpg', '_enh.png'), g)
            print('已保存:', p1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
