#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""code_led_check —— 只验证视觉：能不能认出码、能不能认出灯色。

不导航、不动云台、不需要开别的节点。
它自己占摄像头，所以跑之前要确认没有 traffic_light_node / aim_marker 在跑。

画面：
    左半：原始画面，画 ArUco 框（绿）和灯色框（按颜色）
    右半：送进检测器的增强灰度图（暗光时看这里判断是不是对比度不够）
    顶部：码 = A / 无    灯 = GREEN(0.91) / 无
    底部：按键提示

按键：
    q/ESC 退出    e 增强开关    c/C 增强强度    s 存图    p 参数严格/宽松

用法：
    python3 ~/narsu/ros2/tools/code_led_check.py
    python3 ~/narsu/ros2/tools/code_led_check.py --url http://10.142.138.251:81/stream
"""
import argparse
import os
import time

import cv2
import numpy as np

ARUCO_MAP = {0: 'A', 1: 'B', 2: 'C'}
OUT_DIR = os.path.expanduser('~/led_tests')
LED_COLORS = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'YELLOW': (0, 255, 255)}


def make_aruco(loose):
    ad = cv2.aruco
    d = ad.Dictionary_get(ad.DICT_4X4_50) if hasattr(ad, 'Dictionary_get') \
        else ad.getPredefinedDictionary(ad.DICT_4X4_50)
    params = ad.DetectorParameters_create() if hasattr(ad, 'DetectorParameters_create') \
        else ad.DetectorParameters()
    if loose:
        try:
            params.adaptiveThreshConstant = 12
            params.minMarkerPerimeterRate = 0.015
            params.polygonalApproxAccuracyRate = 0.08
            params.errorCorrectionRate = 0.9
        except Exception:
            pass
    det = None if hasattr(ad, 'detectMarkers') else ad.ArucoDetector(d, params)
    return ad, d, params, det


def find_markers(gray, ad, d, params, det):
    if det is not None:
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = ad.detectMarkers(gray, d, parameters=params)
    out = []
    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            pts = corners[i].reshape(-1, 2)
            out.append({'id': int(ids.ravel()[i]),
                        'name': ARUCO_MAP.get(int(ids.ravel()[i]), 'id%d' % int(ids.ravel()[i])),
                        'pts': pts})
    return out


def find_led(frame, thresh_v=205, s_min=60, min_area=12):
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
        if len(keep) < 15 or float(hp[:, 1].mean()) < 25:
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
        out.append({'color': best, 'conf': cnt[best], 'box': (x, y, w, h), 'area': area})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://10.142.138.251:81/stream')
    ap.add_argument('--clip', type=float, default=3.0)
    ap.add_argument('--thresh-v', type=int, default=205)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        raise SystemExit('打不开视频流: %s（检查摄像头 IP 和是否有别的节点占着）' % args.url)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    enhance = True
    clip = args.clip
    loose = True
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
    ad, d, params, det = make_aruco(loose)
    os.makedirs(OUT_DIR, exist_ok=True)

    t_prev = time.time()
    fps = 0.0
    last_log = 0.0
    print('只看不跑模式。q 退出；e 增强开关；c/C 增强强度；s 存图；p 参数')

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        g = clahe.apply(gray) if enhance else gray
        markers = find_markers(g, ad, d, params, det)
        lamps = find_led(frame, thresh_v=args.thresh_v)

        left = frame.copy()
        for mk in markers:
            cv2.polylines(left, [mk['pts'].astype(int).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            cx, cy = int(mk['pts'][:, 0].mean()), int(mk['pts'][:, 1].mean())
            cv2.circle(left, (cx, cy), 4, (0, 255, 0), -1)
            cv2.putText(left, mk['name'], (cx + 6, cy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
        for lp in lamps:
            x, y, w, h = lp['box']
            col = LED_COLORS.get(lp['color'], (200, 200, 200))
            cv2.rectangle(left, (x, y), (x + w, y + h), col, 2)
            cv2.putText(left, '%s %.2f' % (lp['color'], lp['conf']), (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        right = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        vis = np.hstack([left, right])
        hh, ww = vis.shape[:2]
        half = ww // 2

        now = time.time()
        dt, t_prev = now - t_prev, now
        if dt > 0:
            fps = 0.9 * fps + 0.1 / dt

        code_txt = ('码=%s' % '/'.join(m['name'] for m in markers)) if markers else '码=无'
        if lamps:
            led_txt = '灯=%s(%.2f)' % (lamps[0]['color'], lamps[0]['conf'])
        else:
            led_txt = '灯=无'
        cv2.putText(vis, '%s   %s' % (code_txt, led_txt), (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if markers else (0, 0, 255), 2)
        cv2.putText(vis, 'enhance=%s clip=%.1f mean=%.0f FPS=%.1f'
                    % ('ON' if enhance else 'OFF', clip, float(g.mean()), fps),
                    (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(vis, 'q=quit e=enhance c/C=clip s=save p=params', (8, hh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(vis, 'ORIGINAL', (8, hh - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 0), 1)
        cv2.putText(vis, 'DETECT INPUT', (half + 8, hh - 34), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 0), 1)
        cv2.imshow('code + led check', vis)

        if now - last_log >= 1.0:
            last_log = now
            print('%s   %s   mean=%.0f' % (code_txt, led_txt, float(g.mean())))

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('e'):
            enhance = not enhance
        elif key == ord('c'):
            clip = max(1.0, clip - 1.0)
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
        elif key == ord('C'):
            clip = min(8.0, clip + 1.0)
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
        elif key == ord('p'):
            loose = not loose
            ad, d, params, det = make_aruco(loose)
            print('参数 ->', '宽松' if loose else '严格')
        elif key == ord('s'):
            p1 = os.path.join(OUT_DIR, 'check_%s.jpg' % time.strftime('%H%M%S'))
            cv2.imwrite(p1, frame)
            cv2.imwrite(p1.replace('.jpg', '_enh.png'), g)
            print('已保存:', p1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
