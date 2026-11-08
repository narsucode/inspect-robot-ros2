#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""led_color_live —— 指示灯颜色识别：实时视频测试（窗口 + 实时调参）。

为什么不用灰度阈值：
    红灯 (255,0,0) 的灰度只有 76，绿灯 150，用"灰度>200"会直接把红绿灯滤掉，
    只剩黄灯和白色反光能过。所以亮斑检测改成"三通道最大值"(HSV 的 V)：
    红 V=255、绿 V=255、黄 V=255，任何颜色的灯都能被捞出来。

判色两套并用（互相印证）：
    hue   在饱和像素(S>=s_min)里取主色相，按色相区间判红/黄/绿 —— 主判据
    ratio 通道和比值 —— 备用，饱和像素太少时使用

用法：
    python3 ~/narsu/ros2/tools/led_color_live.py
    python3 ~/narsu/ros2/tools/led_color_live.py --thresh 180 --min-area 10
    python3 ~/narsu/ros2/tools/led_color_live.py --mask v|gray|any

按键：
    q/ESC 退出   s 存图   e/E 阈值∓5   d/D 最小面积∓5   m 切换 mask 方式
"""
import argparse
import os
import time

import cv2
import numpy as np

OUT_DIR = os.path.expanduser('~/led_tests')

COLOR_BGR = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'YELLOW': (0, 255, 255),
             'BLUE': (255, 0, 0), 'DIM': (128, 128, 128), 'SATURATED': (255, 0, 255),
             'OFF': (110, 110, 110), 'WHITE': (150, 150, 150), 'UNKNOWN': (255, 255, 255)}


def build_mask(img, thresh, mode):
    """亮斑掩码。mode=v 取三通道最大值；gray 取灰度；any 任一通道超阈值。"""
    b, g, r = cv2.split(img)
    if mode == 'gray':
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, m = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    elif mode == 'any':
        m = ((b.astype(int) > thresh) | (g.astype(int) > thresh) | (r.astype(int) > thresh))
        m = (m.astype(np.uint8)) * 255
    else:  # v
        v = np.maximum(np.maximum(b, g), r)
        _, m = cv2.threshold(v, thresh, 255, cv2.THRESH_BINARY)
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
    return m


def hue_state(pix_hsv):
    """在饱和像素里取主色相判色。返回 (状态, 占比)。"""
    if len(pix_hsv) == 0:
        return 'SATURATED', 0.0
    h = pix_hsv[:, 0].astype(int)          # 0..179 (OpenCV)
    n = float(len(h))
    red = np.sum((h <= 8) | (h >= 172))
    yellow = np.sum((h >= 12) & (h <= 38))
    green = np.sum((h >= 40) & (h <= 90))
    blue = np.sum((h >= 95) & (h <= 135))
    counts = {'RED': red / n, 'YELLOW': yellow / n, 'GREEN': green / n, 'BLUE': blue / n}
    best = max(counts, key=counts.get)
    if counts[best] < 0.25:
        return 'UNKNOWN', counts[best]
    return best, counts[best]


def ratio_state(r, g, b):
    s = r + g + b
    if s < 1:
        return 'OFF', 0.0
    rr, gg, bb = r / s, g / s, b / s
    if rr > 0.5 and rr / max(bb, 1e-6) > 1.4:
        return 'RED', rr
    if gg > 0.45 and gg > rr + bb:
        return 'GREEN', gg
    if rr > 0.3 and gg > 0.3 and bb < 0.28:
        return 'YELLOW', (rr + gg) / 2
    if bb > 0.5 and bb > rr + gg:
        return 'BLUE', bb
    return 'UNKNOWN', max(rr, gg, bb)


def analyse(img, thresh, min_area, sat_cut, s_min, mode):
    mask = build_mask(img, thresh, mode)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        sel = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(sel, [c], -1, 255, -1)
        m = sel > 0
        pix = img[m]
        hp = hsv[m]
        if pix.size == 0:
            continue

        b_all, g_all, r_all = (float(pix[:, i].sum()) for i in range(3))
        st_ratio, conf_ratio = ratio_state(r_all, g_all, b_all)

        # 判色只用"够亮又够饱和"的像素：纯白核 S≈0 被排除
        keep = (hp[:, 1] >= s_min) & (hp[:, 2] >= thresh)
        st_hue, conf_hue = hue_state(hp[keep])
        sat_pct = 100.0 * (1.0 - np.count_nonzero(keep) / float(len(hp)))
        mean_s = float(hp[:, 1].mean())

        # ???????"?????"???????????? WHITE?
        # ????????????????????????
        if np.count_nonzero(keep) < 20 or mean_s < 25:
            st_hue, conf_hue = 'WHITE', 0.0

        mean_v = float(hp[:, 2].mean())
        if mean_v < thresh * 0.6:
            st_hue = 'DIM'
        x, y, w, h = cv2.boundingRect(c)
        out.append({'box': (x, y, w, h), 'area': area, 'mean_v': mean_v, 'mean_s': mean_s,
                    'sat_pct': sat_pct, 'state': st_hue, 'conf': conf_hue,
                    'ratio_state': st_ratio, 'ratio_conf': conf_ratio,
                    'keep': int(np.count_nonzero(keep)), 'n': int(len(hp))})
    out.sort(key=lambda d: -d['area'])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://10.192.225.251:81/stream')
    ap.add_argument('--thresh', type=int, default=190)
    ap.add_argument('--min-area', type=float, default=10.0)
    ap.add_argument('--s-min', type=int, default=60)
    ap.add_argument('--sat-cut', type=int, default=250)
    ap.add_argument('--mask', choices=['v', 'gray', 'any'], default='v')
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        raise SystemExit('打不开视频流: %s' % args.url)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    thresh, min_area, mode = args.thresh, args.min_area, args.mask
    os.makedirs(OUT_DIR, exist_ok=True)
    t_prev, fps, last_log = time.time(), 0.0, 0.0
    print('连上了。q 退出；e/E 阈值∓5；d/D 面积∓5；m 换 mask(v/gray/any)')

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print('取帧失败，重连…')
            cap.release()
            time.sleep(1.0)
            cap = cv2.VideoCapture(args.url)
            continue

        blobs = analyse(frame, thresh, min_area, args.sat_cut, args.s_min, mode)
        vis = frame.copy()
        h, w = vis.shape[:2]

        for i, d in enumerate(blobs, 1):
            if d['state'] == 'WHITE':
                continue
            x, y, bw, bh = d['box']
            color = COLOR_BGR.get(d['state'], (255, 255, 255))
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(vis, '%d %s %.2f' % (i, d['state'], d['conf']),
                        (x, max(14, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        colored = [d for d in blobs if d['state'] not in ('WHITE',)]
        if colored:
            best = max(colored, key=lambda d: d['conf'])
            top = '%s  conf=%.2f  area=%.0f  V=%.0f' % (
                best['state'], best['conf'], best['area'], best['mean_v'])
            tcol = COLOR_BGR.get(best['state'], (255, 255, 255))
        else:
            top, tcol = 'OFF / no colored light', (170, 170, 170)
        cv2.putText(vis, top, (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, tcol, 2)

        now = time.time()
        dt, t_prev = now - t_prev, now
        if dt > 0:
            fps = 0.9 * fps + 0.1 / dt
        cv2.putText(vis, 'THR=%d MIN=%.0f MASK=%s S>=%d FPS=%.1f n=%d'
                    % (thresh, min_area, mode, args.s_min, fps, len(blobs)),
                    (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(vis, 'q=quit s=save e/E=thr d/D=area m=mask', (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.imshow('LED color test', vis)

        if now - last_log >= 1.0:
            last_log = now
            if not blobs:
                print('无亮斑   THR=%d MASK=%s' % (thresh, mode))
            for i, d in enumerate(blobs[:4], 1):
                if d['state'] == 'WHITE':
                    continue
                print('#%d area=%.0f V=%.0f S=%.0f keep=%d/%d  hue=%s(%.2f)  ratio=%s(%.2f)'
                      % (i, d['area'], d['mean_v'], d['mean_s'], d['keep'], d['n'],
                         d['state'], d['conf'], d['ratio_state'], d['ratio_conf']))

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('s'):
            p = os.path.join(OUT_DIR, 'live_%s.png' % time.strftime('%H%M%S'))
            cv2.imwrite(p, vis)
            print('已保存: %s' % p)
        elif key == ord('e'):
            thresh = max(60, thresh - 5)
        elif key == ord('E'):
            thresh = min(254, thresh + 5)
        elif key == ord('d'):
            min_area = max(3, min_area - 5)
        elif key == ord('D'):
            min_area += 5
        elif key == ord('m'):
            mode = {'v': 'gray', 'gray': 'any', 'any': 'v'}[mode]

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
