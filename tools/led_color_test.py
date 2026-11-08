#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""led_color_test —— 指示灯颜色判定算法的离线验证工具。

用法：
    python3 ~/narsu/ros2/tools/led_color_test.py                 # 直接抓一帧摄像头画面
    python3 ~/narsu/ros2/tools/led_color_test.py --image a.jpg   # 用本地图片
    python3 ~/narsu/ros2/tools/led_color_test.py --thresh 180 --min-area 30
    python3 ~/narsu/ros2/tools/led_color_test.py --sweep          # 200/220/240 三档对比

算法（两步）：
    1) 灰度 > thresh 二值化 → 找轮廓 → 按面积过滤噪点
    2) 对每个亮斑算通道和，用比值判红/黄/绿/灭/不确定

⚠ 关键改进：过曝的灯芯是纯白（三通道都接近 255），三通道和几乎相等，
   直接用比值会把所有灯都判成"不确定"。所以统计时剔除饱和像素
   （max(B,G,R) > sat_cut 的不参与判色），只看光晕边缘。
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

URL = 'http://10.192.225.251/capture'
OUT_DIR = os.path.expanduser('~/led_tests')


def grab(url):
    """从摄像头的 /capture 接口取一张图（比解流快，也干净）。"""
    import urllib.request
    tmp = '/tmp/led_grab.jpg'
    urllib.request.urlretrieve(url, tmp)
    return cv2.imread(tmp)


def classify(r, g, b):
    """纯比值判色。输入是通道和（已排除饱和像素）。"""
    s = r + g + b
    if s < 1:
        return 'OFF', 0.0
    rr, gg, bb = r / s, g / s, b / s

    # 红：R 明显占优
    if rr > 0.55 and rr / max(bb, 1e-6) > 1.6:
        return 'RED', rr
    # 绿：G 明显占优
    if gg > 0.5 and gg > rr + bb:
        return 'GREEN', gg
    # 黄：R、G 都高、B 很低（黄光 = 红+绿）
    if rr > 0.33 and gg > 0.33 and bb < 0.25:
        return 'YELLOW', (rr + gg) / 2.0
    # 蓝：B 占优（本课题用不到，但留着便于排查）
    if bb > 0.5 and bb > rr + gg:
        return 'BLUE', bb
    return 'UNKNOWN', max(rr, gg, bb)


def analyse(img, thresh, min_area, sat_cut, verbose=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rows = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        sel = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(sel, [c], -1, 255, -1)
        pix = img[sel > 0]
        if pix.size == 0:
            continue
        b_all, g_all, r_all = (float(pix[:, i].sum()) for i in range(3))

        # 只保留非饱和像素（灯芯纯白，颜色在边缘）
        mx = pix.max(axis=1)
        keep = pix[mx <= sat_cut]
        sat_ratio = 1.0 - (len(keep) / float(len(pix)))
        if len(keep) > 0:
            b_u, g_u, r_u = (float(keep[:, i].sum()) for i in range(3))
        else:
            b_u = g_u = r_u = 0.0

        state_all, conf_all = classify(r_all, g_all, b_all)
        state_u, conf_u = classify(r_u, g_u, r_u)

        # 亮度太低说明只是反光/噪点，不是点亮的灯
        mean_v = float(gray[sel > 0].mean())
        if mean_v < 120:
            state_u = 'DIM'

        x, y, w, h = cv2.boundingRect(c)
        rows.append({
            'area': area, 'x': x, 'y': y, 'w': w, 'h': h, 'mean_v': mean_v,
            'r': r_all, 'g': g_all, 'b': b_all,
            'state_all': state_all, 'state': state_u, 'conf': conf_u,
            'sat': sat_ratio * 100.0,
        })

    rows.sort(key=lambda d: -d['area'])
    if verbose:
        print('阈值=%d  最小面积=%d  饱和剔除线=%d  亮斑数=%d'
              % (thresh, min_area, sat_cut, len(rows)))
        if not rows:
            print('  画面里没有亮斑：检查灯有没有亮、曝光是不是太低')
        print('%-4s %-8s %-6s %-6s %-7s %-9s %-9s %s'
              % ('#', '面积', '中心', '均亮', '饱和%', '含白核判', '去白核判', '置信'))
        for i, d in enumerate(rows, 1):
            print('%-4d %-8.0f (%3d,%3d) %-6.0f %-6.0f %-9s %-9s %.2f'
                  % (i, d['area'], d['x'] + d['w'] // 2, d['y'] + d['h'] // 2,
                     d['mean_v'], d['sat'], d['state_all'], d['state'], d['conf']))
    return rows


def annotate(img, rows, path):
    vis = img.copy()
    for i, d in enumerate(rows, 1):
        color = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'YELLOW': (0, 255, 255),
                 'BLUE': (255, 0, 0), 'DIM': (128, 128, 128)}.get(d['state'], (255, 255, 255))
        cv2.rectangle(vis, (d['x'], d['y']), (d['x'] + d['w'], d['y'] + d['h']), color, 1)
        cv2.putText(vis, '%d %s %.2f' % (i, d['state'], d['conf']),
                    (d['x'], max(12, d['y'] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    os.makedirs(OUT_DIR, exist_ok=True)
    cv2.imwrite(path, vis)
    print('标注图已保存: %s' % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', default='')
    ap.add_argument('--url', default=URL)
    ap.add_argument('--thresh', type=int, default=200)
    ap.add_argument('--min-area', type=float, default=20.0)
    ap.add_argument('--sat-cut', type=int, default=250)
    ap.add_argument('--sweep', action='store_true', help='对比 180/200/220/240 四档阈值')
    ap.add_argument('--save', action='store_true', default=True)
    args = ap.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print('读不到图片: %s' % args.image)
            sys.exit(1)
        tag = os.path.splitext(os.path.basename(args.image))[0]
    else:
        print('从摄像头取图: %s' % args.url)
        img = grab(args.url)
        if img is None:
            print('取图失败，检查摄像头 IP 或 /capture 是否可用')
            sys.exit(1)
        tag = time.strftime('%H%M%S')

    print('图像尺寸: %dx%d' % (img.shape[1], img.shape[0]))
    if args.sweep:
        for t in (180, 200, 220, 240):
            analyse(img, t, args.min_area, args.sat_cut)
            print('')
        rows = analyse(img, args.thresh, args.min_area, args.sat_cut, verbose=False)
    else:
        rows = analyse(img, args.thresh, args.min_area, args.sat_cut)

    annotate(img, rows, os.path.join(OUT_DIR, 'led_%s.png' % tag))


if __name__ == '__main__':
    main()
