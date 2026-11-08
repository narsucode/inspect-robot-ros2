#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 A/B/C 机柜靶标：ArUco（推荐）+ 二维码（备用）。

ArUco 用 DICT_4X4_50，id 0/1/2 对应 A/B/C。
输出：~/narsu/ros2/tools/markers/{aruco,qr}/<名字>.png
"""
import os

import cv2
import numpy as np
import qrcode
from qrcode.constants import ERROR_CORRECT_M

OUT = os.path.expanduser('~/narsu/ros2/tools/markers')
NAMES = ('A', 'B', 'C')

# ---------- ArUco ----------
ad = cv2.aruco
if hasattr(ad, 'Dictionary_get'):
    dictionary = ad.Dictionary_get(ad.DICT_4X4_50)
else:  # OpenCV >= 4.7
    dictionary = ad.getPredefinedDictionary(ad.DICT_4X4_50)

os.makedirs(os.path.join(OUT, 'aruco'), exist_ok=True)
for idx, name in enumerate(NAMES):
    if hasattr(ad, 'drawMarker'):
        img = ad.drawMarker(dictionary, idx, 600, 1)
    else:  # OpenCV >= 4.7
        img = ad.generateImageMarker(dictionary, idx, 600)
    # 加白边（quiet zone），否则贴到深色机柜上检测不到
    img = cv2.copyMakeBorder(img, 90, 90, 90, 90, cv2.BORDER_CONSTANT, value=255)
    path = os.path.join(OUT, 'aruco', '%s.png' % name)
    cv2.imwrite(path, img)
    print('aruco %s -> %s  %dx%d  (id=%d)' % (name, path, img.shape[1], img.shape[0], idx))

# ---------- QR（备用） ----------
os.makedirs(os.path.join(OUT, 'qr'), exist_ok=True)
for name in NAMES:
    qr = qrcode.QRCode(version=1, error_correction=ERROR_CORRECT_M, box_size=20, border=4)
    qr.add_data(name)
    qr.make(fit=True)
    img = qr.make_image(fill_color='black', back_color='white')
    path = os.path.join(OUT, 'qr', '%s.png' % name)
    img.save(path)
    print('qr    %s -> %s' % (name, path))
