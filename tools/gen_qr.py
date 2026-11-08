#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 A/B/C 三个二维码，供机柜面板张贴。"""
import os

import qrcode
from qrcode.constants import ERROR_CORRECT_M

OUT = os.path.expanduser('~/narsu/ros2/tools/qrcodes')
os.makedirs(OUT, exist_ok=True)

for name in ('A', 'B', 'C'):
    qr = qrcode.QRCode(version=1, error_correction=ERROR_CORRECT_M, box_size=20, border=4)
    qr.add_data(name)
    qr.make(fit=True)
    img = qr.make_image(fill_color='black', back_color='white')
    path = os.path.join(OUT, '%s.png' % name)
    img.save(path)
    print('%s -> %s  %dx%d' % (name, path, img.size[0], img.size[1]))
