#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检可视化界面（PyQt5）

布局：
  顶部：标题 + 统计 + [开始巡检] [停止] [导出报告]
  中间：叠加显示勾选框
  左侧：左边摄像头画面，右边地图（A/B/C/D + 机器人 + 点云/路径/代价地图）
  右侧：巡检顺序（勾选 + ▲▼）/ 进度 / 当前点 / 判定 / 位置 / 提示
  底部：结果表格（点位 / 结果 / 判定 / 期望 / 截图 / 时间）

地图叠加（对应 RViz 的 Display）：
  /scan                       红点，激光点云（经 TF 转到 map）
  /plan                       蓝色线，全局路径
  /local_plan                 绿色线，局部轨迹
  /global_costmap/costmap     红色半透明，全局代价地图
  /local_costmap/costmap      橙色半透明，局部代价地图（随车滚动）

地图底图来源：优先 /map（导航在用），没有就读 room.yaml + room.pgm 兜底。
"""

import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, QThread
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog,
                             QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
                             QMainWindow, QPushButton, QSplitter, QTableWidget, QTextEdit,
                             QTableWidgetItem, QVBoxLayout, QWidget)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

COLOR = {'RED': '#ff4d4f', 'YELLOW': '#faad14', 'GREEN': '#52c41a',
         'OFF': '#8c8c8c', 'UNKNOWN': '#8c8c8c', 'MULTI': '#722ed1',
         'NAV_FAILED': '#ff4d4f', 'ARRIVED': '#1677ff', 'CAPTURED': '#1677ff'}

# 语义等级配色：正常绿、警告黄、故障红、异常灰
LEVEL_COLOR = {'正常': '#52c41a', '警告': '#faad14',
               '故障': '#ff4d4f', '异常': '#8c8c8c', '未知': '#8c8c8c'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosNode(Node):
    def __init__(self):
        super().__init__('patrol_gui')
        self.bridge = CvBridge()
        self.frame = None
        self.status = {}
        self.logs = []
        self.odom = None
        self.pose = None
        self.map_msg = None
        self.scan = None
        self.plan = None
        self.local_plan = None
        self.gcost = None
        self.lcost = None
        self.state = 'OFF'

        self.create_subscription(Image, '/vision/debug_image', self._on_img, 10)
        self.create_subscription(Image, '/vision/aim_image', self._on_aim_img, 10)
        self.aim_frame = None
        self.aim_t = 0.0
        self.create_subscription(String, '/patrol_status', self._on_status, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(String, '/vision/light_state', self._on_light, 10)

        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10)

        # ---- RViz 那几层 ----
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(Path, '/local_plan', self._on_lplan, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_gcost, map_qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_lcost, map_qos)

        # 点云要转到 map 坐标系
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(String, '/patrol_command', 10)

    def send(self, cmd):
        self.cmd_pub.publish(String(data=cmd))

    def _on_img(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _on_aim_img(self, msg):
        try:
            self.aim_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.aim_t = time.time()
        except Exception:
            pass

    def _log(self, text):
        line = '%s  %s' % (time.strftime('%H:%M:%S'), text)
        try:
            self.log_view.append(line)
            with open(self.log_file, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
        except Exception:
            pass

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            self.status = {'message': msg.data}
        st = self.status or {}
        parts = [st.get('phase', ''), str(st.get('index', '')), st.get('point', ''),
                 st.get('state', ''), st.get('message', '')]
        if st.get('level'):
            parts.append('level=%s' % st['level'])
        line = ' | '.join([p for p in parts if p])
        if line:
            self.logs.append(line)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_light(self, msg):
        try:
            self.state = json.loads(msg.data).get('state', 'OFF')
        except Exception:
            pass

    def _on_scan(self, msg):
        self.scan = msg

    def _on_plan(self, msg):
        self.plan = msg

    def _on_lplan(self, msg):
        self.local_plan = msg

    def _on_gcost(self, msg):
        self.gcost = msg

    def _on_lcost(self, msg):
        self.lcost = msg


class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('机房设备状态智能巡检')
        self.resize(1420, 820)

        self.run_start = None
        self.run_end = None
        self.run_done = False
        self.seen = set()
        self.waypoints = {}
        self.current_point = None

        self._map_key = None
        self._map_img = None
        self._map_meta = None      # (w, h, res, ox, oy)
        self._map_src = ''         # 当前底图来源，显示在地图左下角
        self._map_scaled = None
        self._map_scaled_size = (0, 0)
        self._file_map_tried = False

        self._gcost_key = None
        self._gcost_img = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ---------------------- 顶部 ----------------------
        top = QHBoxLayout()
        title = QLabel('机房设备状态智能巡检')
        title.setFont(QFont('Microsoft YaHei', 20, QFont.Bold))
        top.addWidget(title)

        self.lb_stat = QLabel('进度 -/-　正常 0　异常 0　用时 00:00')
        self.lb_stat.setFont(QFont('Microsoft YaHei', 14, QFont.Bold))
        self.lb_stat.setStyleSheet('color:#555; padding-left:16px')
        top.addWidget(self.lb_stat)
        top.addStretch()

        self.btn_start = QPushButton('开始巡检')
        self.btn_stop = QPushButton('停止')
        self.btn_export = QPushButton('导出报告')
        for b in (self.btn_start, self.btn_stop, self.btn_export):
            b.setMinimumHeight(36)
            b.setMinimumWidth(110)
            b.setFont(QFont('Microsoft YaHei', 12, QFont.Bold))
            top.addWidget(b)
        root.addLayout(top)

        # ---------------------- 叠加显示开关 ----------------------
        ov = QHBoxLayout()
        lb_ov = QLabel('叠加')
        lb_ov.setFont(QFont('Microsoft YaHei', 11))
        lb_ov.setStyleSheet('color:#555')
        ov.addWidget(lb_ov)

        self.cb_scan = QCheckBox('点云')
        self.cb_plan = QCheckBox('全局路径')
        self.cb_lplan = QCheckBox('局部轨迹')
        self.cb_cost = QCheckBox('代价地图')
        for cb, on in ((self.cb_scan, True), (self.cb_plan, True),
                       (self.cb_lplan, True), (self.cb_cost, False)):
            cb.setChecked(on)
            cb.setFont(QFont('Microsoft YaHei', 11))
            ov.addWidget(cb)

        ov.addSpacing(10)
        lb_src = QLabel('源')
        lb_src.setFont(QFont('Microsoft YaHei', 11))
        lb_src.setStyleSheet('color:#555')
        ov.addWidget(lb_src)

        self.cb_src = QComboBox()
        self.cb_src.setFont(QFont('Microsoft YaHei', 11))
        self.cb_src.setMaximumWidth(120)
        self.cb_src.addItem('自动', 'auto')
        self.cb_src.addItem('仅 /map', 'ros')
        self.cb_src.addItem('本地文件', 'file')
        self.cb_src.currentIndexChanged.connect(self._set_map_mode)
        self.cb_src.setCurrentIndex(2)   # default: load local map file
        ov.addWidget(self.cb_src)
        ov.addStretch()
        # 这一排不放在顶部，稍后挂到地图上方
        self.ov_row = ov

        # ---------------------- 中间 ----------------------
        mid = QHBoxLayout()

        self.video = QLabel('等待画面…')
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setStyleSheet('background:#111;color:#888;border:1px solid #333')
        self.video.setMinimumSize(460, 300)

        self.map_label = QLabel('等待地图…')
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet('background:#303030;color:#bbb;border:1px solid #333')
        self.map_label.setMinimumSize(460, 300)

        self.split = QSplitter(Qt.Horizontal)
        self.split.addWidget(self.video)

        # 地图上方单独一栏：叠加开关 + 地图源
        map_side = QWidget()
        mv = QVBoxLayout(map_side)
        mv.setContentsMargins(0, 0, 0, 0)
        mv.setSpacing(2)
        mv.addLayout(self.ov_row)
        mv.addWidget(self.map_label, 1)
        self.split.addWidget(map_side)
        self.split.setSizes([520, 520])
        self.split.setChildrenCollapsible(False)
        mid.addWidget(self.split, 3)

        panel = QVBoxLayout()

        lb_ord = QLabel('巡检顺序（勾选 + ▲▼ 调整）')
        lb_ord.setFont(QFont('Microsoft YaHei', 12))
        lb_ord.setStyleSheet('color:#555')
        panel.addWidget(lb_ord)

        self.order_list = QListWidget()
        self.order_list.setFont(QFont('Microsoft YaHei', 12))
        self.order_list.setFixedHeight(130)
        self.order_list.setStyleSheet('border:1px solid #333')
        panel.addWidget(self.order_list)

        row_btns = QHBoxLayout()
        self.btn_up = QPushButton('▲ 上移')
        self.btn_down = QPushButton('▼ 下移')
        for b in (self.btn_up, self.btn_down):
            b.setMinimumHeight(26)
            row_btns.addWidget(b)
        panel.addLayout(row_btns)
        self.btn_up.clicked.connect(lambda: self._move(-1))
        self.btn_down.clicked.connect(lambda: self._move(1))

        self.lb_prog = QLabel('进度: - / -')
        self.lb_point = QLabel('当前点: -')
        self.lb_state = QLabel('判定: -')
        self.lb_odom = QLabel('位置: -')
        self.lb_msg = QLabel('就绪')
        for lb in (self.lb_prog, self.lb_point, self.lb_state, self.lb_odom, self.lb_msg):
            lb.setFont(QFont('Microsoft YaHei', 14))
            lb.setMinimumHeight(34)
            lb.setStyleSheet('padding:6px 8px;border-bottom:1px solid #333')
            panel.addWidget(lb)
        panel.addStretch()

        wrap = QWidget()
        wrap.setLayout(panel)
        wrap.setFixedWidth(310)
        mid.addWidget(wrap)
        root.addLayout(mid, 5)

        # ---------------------- 底部表格 ----------------------
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(['点位', '结果', '判定', '期望', '截图', '时间'])
        self.table.horizontalHeader().setStretchLastSection(True)
        for i, w in enumerate((80, 110, 110, 110, 300, 100)):
            self.table.setColumnWidth(i, w)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setFont(QFont('Microsoft YaHei', 12))
        self.table.horizontalHeader().setFont(QFont('Microsoft YaHei', 12, QFont.Bold))
        self.table.verticalHeader().setDefaultSectionSize(30)
        self.table.cellDoubleClicked.connect(self.open_shot)
        root.addWidget(self.table, 2)

        # ---------- ???? ----------
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(150)
        self.log_view.setStyleSheet(
            'background:#111;color:#ccc;font-family:Consolas;font-size:11pt')
        root.addWidget(self.log_view, 1)
        self.log_file = os.path.expanduser('~/patrol_reports/gui_%s.log'
                                           % time.strftime('%Y%m%d_%H%M%S'))
        try:
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        except Exception:
            pass

        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(lambda: self.node.send('stop'))
        self.btn_export.clicked.connect(self.export)

        self._load_cfg()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(80)

    # ---------- 配置 ----------
    def _load_cfg(self):
        try:
            cfg_dir = os.path.join(get_package_share_directory('roscar_patrol'), 'config')
            with open(os.path.join(cfg_dir, 'patrol.yaml'), 'r', encoding='utf-8') as f:
                order = (yaml.safe_load(f) or {}).get('order') or []
            with open(os.path.join(cfg_dir, 'waypoints.yaml'), 'r', encoding='utf-8') as f:
                data = (yaml.safe_load(f) or {}).get('waypoints') or {}
            self.waypoints = {k: (float(v['x']), float(v['y']), float(v.get('yaw', 0.0)))
                              for k, v in data.items()}
            names = list(self.waypoints.keys())

            seq = [n for n in order if n in names] + [n for n in names if n not in order]
            self.order_list.clear()
            for idx, n in enumerate(seq, 1):
                it = QListWidgetItem('%d. %s' % (idx, n))
                it.setData(Qt.UserRole, str(n))
                it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
                it.setCheckState(Qt.Checked if n in order else Qt.Unchecked)
                self.order_list.addItem(it)
        except Exception:
            pass

    def _renumber(self):
        for i in range(self.order_list.count()):
            it = self.order_list.item(i)
            it.setText('%d. %s' % (i + 1, it.data(Qt.UserRole)))

    def _move(self, delta):
        row = self.order_list.currentRow()
        if row < 0:
            return
        new = row + delta
        if new < 0 or new >= self.order_list.count():
            return
        item = self.order_list.takeItem(row)
        self.order_list.insertItem(new, item)
        self.order_list.setCurrentRow(new)
        self._renumber()

    # ---------- 按钮 ----------
    def on_start(self):
        order = []
        for i in range(self.order_list.count()):
            it = self.order_list.item(i)
            if it.checkState() == Qt.Checked:
                order.append(it.data(Qt.UserRole))
        if not order:
            self.lb_msg.setText('请至少勾选一个点位')
            return

        self.table.setRowCount(0)
        self.seen.clear()
        self.run_start = time.time()
        self.run_end = None
        self.run_done = False
        self.lb_msg.setText('开始巡检…')
        self.node.send('start %s' % ','.join(order))

    # ---------- 刷新 ----------
    def _append_log(self):
        try:
            while self.node.logs:
                line = self.node.logs.pop(0)
                stamp = time.strftime('%H:%M:%S')
                self.log_view.append('%s  %s' % (stamp, line))
                with open(self.log_file, 'a', encoding='utf-8') as f:
                    f.write('%s  %s\n' % (stamp, line))
        except Exception:
            pass

    def refresh(self):
        self._append_log()
        # ????????? ArUco ?????1.2 ??????
        # ???????????????????
        # ?????????? /vision/debug_image ??????
        img = self.node.frame
        if img is not None:
            h, w, _ = img.shape
            qimg = QImage(img.data, w, h, w * 3, QImage.Format_BGR888)
            self.video.setPixmap(QPixmap.fromImage(qimg).scaled(
                self.video.width(), self.video.height(), Qt.KeepAspectRatio))

        try:
            self._render_map()
        except Exception as exc:
            # 叠加层出问题也不能把整个界面拖垮，只报一次
            if not getattr(self, '_map_err', False):
                self._map_err = True
                print('地图渲染出错: %s' % exc, file=sys.stderr)

        st = self.node.status
        idx, total = st.get('index', 0), st.get('total', 0)
        self.current_point = st.get('point') or self.current_point

        self.lb_prog.setText('进度: %s / %s' % (idx if idx else '-', total or '-'))
        self.lb_point.setText('当前点: %s' % (st.get('point') or '-'))

        state = st.get('state') or self.node.state
        level = str(st.get('level') or '')
        # 判定行：前面一个大圆点，颜色跟等级走，不用看字也能一眼分辨
        lc = LEVEL_COLOR.get(level, COLOR.get(state, '#8c8c8c'))
        self.lb_state.setText(
            '<span style="color:%s;font-size:22px">\u25cf</span> '
            '<span style="color:#888">判定:</span> '
            '<span style="color:%s;font-weight:bold">%s</span>'
            % (lc, lc, (level or state or '-')))
        self.lb_state.setStyleSheet('padding:6px 8px;border-bottom:1px solid #333')

        if self.node.odom:
            self.lb_odom.setText('位置: (%.2f, %.2f)' % self.node.odom)
        self.lb_msg.setText(st.get('message', ''))

        if st.get('phase') == 'inspect' and 'ok' in st:
            key = (st.get('index'), st.get('point'))
            if key not in self.seen:
                self.seen.add(key)
                self._add_row(st)

        if st.get('phase') == 'done' and not self.run_done:
            self.run_done = True
            self.run_end = time.time()

        self._update_stats(idx, total)

    # ---------- 地图 ----------
    def _load_file_map(self):
        self._file_map_tried = True
        try:
            yaml_path = None
            names = ('room_20261106.yaml', 'room.yaml')
            for pkg in ('roscar_navigation', 'youdaobot_navigation'):
                try:
                    share = get_package_share_directory(pkg)
                except Exception:
                    continue
                for nm in names:
                    cand = os.path.join(share, 'maps', nm)
                    if os.path.exists(cand):
                        yaml_path = cand
                        break
                if yaml_path:
                    break
            if yaml_path is None:
                return
            with open(yaml_path, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f) or {}
            img_path = meta.get('image', 'room.pgm')
            if not os.path.isabs(img_path):
                img_path = os.path.join(os.path.dirname(yaml_path), img_path)
            g = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if g is None:
                return
            h, w = g.shape
            res = float(meta.get('resolution', 0.05))
            ox, oy = (meta.get('origin') or [0.0, 0.0, 0.0])[:2]

            arr = np.zeros((h, w, 3), np.uint8)
            arr[g > 191] = (245, 245, 245)
            arr[g < 89] = (33, 33, 33)
            mid = (g >= 89) & (g <= 191)
            arr[mid] = (158, 158, 158)

            self._map_img = QImage(arr.data, w, h, w * 3, QImage.Format_BGR888).copy()
            self._map_meta = (w, h, res, float(ox), float(oy))
            self._map_key = ('file', img_path)
            self._map_src = '本地文件  %s' % os.path.basename(img_path)
            self._map_scaled = None
        except Exception:
            self._map_img = None

    def _set_map_mode(self, _idx=None):
        """切换地图来源时清掉缓存，下一帧重建。"""
        self._map_key = None
        self._map_img = None
        self._map_scaled = None
        self._map_scaled_size = (0, 0)
        self._file_map_tried = False

    def _ensure_map(self):
        """根据“地图源”下拉框决定用 /map 还是本地文件。"""
        mode = self.cb_src.currentData()
        msg = self.node.map_msg

        if mode in ('auto', 'ros') and msg is not None:
            key = ('ros', id(msg))
            if self._map_key != key:
                w, h = msg.info.width, msg.info.height
                # OccupancyGrid 的 data 是行主序，**第 0 行对应 y 最小（图的下方）**；
                # 而 QImage 第 0 行是顶部。所以这里必须上下翻转，否则地图会镜像。
                data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
                arr = np.full((h, w, 3), 245, np.uint8)      # 空闲=白
                arr[data < 0] = (158, 158, 158)              # 未知=灰
                arr[data >= 50] = (33, 33, 33)               # 障碍=黑
                arr = arr[::-1].copy()
                self._map_img = QImage(arr.data, w, h, w * 3, QImage.Format_BGR888).copy()
                self._map_meta = (w, h, msg.info.resolution,
                                  msg.info.origin.position.x, msg.info.origin.position.y)
                self._map_key = key
                self._map_scaled = None
                self._map_src = '/map  %dx%d' % (w, h)
            return True

        if mode == 'ros':
            return False

        if not self._file_map_tried:
            self._load_file_map()
        return self._map_img is not None

    def _render_map(self):
        if self.map_label.width() < 50:
            return
        if not self._ensure_map():
            return

        w, h, res, ox, oy = self._map_meta
        lw, lh = self.map_label.width(), self.map_label.height()
        scale = min(lw / w, lh / h)
        dw, dh = max(1, int(w * scale)), max(1, int(h * scale))
        if self._map_scaled is None or self._map_scaled_size != (dw, dh):
            self._map_scaled = self._map_img.scaled(dw, dh, Qt.IgnoreAspectRatio,
                                                    Qt.FastTransformation)
            self._map_scaled_size = (dw, dh)
        left, top = (lw - dw) // 2, (lh - dh) // 2

        canvas = QImage(lw, lh, QImage.Format_RGB32)
        canvas.fill(0x303030)
        p = QPainter(canvas)
        p.drawImage(QPointF(left, top), self._map_scaled)

        def to_screen(wx, wy):
            fx = (wx - ox) / res / w
            fy = 1.0 - (wy - oy) / res / h
            return QPointF(left + fx * dw, top + fy * dh)

        # ---- 代价地图（先画，压在最底层）----
        if self.cb_cost.isChecked():
            self._draw_costmaps(p, to_screen)

        # ---- 全局路径 ----
        if self.cb_plan.isChecked() and self.node.plan is not None:
            self._draw_path(p, self.node.plan, to_screen, QColor('#2196f3'), 2)

        # ---- 局部轨迹 ----
        if self.cb_lplan.isChecked() and self.node.local_plan is not None:
            self._draw_path(p, self.node.local_plan, to_screen, QColor('#00e676'), 2)

        # ---- 点云 ----
        if self.cb_scan.isChecked():
            self._draw_scan(p, to_screen)

        # ---- 点位 ----
        for name, (wx, wy, wyaw) in self.waypoints.items():
            pt = to_screen(wx, wy)
            cur = (name == self.current_point)
            r = 7 if cur else 4

            # 朝向箭头：长度 14 像素，屏幕角度 = -yaw
            a = -wyaw
            p.setPen(QPen(QColor('#9ad0ff'), 2))
            p.drawLine(pt, QPointF(pt.x() + 14 * math.cos(a),
                                 pt.y() + 14 * math.sin(a)))

            p.setPen(QPen(QColor('#ffffff'), 1))
            p.setBrush(QColor('#ff4d4f') if cur else QColor('#1677ff'))
            p.drawEllipse(pt, r, r)
            # 字母用黑色 + 加大字号，浅色地图上更清楚
            p.setPen(QPen(QColor('#000000'), 1))
            p.setFont(QFont('Microsoft YaHei', 13, QFont.Bold))
            p.drawText(QPointF(pt.x() + r + 3, pt.y() - r - 2), name)

        # ---- 机器人 ----
        if self.node.pose:
            rx, ry, ryaw = self.node.pose
            c = to_screen(rx, ry)
            L = 12
            a = -ryaw
            tip = QPointF(c.x() + L * math.cos(a), c.y() + L * math.sin(a))
            lef = QPointF(c.x() + 0.6 * L * math.cos(a + 2.5), c.y() + 0.6 * L * math.sin(a + 2.5))
            rig = QPointF(c.x() + 0.6 * L * math.cos(a - 2.5), c.y() + 0.6 * L * math.sin(a - 2.5))
            p.setPen(QPen(QColor('#00e676'), 1))
            p.setBrush(QColor('#00e676'))
            p.drawPolygon(QPolygonF([tip, lef, rig]))

        p.setPen(QPen(QColor('#ffffff'), 1))
        p.setFont(QFont('Microsoft YaHei', 9))
        p.drawText(8, 18, '蓝点=点位  红点=当前目标  绿箭头=机器人')
        if self._map_src:
            p.setPen(QPen(QColor('#bbbbbb'), 1))
            p.drawText(8, lh - 10, '源: ' + self._map_src)
        p.end()

        self.map_label.setPixmap(QPixmap.fromImage(canvas))

    def _draw_path(self, p, msg, to_screen, color, width):
        if not msg.poses:
            return
        poly = QPolygonF()
        for ps in msg.poses:
            poly.append(to_screen(ps.pose.position.x, ps.pose.position.y))
        p.setPen(QPen(color, width))
        p.setBrush(Qt.NoBrush)
        p.drawPolyline(poly)

    def _draw_scan(self, p, to_screen):
        scan = self.node.scan
        if scan is None:
            return
        try:
            tf = self.node.tf_buffer.lookup_transform(
                'map', scan.header.frame_id, Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tf = self.node.tf_buffer.lookup_transform(
                    'map', scan.header.frame_id, Time(), timeout=Duration(seconds=0.05))
            except Exception:
                return

        t = tf.transform.translation
        yaw = yaw_of(tf.transform.rotation)
        cy, sy = math.cos(yaw), math.sin(yaw)

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges >= max(scan.range_min, 0.05)) & \
            (ranges <= min(scan.range_max, 6.0))
        ranges, angles = ranges[ok], angles[ok]
        if len(ranges) > 720:
            step = max(1, len(ranges) // 720)
            ranges, angles = ranges[::step], angles[::step]

        p.setPen(QPen(QColor(255, 70, 70), 2))
        for r, a in zip(ranges, angles):
            lx, ly = r * math.cos(a), r * math.sin(a)
            wx = t.x + lx * cy - ly * sy
            wy = t.y + lx * sy + ly * cy
            pt = to_screen(wx, wy)
            p.drawPoint(pt)

    def _draw_costmaps(self, p, to_screen):
        # 全局代价地图：换成一张图缓存，避免每帧遍历几万格
        g = self.node.gcost
        if g is not None:
            key = id(g)
            if self._gcost_key != key:
                w, h = g.info.width, g.info.height
                img = QImage(w, h, QImage.Format_ARGB32)
                img.fill(0)
                data = g.data
                for row in range(h):
                    base = row * w
                    for col in range(w):
                        v = data[base + col]
                        if v >= 50:
                            # 同样要翻转行号：数据第 0 行在图的下方
                            img.setPixel(col, h - 1 - row, 0xB4FF4040)     # 障碍：红
                        elif v > 0:
                            img.setPixel(col, h - 1 - row, 0x3CFFA040)     # 膨胀：橙
                self._gcost_img = img
                self._gcost_key = key
            if self._gcost_img is not None:
                res = g.info.resolution
                ox = g.info.origin.position.x
                oy = g.info.origin.position.y
                p.setOpacity(0.55)
                p.drawImage(QRectF(to_screen(ox, oy + g.info.height * res),
                                   to_screen(ox + g.info.width * res, oy)),
                            self._gcost_img)
                p.setOpacity(1.0)

        # 局部代价地图：尺寸小（3x3m），逐格画，并转到 map 坐标系
        l = self.node.lcost
        if l is not None:
            try:
                tf = self.node.tf_buffer.lookup_transform(
                    'map', l.header.frame_id, Time(), timeout=Duration(seconds=0.05))
                t = tf.transform.translation
                yaw = yaw_of(tf.transform.rotation)
                cy, sy = math.cos(yaw), math.sin(yaw)
            except Exception:
                cy = sy = None
                t = None

            res = l.info.resolution
            ox = l.info.origin.position.x
            oy = l.info.origin.position.y
            w, h = l.info.width, l.info.height
            data = l.data
            p.setPen(Qt.NoPen)
            drawn = 0
            for row in range(h):
                base = row * w
                for col in range(w):
                    v = data[base + col]
                    if v <= 0:
                        continue
                    cx = ox + (col + 0.5) * res
                    cy_w = oy + (row + 0.5) * res
                    if cy is not None:
                        wx = t.x + cx * cy - cy_w * sy
                        wy = t.y + cx * sy + cy_w * cy
                    else:
                        wx, wy = cx, cy_w
                    pt = to_screen(wx, wy)
                    p.setBrush(QColor(255, 160, 64, 90) if v < 50 else QColor(255, 64, 64, 140))
                    p.drawRect(QRectF(pt.x() - 1.5, pt.y() - 1.5, 3, 3))
                    drawn += 1
                    if drawn > 2500:
                        return

    # ---------- 统计 / 表格 ----------
    def _update_stats(self, idx, total):
        rows = self.table.rowCount()
        ok = bad = 0
        for r in range(rows):
            item = self.table.item(r, 2)
            if item is not None:
                if item.text() == '正常':
                    ok += 1
                elif item.text() == '异常':
                    bad += 1
        if self.run_start is None:
            used = '00:00'
        else:
            end = self.run_end or time.time()
            sec = max(0, int(end - self.run_start))
            used = '%02d:%02d' % (sec // 60, sec % 60)
        self.lb_stat.setText('进度 %s/%s　正常 %d　异常 %d　用时 %s'
                             % (idx or rows, total or rows, ok, bad, used))

    def _add_row(self, st):
        r = self.table.rowCount()
        self.table.insertRow(r)

        point = str(st.get('point', '-'))
        state = str(st.get('state', '-'))
        expect = str(st.get('expect') or '-')
        ok = bool(st.get('ok', False))
        level = str(st.get('level') or '')
        image = st.get('image') or ''

        self.table.setItem(r, 0, QTableWidgetItem(point))

        it_state = QTableWidgetItem(state)
        it_state.setForeground(QColor(COLOR.get(state, '#dddddd')))
        it_state.setFont(QFont('Microsoft YaHei', 12, QFont.Bold))
        self.table.setItem(r, 1, it_state)

        it_ok = QTableWidgetItem('\u25cf ' + (level or ('正常' if ok else '异常')))
        it_ok.setForeground(QColor(LEVEL_COLOR.get(level, '#52c41a' if ok else '#ff4d4f')))
        it_ok.setFont(QFont('Microsoft YaHei', 13, QFont.Bold))
        self.table.setItem(r, 2, it_ok)

        self.table.setItem(r, 3, QTableWidgetItem(expect))

        shot = QTableWidgetItem(os.path.basename(image) if image else '-')
        if image:
            shot.setData(Qt.UserRole, image)
            shot.setForeground(QColor('#1677ff'))
        self.table.setItem(r, 4, shot)

        self.table.setItem(r, 5, QTableWidgetItem(
            st.get('time') or time.strftime('%H:%M:%S')))

        self.table.scrollToBottom()

    def open_shot(self, row, _col):
        item = self.table.item(row, 4)
        if item is None:
            return
        path = item.data(Qt.UserRole)
        if path and os.path.exists(path):
            subprocess.Popen(['xdg-open', path])
        else:
            self.lb_msg.setText('该点位没有截图')

    def export(self):
        d = os.path.expanduser('~/patrol_reports')
        os.makedirs(d, exist_ok=True)
        default = os.path.join(d, 'patrol_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        path, _ = QFileDialog.getSaveFileName(self, '另存报告', default, 'CSV (*.csv)')
        if not path:
            return
        with open(path, 'w', encoding='utf-8-sig') as f:
            f.write('点位,结果,判定,期望,截图,时间\n')
            for r in range(self.table.rowCount()):
                vals = []
                for c in range(6):
                    item = self.table.item(r, c)
                    if c == 4 and item is not None:
                        vals.append(item.data(Qt.UserRole) or '')
                    else:
                        vals.append(item.text() if item else '')
                f.write(','.join(vals) + '\n')
        self.lb_msg.setText('已导出: %s' % path)


def main():
    rclpy.init()
    node = RosNode()
    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()
    th = RosThread(node)
    th.start()
    try:
        app.exec_()
    finally:
        rclpy.shutdown()
        th.wait(1000)


if __name__ == '__main__':
    main()
