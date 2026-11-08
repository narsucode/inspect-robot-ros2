#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aim_marker —— 云台主动扫视 + 靶标对准（带预览窗口）。

为什么要有这个节点：
    巡检点位的朝向、机柜的摆放、车每次停车的位姿都会有几度偏差，
    靠"预先标一个固定角度"很脆。这里改成主动搜索：
    云台按栅格扫一圈，看到靶标就把它移到画面指定位置，然后停住。

靶标支持两种（参数 code_type）：
    aruco  机柜专用码，检测快、小尺寸也能认，推荐
    qr     二维码，肉眼可读、能写文字，备用
    both   两种都试，aruco 优先

协议（和 patrol_node 配对）：
    /aim_request (std_msgs/String)  请求值 = 点位名，如 "A"
    /aim_result  (geometry_msgs/Vector3)  x=pan  y=tilt  z=1 成功 / 0 失败
    /aim_text    (std_msgs/String)  人看的说明：靶标内容 / 失败原因

⚠ 关于画面延迟：
    MJPEG 流在 OpenCV 里带缓冲，云台刚转完直接读帧，读到的还是旧画面。
    如果拿旧画面算"还差多少度"，控制就会一直往一个方向推，最后顶到限位。
    所以这里每次都先空读 drain 帧丢弃缓冲，再取用于检测的那一帧。
"""
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

CFG = os.path.expanduser('~/narsu/ros2/patrol_ws/src/roscar_patrol/config')
PT = os.path.join(CFG, 'patrol.yaml')

HEADER = """# 巡检点位配置
#
# order: 巡检顺序，必须是 waypoints.yaml 里存在的点位名
# points: 每个点位的云台预设角度（度）和期望状态（可选，用于自动核对）
#         pan  水平角度，正值向左（2026-11-02 实测：pan 减小 = 向右）
#         tilt 俯仰角度，正值向上（tilt 减小 = 向下）
#         expect 期望识别到的状态：RED / YELLOW / GREEN，不写就只记录不核对
"""


class AimMarker(Node):
    def __init__(self):
        super().__init__('aim_marker')
        p = self.declare_parameter

        # 摄像头流地址（仅在没人在发 /vision/image_raw 时才直连）
        p('source', 'http://10.189.154.251:81/stream')

        # 图像来源：默认订阅识别节点转发的原始图。
        # ESP32-CAM 的 MJPEG 服务只容一个客户端，两个节点各自拉流会互相阻塞。
        p('image_topic', '/vision/image_raw')
        # ESP32-CAM ??????????????? VideoCapture ??????
        # ??????????????????????
        p('allow_direct_stream', False)
        p('debug_dump', False)

        # 云台话题名：patrol_node 用的是 /cmd_ptz_angle（x=pan 度, y=tilt 度，绝对值）
        p('ptz_topic', '/cmd_ptz_angle')

        # 靶标类型：aruco / qr / both
        p('code_type', 'aruco')
        # ArUco 字典名与 id→点位名 的映射（生成脚本用的 DICT_4X4_50，A=0 B=1 C=2）
        p('aruco_dict', 'DICT_4X4_50')
        p('aruco_map', 'A:0,B:1,C:2')

        # 扫视范围。角度是云台指令值（度），不是画面角度。
        p('pan_min', -40.0)
        p('pan_max', 40.0)
        p('pan_step', 15.0)
        p('tilt_min', -12.0)
        p('tilt_max', 8.0)
        p('tilt_step', 6.0)

        # 每格停留时间（秒）：aruco 检测快，可以缩短。
        p('settle', 0.35)

        # 对准时希望靶标落在画面上的归一化位置。
        # u=0.5 正中；v 略小于 0.5，让靶标下方的指示灯落进画面中间。
        p('target_u', 0.5)
        p('target_v', 0.38)

        # 估计视场角，用于把"画面偏差"换算成"云台角度"。
        p('fov_h', 65.0)
        p('fov_v', 45.0)

        # 微调：最多几轮、单轮增益、容差（归一化）。
        p('max_refine', 4)
        p('refine_gain', 0.5)
        p('tolerate', 0.07)

        # 硬件方向（2026-11-02 实测）：pan 减小=向右，tilt 减小=向下。
        p('pan_sign', -1.0)
        p('tilt_sign', -1.0)

        # 云台渐变：每次最多走多少度、每步停多久（舵机直接跳大角度会猛冲）
        p('ramp_deg', 6.0)
        p('ramp_ms', 50.0)

        # 读帧前空读多少帧，丢掉 MJPEG 缓冲里的旧画面。
        # 太小 → 拿旧画面算偏差，越调越偏；太大 → 每格变慢。
        p('drain', 6)

        # ??????????????????
        # ????????????????/control ?? 500??????????
        p('enhance', True)
        p('clahe_clip', 3.0)

        # 是否弹预览窗口（画靶标框、目标十字、当前角度）
        p('show', True)

        # 找到后是否把角度写回 patrol.yaml，下次直接从这里开始。
        p('save_to_config', True)

        # 严格模式：解出来的靶标必须和当前点位名一致，否则当作没看到继续扫。
        p('strict_code', False)

        # true=先试上次记住的角度（快）；false=每次都完整扫一遍（更稳）
        p('preset_first', False)

        # 到点先不动，直接看一眼当前画面：有码就立即对齐，没码再扫。
        # 比“先转到记住的角度”更直接，也不会因为记忆过期而白转一次。
        p('check_first', True)
        # false = ???????????????????
        p('scan_enable', True)

        # ?????????????????????????????
        p('smooth_sweep', True)
        p('sweep_deg', 35.0)
        p('sweep_speed', 8.0)
        p('sweep_tilt', -4.0)

        # 每个点位开始时假定云台在 0/0（patrol_node 每点结束都回正）
        p('assume_center', True)

        # 开场先左右摆一下：答辩时看得出云台在工作，也能顺手抓到靶标
        p('sweep_first', True)

        self.source = self.get_parameter('source').value
        self.code_type = str(self.get_parameter('code_type').value).strip().lower()
        self.aruco_map = self._parse_map(str(self.get_parameter('aruco_map').value))
        self.pan_min = float(self.get_parameter('pan_min').value)
        self.pan_max = float(self.get_parameter('pan_max').value)
        self.pan_step = float(self.get_parameter('pan_step').value)
        self.tilt_min = float(self.get_parameter('tilt_min').value)
        self.tilt_max = float(self.get_parameter('tilt_max').value)
        self.tilt_step = float(self.get_parameter('tilt_step').value)
        self.settle = float(self.get_parameter('settle').value)
        self.target_u = float(self.get_parameter('target_u').value)
        self.target_v = float(self.get_parameter('target_v').value)
        self.fov_h = float(self.get_parameter('fov_h').value)
        self.fov_v = float(self.get_parameter('fov_v').value)
        self.max_refine = int(self.get_parameter('max_refine').value)
        self.gain = float(self.get_parameter('refine_gain').value)
        self.tol = float(self.get_parameter('tolerate').value)
        self.pan_sign = float(self.get_parameter('pan_sign').value)
        self.tilt_sign = float(self.get_parameter('tilt_sign').value)
        self.ramp_deg = float(self.get_parameter('ramp_deg').value)
        self.ramp_ms = float(self.get_parameter('ramp_ms').value)
        self.drain = int(self.get_parameter('drain').value)
        self.enhance = bool(self.get_parameter('enhance').value)
        self.clahe = cv2.createCLAHE(clipLimit=float(self.get_parameter('clahe_clip').value),
                                     tileGridSize=(8, 8))
        self.show = bool(self.get_parameter('show').value)
        self.save_cfg = bool(self.get_parameter('save_to_config').value)
        self.strict_code = bool(self.get_parameter('strict_code').value)
        self.preset_first = bool(self.get_parameter('preset_first').value)
        self.check_first = bool(self.get_parameter('check_first').value)
        self.scan_enable = bool(self.get_parameter('scan_enable').value)
        self.smooth_sweep = bool(self.get_parameter('smooth_sweep').value)
        self.sweep_deg = float(self.get_parameter('sweep_deg').value)
        self.sweep_speed = float(self.get_parameter('sweep_speed').value)
        self.sweep_tilt = float(self.get_parameter('sweep_tilt').value)
        self.assume_center = bool(self.get_parameter('assume_center').value)
        self.sweep_first = bool(self.get_parameter('sweep_first').value)
        self.sweep_deg = float(self.get_parameter('sweep_deg').value)

        self.cur_pan = 0.0
        self.cur_tilt = 0.0

        self.ptz_pub = self.create_publisher(Vector3, str(self.get_parameter('ptz_topic').value), 10)
        self.res_pub = self.create_publisher(Vector3, '/aim_result', 10)
        self.pub_vis = self.create_publisher(Image, '/vision/aim_image', 10)
        self.pub_done = self.create_publisher(Bool, '/vision/aim_done', 10)
        self.txt_pub = self.create_publisher(String, '/aim_text', 10)
        self.create_subscription(String, '/aim_request', self._on_request, 10)

        self.job = None
        self.job_lock = threading.Lock()
        self.cap = None
        self.bridge = CvBridge()
        self.latest = None
        self.stable_at = 0.0
        self.debug_dump = bool(self.get_parameter('debug_dump').value)
        self.latest_t = 0.0
        self.allow_direct = bool(self.get_parameter('allow_direct_stream').value)
        topic = str(self.get_parameter('image_topic').value)
        if topic:
            self.create_subscription(Image, topic, self._on_image, 10)

        # 检测器准备（两种可按需启用）
        self.ad = cv2.aruco
        self.aruco_dict = self._make_dict(str(self.get_parameter('aruco_dict').value))
        if hasattr(self.ad, 'DetectorParameters_create'):
            self.aruco_params = self.ad.DetectorParameters_create()
        else:
            self.aruco_params = self.ad.DetectorParameters()
        # ??/????????????????????????????????
        try:
            self.aruco_params.adaptiveThreshConstant = 10
            self.aruco_params.minMarkerPerimeterRate = 0.02
            self.aruco_params.polygonalApproxAccuracyRate = 0.06
        except Exception:
            pass
        self.aruco_detector = None
        if not hasattr(self.ad, 'detectMarkers'):
            self.aruco_detector = self.ad.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.qr_detector = cv2.QRCodeDetector()

        self.get_logger().info(
            '靶标=%s  扫视 pan[%.0f,%.0f]x%.0f  tilt[%.0f,%.0f]x%.0f  预览=%s  源=%s'
            % (self.code_type, self.pan_min, self.pan_max, self.pan_step,
               self.tilt_min, self.tilt_max, self.tilt_step, self.show, self.source))

    # ---------- 初始化辅助 ----------
    @staticmethod
    def _parse_map(txt):
        out = {}
        for item in txt.split(','):
            if ':' not in item:
                continue
            name, idx = item.split(':', 1)
            try:
                out[int(idx.strip())] = name.strip().upper()
            except ValueError:
                continue
        return out

    def _make_dict(self, name):
        key = getattr(self.ad, name, None) or self.ad.DICT_4X4_50
        if hasattr(self.ad, 'Dictionary_get'):
            return self.ad.Dictionary_get(key)
        return self.ad.getPredefinedDictionary(key)

    # ---------- 对外接口 ----------
    def _on_image(self, msg):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_t = time.time()
        except Exception:
            pass

    def _on_request(self, msg):
        name = (msg.data or '').strip()
        if not name:
            return
        with self.job_lock:
            self.job = name
        self.get_logger().info('收到对准请求: %s' % name)

    def _publish_result(self, pan, tilt, ok, text):
        v = Vector3()
        v.x = float(pan)
        v.y = float(tilt)
        v.z = 1.0 if ok else 0.0
        self.res_pub.publish(v)
        self.txt_pub.publish(String(data=text))
        self.get_logger().info('%s: pan=%.1f tilt=%.1f ok=%s %s'
                               % ('对准成功' if ok else '对准失败', pan, tilt, ok, text))

    # ---------- 云台 / 相机 ----------
    def _pump(self, seconds):
        t_end = time.time() + float(seconds)
        while time.time() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def _ptz(self, pan, tilt, hold=0.35):
        """云台是一次性指令，丢包就不动了，所以在 hold 内补发几次。

        另外这里不是一步跳到位：舵机跟上 15° 的跳变会猛冲、还会拖影，
        所以按 ramp_deg 分几步走，最后再停在目标上补发 hold 秒。
        """
        steps = max(1, int(max(abs(pan - self.cur_pan),
                              abs(tilt - self.cur_tilt)) / max(self.ramp_deg, 0.1)))
        p0, t0 = self.cur_pan, self.cur_tilt
        for i in range(1, steps + 1):
            m = Vector3()
            m.x = float(p0 + (pan - p0) * i / steps)
            m.y = float(t0 + (tilt - t0) * i / steps)
            self.ptz_pub.publish(m)
            self._pump(self.ramp_ms / 1000.0)
            self._publish_live()
        self.cur_pan, self.cur_tilt = float(pan), float(tilt)
        # ????????????????????????????
        self.stable_at = time.time()

        # 目标位停稳 + 补发，防丢包
        m = Vector3()
        m.x = float(pan)
        m.y = float(tilt)
        t_end = time.time() + hold
        while time.time() < t_end:
            self.ptz_pub.publish(m)
            self._pump(0.12)
            self._publish_live()

    def _open(self):
        """没人在发图像话题时才自己拉流（会阻塞，所以只在降级时用）。"""
        if self.cap is not None:
            return self.cap
        os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'timeout;3000')
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            self.get_logger().error('打不开视频流: %s' % self.source)
            return None
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.cap = cap
        return cap

    def _detect(self, img):
        """在一帧里找靶标，返回 dict 或 None。

        用 dict 而不是元组，是为了顺带把角点带出来给预览窗口画框。
        """
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        if self.enhance:
            # ??????????????CLAHE ????????
            gray = self.clahe.apply(gray)

        if self.code_type in ('aruco', 'both'):
            if self.aruco_detector is not None:
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            else:
                corners, ids, _ = self.ad.detectMarkers(gray, self.aruco_dict,
                                                        parameters=self.aruco_params)
            if ids is not None and len(ids) > 0:
                pts = corners[0].reshape(-1, 2)
                idx = int(ids.ravel()[0])
                return {'code': self.aruco_map.get(idx, 'id%d' % idx),
                        'u': float(pts[:, 0].mean()) / w,
                        'v': float(pts[:, 1].mean()) / h,
                        'pts': pts, 'w': w, 'h': h}

        if self.code_type in ('qr', 'both'):
            data, pts, _ = self.qr_detector.detectAndDecode(gray)
            if pts is not None and data:
                pts = pts.reshape(-1, 2)
                return {'code': data,
                        'u': float(pts[:, 0].mean()) / w,
                        'v': float(pts[:, 1].mean()) / h,
                        'pts': pts, 'w': w, 'h': h}
        return None

    def _read_fresh(self, cap=None):
        """取一张当前时间的画面。

        优先用识别节点转发的 /vision/image_raw：ESP32-CAM 只容一个流客户端，
        两个节点各自 VideoCapture 会永久阻塞（云台看起来“不动”就是卡在这里）。
        话题没数据时才降级到自己拉流。
        """
        if self.latest is not None and time.time() - self.latest_t < 1.0:
            return self.latest.copy()
        if not self.allow_direct:
            # ??????????? None?????????????
            # ???????? VideoCapture ?? ????????????
            return None
        if cap is None:
            cap = self._open()
        if cap is None:
            return None
        for _ in range(max(0, self.drain)):
            cap.grab()
        ok, frame = cap.read()
        if not ok or frame is None:
            return None
        return frame

    def _sharpness(self, frame):
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(g, cv2.CV_64F).var())

    def _grab(self, cap=None):
        # ????????????? + JPEG ???????????????
        # ???????????????????????????
        best = None
        best_score = -1.0
        for _ in range(3):
            deadline = time.time() + 2.0
            while time.time() < deadline and self.latest_t <= self.stable_at:
                self._pump(0.05)
            if self.latest is None:
                continue
            frame = self.latest.copy()
            score = self._sharpness(frame)
            if score > best_score:
                best, best_score = frame, score
            self.stable_at = self.latest_t   # ????????
            self._pump(0.12)
        if best is None:
            return None
        self.last_mean = float(cv2.cvtColor(best, cv2.COLOR_BGR2GRAY).mean())
        if self.debug_dump:
            try:
                import os as _os
                d = _os.path.expanduser('~/led_tests/aimdump')
                _os.makedirs(d, exist_ok=True)
                cv2.imwrite(_os.path.join(d, 'c_%s_p%d_t%d.jpg'
                           % (time.strftime('%H%M%S'), int(self.cur_pan), int(self.cur_tilt))), best)
            except Exception:
                pass
        hit = self._detect(best)
        self._preview(best, hit)
        return hit

    def _grab_fast(self):
        """扫视专用：直接用最新一帧检测，不等新帧。

        为什么扫视不能复用 _grab()：_grab 为了拿清晰帧会等 latest_t 变化，
        每步最多等 2 秒×3 轮，扫视就变成一顺一顺的。扫视时云台在连续动，
        用当前帧就够：帧新旧只差几十毫秒，检测到就停。
        """
        if self.latest is None:
            return None
        if time.time() - self.latest_t > 1.0:
            return None
        hit = self._detect(self.latest)
        self._preview(self.latest, hit)
        return hit

    def _publish_live(self):
        """????????????????????????????"""
        if self.latest is None:
            return
        try:
            vis = self.latest.copy()
            h, w = vis.shape[:2]
            cv2.drawMarker(vis, (int(self.target_u * w), int(self.target_v * h)),
                           (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
            cv2.putText(vis, 'pan=%.1f tilt=%.1f' % (self.cur_pan, self.cur_tilt),
                        (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            m = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_vis.publish(m)
        except Exception:
            pass

    def _preview(self, frame, hit):
        vis = frame.copy()
        h, w = vis.shape[:2]
        tx, ty = int(self.target_u * w), int(self.target_v * h)
        cv2.drawMarker(vis, (tx, ty), (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
        if hit:
            if hit.get('pts') is not None:
                cv2.polylines(vis, [hit['pts'].astype(int).reshape(-1, 1, 2)],
                              True, (0, 0, 255), 2)
            du = hit['u'] - self.target_u
            dv = hit['v'] - self.target_v
            cv2.putText(vis, '%s u=%.2f v=%.2f du=%+.2f dv=%+.2f'
                        % (hit['code'], hit['u'], hit['v'], du, dv),
                        (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(vis, 'pan=%.1f tilt=%.1f' % (self.cur_pan, self.cur_tilt),
                    (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        try:
            m = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_vis.publish(m)
        except Exception:
            pass
        if not self.show:
            return
        try:
            cv2.imshow('aim_marker', vis)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn('preview unavailable: %s' % e)
            self.show = False

    def _load_preset(self, name):
        try:
            with open(PT, encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            pt = (cfg.get('points') or {}).get(name) or {}
            if 'pan' in pt and 'tilt' in pt:
                return float(pt['pan']), float(pt['tilt'])
        except Exception:
            pass
        return None

    def _save_preset(self, name, pan, tilt):
        if not self.save_cfg:
            return
        try:
            with open(PT, encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            cfg.setdefault('points', {}).setdefault(name, {})
            cfg['points'][name]['pan'] = round(float(pan), 1)
            cfg['points'][name]['tilt'] = round(float(tilt), 1)
            body = yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False)
            with open(PT, 'w', encoding='utf-8') as f:
                f.write(HEADER + '\n' + body)
            self.get_logger().info('已把 %s 的对准角度写回 patrol.yaml' % name)
        except Exception as e:
            self.get_logger().warn('写配置失败: %s' % e)

    # ---------- 主流程 ----------
    def _dump_fail(self, name, img):
        try:
            import os as _os
            d = _os.path.expanduser('~/led_tests/aim_fail')
            _os.makedirs(d, exist_ok=True)
            if img is None:
                self.get_logger().warn('scan failed, no frame')
                return
            t = time.strftime('%H%M%S')
            p1 = _os.path.join(d, '%s_%s.jpg' % (name, t))
            cv2.imwrite(p1, img)
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.get_logger().warn('scan failed, saved %s  mean=%.0f' % (p1, float(g.mean())))
        except Exception as e:
            self.get_logger().warn('dump fail: %s' % e)

    def _scan_grid(self, name):
        """先看当前画面，不中再扫。返回 (pan, tilt, code) 或 None。

        第一步不动云台：保持当前角度拍一帧，有码就直接用这个角度去微调；
        没码才进入左右扫视和栅格搜索。这样无需依赖“上次记住的角度”。
        """
        # 第 0 步：不动，直接看
        if self.check_first:
            hit = self._grab()
            if hit and (not self.strict_code or hit['code'].upper() == name.upper()):
                self.get_logger().info('不用扫：当前画面已有靶标 %s（pan=%.1f tilt=%.1f）'
                                       % (hit['code'], self.cur_pan, self.cur_tilt))
                return self.cur_pan, self.cur_tilt, hit['code']

        if not self.scan_enable:
            self.get_logger().info('scan disabled, give up after one look')
            return None

        # 连续扫视：多条水平线，每条都从右往左
        if self.smooth_sweep:
            # 匀速扫视：角度由时间算出来，不靠步数累加。
            # 检测耗时不固定，若用“走一步等 0.06s”，
            # 实际速度会被拖慢且忽快忽慢。
            tilts = [self.sweep_tilt,
                     self.sweep_tilt + 6.0,
                     self.sweep_tilt - 6.0]
            tilts = [t for t in tilts if self.tilt_min <= t <= self.tilt_max]
            for row, tl in enumerate(tilts, 1):
                self.get_logger().info(
                    '\u626b\u89c6\u7b2c %d/%d \u884c  tilt=%.0f  %.0f\u00b0/s  '
                    'pan %.0f -> %.0f'
                    % (row, len(tilts), tl, self.sweep_speed,
                       -self.sweep_deg, self.sweep_deg))
                t0 = time.time()
                last_cmd = 0.0
                last_det = 0.0
                while True:
                    elapsed = time.time() - t0
                    pan = -self.sweep_deg + self.sweep_speed * elapsed
                    if pan > self.sweep_deg + 1e-6:
                        break
                    now = time.time()
                    if now - last_cmd >= 0.04:
                        self._ptz(pan, tl, hold=0.0)
                        last_cmd = now
                    # 检测节流到 ~10Hz，不让它拖慢云台运动
                    if now - last_det >= 0.10:
                        last_det = now
                        hit = self._grab_fast()
                        if hit:
                            if self.strict_code and hit['code'].upper() != name.upper():
                                self.get_logger().warn(
                                    '\u770b\u5230 %s \u7684\u7801\uff08\u5f53\u524d\u70b9\u4f4d %s\uff09\uff0c\u5ffd\u7565'
                                    % (hit['code'], name))
                            else:
                                self.get_logger().info(
                                    '\u547d\u4e2d: pan=%.1f tilt=%.1f \u9776\u6807=%s'
                                    % (pan, tl, hit['code']))
                                return pan, tl, hit['code']
                    self._pump(0.02)



        seq = []
        preset = self._load_preset(name) if self.preset_first else None

        # 开场扫一眼：左右各摆一次，既有观感，靶标正好在视野里也能直接命中。
        if self.sweep_first:
            seq.append((round(self.sweep_deg, 1), 0.0))
            seq.append((round(-self.sweep_deg, 1), 0.0))

        if preset:
            seq.append(preset)

        # 栅格由中心向外：机柜一般就在正前方附近，先扫中间命中最快。
        grid = []
        tilt = self.tilt_max
        while tilt >= self.tilt_min - 1e-6:
            pan = self.pan_min
            while pan <= self.pan_max + 1e-6:
                grid.append((round(pan, 1), round(tilt, 1)))
                pan += self.pan_step
            tilt -= self.tilt_step
        grid.sort(key=lambda q: (abs(q[0]) / max(self.pan_step, 1.0)
                                 + 1.3 * abs(q[1]) / max(self.tilt_step, 1.0),
                                 -q[1], q[0]))

        for pt in grid:
            if pt in seq:
                continue
            if preset is None or abs(pt[0] - preset[0]) > 1e-3 or abs(pt[1] - preset[1]) > 1e-3:
                seq.append(pt)

        # 不再提前拉流：有话题就靠话题，没有才在 _grab 里降级开流
        total = len(seq)
        last_frame = []
        for i, (pan, tilt) in enumerate(seq, 1):
            self._ptz(pan, tilt)
            self._pump(self.settle)
            hit = self._grab()
            if hit:
                # 严格模式：码对不上当前点位就当没看到，继续扫下一格。
                # 这样即使邻柜的码混进画面，也不会锁错目标。
                if self.strict_code and hit['code'].upper() != name.upper():
                    self.get_logger().warn(
                        '第 %d/%d 格看到 %s 的码（当前点位 %s），忽略继续扫'
                        % (i, total, hit['code'], name))
                else:
                    self.get_logger().info('第 %d/%d 格命中: pan=%.0f tilt=%.0f 靶标=%s'
                                           % (i, total, pan, tilt, hit['code']))
                    return pan, tilt, hit['code']
            if self.latest is not None:
                last_frame.append(self.latest.copy())
            self.get_logger().info('cell %d/%d pan=%.0f tilt=%.0f mean=%.0f' %
                                   (i, total, pan, tilt, getattr(self, 'last_mean', -1.0)))
            if i % 6 == 0:
                self.get_logger().info('已扫 %d/%d 格，暂无目标' % (i, total))
        self._dump_fail(name, last_frame[-1] if last_frame else None)
        return None

    def _refine(self, pan, tilt, cap=None):
        """把靶标挪到 (target_u, target_v)，返回最终角度和内容。

        带防发散：如果连续两次偏差没变小，说明方向反了或画面还是旧的，
        立刻停手，保留当前角度并告警，不再往限位里顶。
        """
        code = ''
        last_err = None
        for k in range(self.max_refine + 1):
            hit = self._grab()
            if not hit:
                self._ptz(pan, tilt)
                self._pump(self.settle)
                hit = self._grab()
                if not hit:
                    self.get_logger().warn('微调中丢失靶标，用当前角度')
                    return pan, tilt, code
            code = hit['code']
            du = hit['u'] - self.target_u
            dv = hit['v'] - self.target_v
            err = max(abs(du), abs(dv))
            self.get_logger().info('微调 %d: pan=%.1f tilt=%.1f du=%+.2f dv=%+.2f'
                                   % (k, pan, tilt, du, dv))
            if err <= self.tol:
                break
            if last_err is not None and err > last_err * 1.05:
                self.get_logger().warn('偏差变大（%.2f→%.2f），停止微调防发散'
                                       % (last_err, err))
                break
            last_err = err
            pan += self.pan_sign * du * self.fov_h * self.gain
            tilt += self.tilt_sign * dv * self.fov_v * self.gain
            pan = max(-60.0, min(60.0, pan))
            tilt = max(-30.0, min(30.0, tilt))
            self._ptz(pan, tilt)
            self._pump(self.settle)
        return pan, tilt, code

    def run_job(self, name):
        try:
            # patrol_node 每点结束会把云台回正，所以这里按 0/0 起算
            self.pub_done.publish(Bool(data=False))
            if self.assume_center:
                self.cur_pan, self.cur_tilt = 0.0, 0.0

            # ???????????? 39 ???????????
            t_wait = time.time() + 3.0
            while time.time() < t_wait and time.time() - self.latest_t > 1.0:
                self._pump(0.1)
            if time.time() - self.latest_t > 1.5:
                self._publish_result(0.0, 0.0, False,
                                     '????????? traffic_light ?????/vision/image_raw ?????')
                return

            hit = self._scan_grid(name)
            if not hit:
                self._publish_result(0.0, 0.0, False, '扫视范围内没找到靶标')
                return
            pan, tilt = hit[0], hit[1]
            pan, tilt, code2 = self._refine(pan, tilt)
            code = code2 or hit[2]
            self._ptz(pan, tilt, hold=0.6)
            self._save_preset(name, pan, tilt)
            note = '靶标=%s' % code
            if code and code.upper() != name.upper() and not code.startswith('id'):
                note += '（与点位 %s 不一致，检查是否停错点位）' % name
                self.get_logger().warn(note)
            self.pub_done.publish(Bool(data=True))
            self._publish_result(pan, tilt, True, note)
        except Exception as e:
            self.get_logger().error('扫视异常: %s' % e)
            self._publish_result(0.0, 0.0, False, '异常: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = AimMarker()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        with node.job_lock:
            job = node.job
            node.job = None
        if job:
            node.run_job(job)
    if node.show:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
