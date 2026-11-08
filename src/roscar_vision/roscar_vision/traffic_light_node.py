#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""红黄绿灯识别节点（2026-10-23 定稿版）

与调参台验证过的算法完全一致：
  1. 泄漏基线差分：baseline = min(当前帧, baseline + rise)
     —— 灯一亮就长时间保持检出，不会"只框一下"。
  2. 通道判色（不用 HSV）：过曝时 HSV 全废，BGR 相对关系仍然可分。
  3. 连续帧确认：同色必须连续两帧都出现，滤掉手/反光的瞬时变化。
  4. 只输出红/黄/绿，白色杂散亮斑直接丢弃。
  5. 显示提亮只作用于调试图，判定始终用原始帧。

摄像头侧前提（已烧进模块固件）：
  AEC 关闭、曝光固定 120（AEC=0, 0x03=0, 0x04=120）
  MJPEG: http://10.143.115.211:81/stream   480x320

话题：
  /vision/light_state  (std_msgs/String)  JSON：state + 每个亮斑的 RGB/位置
  /vision/debug_image  (sensor_msgs/Image)
"""

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

COLOR_BGR = {"RED": (0, 0, 255), "YELLOW": (0, 255, 255), "GREEN": (0, 255, 0)}


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__("traffic_light")
        p = self.declare_parameter
        p("source", "http://10.189.154.251:81/stream")
        p("show", True)
        p("baseline_rise", 1.0)     # 基线每帧最大上升量（越小框得越久）
        p("diff_thresh", 28)        # 比基线亮多少算点亮
        p("min_area", 8)          # 远距离小目标也能进（原 45 太大）
        p("min_roundness", 0.35)   # 圆度过滤：4πA/P²，圆≈1，手/反光≈0
        p("gate_px", 90)           # 锁定后只在此半径内继续找
        p("red_margin", 20)
        p("yel_margin", 20)
        p("grn_margin", 12)
        p("confirm_frames", 2)      # 连续确认帧数
        p("display_gain", 1.0)      # 仅调试图提亮（新模块本身较亮，默认不提亮）
        p("detect", True)           # false=只取流不判色（巡检只采图时用）

        # detector: hue=V 通道找亮斑 + 色相判色（实测稳）；baseline=旧基线差分法
        p("detector", "hue")
        p("detect_code", True)
        p("gated", False)
        p("thresh_v", 200)         # 三通道最大值的阈值（不是灰度！）
        p("s_min", 60)             # 参与判色的最小饱和度
        p("hue_min_ratio", 0.25)   # 主色相占比低于此值判不出

        self.source = self.get_parameter("source").value
        self.show = self.get_parameter("show").value
        self.gain = float(self.get_parameter("display_gain").value)
        self.rise = float(self.get_parameter("baseline_rise").value)
        self.diff_th = int(self.get_parameter("diff_thresh").value)
        self.min_area = int(self.get_parameter("min_area").value)
        self.min_round = float(self.get_parameter("min_roundness").value)
        self.gate_px = int(self.get_parameter("gate_px").value)
        self.red_m = int(self.get_parameter("red_margin").value)
        self.yel_m = int(self.get_parameter("yel_margin").value)
        self.grn_m = int(self.get_parameter("grn_margin").value)
        self.confirm = int(self.get_parameter("confirm_frames").value)
        self.detect = bool(self.get_parameter("detect").value)
        self.detector = str(self.get_parameter("detector").value).strip().lower()
        self.detect_code = bool(self.get_parameter("detect_code").value)
        self.gated = bool(self.get_parameter("gated").value)
        self.thresh_v = int(self.get_parameter("thresh_v").value)
        self.s_min = int(self.get_parameter("s_min").value)
        self.hue_min_ratio = float(self.get_parameter("hue_min_ratio").value)

        self.pub_state = self.create_publisher(String, "/vision/light_state", 10)
        self.pub_img = self.create_publisher(Image, "/vision/debug_image", 10)
        # 原始画面：给 aim_marker 用。ESP32-CAM 只容一个流客户端，
        # 所以全系统只让本节点拉流，其它节点订阅这个话题。
        self.pub_raw = self.create_publisher(Image, "/vision/image_raw", 10)
        self.bridge = CvBridge()
        self.aim_done = False
        self.ARUCO_MAP = {0: "A", 1: "B", 2: "C"}
        self._ad = None
        self._last_corners = None
        self._last_ids = None
        try:
            _ad = cv2.aruco
            self._audi = _ad.Dictionary_get(_ad.DICT_4X4_50) if hasattr(_ad, "Dictionary_get") else _ad.getPredefinedDictionary(_ad.DICT_4X4_50)
            self._aparams = _ad.DetectorParameters_create() if hasattr(_ad, "DetectorParameters_create") else _ad.DetectorParameters()
            try:
                self._aparams.adaptiveThreshConstant = 10
                self._aparams.minMarkerPerimeterRate = 0.02
                self._aparams.polygonalApproxAccuracyRate = 0.06
            except Exception:
                pass
            self._adet = None if hasattr(_ad, "detectMarkers") else _ad.ArucoDetector(self._audi, self._aparams)
            self._ad = _ad
            self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        except Exception as _e:  # noqa: BLE001
            self.get_logger().warn(f'ArUco 初始化失败（不影响红灯判色）: {_e}')

        # 门控信号必须在 try 外面建：ArUco 初始化失败不能连带把门控也废掉，
        # 否则 aim_done 永远是 False，判断灯色会一直卡在 WAIT。
        self.aim_done = False
        self.create_subscription(Bool, '/vision/aim_done', self._on_aim_done, 10)

        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.stopped = False
        self.baseline = None
        self.prev_colors = set()
        self.last_boxes = []        # 上一帧确认过的灯位置，用作位置闸门
        self.last_log = 0.0

        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(0.08, self._tick)
        self._expo_checked = False
        self.get_logger().info(f"取流 {self.source}  show={self.show}")

    def _reader(self):
        while not self.stopped:
            if self.cap is None or not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.source)
                if not self.cap.isOpened():
                    self.get_logger().warn("打不开视频流，2 秒后重试")
                    time.sleep(2.0)
                    continue
                self.baseline = None
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            with self.lock:
                self.frame = f

    def _classify(self, frame, sel):
        b, g, r = (frame[:, :, i].astype(np.int16) for i in range(3))
        R, G, B = float(r[sel].mean()), float(g[sel].mean()), float(b[sel].mean())
        if min(R, G) - B >= self.yel_m and abs(R - G) <= 55:
            return "YELLOW", R, G, B
        if R - max(G, B) >= self.red_m:
            return "RED", R, G, B
        if G - max(R, B) >= self.grn_m:
            return "GREEN", R, G, B
        return "WHITE", R, G, B

    def _on_aim_done(self, msg):
        self.aim_done = bool(msg.data)

    def _publish_tick_common(self, vis, state, lamps, raw=None, code=None):
        """两种检测器共用的收尾：画状态字、发状态与调试图、打日志。"""
        if raw is not None:
            try:
                m = self.bridge.cv2_to_imgmsg(raw, encoding="bgr8")
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = "camera"
                self.pub_raw.publish(m)
            except Exception:
                pass
        cv2.putText(vis, f"state={state}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (0, 255, 0) if lamps else (180, 180, 180), 2)
        msg = String()
        msg.data = json.dumps({"state": state, "lamps": lamps, "code": list(code or [])},
                               ensure_ascii=False)
        self.pub_state.publish(msg)

        now = time.time()
        if now - self.last_log > 1.0:
            self.last_log = now
            brief = ", ".join(f"{l['color']}:{l['area']}px" for l in lamps) or "无"
            self.get_logger().info(f"state={state}  {brief}")

        try:
            im = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            im.header.stamp = self.get_clock().now().to_msg()
            im.header.frame_id = "camera"
            self.pub_img.publish(im)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"发布调试图失败: {e}")

        if self.show:
            try:
                cv2.imshow("traffic_light", vis)
                cv2.waitKey(1)
            except Exception:
                pass

    def _hue_state(self, hp):
        """在饱和像素里数色相子，返回 (颜色, 置信度)。"""
        if len(hp) == 0:
            return "WHITE", 0.0
        h = hp[:, 0].astype(np.int32)
        n = float(len(h))
        cnt = {
            "RED": float(np.count_nonzero((h <= 8) | (h >= 172))) / n,
            "YELLOW": float(np.count_nonzero((h >= 12) & (h <= 38))) / n,
            "GREEN": float(np.count_nonzero((h >= 40) & (h <= 90))) / n,
            "BLUE": float(np.count_nonzero((h >= 95) & (h <= 135))) / n,
        }
        best = max(cnt, key=cnt.get)
        return best, cnt[best]

    def _tick_hue(self):
        """V 通道找亮斑 + 色相判色。

        为什么不用灰度：红灯 (255,0,0) 灰度只有 76、绿 150，
        “灰度>200”会把红绿灯直接滤掉，只剩黄灯和白反光。
        用三通道最大值 V 则任何颜色的灯都是 255。
        """
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return

        if not self.detect:
            vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
            cv2.putText(vis, "DETECT OFF", (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 200, 255), 2)
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
            if time.time() - self.last_log > 5.0:
                self.last_log = time.time()
                self.pub_state.publish(String(
                    data=json.dumps({"state": "DISABLED", "lamps": []})))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2]
        _, m = cv2.threshold(v, self.thresh_v, 255, cv2.THRESH_BINARY)
        k = np.ones((3, 3), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)

        aruco_found = []
        if self.detect_code and self._ad is not None:
            try:
                _g = self.clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                if self._adet is not None:
                    _c, _ids, _ = self._adet.detectMarkers(_g)
                else:
                    _c, _ids, _ = self._ad.detectMarkers(_g, self._audi, parameters=self._aparams)
                if _ids is not None and len(_ids) > 0:
                    aruco_found = [self.ARUCO_MAP.get(int(i), 'id%d' % int(i)) for i in _ids.ravel()]
                    self._last_corners, self._last_ids = _c, _ids
                else:
                    self._last_corners = self._last_ids = None
            except Exception:
                pass

        lamps = []
        if self.gated and not self.aim_done:
            # ?????????????????????
            vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
            if self._last_ids is not None:
                try:
                    self._ad.drawDetectedMarkers(vis, self._last_corners, self._last_ids)
                except Exception:
                    pass
            cv2.putText(vis, 'SEARCH/ALIGN  code=%s' % ('/'.join(aruco_found) or '-'),
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self._publish_tick_common(vis, 'WAIT', [], raw=frame, code=aruco_found)
            return

        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:6]:
            area = float(cv2.contourArea(c))
            if area < self.min_area:
                continue
            if area > frame.shape[0] * frame.shape[1] * 0.25:
                continue                      # 整片白墙/窗，不是灯
            x, y, w, h = cv2.boundingRect(c)
            sel = cv2.drawContours(np.zeros(m.shape, np.uint8), [c], -1, 1, -1) > 0
            hp = hsv[sel]
            keep = hp[(hp[:, 1] >= self.s_min) & (hp[:, 2] >= self.thresh_v)]
            # 白纸、墙面：亮但没颜色，可用像素太少就丢掉，不画框
            if len(keep) < 8 or float(hp[:, 1].mean()) < 15:
                continue
            color, conf = self._hue_state(keep)
            if color in ("WHITE", "BLUE") or conf < self.hue_min_ratio:
                continue
            b_g, g_g, r_g = (float(keep[:, 2].mean()), float(keep[:, 1].mean()),
                             float(keep[:, 0].mean()))
            lamps.append({"color": color, "area": int(area), "conf": round(conf, 2),
                          "box": [int(x), int(y), int(w), int(h)],
                          "R": round(r_g), "G": round(g_g), "B": round(b_g)})

        cur_colors = {l["color"] for l in lamps}
        if self.confirm >= 2 and self.prev_colors:
            lamps = [l for l in lamps if l["color"] in self.prev_colors]
        self.prev_colors = cur_colors

        if not lamps:
            state = "OFF"
        elif len({l["color"] for l in lamps}) == 1:
            state = lamps[0]["color"]
        else:
            state = "MULTI"

        vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
        if self._last_ids is not None:
            try:
                self._ad.drawDetectedMarkers(vis, self._last_corners, self._last_ids)
            except Exception:
                pass
        for l in lamps:
            x, y, w, h = l["box"]
            col = COLOR_BGR.get(l["color"], (200, 200, 200))
            cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
            cv2.putText(vis, f"{l['color']} {l['conf']:.2f}", (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        self._publish_tick_common(vis, state, lamps, raw=frame, code=aruco_found)

    def _tick(self):
        if self.detector == 'hue':
            return self._tick_hue()

        # 启动后检查一次曝光：若是满量程（全白）或明显异常，自动修正
        if not self._expo_checked:
            self._expo_checked = True
            try:
                import urllib.request
                base = self.source.split("//")[1].split("/")[0].split(":")[0]
                hi = urllib.request.urlopen(
                    f"http://{base}/greg?reg=3&mask=255", timeout=3).read().decode().strip()
                lo = urllib.request.urlopen(
                    f"http://{base}/greg?reg=4&mask=255", timeout=3).read().decode().strip()
                exp = int(hi or 0) * 256 + int(lo or 0)
                if exp > 400:
                    urllib.request.urlopen(
                        f"http://{base}/reg?reg=3&mask=255&val=0", timeout=3).read()
                    urllib.request.urlopen(
                        f"http://{base}/reg?reg=4&mask=255&val=60", timeout=3).read()
                    self.get_logger().warn(f"曝光异常({exp})，已自动修正为 60")
            except Exception:
                pass

        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return

        # detect=False：只取流、不判色。仍然发调试图（界面显示/到点存图要用），
        # 状态固定为 DISABLED，避免上层误把它当成“全灭 OFF”。
        if not self.detect:
            vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
            cv2.putText(vis, "DETECT OFF", (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 200, 255), 2)
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
            if time.time() - self.last_log > 5.0:
                self.last_log = time.time()
                self.pub_state.publish(String(
                    data=json.dumps({"state": "DISABLED", "lamps": []})))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        if self.baseline is None or self.baseline.shape != gray.shape:
            self.baseline = gray.copy()
        else:
            self.baseline = np.minimum(gray, self.baseline + self.rise)

        diff = np.clip(gray - self.baseline, 0, 255).astype(np.uint8)
        _, m = cv2.threshold(diff, self.diff_th, 255, cv2.THRESH_BINARY)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        lamps = []
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:4]:
            area = float(cv2.contourArea(c))
            if area < self.min_area:
                continue
            if area > frame.shape[0] * frame.shape[1] * 0.25:
                continue                      # 整体亮度变化，不是灯
            x, y, w, h = cv2.boundingRect(c)
            sel = cv2.drawContours(np.zeros(m.shape, np.uint8), [c], -1, 1, -1) > 0

            # 圆度过滤：把细长/不规则亮块（手指、灯带反光）挡在外面
            perim = cv2.arcLength(c, True)
            roundness = (4.0 * np.pi * area / (perim * perim)) if perim > 0 else 0.0
            if roundness < self.min_round:
                continue

            # 位置闸门：已经锁定的灯附近才继续判，避免远处噪点抢位
            cx_, cy_ = x + w / 2.0, y + h / 2.0
            if self.last_boxes:
                near = any(abs(cx_ - bx) < self.gate_px and abs(cy_ - by) < self.gate_px
                           for bx, by in self.last_boxes)
                if not near:
                    continue
            color, R, G, B = self._classify(frame, sel)
            if color == "WHITE":
                continue
            lamps.append({"color": color, "area": int(area),
                          "box": [int(x), int(y), int(w), int(h)],
                          "R": round(R), "G": round(G), "B": round(B)})

        # 连续帧确认
        cur_colors = {l["color"] for l in lamps}
        if self.confirm >= 2:
            lamps = [l for l in lamps if l["color"] in self.prev_colors]
        self.prev_colors = cur_colors
        if lamps:
            self.last_boxes = [(l["box"][0] + l["box"][2] / 2.0,
                                l["box"][1] + l["box"][3] / 2.0) for l in lamps]

        if not lamps:
            state = "OFF"
        elif len({l["color"] for l in lamps}) == 1:
            state = lamps[0]["color"]
        else:
            state = "MULTI"

        # 调试图：显示提亮 + 过曝标记
        vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
        for l in lamps:
            x, y, w, h = l["box"]
            col = COLOR_BGR.get(l["color"], (200, 200, 200))
            cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
            cv2.putText(vis, f"{l['color']} {l['area']} R{l['R']}G{l['G']}B{l['B']}",
                        (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

        clip = 100.0 * float(np.count_nonzero(
            (frame[:, :, 2] >= 250) & (frame[:, :, 1] >= 250) & (frame[:, :, 0] >= 250))) / frame[:, :, 0].size
        cv2.putText(vis, f"state={state}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (0, 255, 0) if lamps else (180, 180, 180), 2)
        cv2.putText(vis, f"clip={clip:.1f}%", (8, 48), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 255), 1)

        msg = String()
        msg.data = json.dumps({"state": state, "lamps": lamps}, ensure_ascii=False)
        self.pub_state.publish(msg)

        now = time.time()
        if now - self.last_log > 1.0:
            self.last_log = now
            brief = ", ".join(f"{l['color']}:{l['area']}px@({l['box'][0]},{l['box'][1]})"
                              for l in lamps) or "无"
            self.get_logger().info(f"state={state}  {brief}")

        try:
            im = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            im.header.stamp = self.get_clock().now().to_msg()
            im.header.frame_id = "camera"
            self.pub_img.publish(im)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"发布调试图失败: {e}")

        if self.show:
            try:
                cv2.imshow("traffic_light", vis)
                cv2.waitKey(1)
            except Exception:
                pass

    def destroy_node(self):
        self.stopped = True
        if self.cap is not None:
            self.cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
