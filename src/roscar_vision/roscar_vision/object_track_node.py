#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物体追踪节点：找画面里指定颜色（或最亮）的物体，驱动云台把它拉到画面中心。

原理
----
1. 从摄像头 MJPEG 取帧。
2. 按 target 参数选目标：
     red/yellow/green → 对应颜色的最大色块
     bright           → 画面里最亮的连通块（白纸、手电、灯都行）
3. 目标中心与画面中心做差，按比例（Kp）换算成云台角度增量。
4. 发布 /cmd_ptz_angle（x=pan 度, y=tilt 度，绝对值）。
   固件收到后 1 秒内保持"遥控态"，自动追踪会被暂停，所以两者不会打架。

参数
----
  source   取流地址
  target   red / yellow / green / bright
  kp       每像素对应的角度（默认 0.04）
  deadband 中心死区（像素，默认 25）
  show     是否弹窗
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node


class ObjectTrackNode(Node):
    def __init__(self):
        super().__init__("object_track")
        p = self.declare_parameter
        p("source", "http://10.143.115.211:81/stream")
        p("target", "red")
        p("kp", 0.04)
        p("deadband", 25)
        p("show", True)
        p("display_gain", 2.4)

        self.source = self.get_parameter("source").value
        self.target = str(self.get_parameter("target").value).lower()
        self.kp = float(self.get_parameter("kp").value)
        self.deadband = int(self.get_parameter("deadband").value)
        self.show = bool(self.get_parameter("show").value)
        self.gain = float(self.get_parameter("display_gain").value)

        self.pub = self.create_publisher(Vector3, "/cmd_ptz_angle", 10)
        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.stopped = False
        self.pan = 0.0
        self.tilt = 0.0
        self.last_see = 0.0

        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(0.2, self._tick)       # 5 Hz
        self.get_logger().info(
            f"追踪目标={self.target}  kp={self.kp}  死区={self.deadband}px")

    def _reader(self):
        while not self.stopped:
            if self.cap is None or not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.source)
                if not self.cap.isOpened():
                    time.sleep(2.0)
                    continue
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            with self.lock:
                self.frame = f

    def _find_target(self, frame):
        b, g, r = (frame[:, :, i].astype(np.int16) for i in range(3))
        v = np.maximum(np.maximum(r, g), b)

        if self.target == "bright":
            mask = (v >= 170).astype(np.uint8) * 255
        else:
            if self.target == "red":
                m = (r - np.maximum(g, b)) >= 25
            elif self.target == "yellow":
                m = ((np.minimum(r, g) - b) >= 20) & (np.abs(r - g) <= 55)
            elif self.target == "green":
                m = (g - np.maximum(r, b)) >= 15
            else:
                m = (v >= 170)
            mask = (m & (v >= 80)).astype(np.uint8) * 255

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None, 0
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 150:
            return None, 0
        x, y, w, h = cv2.boundingRect(c)
        return (x + w // 2, y + h // 2, x, y, w, h), area

    def _tick(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        found, area = self._find_target(frame)

        vis = np.clip(frame.astype(np.float32) * self.gain, 0, 255).astype(np.uint8)
        cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 24, 1)

        if found:
            tx, ty, x, y, bw, bh = found
            self.last_see = time.time()
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 200, 255), 2)
            cv2.line(vis, (cx, cy), (tx, ty), (0, 200, 255), 1)

            ex, ey = tx - cx, ty - cy
            if abs(ex) > self.deadband:
                self.pan = float(np.clip(self.pan + self.kp * ex, -90, 90))
            if abs(ey) > self.deadband:
                self.tilt = float(np.clip(self.tilt + self.kp * ey, -90, 90))

            cv2.putText(vis, f"err=({ex},{ey}) pan={self.pan:.0f} tilt={self.tilt:.0f}",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        else:
            cv2.putText(vis, "目标丢失", (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)

        # 丢了超过 2 秒就把角度慢慢回中位
        if time.time() - self.last_see > 2.0:
            self.pan *= 0.9
            self.tilt *= 0.9

        m = Vector3()
        m.x = self.pan
        m.y = self.tilt
        m.z = 0.0
        self.pub.publish(m)

        if self.show:
            try:
                cv2.imshow("object_track", vis)
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
    node = ObjectTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
