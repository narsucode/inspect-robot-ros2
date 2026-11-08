#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检节点 v2：命令接口 + JSON 状态输出，供 GUI 控制。

整体思路（读代码时按这个顺序看）：
    1. 启动时读两份 YAML：
       - waypoints.yaml：每个点位的 (x, y, yaw)，给 Nav2 用；
       - patrol.yaml  ：巡检顺序、每个点位的云台角度 pan/tilt、期望状态 expect。
    2. 平时空转，等 /patrol_command 收到 "start" 才开始一轮巡检。
    3. 一轮巡检 = 对 order 里的每个点位重复：
       导航到位 → 云台对准 → 采样识别 → （识别不清就左右扫视重试）→ 记结果。
    4. 全部跑完把结果写成 JSON 报告，并播报结束语音。

为什么导航用 Action 而不是 Topic：
    NavigateToPose 是"长任务"，Action 能返回 接受/拒绝、进度、最终结果，
    还能中途取消；Topic 只能发出去，不知道有没有走到。

话题一览：
    订阅 /patrol_command  std_msgs/String  "start" / "stop" / "reset"
    订阅 /vision/light_state std_msgs/String JSON：{"state":"RED", ...}
    发布 /patrol_status   std_msgs/String  JSON，给 GUI 显示进度
    发布 /cmd_ptz_angle   geometry_msgs/Vector3  x=pan(度) y=tilt(度)
    发布 /speak           std_msgs/String  语音事件码
"""

import json
import os
import time

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Vector3
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf_transformations import quaternion_from_euler


# 颜色 → 语义等级。全局规则，不针对某一台设备：
# 绿灯=正常，黄灯=警告，红灯=故障，灯灭或识别不出都算异常。
LEVEL_OF = {
    'GREEN': '正常',
    'YELLOW': '警告',
    'RED': '故障',
    'OFF': '熄灭',
    'UNKNOWN': '未识别',
    'MULTI': '多灯同亮',
}


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # ---------- 1. 声明 ROS 参数 ----------
        # 声明之后才能在 launch 里覆盖；没传就用默认值。
        # patrol_order: 逗号分隔的点位名，优先级高于 patrol.yaml 里的 order
        # dry_run     : true 时不真的发导航，只走后面的识别流程（无车也能调）
        # auto_start  : true 时节点一启动就跑，不等 GUI 发 start
        # dwell_sec   : 每个点位采样识别的时长（秒），越长越稳但越慢
        # nav_timeout : 单次导航的超时（秒），超时就取消并记 NAV_FAILED
        self.declare_parameter('patrol_order', '')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('dwell_sec', 2.5)
        self.declare_parameter('nav_timeout', 180.0)
        # skip_vision: true 时跳过云台与识别，只验证导航能不能到点。
        self.declare_parameter('skip_vision', False)
        # capture_only: true 时云台照转、到点存一张图，但不做颜色判定。
        self.declare_parameter('capture_only', False)
        # snapshot_topic: 存图用的话题（视觉节点发的调试图）。
        self.declare_parameter('snapshot_topic', '/vision/debug_image')
        # home_point : 巡检结束后要返回的原点名称；留空字符串则不返回。
        self.declare_parameter('home_point', 'O')
        # aim_mode    : 'marker' 时到点先让云台扫视找二维码再识别（需 aim_marker 节点）；
        #               'off' 则直接用 patrol.yaml 里的固定角度。
        # aim_timeout : 等一次扫视对准的最长时间（秒），超时就退回固定角度。
        self.declare_parameter('aim_mode', 'off')
        self.declare_parameter('aim_timeout', 100.0)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.dwell = float(self.get_parameter('dwell_sec').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.skip_vision = bool(self.get_parameter('skip_vision').value)
        self.capture_only = bool(self.get_parameter('capture_only').value)
        self.home_point = str(self.get_parameter('home_point').value or '').strip()
        _am = self.get_parameter('aim_mode').value
        self.aim_mode = str(_am).strip().lower() if not isinstance(_am, bool) else ('none' if not _am else 'marker')
        if self.aim_mode in ('off', 'false', ''):
            self.aim_mode = 'none'
        self.aim_timeout = float(self.get_parameter('aim_timeout').value)
        self._aim_res = None
        self.aim_req_pub = self.create_publisher(String, '/aim_request', 10)
        self.create_subscription(Vector3, '/aim_result', self._on_aim_result, 10)

        # ---------- 2. 读配置 ----------
        # get_package_share_directory 拿到的是 install/ 下的路径，
        # 所以改完 YAML 要重新 colcon build（或用 --symlink-install）。
        pkg_dir = get_package_share_directory('roscar_patrol')
        self.waypoints = self._load_waypoints(os.path.join(pkg_dir, 'config', 'waypoints.yaml'))
        self.patrol_cfg = self._load_patrol(os.path.join(pkg_dir, 'config', 'patrol.yaml'))

        # 顺序优先级：launch 参数 > patrol.yaml 的 order > waypoints 里的全部点位
        order_param = self.get_parameter('patrol_order').value
        if order_param:
            self.order = [s.strip() for s in order_param.split(',') if s.strip()]
        else:
            self.order = self.patrol_cfg.get('order') or list(self.waypoints.keys())

        # ---------- 3. 通信接口 ----------
        # ActionClient 指向 Nav2 的 navigate_to_pose，节点名必须完全一致。
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 云台：x=水平 pan，y=俯仰 tilt，单位是“度”，由下位机固件解释。
        self.ptz_pub = self.create_publisher(Vector3, '/cmd_ptz_angle', 10)
        # 语音：只发事件码，具体播什么由车端语音模块决定。
        self.speak_pub = self.create_publisher(String, '/speak', 10)
        # 给 GUI 的状态（JSON 字符串）。
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        # ????????????? aim_done ? False?
        # ?????????? traffic_light ???????????????
        self.aim_gate_pub = self.create_publisher(Bool, '/vision/aim_done', 10)

        # 命令入口：GUI 按钮或命令行 ros2 topic pub 都走这里。
        self.create_subscription(String, '/patrol_command', self._on_command, 10)

        # 识别结果：由视觉节点持续发布，这里只缓存最新一条。
        self.light = {}
        self.create_subscription(String, '/vision/light_state', self._on_light, 10)

        # 画面：capture_only 模式下到点存一张图，用视觉节点的调试图即可。
        self.bridge = CvBridge()
        self.last_img = None
        self.img_count = 0
        self.create_subscription(Image, self.get_parameter('snapshot_topic').value,
                                 self._on_image, 10)

        # 运行时状态
        self.running = False        # 是否有一轮巡检正在进行
        self.stop_flag = False      # 是否收到 stop（在当前点位结束后退出）
        self.results = []           # 每个点位一条记录
        self._last_speak = (None, 0.0)  # 语音去重：(上次的码, 时间)

        self.get_logger().info('巡检点位 %s  dry_run=%s  auto_start=%s  skip_vision=%s  capture_only=%s'
                               % (self.order, self.dry_run, self.auto_start,
                                  self.skip_vision, self.capture_only))
        self._status('idle', 0, '', '', '就绪')
        if self.auto_start:
            self.running = True

    # ---------- 4. 配置读取 ----------
    def _load_waypoints(self, path):
        """waypoints.yaml → {点名: (x, y, yaw)}，缺 yaw 时按 0 处理。"""
        if not os.path.exists(path):
            self.get_logger().error('找不到 %s' % path)
            return {}
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        return {n: (float(p['x']), float(p['y']), float(p.get('yaw', 0.0)))
                for n, p in (data.get('waypoints') or {}).items()}

    def _load_patrol(self, path):
        """patrol.yaml → {order: [...], points: {点名: {pan, tilt, expect}}}。"""
        if not os.path.exists(path):
            return {'order': [], 'points': {}}
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {'order': [], 'points': {}}

    # ---------- 5. 订阅回调 ----------
    def _on_command(self, msg):
        """命令格式：
             start              按 patrol.yaml 的顺序
             start B,A,D,C      本次临时指定顺序（可只写一部分，也可重复）
             stop / reset
        """
        raw = msg.data.strip()
        cmd = raw.lower()
        self.get_logger().info('收到命令: %s' % raw)

        if cmd == 'start' or cmd.startswith('start ') or cmd.startswith('start\t'):
            # 允许 “start B,A,D,C” 这种带顺序的写法；
            # 中文逗号也兼容，避免界面里输入法打错。
            order_txt = raw[5:].strip().replace('，', ',')
            if order_txt:
                order = [s.strip() for s in order_txt.split(',') if s.strip()]
                if order:
                    self.order = order
                    self.get_logger().info('本次巡检顺序: %s' % self.order)
            self.stop_flag = False
            self.results = []
            self.running = True
        elif cmd == 'stop':
            # 注意：这里是“软停”，不会打断正在进行的导航，
            # 当前点位跑完后在主循环里检查 stop_flag 退出，避免把车扔在半路。
            self.stop_flag = True
            self._status('idle', 0, '', '', '已请求停止')
        elif cmd == 'reset':
            self.results = []
            self._status('idle', 0, '', '', '已清空')

    def _on_light(self, msg):
        """缓存视觉节点发来的最新识别结果，解析失败就忽略（保持上一条）。"""
        try:
            self.light = json.loads(msg.data)
        except Exception:
            pass

    def _on_image(self, msg):
        """只保留最新一帧，存图时再转 OpenCV，避免占内存。"""
        self.last_img = msg
        self.img_count += 1

    # ---------- 6. 工具函数 ----------
    def _status(self, phase, index, point, state, message, expect=None, ok=None,
                image=None, level=None):
        """统一的状态广播：phase 取 idle/navigate/inspect/done。

        expect/ok/image 是本轮新增的：界面要靠它们填“期望/判定/截图”三列，
        以前只有 state，导致表格里期望永远是 '-'、异常点也看不出结论。
        """
        payload = {'phase': phase, 'index': index, 'total': len(self.order),
                   'point': point, 'state': state, 'message': message}
        if expect is not None:
            payload['expect'] = expect
        if ok is not None:
            payload['ok'] = bool(ok)
        if level is not None:
            payload['level'] = level
        if image is not None:
            payload['image'] = image
        try:
            self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception:
            pass

    def speak(self, code, gap=1.0):
        """向 /speak 发事件码，车端语音模块按配置播报。
        同一个码 1 秒内重复发会被丢弃，避免连播刷屏。"""
        now = time.time()
        last_code, last_t = self._last_speak
        if code == last_code and (now - last_t) < gap:
            return
        self._last_speak = (code, now)
        try:
            self.speak_pub.publish(String(data=code))
        except Exception:
            pass

    def set_ptz(self, pan, tilt, hold=1.2):
        """把云台转到指定角度。

        为什么是 for 循环重复发、而不是发一次就完事：
        下位机是“收到指令就动”的一次性范式，UDP 丢一包云台就不动了；
        这里在 hold 秒内每 0.4 秒补发一次，相当于软重传 + 等它转到位。
        """
        t0 = time.time()
        while time.time() - t0 < hold:
            m = Vector3()
            m.x = float(pan)
            m.y = float(tilt)
            try:
                self.ptz_pub.publish(m)
            except Exception:
                return
            rclpy.spin_once(self, timeout_sec=0.4)

    def reset_ptz(self):
        """云台回正（0°/0°）。

        每个点位处理完都调一次：既保证下一个点从正前方开始扫，
        也让现场看起来“做完了会归位”，不会停在歪着的角度。
        """
        try:
            self.set_ptz(0.0, 0.0, hold=1.0)
        except Exception:
            pass

    # ---------- 6.5 云台扫视对准 ----------
    def _on_aim_result(self, msg):
        """aim_marker 回报的对准结果：x=pan  y=tilt  z=1 成功 / 0 失败。"""
        self._aim_res = (float(msg.x), float(msg.y), float(msg.z) > 0.5)

    def aim_by_marker(self, name):
        """请求 aim_marker 扫视对准，成功返回 (pan, tilt)，失败或超时返回 None。

        为什么要阻塞等：后面的识别必须在云台停稳、二维码居中之后再做，
        否则采到的是云台还在转的模糊画面。
        """
        self._aim_res = None
        self.aim_req_pub.publish(String(data=name))
        t0 = time.time()
        while time.time() - t0 < self.aim_timeout:
            if self.stop_flag:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._aim_res is not None:
                pan, tilt, ok = self._aim_res
                self._aim_res = None
                return (pan, tilt) if ok else None
        self.get_logger().warn('%s 扫视对准超时，退回固定角度' % name)
        return None

    # ---------- 7. 导航 ----------
    def go_to_pose(self, name):
        """同步阻塞地走到某个点位，成功 True / 失败 False。

        完整流程（Action 的标准四步）：
            发目标 → 等接受 → 等结果 → （超时就取消）
        """
        if name not in self.waypoints:
            return False

        # dry_run：只假装走了一下，方便无车调识别流程。
        if self.dry_run:
            time.sleep(0.5)
            return True

        x, y, yaw = self.waypoints[name]

        # 目标位姿：位置用 x/y，朝向把 yaw 转成四元数（导航只认四元数）。
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation = Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

        # 1) 等 Action 服务出现（nav2 没起来就直接报错，别干等）。
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose 不可用')
            return False

        # 2) 发送目标并等“是否接受”。
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if not gh or not gh.accepted:
            return False

        # 3) 等最终结果；超时就主动取消，避免车卡在半路还占着任务。
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=self.nav_timeout)
        if not res_future.done():
            gh.cancel_goal_async()
            return False
        return True

    # ---------- 8. 识别采样 ----------
    def sweep_light(self, pan_center, tilt, sweep_deg=40.0, speed=8.0):
        """判色失败后：从右往左匀速扫一遍，边扫边累积投票。

        为什么不用跳档：跳档只试 5 个点，中间全是盲区；
        匀速扫是连续覆盖，灯只要在视野里就一定能看到。
        代价是慢（80° / 8°/s = 10 秒），但比漏判强。
        """
        votes = {}
        seen = 0
        t0 = time.time()
        dur = (2.0 * sweep_deg) / max(speed, 1.0)
        self.get_logger().info(
            '扫视找灯 %.0f°/s  %.0f° -> %.0f°  约 %.0f 秒'
            % (speed, pan_center - sweep_deg, pan_center + sweep_deg, dur))
        while True:
            el = time.time() - t0
            if el > dur or self.stop_flag:
                break
            pan = pan_center - sweep_deg + speed * el
            pan = max(-60.0, min(60.0, pan))
            self.set_ptz(pan, tilt, hold=0.0)
            rclpy.spin_once(self, timeout_sec=0.06)
            st = self.light.get('state', 'NONE')
            if st in ('RED', 'YELLOW', 'GREEN'):
                votes[st] = votes.get(st, 0) + 1
                seen += 1
                if votes[st] >= 5:
                    self.get_logger().info(
                        '扫视中看到 %s（%d 票），停' % (st, votes[st]))
                    break
        if not votes:
            return 'UNKNOWN', 0, {}
        best = max(votes.items(), key=lambda kv: kv[1])
        return best[0], best[1], votes

    def sample_state(self, seconds):
        """在 seconds 秒内多次读取识别结果，取“出现次数最多”的那个状态。

        为什么要投票而不是读一帧：
        单帧容易受反光、曝光、云台还在动的影响，偶尔会闪成 OFF 或别的颜色；
        多帧多数表决能把这类瞬时误判压掉，代价是慢一点。
        """
        # ??????????????????????????
        t_pre = time.time()
        while time.time() - t_pre < 1.0 and not self.stop_flag:
            rclpy.spin_once(self, timeout_sec=0.05)

        votes = {}
        t0 = time.time()
        while time.time() - t0 < seconds:
            # ?? spin?? spin ???????self.light ?????
            rclpy.spin_once(self, timeout_sec=0.2)
            st = self.light.get('state', 'NONE')
            if st not in ('NONE', ''):
                votes[st] = votes.get(st, 0) + 1
        if not votes:
            return 'UNKNOWN', 0, {}
        best = max(votes.items(), key=lambda kv: kv[1])
        return best[0], best[1], votes

    def _snapshot(self, name, timeout=3.0):
        """等一帧新画面再存盘，返回文件路径；没画面就返回空串。

        注意：这里用 spin_once 而不是 time.sleep，否则图像回调不会被执行，
        永远等不到新帧，存下来的就是上一张旧图。
        """
        if self.last_img is None:
            return ''
        stamp = self.img_count
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.img_count != stamp:
                break
        try:
            frame = self.bridge.imgmsg_to_cv2(self.last_img, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('取图失败: %s' % exc)
            return ''
        out_dir = os.path.expanduser('~/patrol_shots')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, '%s_%s.jpg' % (time.strftime('%Y%m%d_%H%M%S'), name))
        cv2.imwrite(path, frame)
        self.get_logger().info('已存图: %s' % path)
        return path

    # ---------- 9. 一轮巡检的主流程 ----------
    def run_once(self):
        """按 order 依次巡检。每个点位的流程见下方注释。"""
        self.results = []
        total = len(self.order)
        # ???????????????????????
        self.reset_ptz()
        self.speak('PATROL_START')

        for i, name in enumerate(self.order, 1):
            # 每个点位开始前先看有没有 stop 请求。
            if self.stop_flag:
                self._status('idle', i - 1, name, '', '已停止')
                break

            # (0) 云台回正：上一个点用完就归位，本点从正前方开始
            self.reset_ptz()
            # 门控复位：本点还没对准，先别让识别节点画灯框
            self.aim_gate_pub.publish(Bool(data=False))

            # (1) 导航到点位
            self._status('navigate', i, name, '', '前往 %s' % name)
            if not self.go_to_pose(name):
                # 导航失败也要留痕，继续下一个点，不整轮中断。
                _pt = (self.patrol_cfg.get('points') or {}).get(name, {}) or {}
                rec = {'point': name, 'state': 'NAV_FAILED', 'time': time.strftime('%H:%M:%S')}
                self.results.append(rec)
                self.speak('NAV_FAIL')
                self._status('inspect', i, name, 'NAV_FAILED', '导航失败',
                             expect=_pt.get('expect'), ok=False)
                continue
            if self.stop_flag:
                break

            # (2) 查该点位的云台预设角度（patrol.yaml 里没写就默认 0/0）
            pt = (self.patrol_cfg.get('points') or {}).get(name, {}) or {}
            pan = float(pt.get('pan', 0.0))
            tilt = float(pt.get('tilt', 0.0))

            # 该点位要不要做云台对准和识别：patrol.yaml 里写 inspect: false 就只到点。
            # 用途：机柜还没做完时，让那些点只验证导航，不空转云台也不误报异常。
            if not bool(pt.get('inspect', True)):
                rec = {'point': name, 'state': 'ARRIVED', 'expect': pt.get('expect'),
                       'ok': True, 'time': time.strftime('%H:%M:%S')}
                self.results.append(rec)
                self._status('inspect', i, name, 'ARRIVED', '已到达（本点不检查）',
                             expect=pt.get('expect'), ok=True)
                self.reset_ptz()
                self.speak('POINT_DONE')
                continue

            # 到点后先让云台主动扫视找二维码：找到就用它的角度，
            # 找不到（或没开）就退回上面读到的固定角度。
            if self.aim_mode == 'marker' and not self.stop_flag \
                    and bool(pt.get('aim', True)):
                self._status('inspect', i, name, '', '云台扫视对准')
                got = self.aim_by_marker(name)
                if got:
                    pan, tilt = got

            # 只测导航时：到点就记一条 ARRIVED，不碰云台、不做识别。
            if self.skip_vision:
                rec = {'point': name, 'state': 'ARRIVED', 'expect': pt.get('expect'),
                       'ok': True, 'time': time.strftime('%H:%M:%S')}
                self.results.append(rec)
                self._status('inspect', i, name, 'ARRIVED', '已到达（跳过识别）',
                             expect=pt.get('expect'), ok=True)
                self.reset_ptz()
                self.speak('POINT_DONE')
                continue

            # 只采图：云台照转，到点存一张，不做颜色判定。
            if self.capture_only:
                self._status('inspect', i, name, '', '对准云台')
                self.set_ptz(pan, tilt)
                path = self._snapshot(name)
                rec = {'point': name, 'state': 'CAPTURED', 'image': path,
                       'expect': pt.get('expect'), 'ok': True,
                       'time': time.strftime('%H:%M:%S')}
                self.results.append(rec)
                self.speak('POINT_DONE')
                self._status('inspect', i, name, 'CAPTURED', '已拍照',
                             expect=pt.get('expect'), ok=True, image=path)
                self.reset_ptz()
                continue

            # (3) 云台对准 → 采样 dwell 秒 → 投票
            self._status('inspect', i, name, '', '对准云台')
            self.set_ptz(pan, tilt)
            state, hits, votes = self.sample_state(self.dwell)

            # (4) 识别不清（未知/全灭/多灯）时，先等 3 秒重采，
            #     还不行就从右往左匀速扫一遍，不再跳档。
            if state in ('UNKNOWN', 'OFF', 'NONE', 'MULTI'):
                # ① 原地再等 3 秒重采：可能是灯刚亮、曝光没稳
                self._status('inspect', i, name, state, '等待 3 秒重采')
                t_wait = time.time() + 3.0
                while time.time() < t_wait and not self.stop_flag:
                    rclpy.spin_once(self, timeout_sec=0.05)
                s1, h1, v1 = self.sample_state(self.dwell)
                if s1 in ('RED', 'YELLOW', 'GREEN'):
                    state, hits, votes = s1, h1, v1
                elif not self.stop_flag:
                    # ② 还不行，从右往左匀速扫一遍，边扫边投票
                    self._status('inspect', i, name, state, '匀速扫视找灯')
                    s2, h2, v2 = self.sweep_light(pan, tilt)
                    if s2 in ('RED', 'YELLOW', 'GREEN'):
                        state, hits, votes = s2, h2, v2
                    # 扫完不管结果，都回到对准的角度，不歪着开走
                    self.set_ptz(pan, tilt, hold=0.6)

            # (5) 判定：颜色 → 语义等级。
            #     全局规则：绿=正常，黄=警告，红=故障，灭/未识别=异常。
            #     个别设备“亮红才正常”时，在 patrol.yaml 里写 expect 覆盖。
            image = self._snapshot(name, timeout=1.0)
            expect = pt.get('expect')
            level = LEVEL_OF.get(state, '未知')
            ok = (state == expect) if expect else (level == '正常')
            rec = {'point': name, 'state': state, 'level': level,
                   'hits': hits, 'votes': votes, 'image': image,
                   'expect': expect, 'ok': ok,
                   'time': time.strftime('%H:%M:%S')}
            self.results.append(rec)
            self.reset_ptz()
            self.speak('POINT_DONE')
            self._status('inspect', i, name, state, '判定 %s（%s）' % (level, state),
                         expect=expect, ok=ok, image=image, level=level)

        # 最后一个点也回正，带着正的角度返航
        self.reset_ptz()

        # (6) 先把报告写到磁盘：返航途中被打断也不会丢结果。
        #     返航结束后再补一次状态，覆盖同一份文件。
        report_path = self._save_report()

        # (7) 回原点：最后一站巡检完还得走回 home_point，回到起点才算整轮结束。
        #     回不去不中断流程（报告照样写），只在状态里标出来。
        back_ok = True
        if self.home_point and self.order and not self.stop_flag:
            if self.home_point not in self.waypoints:
                self.get_logger().warn('home_point %s 不在 waypoints.yaml 里，跳过回原点'
                                       % self.home_point)
            elif self.order[-1] != self.home_point:
                self._status('navigate', total, self.home_point, '',
                             '返回原点 %s' % self.home_point)
                back_ok = self.go_to_pose(self.home_point)
                self._status('return', total, self.home_point, '',
                             '已回到原点' if back_ok else '回原点失败')

        # (8) 收尾：把返航结果补进已写好的报告（同一份文件）。
        self._save_report(report_path, back_ok=back_ok)
        self._status('done', len(self.order), '', '',
                     '巡检完成' if back_ok else '巡检完成（未回到原点）')
        abnormal = [r for r in self.results if not r.get('ok', False)]
        if abnormal:
            self.speak('ALARM')
        self.speak('PATROL_DONE')
        self.running = False

    def _save_report(self, path=None, back_ok=None):
        """把本轮结果写到 ~/patrol_reports/patrol_<时间戳>.json。

        path 传入时就写到那个文件（返航后补写用），
        不传就按当前时间新建一份。
        """
        out_dir = os.path.expanduser('~/patrol_reports')
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        if path is None:
            path = os.path.join(out_dir, 'patrol_%s.json' % stamp)
        data = {'time': stamp, 'results': self.results}
        if back_ok is not None:
            data['back_ok'] = bool(back_ok)
        with open(path, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self.get_logger().info('报告已保存: %s' % path)
        return path


def main(args=None):
    """主循环：每 0.2 秒处理一次回调；发现 running=True 就跑一轮。

    注意 run_once() 是阻塞的（里面用 spin_until_future_complete 等导航），
    所以一轮巡检期间 /patrol_command 的 stop 要等它内部 spin 时才被处理。
    """
    rclpy.init(args=args)
    node = PatrolNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.running and not node.stop_flag:
                node.run_once()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
