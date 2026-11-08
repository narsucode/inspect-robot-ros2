# 机房设备状态智能巡检机器人

> 本科毕业设计 · ESP32-S3 + ROS 2
> 建图 → 导航 → 云台对准 → 指示灯识别 → 自动报告，全流程无人干预

<!-- 放一张效果图或 GIF，剪辑里截一帧就行 -->
![effect](docs/images/effect.jpg)

---

## 硬件

| 部件 | 型号 | 说明 |
|---|---|---|
| 主控 | ESP32-S3 | 上下位机一体，跑 micro-ROS |
| 激光雷达 | YDLidar X3 Pro | 360°，10 Hz，UART 115200 |
| 摄像头 | ESP32-CAM（OV2640） | MJPEG 推流，80 端口 |
| 云台 | 二自由度舵机 | LEDC PWM 50 Hz，1500 µs 中位 |
| 底盘 | 四轮差速 | 编码器闭环，PID 调速 |

---

## 软件架构

```
┌─────────────────────── 上位机（Ubuntu 22.04 / ROS 2 Humble）───────────────────────┐
│                                                                                    │
│   patrol_gui (PyQt5)  ──►  patrol_node  ──►  Nav2 / AMCL  ──►  /cmd_vel            │
│         │                    │                                                     │
│         │                    ├──►  aim_marker  ──►  /cmd_ptz_angle                 │
│         │                    │       （扫视 + ArUco + 闭环对准）                    │
│         │                    │                                                     │
│         └────────────────────┴──►  traffic_light  ──►  /vision/light_state        │
│                                      （唯一拉流者，转发 /vision/image_raw）          │
│                                                                                    │
└────────────────────────────────────┬───────────────────────────────────────────────┘
                                     │  micro-ROS over UDP 8888
┌────────────────────────────────────┴───────────────────────────────────────────────┐
│                          下位机（ESP32-S3 / ESP-IDF + FreeRTOS）                    │
│                                                                                    │
│   /cmd_vel ──► 差速运动学 ──► PID ──► 电机 PWM                                     │
│   /cmd_ptz_angle ──► 云台角度 ──► LEDC                                             │
│   /scan ◄── YDLidar 解析（Core 1）      /odom ◄── 编码器航位推算                    │
│   /imu  ◄── MPU6050 + Madgwick          /battery ◄── ADC                           │
└────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 核心功能

### 1. 建图与导航

- `slam_toolbox` 建图，保存为 `maps/room_*.yaml`
- `AMCL` 蒙特卡洛定位，`Nav2` 全局 + 局部规划
- 多点巡检后自动返航，停车误差厘米级

### 2. 云台主动对准

停车位姿有偏差时，机柜可能不在画面正中。做法：

1. 云台从右（-40°）往左（+40°）**匀速 8°/s** 扫视
2. 任一行扫到 ArUco 码，立刻停下
3. PD 闭环微调 2–4 次，把码拉到画面 `(0.5, 0.38)` 位置
4. 偏差连续两次没变小就停手，防止往限位里顶

> 为什么匀速而不是跳档：跳档只试几个点，中间是盲区；匀速连续覆盖，码只要在视野里就一定能看到。

### 3. 指示灯颜色识别

不用简单阈值，分两步：

1. **定位亮斑**：取 R/G/B 三通道最大值，亮斑在 V 通道上一定突出
2. **判色**：只在饱和像素（S ≥ 60）里统计色相区间，取占比最高的颜色

好处是过曝的白芯不会干扰判断（白芯 S 很低，会被过滤掉）。

判定映射：

| 颜色 | 状态 | 含义 |
|---|---|---|
| 绿 | 正常 | 设备运行中 |
| 黄 | 警告 | 需要关注 |
| 红 | 故障 | 需要处理 |
| 灭 | 异常 | 掉电或损坏 |

### 4. 报告生成

巡检结束自动落盘（在返航之前写，返航被打断也不丢）：

```json
{
  "time": "20261108_111525",
  "results": [
    {"point": "A", "state": "GREEN", "level": "正常", "hits": 63, "votes": {"GREEN": 63},
     "image": "~/patrol_shots/xxx_A.jpg", "ok": true}
  ],
  "back_ok": true
}
```

`tools/make_report_html.py` 可以把 JSON 渲染成一张可视化网页（含截图缩略图），方便存档和展示。

### 5. 动态避障

靠 Nav2 的局部代价地图 + 全局重规划，不是写死路线。路上放障碍物会自己改道。

---

## 目录结构

```
.
├── src/
│   ├── roscar_bringup/       底盘启动、TF、URDF
│   ├── roscar_navigation/    Nav2 参数、地图
│   ├── roscar_slam/          slam_toolbox 配置
│   ├── roscar_patrol/        巡检节点、GUI、对准节点
│   ├── roscar_vision/        指示灯识别节点
│   └── roscar_description/   机器人模型
├── tools/
│   ├── ptz_teleop.py         云台键盘遥控
│   ├── rviz_mark.py          RViz 里标点位
│   ├── make_report_html.py   JSON → 可视化网页
│   └── check_env.sh          环境自检
└── config/
    └── mapper_params.yaml    slam_toolbox 参数
```

---

## 怎么跑起来

### 依赖

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-ros2-control
pip install opencv-python pyqt5
```

### 启动

```bash
# 1. 底盘（含雷达、里程计、云台）
ros2 launch roscar_bringup bringup.launch.py use_rviz:=false

# 2. 导航
ros2 launch roscar_navigation navigation2.launch.py use_rviz:=true
#    RViz 里点 2D Pose Estimate 给初始位姿

# 3. 巡检界面
ros2 launch roscar_patrol patrol_gui.launch.py
#    点「开始巡检」
```

### 下位机

下位机固件在单独仓库：[inspect-robot-firmware](https://github.com/narsucode/inspect-robot-firmware)

```bash
cd patrol_car
idf.py menuconfig      # 填 WiFi SSID / 密码 / agent IP
idf.py -p /dev/ttyUSB0 flash
```

---

## 踩过的坑

调试过程中记录下来的，写在这里给后来人省点时间。

1. **ESP32-CAM 的 MJPEG 只允许一个客户端**
   两个节点同时拉流会互相挤掉，都收不到图。改成只让识别节点拉流，其他节点订阅它转发出来的 `/vision/image_raw`。

2. **`/map` 的图像行 0 在底部**
   `OccupancyGrid.data` 的行序和图上的 y 轴相反，直接画会上下颠倒，需要 `arr[::-1]`。

3. **YAML 里写 `off` 会被解析成 bool**
   `default_value: off` 变成 `False`，参数类型对不上直接抛 `InvalidParameterTypeException`。改用 `none` 并在代码里做兼容。

4. **云台指令发一次就动一下**
   下位机是"收到就动"的绝对值范式，UDP 丢一包云台就停住。改成 `hold` 秒内每 0.4 秒补发一次。

5. **`time.sleep` 会让图像回调饿死**
   ROS 2 单线程执行器里 `sleep` 期间不处理回调，`latest` 一直是旧帧，导致"画面对、判定错"。统一改成 `rclpy.spin_once` 轮询。

6. **门控订阅写在 try 块里被吞掉**
   `traffic_light` 把 `/vision/aim_done` 的订阅放在 ArUco 初始化的 `try` 里，ArUco 一报错订阅就没建，永远收不到对准完成信号，一直卡在 `WAIT`。订阅必须无条件创建。

7. **串口和雷达抢 LEDC 通道**
   雷达电机占用了 `LEDC_CHANNEL_0` 和 `TIMER_0`，云台必须避开，改用通道 2/3 和 `TIMER_1`。

---

## 演示

<!-- 视频链接 -->
B 站：https://www.bilibili.com/video/BV1LUWJ6yEtp

---

## 许可

MIT
