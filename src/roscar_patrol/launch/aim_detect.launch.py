"""aim_detect.launch.py —— 一条命令跑完「云台复位 → 扫视找码 → 识别灯色」。

启动三个进程（不用开三个终端）：
    traffic_light_node  识别灯色，弹窗口
    aim_marker          云台扫视对准，弹窗口
    aim_detect_test.py  编排流程：复位 → 请求对准 → 采样投票 → 给结论

用法：
    ros2 launch roscar_patrol aim_detect.launch.py
    ros2 launch roscar_patrol aim_detect.launch.py point:=B
    ros2 launch roscar_patrol aim_detect.launch.py show:=false    # 不弹窗口（无显示器时）

Ctrl+C 一次全停。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOOLS = os.path.expanduser('~/narsu/ros2/tools')

# 终端里 DISPLAY 常是空的（图形会话在 :1），这里自动补上，
# 否则 cv2/Qt 开窗口会失败，看起来就是“没弹窗口”。
DISP = os.environ.get('DISPLAY') or ':1'
ENV = {'DISPLAY': DISP}


def generate_launch_description():
    point = LaunchConfiguration('point')
    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')

    return LaunchDescription([
        DeclareLaunchArgument('point', default_value='A',
                              description='测哪个点位，如 A / B / C'),
        DeclareLaunchArgument('source', default_value='http://10.189.154.251:81/stream',
                              description='摄像头 MJPEG 流地址'),
        DeclareLaunchArgument('gain', default_value='2.0',
                              description='preview brightness, does not affect detection'),
        DeclareLaunchArgument('first_look', default_value='true',
                              description='false = always sweep from the side'),
        DeclareLaunchArgument('scan', default_value='true',
                              description='false = one look only, no sweep'),
        DeclareLaunchArgument('show', default_value='true',
                              description='是否弹识别/对准的预览窗口'),

        # 识别节点：判灯色，发 /vision/light_state
        Node(
            package='roscar_vision',
            executable='traffic_light_node',
            name='traffic_light',
            output='screen',
            additional_env=ENV,
            parameters=[{
                'source': source,
                'show': show,
                'detector': 'hue',
                'display_gain': LaunchConfiguration('gain'),
                'gated': True,
            }],
        ),

        # 扫视对准节点：收到 /aim_request 后转云台找 ArUco
        Node(
            package='roscar_patrol',
            executable='aim_marker',
            name='aim_marker',
            output='screen',
            additional_env=ENV,
            parameters=[{
                'source': source,
                'show': False,          # ??????????????????
                'scan_enable': LaunchConfiguration('scan'),
                'check_first': LaunchConfiguration('first_look'),
            }],
        ),

        # 编排脚本：等两个节点起来再开始，4 秒足够
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.join(TOOLS, 'aim_detect_test.py'), point],
                    output='screen',
                    emulate_tty=True,
                    additional_env=ENV,
                ),
            ],
        ),
    ])
