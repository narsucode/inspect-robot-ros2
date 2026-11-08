"""patrol_gui.launch.py —— 一键启动：识别节点 + 巡检节点 + 图形界面。

启动后各进程的关系：
    roscar_vision/traffic_light_node  → 持续发布 /vision/light_state
    roscar_patrol/aim_marker          → 收到 /aim_request 后扫视找二维码，回报角度
    roscar_patrol/patrol_node         → 导航 + 云台 + 判定
    roscar_patrol/patrol_gui          → 订阅 /patrol_status，按钮发 /patrol_command

常见用法：
    ros2 launch roscar_patrol patrol_gui.launch.py
    ros2 launch roscar_patrol patrol_gui.launch.py camera:=false        # 只开界面，不启识别
    ros2 launch roscar_patrol patrol_gui.launch.py aim:=off             # 不做扫视对准
    ros2 launch roscar_patrol patrol_gui.launch.py source:=http://192.168.5.1/stream
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 终端里 DISPLAY 常是空的（图形会话在 :1），这里自动补上，
# 否则识别窗口和 Qt 界面都起不来。
DISP = os.environ.get('DISPLAY') or ':1'
ENV = {'DISPLAY': DISP}


def generate_launch_description():
    return LaunchDescription([
        # true：巡检只走识别流程、不真的开车，用来验证云台和识别。
        DeclareLaunchArgument('dry_run', default_value='false'),

        # 是否同时启动视觉识别节点；只想调界面时传 false。
        DeclareLaunchArgument('camera', default_value='true',
                              description='是否同时启动识别节点'),

        # true：巡检只开车不识别（云台也不动），用来验收导航。
        DeclareLaunchArgument('skip_vision', default_value='false',
                              description='true=只跑导航，不转云台不识别'),

        # 巡检顺序：逗号分隔的点位名，留空则用 patrol.yaml 里的 order。
        DeclareLaunchArgument('patrol_order', default_value='',
                              description='逗号分隔的点位顺序，如 B,A,D,C；空=用 patrol.yaml'),

        # true：云台照转、到点存图，但不做颜色判定。
        DeclareLaunchArgument('capture_only', default_value='false',
                              description='true=只拍照不识别'),

        # false：视觉节点只取流、不识别（给 capture_only 用）。
        DeclareLaunchArgument('detect', default_value='true',
                              description='false=视觉节点不判色'),

        # 摄像头流地址：ESP32-S3-Cam 的热点默认是 192.168.5.1，
        # 之前的手机 IP 摄像头是 http://192.168.70.251:81/stream。
        DeclareLaunchArgument('source',
                              default_value='http://10.189.154.251:81/stream'),

        # marker：到点先让云台扫视找二维码对准，再识别；off：直接用配置角度。
        DeclareLaunchArgument('aim', default_value='marker',
                              description='marker=扫视找二维码对准，off=用固定角度'),

        # true=先试 patrol.yaml 里记住的角度（快）；false=每次都完整扫一遍（不看记忆）
        DeclareLaunchArgument('preset_first', default_value='false',
                              description='true=先试记住的角度，false=每次全扫'),

        # 识别节点：show=False 表示不弹 OpenCV 窗口，画面交给 GUI 显示。
        DeclareLaunchArgument('scan', default_value='true',
                              description='false=no sweep, only one look'),

        DeclareLaunchArgument('strict', default_value='true',
                              description='true=check code id matches point, false=accept any'),

        Node(
            package='roscar_vision',
            executable='traffic_light_node',
            name='traffic_light',
            output='screen',
            additional_env=ENV,
            condition=IfCondition(LaunchConfiguration('camera')),
            parameters=[{
                'show': False,
                'source': LaunchConfiguration('source'),
                'detect': LaunchConfiguration('detect'),
                'gated': True,
            }],
        ),

        # 扫视对准节点：常驻但不主动做事，只有巡检节点发 /aim_request 它才扫。
        # 不想用它只需 aim:=off（巡检节点就不会发请求）。
        Node(
            package='roscar_patrol',
            executable='aim_marker',
            name='aim_marker',
            output='screen',
            additional_env=ENV,
            parameters=[{
                'source': LaunchConfiguration('source'),
                'preset_first': LaunchConfiguration('preset_first'),
                'strict_code': LaunchConfiguration('strict'),
                'scan_enable': LaunchConfiguration('scan'),
                'show': False,
            }],
        ),

        # 巡检节点：auto_start=False，启动后先待命，等界面上的“开始”按钮。
        Node(
            package='roscar_patrol',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            additional_env=ENV,
            parameters=[{
                'auto_start': False,
                'dry_run': LaunchConfiguration('dry_run'),
                'skip_vision': LaunchConfiguration('skip_vision'),
                'capture_only': LaunchConfiguration('capture_only'),
                'patrol_order': LaunchConfiguration('patrol_order'),
                'aim_mode': LaunchConfiguration('aim'),
            }],
        ),

        # 图形界面本身也是一个节点，只有它需要显示窗口（DISPLAY）。
        Node(
            package='roscar_patrol',
            executable='patrol_gui',
            name='patrol_gui',
            output='screen',
            additional_env=ENV,
        ),
    ])
