"""point_test.launch.py —— 一条命令测「转云台 + 识别灯色」，不导航。

只起两个进程：
    traffic_light_node  拉流 + 判灯色（默认不弹窗）
    point_test.py       读 patrol.yaml 的角度 → 转云台 → 采 2.5 秒 → 报结果

用法：
    ros2 launch roscar_patrol point_test.launch.py
    ros2 launch roscar_patrol point_test.launch.py point:=B
    ros2 launch roscar_patrol point_test.launch.py point:=B pan:=12.0 tilt:=-4.0   # 临时试角度
    ros2 launch roscar_patrol point_test.launch.py show:=true                       # 弹识别窗口
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOOLS = os.path.expanduser('~/narsu/ros2/tools')
DISP = os.environ.get('DISPLAY') or ':1'
ENV = {'DISPLAY': DISP}


def generate_launch_description():
    point = LaunchConfiguration('point')
    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')
    pan = LaunchConfiguration('pan')
    tilt = LaunchConfiguration('tilt')

    return LaunchDescription([
        DeclareLaunchArgument('point', default_value='A'),
        DeclareLaunchArgument('source', default_value='http://10.189.154.251:81/stream'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('pan', default_value=''),
        DeclareLaunchArgument('tilt', default_value=''),

        Node(
            package='roscar_vision',
            executable='traffic_light_node',
            name='traffic_light',
            output='screen',
            additional_env=ENV,
            parameters=[{'source': source, 'show': show, 'detector': 'hue'}],
        ),

        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.join(TOOLS, 'point_test.py'), point,
                         '--pan', pan, '--tilt', tilt],
                    output='screen',
                    emulate_tty=True,
                    additional_env=ENV,
                ),
            ],
        ),
    ])
