"""led_demo.launch.py —— 一条命令演示「云台巡视 + 直接识别灯色」。

三个进程：
    ptz_hold.py         云台固定角度 / 左右扫视
    traffic_light_node  拉流 + 判灯色，弹窗（框住灯 + 码）
    led_monitor.py      终端实时报出颜色变化

用法：
    ros2 launch roscar_patrol led_demo.launch.py sweep:=35          # 左右扫 ±35°
    ros2 launch roscar_patrol led_demo.launch.py pan:=3.3 tilt:=0.1 # 锁在固定角度
    ros2 launch roscar_patrol led_demo.launch.py                     # 云台不动
    ros2 launch roscar_patrol led_demo.launch.py sweep:=35 gain:=2.2
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOOLS = os.path.expanduser('~/narsu/ros2/tools')
DISP = os.environ.get('DISPLAY') or ':1'
ENV = {'DISPLAY': DISP}


def generate_launch_description():
    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')
    gain = LaunchConfiguration('gain')
    pan = LaunchConfiguration('pan')
    tilt = LaunchConfiguration('tilt')
    sweep = LaunchConfiguration('sweep')
    speed = LaunchConfiguration('speed')

    # ptz_hold 的参数：扫视优先，其次固定角度，都没有就空转
    ptz_cmd = ['python3', os.path.join(TOOLS, 'ptz_hold.py'),
               pan, tilt,
               '--sweep', sweep,
               '--speed', speed]

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='http://10.189.154.251:81/stream'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('gain', default_value='1.6'),
        DeclareLaunchArgument('pan', default_value='0'),
        DeclareLaunchArgument('tilt', default_value='0'),
        DeclareLaunchArgument('sweep', default_value='0',
                              description='扫视幅度（度），0=不扫视'),
        DeclareLaunchArgument('speed', default_value='20',
                              description='扫视角速度 度/秒'),

        Node(
            package='roscar_vision',
            executable='traffic_light_node',
            name='traffic_light',
            output='screen',
            additional_env=ENV,
            parameters=[{'source': source, 'show': show, 'detector': 'hue',
                         'display_gain': gain}],
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=ptz_cmd,
                    output='screen',
                    emulate_tty=True,
                    additional_env=ENV,
                ),
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.join(TOOLS, 'led_monitor.py')],
                    output='screen',
                    emulate_tty=True,
                    additional_env=ENV,
                ),
            ],
        ),
    ])
