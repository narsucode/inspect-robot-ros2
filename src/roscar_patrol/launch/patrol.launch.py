"""patrol.launch.py —— 只启动巡检节点（无界面）。

什么时候用：
    调试巡检流程、或想让别的程序通过 /patrol_command 控制巡检时。
    界面版本见 patrol_gui.launch.py。

launch 文件的三件套（下面注释按这三步看）：
    1. DeclareLaunchArgument：声明“这个 launch 能接收哪些参数”，并给默认值；
    2. LaunchConfiguration   ：在文件里引用这些参数的值；
    3. Node                  ：真正要启动的进程，参数通过 parameters=[{...}] 传进去。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """launch 的入口函数，名字固定，返回一个 LaunchDescription。"""
    return LaunchDescription([
        # 巡检顺序：逗号分隔的点位名；留空则用 patrol.yaml 里的 order。
        DeclareLaunchArgument('patrol_order', default_value='',
                              description='逗号分隔的点位顺序，空=用 patrol.yaml'),

        # true 时不做真实导航，只跑 云台/识别/判定 流程，方便无车调试。
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true=不导航，只跑识别流程'),

        # 每个点位采样识别的时长（秒）。调大更稳，但一轮巡检更慢。
        DeclareLaunchArgument('dwell_sec', default_value='2.5'),

        # true：跳过云台与识别，只验证导航能不能到达各点位。
        DeclareLaunchArgument('skip_vision', default_value='false',
                              description='true=只跑导航，不转云台不识别'),

        # true：云台照转、到点存图，但不做颜色判定。
        DeclareLaunchArgument('capture_only', default_value='false',
                              description='true=只拍照不识别'),

        Node(
            package='roscar_patrol',        # 功能包名（package.xml 里的 name）
            executable='patrol_node',       # setup.py 里 entry_points 注册的可执行名
            name='patrol_node',             # ROS 图里显示的节点名
            output='screen',                # 日志直接打到终端
            parameters=[{
                # LaunchConfiguration 的值是字符串，节点里已按需转成 bool/float。
                'patrol_order': LaunchConfiguration('patrol_order'),
                'dry_run': LaunchConfiguration('dry_run'),
                'dwell_sec': LaunchConfiguration('dwell_sec'),
                'skip_vision': LaunchConfiguration('skip_vision'),
                'capture_only': LaunchConfiguration('capture_only'),
            }],
        ),
    ])
