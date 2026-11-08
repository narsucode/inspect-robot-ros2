# 只加载地图并显示：给标点、看地图用，不需要机器人、不需要导航栈。
#
# 用法：
#   ros2 launch roscar_navigation map_only.launch.py \
#       map:=$HOME/narsu/ros2/patrol_ws/src/roscar_navigation/maps/room_20261101.yaml
#
# 不传 map 时默认加载 maps/room.yaml。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory('roscar_navigation'), 'maps', 'room_20261108_demo.yaml')
    nav2_rviz = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='要加载的地图 yaml 路径'),
        # 标点专用 RViz 配置：比 nav2 默认多一个 “2D Goal Pose” 工具（发 /goal_pose）
        DeclareLaunchArgument('rviz_config',
                              default_value='/home/ros/narsu/ros2/tools/mark.rviz'),

        # 地图服务器：读 yaml + pgm，发布 /map（latched）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}],
            output='screen'
        ),

        # map_server 是生命周期节点，必须被激活才会真正发布地图
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
            output='screen'
        ),

        # 没有导航栈时 TF 里没有 map 帧，RViz 会报 Fixed Frame 不存在。
        # 这里补一个 map -> odom 的单位变换，只为显示用。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # RViz 用 nav2 的配置，Fixed Frame 默认就是 map
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        ),
    ])
