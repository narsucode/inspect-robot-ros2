import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('roscar_vision')
    rviz_cfg = os.path.join(pkg, 'rviz', 'traffic_light.rviz')

    source = LaunchConfiguration('source')
    show = LaunchConfiguration('show')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('source',
                              default_value='http://10.189.154.251:81/stream',
                              description='视觉模块 MJPEG 流地址'),
        DeclareLaunchArgument('show', default_value='true',
                              description='是否弹 OpenCV 窗口'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否同时开 rviz 看 /vision/debug_image'),

        Node(
            package='roscar_vision',
            executable='traffic_light_node',
            name='traffic_light',
            output='screen',
            parameters=[{
                'source': source,
                'show': show,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
