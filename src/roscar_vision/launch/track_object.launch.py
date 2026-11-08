import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('roscar_vision')
    rviz_cfg = os.path.join(pkg, 'rviz', 'traffic_light.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('source',
                              default_value='http://10.143.115.211:81/stream'),
        DeclareLaunchArgument('target', default_value='red',
                              description='red / yellow / green / bright'),
        DeclareLaunchArgument('kp', default_value='0.04'),
        DeclareLaunchArgument('deadband', default_value='25'),
        DeclareLaunchArgument('show', default_value='true'),

        Node(
            package='roscar_vision',
            executable='object_track_node',
            name='object_track',
            output='screen',
            parameters=[{
                'source': LaunchConfiguration('source'),
                'target': LaunchConfiguration('target'),
                'kp': LaunchConfiguration('kp'),
                'deadband': LaunchConfiguration('deadband'),
                'show': LaunchConfiguration('show'),
            }],
        ),
    ])
