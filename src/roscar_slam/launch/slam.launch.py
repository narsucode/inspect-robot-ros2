import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_dir = get_package_share_directory('roscar_slam')
    rviz_cfg = os.path.join(slam_dir, 'rviz', 'slam.rviz')

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'mode': 'mapping'},
                {'scan_topic': '/scan'},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'base_footprint'},
                {'laser_frame': 'laser'},
                {'laser_min_range': 0.12},
                {'max_laser_range': 8.0},
                {'resolution': 0.05},
                {'minimum_travel_distance': 0.05},
                {'minimum_travel_heading': 0.1},
                {'minimum_time_interval': 0.2},
                {'map_update_interval': 1.0},
                {'transform_tolerance': 1.0},
                {'tf_buffer_duration': 30.0},
                {'use_scan_matching': True},
                {'use_scan_barycenter': True},
                {'do_loop_closing': True},
                {'loop_search_maximum_distance': 3.0},
            ],
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
