import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():                                       
    roscar_navigation_dir = get_package_share_directory('roscar_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')          
    
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = launch.substitutions.LaunchConfiguration(               
        'map', default=os.path.join(roscar_navigation_dir, 'maps', 'room_20261108_demo.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(             
        'params_file', default=os.path.join(roscar_navigation_dir, 'config', 'nav2_params.yaml'))
    
    # 是否启动 RViz，默认 true
    use_rviz = launch.substitutions.LaunchConfiguration('use_rviz', default='true')
    # 默认使用 Nav2 自带的 rviz 配置
    rviz_config = launch.substitutions.LaunchConfiguration(
        'rviz_config', 
        default=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path),       
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path), 
        launch.actions.DeclareLaunchArgument('use_rviz', default_value=use_rviz),
        launch.actions.DeclareLaunchArgument('rviz_config', default_value=rviz_config),

        # Nav2导航核心
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={                                             
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
        ),
        
        # RViz2 可视化（默认启动）
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])