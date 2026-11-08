import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取功能包路径
    desc_dir = get_package_share_directory('roscar_description')
    bringup_dir = get_package_share_directory('roscar_bringup')
    
    # 读取机器人URDF模型
    urdf_path = os.path.join(desc_dir, 'urdf', 'roscar.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    # 是否启动RVIZ参数
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_ekf = LaunchConfiguration('use_ekf', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='false',
                              description='true=用 EKF 融合；false=直接用 /odom 发 TF'),
        
        # 根据URDF文件发布机器人的TF变换
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # # IMU姿态滤波融合
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter',
        #     output='screen',
        #     parameters=[{
        #         'use_mag': False,
        #         'publish_tf': False,
        #         'world_frame': 'enu',
        #         'frequency': 100.0,
        #         'gain': 0.1,
        #     }],
        # ),
        
        # EKF里程融合滤波
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(use_ekf),
            parameters=[os.path.join(bringup_dir, 'config', 'ekf.yaml')]
        ),

        # use_ekf:=false 时，直接把 /odom 转成 odom -> base_footprint
        Node(
            package='roscar_bringup',
            executable='odom2tf',
            name='odom2tf',
            output='screen',
            condition=UnlessCondition(use_ekf)
        ),
        
        # # RVIZ可视化，条件启动
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     condition=IfCondition(use_rviz)
        # ),
    ])