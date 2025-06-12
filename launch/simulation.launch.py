#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 获取包路径 (使用可靠方法)
    pkg_dir = get_package_share_directory('vision_tracking')
    
    # 查找资源文件 (优先使用安装路径)
    def find_resource(relative_path):
        # 检查安装目录
        install_path = os.path.join(pkg_dir, relative_path)
        if os.path.exists(install_path):
            return install_path
        
        # 检查源目录
        source_path = os.path.join(os.path.dirname(__file__), "../", relative_path)
        if os.path.exists(source_path):
            return os.path.abspath(source_path)
        
        # 未找到文件
        raise FileNotFoundError(f"找不到资源文件: {relative_path}")
    
    # 获取资源文件
    try:
        urdf_path = find_resource("urdf/robot.xacro")
        world_path = find_resource("worlds/empty.world")
        rviz_config = find_resource("rviz/tracking.rviz")
        color_params = find_resource("params/color_params.yaml")
    except FileNotFoundError as e:
        print(f"错误: {e}")
        exit(1)
    
    # 处理 Xacro 文件
    try:
        robot_description = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        print(f"URDF处理错误: {e}")
        exit(1)
    
    # 基础环境变量
    env_vars = dict(os.environ)
    env_vars.update({
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'ROS_DOMAIN_ID': '0'
    })
    
    return LaunchDescription([
        # 启动 Gazebo 服务器
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path],
            output='screen',
            name='gazebo_server',
            env=env_vars
        ),
        
        # 机器人状态发布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            env=env_vars
        ),
        
        # 生成机器人实体 (延迟3秒)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_robot',
                    arguments=[
                        '-entity', 'vision_tracking_robot',
                        '-topic', 'robot_description',
                        '-x', '0.0', '-y', '0.0', '-z', '0.1',
                        '-Y', '0.0'
                    ],
                    output='screen',
                    # 不覆盖环境变量
                )
            ]
        ),
        
        # 图像处理节点
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[color_params]
        ),
        
        # 深度处理节点
        Node(
            package='vision_tracking',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 控制器节点
        Node(
            package='vision_tracking',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'kp_linear': 0.3},
                {'kp_angular': 0.8},
                {'safe_distance': 0.5},
                {'target_lost_timeout': 2.0},
                {'max_linear_vel': 0.5},
                {'max_angular_vel': 1.0}
            ]
        ),
        
        # RViz 可视化 (延迟10秒)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        )
    ])