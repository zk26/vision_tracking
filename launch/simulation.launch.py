#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('vision_tracking')
    
    # 处理 Xacro 文件
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.xacro')
    robot_description = xacro.process_file(urdf_path).toxml()
    
    # 资源文件路径
    world_path = os.path.join(pkg_dir, 'worlds', 'empty.world')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'tracking.rviz')
    color_params = os.path.join(pkg_dir, 'params', 'color_params.yaml')
    
    # 基础环境变量
    env_vars = dict(os.environ)
    env_vars.update({
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'ROS_DOMAIN_ID': '0'
    })
    
    # 确保关键ROS变量存在
    required_vars = ['AMENT_PREFIX_PATH', 'ROS_DISTRO', 'PYTHONPATH']
    for var in required_vars:
        if var not in env_vars:
            print(f"WARNING: Required environment variable {var} not set")
            env_vars[var] = env_vars.get(var, '/opt/ros/humble')
    
    return LaunchDescription([
        # 启动 Gazebo 服务器
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path],
            output='screen',
            name='gazebo_server',
            env=env_vars
        ),
        
        # 机器人状态发布器 - 仅传递必要参数
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            # 不覆盖环境变量
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