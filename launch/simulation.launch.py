#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction, RegisterEventHandler,
                            DeclareLaunchArgument)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  Command, PythonExpression)
from launch_ros.actions import Node
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('vision_tracking')
    
    # 设置环境变量解决Gazebo问题
    env_vars = {
        'GAZEBO_MODEL_PATH': os.path.join(os.environ['HOME'], '.gazebo/models'),
        'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11',
        'LIBGL_ALWAYS_SOFTWARE': '1',  # 解决渲染问题
        'MESA_GL_VERSION_OVERRIDE': '3.3'  # 兼容性设置
    }
    
    # 清理命令 (确保没有残留进程)
    cleanup_commands = [
        'pkill -f gzserver',
        'pkill -f gzclient',
        'rm -rf ~/.gazebo/lock',
        'find /dev/shm -name "rtps_*" -delete'
    ]
    
    # 机器人xacro文件路径
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.xacro')
    
    # 使用xacro处理机器人描述
    robot_description = xacro.process_file(xacro_path).toxml()
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('world', default_value='/usr/share/gazebo-11/worlds/empty.world',
                              description='Gazebo world file'),
        
        # 执行清理命令
        ExecuteProcess(
            cmd=['bash', '-c', '; '.join(cleanup_commands)],
            output='screen'
        ),
        
        # 启动Gazebo服务器
        ExecuteProcess(
            cmd=['gazebo', '--verbose', LaunchConfiguration('world'),
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            name='gazebo_server',
            env=env_vars
        ),
        
        # 启动Gazebo客户端
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            name='gazebo_client',
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
            }]
        ),
        
        # 在Gazebo中生成机器人实体
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-entity', 'vision_tracking_robot',
                '-topic', 'robot_description',  # 从话题获取URDF
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen',
            env=env_vars
        ),
        
        # 视觉处理节点 (5秒后启动)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='vision_tracking',
                    executable='image_processor',
                    name='image_processor',
                    output='screen',
                    parameters=[os.path.join(pkg_dir, 'params', 'color_params.yaml')]
                ),
                Node(
                    package='vision_tracking',
                    executable='depth_processor',
                    name='depth_processor',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
                Node(
                    package='vision_tracking',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True,
                         'kp_linear': 0.3, 
                         'kp_angular': 0.8,
                         'safe_distance': 0.5,
                         'target_lost_timeout': 2.0,
                         'max_linear_vel': 0.5,
                         'max_angular_vel': 1.0}
                    ]
                )
            ]
        )
    ])