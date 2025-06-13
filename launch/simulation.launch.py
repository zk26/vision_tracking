#!/usr/bin/env python3
import os
import sys  # 新增：用于 SystemExit
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_tracking')
    
    # 定义资源路径查找函数（保持原逻辑）
    def find_resource(relative_path):
        # 优先检查安装路径（colcon install后）
        install_path = os.path.join(pkg_dir, '..', 'install', 'vision_tracking', relative_path)
        if os.path.exists(install_path):
            return install_path
        
        # 检查源路径（开发时）
        source_path = os.path.join(pkg_dir, relative_path)
        if os.path.exists(source_path):
            return os.path.abspath(source_path)
        
        # 未找到文件时抛出 SystemExit（关键修改）
        print(f"错误: 资源未找到: {relative_path}", file=sys.stderr)
        raise SystemExit(1)  # 直接终止进程，退出码1表示错误

    try:
        urdf_path = find_resource("urdf/robot.xacro")
        world_path = find_resource("worlds/empty.world")
        rviz_config = find_resource("rviz/tracking.rviz")
        color_params = find_resource("params/color_params.yaml")
    except SystemExit:
        # 资源加载失败时，launch 会捕获退出码并终止启动
        return LaunchDescription([])  # 返回空列表，不启动任何节点

    try:
        robot_description = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        print(f"URDF处理错误: {e}", file=sys.stderr)
        raise SystemExit(1)  # URDF处理失败时终止

    env_vars = dict(os.environ)
    env_vars.update({
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'ROS_DOMAIN_ID': '0'
    })

    return LaunchDescription([
        # 启动Gazebo服务器
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path],
            output='screen',
            name='gazebo_server',
            env=env_vars
        ),

        # 发布机器人状态（延迟3秒）
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
                    output='screen'
                )
            ]
        ),

        # 图像处理节点（传递颜色参数）
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[
                color_params,
                {'use_sim_time': True}
            ]
        ),

        # 深度处理节点
        Node(
            package='vision_tracking',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # 控制节点
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

        # RViz可视化（延迟10秒）
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                    parameters=[
                        {'use_sim_time': True}
                    ]
                )
            ]
        )
    ])