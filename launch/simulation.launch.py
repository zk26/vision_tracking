#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, Abort
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_tracking')
    
    def find_resource(relative_path):
        # 优先检查安装路径（colcon install后）
        install_path = os.path.join(pkg_dir, '..', 'install', 'vision_tracking', relative_path)
        if os.path.exists(install_path):
            return install_path
        
        # 检查源路径（开发时）
        source_path = os.path.join(pkg_dir, relative_path)
        if os.path.exists(source_path):
            return os.path.abspath(source_path)
        
        raise FileNotFoundError(f"资源未找到: {relative_path}")

    try:
        urdf_path = find_resource("urdf/robot.xacro")
        world_path = find_resource("worlds/empty.world")
        rviz_config = find_resource("rviz/tracking.rviz")
        color_params = find_resource("params/color_params.yaml")
    except FileNotFoundError as e:
        return LaunchDescription([Abort(reason=f"资源未找到: {str(e)}")])

    try:
        robot_description = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        return LaunchDescription([Abort(reason=f"URDF处理失败: {e}")])

    env_vars = dict(os.environ)
    env_vars.update({
        'LIBGL_ALWAYS_SOFTWARE': '1',  # 解决Gazebo渲染问题
        'ROS_DOMAIN_ID': '0'          # 避免多机器人干扰
    })

    return LaunchDescription([
        # 启动Gazebo服务器
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path],
            output='screen',
            name='gazebo_server',
            env=env_vars
        ),

        # 发布机器人状态
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True}  # 使用仿真时间
            ],
            env=env_vars
        ),

        # 延迟生成机器人实体（等待Gazebo初始化）
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_robot',
                    arguments=[
                        '-entity', 'vision_tracking_robot',  # 机器人实体名
                        '-topic', 'robot_description',       # 从robot_description话题获取URDF
                        '-x', '0.0', '-y', '0.0', '-z', '0.1',  # 初始位置（z=0.1避免地面穿透）
                        '-Y', '0.0'                           # 初始朝向（无旋转）
                    ],
                    output='screen'
                )
            ]
        ),

        # 图像处理节点（依赖颜色参数）
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[
                color_params,  # 加载颜色阈值参数
                {'use_sim_time': True}  # 使用仿真时间
            ]
        ),

        # 深度处理节点（依赖仿真时间）
        Node(
            package='vision_tracking',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[
                {'use_sim_time': True}  # 使用仿真时间
            ]
        ),

        # 控制节点（依赖仿真时间和参数）
        Node(
            package='vision_tracking',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # 使用仿真时间
                {'kp_linear': 0.3},      # 线速度比例系数
                {'kp_angular': 0.8},     # 角速度比例系数
                {'safe_distance': 0.5},  # 安全距离（米）
                {'target_lost_timeout': 2.0},  # 目标丢失超时时间（秒）
                {'max_linear_vel': 0.5},  # 最大线速度（米/秒）
                {'max_angular_vel': 1.0}  # 最大角速度（弧度/秒）
            ]
        ),

        # RViz可视化（延迟启动确保节点就绪）
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],  # 加载RViz配置
                    output='screen',
                    parameters=[
                        {'use_sim_time': True}  # 使用仿真时间
                    ]
                )
            ]
        )
    ])