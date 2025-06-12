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
        return LaunchDescription([Abort(reason=str(e))])

    try:
        robot_description = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        return LaunchDescription([Abort(reason=f"URDF处理失败: {e}")])

    env_vars = dict(os.environ)
    env_vars.update({
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'ROS_DOMAIN_ID': '0'
    })

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path],
            output='screen',
            name='gazebo_server',
            env=env_vars
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True}
            ],
            env=env_vars
        ),

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

        Node(
            package='vision_tracking',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

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