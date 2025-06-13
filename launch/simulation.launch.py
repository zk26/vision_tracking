from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('vision_tracking')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'vision_tracking.rviz')

    return LaunchDescription([
        # Step 1: robot_state_publisher（必须最先）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': True}],
        ),

        # Step 2: Gazebo Server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        # Step 3: Gazebo Client，用 X11
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'}
        ),

        # Step 4: spawn_entity，加载机器人
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'vision_robot',
                 '-file', urdf_path],
            output='screen'
        ),

        # Step 5: 图像、深度、控制器
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[{'use_sim_time': True}]
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
            parameters=[{'use_sim_time': True}]
        ),

        # Step 6: 延迟启动 RViz（延迟 5 秒）
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],
                    additional_env={'QT_QPA_PLATFORM': 'xcb'}
                )
            ]
        )
    ])
