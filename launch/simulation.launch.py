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

    try:
        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()
    except Exception as e:
        robot_desc = ''
        print(f"[ERROR] 读取URDF文件失败: {e}")

    return LaunchDescription([
        # 1. 启动 robot_state_publisher 发布 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }],
        ),

        # 2. 启动 Gazebo 服务器，加载空世界
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        # 3. 启动 Gazebo 客户端，用于可视化
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'}
        ),

        # 4. 延迟6秒启动 spawn_entity，确保 Gazebo 和 robot_description 准备好
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                         '-entity', 'minimal_robot',
                         '-topic', 'robot_description'],
                    output='screen'
                )
            ]
        ),

        # 5. 启动图像处理器节点
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 6. 启动深度处理器节点
        Node(
            package='vision_tracking',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 7. 启动控制器节点
        Node(
            package='vision_tracking',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 8. 延迟5秒启动 RViz，确保模型加载完成
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
