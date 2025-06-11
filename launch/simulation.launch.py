import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 获取当前功能包路径
    pkg_dir = get_package_share_directory('vision_tracking')
    
    # 使用项目内部的URDF文件
    robot_urdf = os.path.join(pkg_dir, 'urdf', 'camera.urdf')  # 使用存在的camera.urdf
    
    # 检查URDF文件是否存在，如果不存在则使用备用方案
    if not os.path.exists(robot_urdf):
        # 创建简单的备用URDF描述
        robot_desc = '''
            <robot name="simple_robot">
                <link name="base_link">
                    <visual>
                        <geometry>
                            <box size="0.3 0.3 0.1"/>
                        </geometry>
                        <material name="blue">
                            <color rgba="0 0 0.8 1"/>
                        </material>
                    </visual>
                </link>
            </robot>
        '''
    else:
        with open(robot_urdf, 'r') as f:
            robot_desc = f.read()
    
    # 使用Gazebo默认世界文件
    world_path = os.path.join('/opt/ros/humble/share', 'turtlebot3_gazebo', 'worlds', 'empty.world')
    
    return LaunchDescription([
        # 设置必要的环境变量
        ExecuteProcess(
            cmd=['printenv', 'GAZEBO_MODEL_PATH'],
            output='screen'
        ),
        
        # 启动Gazebo服务
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            name='gazebo_server'
        ),
        
        # 启动机器人状态发布器 (5秒后启动)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'robot_description': robot_desc
                    }]
                )
            ]
        ),
        
        # 在Gazebo中生成机器人实体 (7秒后启动)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'vision_robot',
                        '-x', '0.0', '-y', '0.0', '-z', '0.01',
                        '-topic', '/robot_description'  # 使用话题而不是文件路径
                    ],
                    output='screen',
                    name='spawn_robot'
                )
            ]
        ),
        
        # 启动视觉跟踪节点 (10秒后启动)
        TimerAction(
            period=10.0,
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
                    output='screen'
                ),
                Node(
                    package='vision_tracking',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[
                        {'kp_linear': 0.3, 
                         'kp_angular': 0.8,
                         'safe_distance': 0.5,
                         'max_linear_vel': 0.5,
                         'max_angular_vel': 1.0}
                    ]
                )
            ]
        )
    ])