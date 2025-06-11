import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import TimerAction
import xacro

def generate_launch_description():
    # 获取包路径
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_description_pkg = get_package_share_directory('turtlebot3_description')
    
    # 获取世界文件路径
    world_path = os.path.join(tb3_gazebo_pkg, 'worlds', 'turtlebot3_world.world')
    
    # 加载URDF模型并解析
    urdf_path = os.path.join(tb3_description_pkg, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='Turtlebot3 model'
        ),
        
        # 启动Gazebo服务
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path,
                 '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so',
                 '--model-database', f'file://{tb3_description_pkg}/models'],
            output='screen',
            prefix='x-terminal-emulator -e',  # 强制在新终端打开
            name='gazebo'
        ),
        
        # 等待3秒后启动机器人状态发布器
        TimerAction(
            period=3.0,
            actions=[
                # 机器人状态发布器
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'robot_description': robot_desc
                    }]
                ),
                
                # 在Gazebo中生成机器人实体
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'turtlebot3_waffle',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.01',
                        '-topic', '/robot_description'
                    ],
                    output='screen',
                    name='spawn_entity'
                )
            ]
        ),
        
        # 等待10秒后启动跟踪节点
        TimerAction(
            period=10.0,
            actions=[
                # 启动您的跟踪节点
                Node(
                    package='vision_tracking',
                    executable='image_processor',
                    name='camera_node',
                    output='screen',
                    prefix='x-terminal-emulator -e'  # 在新终端打开
                ),
                Node(
                    package='vision_tracking',
                    executable='depth_processor',
                    name='ball_tracker',
                    output='screen',
                    prefix='x-terminal-emulator -e'  # 在新终端打开
                ),
                Node(
                    package='vision_tracking',
                    executable='controller',
                    name='commander',
                    output='screen',
                    prefix='x-terminal-emulator -e'  # 在新终端打开
                )
            ]
        )
    ])