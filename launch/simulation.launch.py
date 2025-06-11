import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    vision_tracking_pkg = get_package_share_directory('vision_tracking')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_description_pkg = get_package_share_directory('turtlebot3_description')
    
    # 获取世界文件路径
    world_path = os.path.join(tb3_gazebo_pkg, 'worlds', 'turtlebot3_world.world')
    
    # 加载URDF模型并解析
    urdf_path = os.path.join(tb3_description_pkg, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # 创建Gazebo进程
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        name='gazebo'
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )
    
    # 在Gazebo中生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-topic', '/robot_description'
        ],
        output='screen'
    )
    
    # 创建事件处理
    gazebo_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=3.0,  # 给Gazebo额外启动时间
                    actions=[
                        spawn_entity
                    ]
                )
            ]
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='Turtlebot3 model'
        ),
        
        gazebo,
        robot_state_publisher,
        gazebo_handler,
        
        # 在机器人生成后启动跟踪节点
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    # 启动您的跟踪节点
                    Node(
                        package='vision_tracking',
                        executable='camera_node',
                        name='camera_node',
                        output='screen'
                    ),
                    Node(
                        package='vision_tracking',
                        executable='ball_tracker',
                        name='ball_tracker',
                        output='screen'
                    ),
                    Node(
                        package='vision_tracking',
                        executable='commander',
                        name='commander',
                        output='screen'
                    ),
                    Node(
                        package='vision_tracking',
                        executable='drive_node',
                        name='drive_node',
                        output='screen'
                    )
                ]
            )
        )
    ])