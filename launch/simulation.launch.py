import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    world_file = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    
    return LaunchDescription([
        # 设置Turtlebot3型号
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='Turtlebot3 model (burger or waffle)'
        ),
        
        # 启动Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),
        
        # 启动机器人
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_simulation',
            name='turtlebot3_simulation',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model')
            }]
        ),
        
        # 在Gazebo中添加一个红色球体
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
                 '-entity', 'red_ball',
                 '-x', '0.5', '-y', '0.0', '-z', '0.2',
                 '-file', os.path.join(pkg_tb3_gazebo, 'models', 'red_ball', 'model.sdf')],
            output='screen'
        )
    ])