import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tb3_pkg = 'turtlebot3_gazebo'
    world_path = os.path.join(get_package_share_directory(tb3_pkg), 'worlds', 'turtlebot3_world.world')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='Turtlebot3 model'
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_simulation',
            name='turtlebot3_simulation',
            output='screen',
            parameters=[{'model': LaunchConfiguration('model')}]
        )
    ])