import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument  # 添加声明参数
from launch.substitutions import LaunchConfiguration  # 添加配置

def generate_launch_description():
    package_dir = get_package_share_directory('vision_tracking')
    params_file = os.path.join(package_dir, 'params', 'color_params.yaml')
    
    # 添加可配置的目标颜色参数
    declare_target_arg = DeclareLaunchArgument(
        'target_color',
        default_value='red',
        description='Target color name (red/blue/green/yellow)')
    
    return LaunchDescription([
        declare_target_arg,
        
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[params_file, 
                       {'target_color': LaunchConfiguration('target_color')}]  # 使用参数
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
            parameters=[{
                'kp_linear': 0.3,
                'kp_angular': 0.8,
                'safe_distance': 0.5,
                'target_lost_timeout': 2.0,
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.0
            }]
        )
    ])