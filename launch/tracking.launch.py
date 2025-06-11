import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_tracking')
    color_params = os.path.join(pkg_dir, 'params', 'color_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_color',
            default_value='red',
            description='Target color for tracking (red/blue/green/yellow)'
        ),
        
        # 图像处理节点
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[color_params, {'target_color': LaunchConfiguration('target_color')}]
        ),
        
        # 深度处理节点
        Node(
            package='vision_tracking',
            executable='depth_processor',
            name='depth_processor',
            output='screen'
        ),
        
        # 控制节点
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
        ),
        
        # RViz可视化
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(pkg_dir, 'rviz', 'tracking.rviz')],
            output='screen',
            name='rviz'
        )
    ])