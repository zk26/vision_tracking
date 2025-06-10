import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('vision_tracking')
    params_file = os.path.join(package_dir, 'params', 'color_params.yaml')
    rviz_config = os.path.join(package_dir, 'rviz', 'tracking.rviz')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'target_color',
            default_value='red',
            description='Target object color (red, blue, green, etc.)'
        ),
        
        # 图像处理节点 - 直接加载参数文件
        Node(
            package='vision_tracking',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[params_file, {'target_color': LaunchConfiguration('target_color')}]
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
            parameters=[{
                'kp_linear': 0.3,
                'kp_angular': 0.8,
                'safe_distance': 0.5,
                'target_lost_timeout': 2.0,
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.0
            }]
        ),
        
        # RViz可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])