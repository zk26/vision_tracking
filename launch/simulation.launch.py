import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_tracking')
    
    # 设置机器人URDF路径
    robot_urdf = os.path.join(pkg_dir, 'urdf', 'simple_robot.urdf')
    
    # 创建简单的URDF文件作为备用
    simple_urdf = os.path.join(pkg_dir, 'urdf', 'simple_robot.urdf')
    with open(simple_urdf, 'w') as f:
        f.write('''
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
    <collision>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
        ''')
    
    return LaunchDescription([
        # 设置环境变量
        ExecuteProcess(
            cmd=['printenv', 'GAZEBO_MODEL_PATH'],
            output='screen'
        ),
        
        # 启动Gazebo服务
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/usr/share/gazebo-11/worlds/empty.world',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            name='gazebo_server'
        ),
        
        # 等待2秒后生成机器人实体
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'simple_robot',
                        '-file', simple_urdf,
                        '-x', '0.0', '-y', '0.0', '-z', '0.5',
                        '-Y', '0.0'
                    ],
                    output='screen',
                    name='spawn_robot'
                )
            ]
        ),
        
        # 启动视觉跟踪节点 (5秒后启动)
        TimerAction(
            period=5.0,
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
                        {'use_sim_time': True},
                        {'kp_linear': 0.3}, 
                        {'kp_angular': 0.8},
                        {'safe_distance': 0.5},
                        {'max_linear_vel': 0.5},
                        {'max_angular_vel': 1.0}
                    ]
                )
            ]
        )
    ])