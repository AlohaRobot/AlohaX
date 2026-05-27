import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import rclpy
from rclpy.node import Node as RclpyNode
from visualization_msgs.msg import Marker

def generate_launch_description():
    # 获取包的目录
    bringup_dir = get_package_share_directory('alohax_urdf')
    
    # URDF 文件路径
    urdf_path = os.path.join(bringup_dir, 'urdf', 'alohax.urdf')
    
    # RViz 配置文件路径
    rviz_config_path = os.path.join(bringup_dir, 'rviz', 'urdf.rviz')
    
    # 读取 URDF 文件
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    # 创建节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    marker_publisher = Node(
        package='alohax_urdf',
        executable='marker_publisher.py',
        name='marker_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
        marker_publisher
    ])
