import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('alohax_hw_bridge'), 'config')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'port1',
            default_value='/dev/so101_L',
            description='Left bus port'
        ),
        DeclareLaunchArgument(
            'port2',
            default_value='/dev/so101_R',
            description='Right bus port'
        ),
        DeclareLaunchArgument(
            'calib_file',
            default_value=os.path.join(config_dir, 'alohax_calibration.yaml'),
            description='Calibration file path'
        ),
        DeclareLaunchArgument(
            'lift_travel_m',
            default_value='0.25',
            description='Left lift travel in meters'
        ),
        DeclareLaunchArgument(
            'lift_motor_degrees_per_meter',
            default_value='144000.0',
            description='Lift motor degrees per meter'
        ),
        Node(
            package='alohax_hw_bridge',
            executable='alohax_bridge',
            name='alohax_bridge',
            output='screen',
            parameters=[{
                'port1': LaunchConfiguration('port1'),
                'port2': LaunchConfiguration('port2'),
                'calib_file': LaunchConfiguration('calib_file'),
                'lift_travel_m': LaunchConfiguration('lift_travel_m'),
                'lift_motor_degrees_per_meter': LaunchConfiguration('lift_motor_degrees_per_meter'),
            }]
        ),
    ])
