"""alohax_bringup/launch/alohax.launch.py

整机一键启动：
  1. robot_state_publisher  ← 整机 URDF
  2. alohax_hw_bridge ← 双路 Feetech 总线桥接
  3. rviz2                  ← 可视化（可关闭）

用法示例：
  # 真实硬件（默认串口）
  ros2 launch alohax_bringup alohax.launch.py

  # 指定串口
  ros2 launch alohax_bringup alohax.launch.py port1:=/dev/ttyACM0 port2:=/dev/ttyACM1

  # 仅可视化（不连硬件）
  ros2 launch alohax_bringup alohax.launch.py hardware:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_pkg = get_package_share_directory("alohax_urdf")
    with open(os.path.join(urdf_pkg, "urdf", "alohax.urdf"), "r") as f:
        robot_desc = f.read()

    moveit_pkg = get_package_share_directory("alohax_moveit_config")
    srdf_file  = os.path.join(moveit_pkg, "config", "alohax.srdf")
    with open(srdf_file, "r") as f:
        robot_desc_semantic = f.read()

    hw_pkg       = get_package_share_directory("alohax_hw_bridge")
    default_calib = os.path.join(hw_pkg, "config", "alohax_calibration.yaml")

    hardware   = LaunchConfiguration("hardware")
    gui        = LaunchConfiguration("gui")
    port1      = LaunchConfiguration("port1")
    port2      = LaunchConfiguration("port2")
    calib_file = LaunchConfiguration("calib_file")
    publish_hz = LaunchConfiguration("publish_hz")
    lift_travel_m = LaunchConfiguration("lift_travel_m")
    lift_motor_degrees_per_meter = LaunchConfiguration("lift_motor_degrees_per_meter")
    rviz       = LaunchConfiguration("rviz")
    moveit    = LaunchConfiguration("moveit")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"publish_robot_description": True},
        ],
        remappings=[("joint_states", "/xlerobot/joint_states")],
    )

    # hardware:=false 时用 joint_state_publisher 提供默认关节状态
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
        remappings=[("joint_states", "/xlerobot/joint_states")],
        condition=UnlessCondition(hardware),
    )

    hw_bridge = Node(
        package="alohax_hw_bridge",
        executable="alohax_bridge",
        name="alohax_bridge",
        output="screen",
        parameters=[{
            "port1":      port1,
            "port2":      port2,
            "calib_file": calib_file,
            "publish_hz": publish_hz,
            "lift_travel_m": lift_travel_m,
            "lift_motor_degrees_per_meter": lift_motor_degrees_per_meter,
        }],
        condition=IfCondition(hardware),
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
        output="screen",
        remappings=[("joint_states", "/xlerobot/joint_states")],
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("alohax_bringup"), "rviz", "xlerobot_moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="log",
        parameters=[{
            "robot_description": robot_desc,
            "robot_description_semantic": robot_desc_semantic,
        }],
        remappings=[("joint_states", "/xlerobot/joint_states")],
        condition=IfCondition(rviz),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("alohax_moveit_config"), "launch", "move_group.launch.py")
        ),
        condition=IfCondition(moveit),
    )

    delayed_move_group_launch = TimerAction(
        period=5.0,
        actions=[move_group_launch],
        condition=IfCondition(moveit),
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware",    default_value="true"),
        DeclareLaunchArgument("gui",         default_value="false"),
        DeclareLaunchArgument("moveit",      default_value="true"),
        DeclareLaunchArgument("port1",       default_value="/dev/so101_L"),
        DeclareLaunchArgument("port2",       default_value="/dev/so101_R"),
        DeclareLaunchArgument("calib_file",  default_value=default_calib),
        DeclareLaunchArgument("publish_hz",  default_value="50.0"),
        DeclareLaunchArgument("lift_travel_m", default_value="0.25"),
        DeclareLaunchArgument("lift_motor_degrees_per_meter", default_value="144000.0"),
        DeclareLaunchArgument("rviz",        default_value="true"),

        rsp,
        jsp,
        hw_bridge,
        jsp_gui,
        delayed_move_group_launch,
        rviz_node,
    ])
