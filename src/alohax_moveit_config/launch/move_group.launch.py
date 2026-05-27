"""alohax_moveit_config/launch/move_group.launch.py"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def to_space_separated_string(value):
    if isinstance(value, (list, tuple)):
        return " ".join(str(item) for item in value)
    if value is None:
        return ""
    return str(value)


def generate_launch_description():
    pkg_urdf   = get_package_share_directory("alohax_urdf")
    pkg_moveit = get_package_share_directory("alohax_moveit_config")

    with open(os.path.join(pkg_urdf, "urdf", "alohax.urdf"), "r") as f:
        robot_desc = f.read()

    with open(os.path.join(pkg_moveit, "config", "alohax.srdf"), "r") as f:
        robot_desc_semantic = f.read()

    kinematics   = load_yaml(os.path.join(pkg_moveit, "config", "kinematics.yaml"))
    joint_limits = load_yaml(os.path.join(pkg_moveit, "config", "joint_limits.yaml"))
    controllers  = load_yaml(os.path.join(pkg_moveit, "config", "moveit_controllers.yaml"))

    ompl_yaml = load_yaml(os.path.join(pkg_moveit, "config", "ompl_planning.yaml"))
    ompl_yaml.setdefault("planning_plugin", "ompl_interface/OMPLPlanner")
    ompl_yaml["request_adapters"] = to_space_separated_string(ompl_yaml.get("request_adapters"))

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"robot_description_semantic": robot_desc_semantic},
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning": joint_limits},
            controllers,
            {"ompl": ompl_yaml},
            # 明确指定使用 OMPL 规划管道
            {"use_sim_time": False},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            {"monitor_dynamics": False},
            # 放宽碰撞检测容差，允许起始状态有轻微碰撞
            {"default_constrained_link_safety_margin": 0.05},
            {"default_link_padding": 0.02},
            # 指定 joint_states topic，使 current_state_monitor 能正确订阅
            {"joint_states_topic": "/xlerobot/joint_states"},
            # 增加轨迹执行等待时间，确保能接收到有效的机器人状态
            {"trajectory_execution/allowed_start_tolerance": 0.5},
            {"trajectory_execution/allowed_goal_duration_margin": 5.0},
            # 指定默认规划器为 RRTConnect
            {"default_planning_pipeline": "ompl"},
            {"planning_pipelines": ["ompl"]},
            # 禁用 CHOMP 规划器加载
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
        ],
        remappings=[
            ("joint_states", "/xlerobot/joint_states"),
            ("robot_state", "/xlerobot/robot_state"),
        ],
    )

    return LaunchDescription([move_group])
