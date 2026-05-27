#!/usr/bin/env python3
"""alohax_hw_bridge/alohax_bridge.py

AlohaX 统一硬件桥接节点：
  bus1 (/dev/so101_L): 左臂(ID1-6) + 驱动轮(ID7-8) + 升降(ID11)
  bus2 (/dev/so101_R): 右臂(ID1-6)

硬件变更说明：
1. 两个机械臂从肩膀上面变更为安装在肩膀下面，0位时自然下垂
2. 增加了一个升降自由度在左臂，st3215控制，id11
3. 两个驱动后轮更换到左臂控制，st3215控制，id7,8
4. 取消所有头部电机，头部相机固定

发布话题:
  /xlerobot/joint_states  (sensor_msgs/JointState) ← 整机实际状态

订阅话题：
  /joint_states                      ← joint_state_publisher_gui 滑块指令
  /xlerobot/left_arm/joint_commands  ← 左臂精细指令
  /xlerobot/right_arm/joint_commands ← 右臂精细指令
  /xlerobot/base/cmd_vel             ← 底盘 Twist 指令
"""

from __future__ import annotations

import math
import pathlib
import sys
import threading
import time
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

for _mod, _attr, _val in [("deepdiff", "DeepDiff", dict), ("tqdm", "tqdm", lambda it=None, **k: it or [])]:
    try:
        __import__(_mod)
    except ImportError:
        import types
        _m = types.ModuleType(_mod)
        setattr(_m, _attr, _val)
        sys.modules[_mod] = _m

from alohax_hw_bridge.motors.feetech.feetech import FeetechMotorsBus, OperatingMode
from alohax_hw_bridge.motors import Motor, MotorNormMode
from alohax_hw_bridge.motors_bus import MotorCalibration

# AlohaX 电机配置
# bus1: 左臂(ID1-6) + 驱动轮(ID7-8) + 升降(ID11)
BUS1_MOTORS = {
    "left_arm_shoulder_pan":  Motor(1, "sts3215", MotorNormMode.DEGREES),
    "left_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "left_arm_elbow_flex":    Motor(3, "sts3215", MotorNormMode.DEGREES),
    "left_arm_wrist_flex":    Motor(4, "sts3215", MotorNormMode.DEGREES),
    "left_arm_wrist_roll":    Motor(5, "sts3215", MotorNormMode.DEGREES),
    "left_arm_gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    "base_left_wheel":        Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
    "base_right_wheel":       Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
    "vertical_lift":          Motor(11, "sts3215", MotorNormMode.DEGREES),  # 新增升降自由度
}

# bus2: 右臂(ID1-6)
BUS2_MOTORS = {
    "right_arm_shoulder_pan":  Motor(1, "sts3215", MotorNormMode.DEGREES),
    "right_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "right_arm_elbow_flex":    Motor(3, "sts3215", MotorNormMode.DEGREES),
    "right_arm_wrist_flex":    Motor(4, "sts3215", MotorNormMode.DEGREES),
    "right_arm_wrist_roll":    Motor(5, "sts3215", MotorNormMode.DEGREES),
    "right_arm_gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

LEFT_ARM_JOINTS  = [k for k in BUS1_MOTORS if k.startswith("left_arm")]
RIGHT_ARM_JOINTS = [k for k in BUS2_MOTORS if k.startswith("right_arm")]
BASE_WHEELS      = [k for k in BUS1_MOTORS if k.startswith("base")]
LIFT_JOINT       = "vertical_lift"
LIFT_JOINTS      = [LIFT_JOINT]
ALL_POSITION_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + LIFT_JOINTS

URDF_JOINT_MAP = {k: k for k in {**BUS1_MOTORS, **BUS2_MOTORS}}

WHEEL_RADIUS  = 0.05
BASE_RADIUS   = 0.125
STEPS_PER_DEG = 4096.0 / 360.0
MAX_RAW       = 3000
RAW_POSITION_MIN = 0
RAW_POSITION_MAX = 4095
BASE_WHEEL_SIGN = {
    "base_left_wheel": -1.0,
    "base_right_wheel": 1.0,
}
LIFT_DIRECTION_SIGN = -1.0


class AlohaXBridge(Node):
    def __init__(self):
        super().__init__("alohax_bridge")

        self.declare_parameter("port1",      "/dev/so101_L")
        self.declare_parameter("port2",      "/dev/so101_R")
        self.declare_parameter("calib_file", "")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("lift_travel_m", 0.25)
        self.declare_parameter("lift_motor_degrees_per_meter", 144000.0)

        port1      = self.get_parameter("port1").value
        port2      = self.get_parameter("port2").value
        calib_file = self.get_parameter("calib_file").value
        hz         = self.get_parameter("publish_hz").value
        self.lift_travel_m = float(self.get_parameter("lift_travel_m").value)
        self.lift_motor_degrees_per_meter = float(self.get_parameter("lift_motor_degrees_per_meter").value)

        self._port1 = port1
        self._port2 = port2
        self._calib_file = calib_file
        self._hz = hz
        self._ready = False
        self._active_bus1: list[str] = []
        self._active_bus2: list[str] = []

        self.bus1 = FeetechMotorsBus(port1, BUS1_MOTORS)
        self.bus2 = FeetechMotorsBus(port2, BUS2_MOTORS)

        self._write_lock = threading.Lock()
        self._traj_active = False

        self._cmd: dict[str, float] = {k: 0.0 for k in {**BUS1_MOTORS, **BUS2_MOTORS}}

        self._pub_states = self.create_publisher(JointState, "/xlerobot/joint_states", 10)
        self._pub_left_arm_state = self.create_publisher(JointState, "left_arm_state", 10)
        self._pub_right_arm_state = self.create_publisher(JointState, "right_arm_state", 10)
        self._pub_lift_state = self.create_publisher(JointState, "lift_state", 10)

        self.create_subscription(JointState, "/joint_states",
                                 self._cb_gui_cmd, 10)
        self.create_subscription(JointState, "/xlerobot/left_arm/joint_commands",
                                 self._cb_left_cmd,  10)
        self.create_subscription(JointState, "/xlerobot/right_arm/joint_commands",
                                 self._cb_right_cmd, 10)
        self.create_subscription(Twist, "/xlerobot/base/cmd_vel",
                                 self._cb_cmd_vel,   10)
        self.create_subscription(JointState, "lift_target",
                                 self._cb_lift_cmd,  10)

        self.create_timer(1.0 / hz, self._timer_cb)

        threading.Thread(target=self._connect_loop, args=(calib_file,), daemon=True).start()

        self._cb_group = ReentrantCallbackGroup()
        self._fjt_servers = {}
        for ctrl_name, joints in [
            ("both_arms_controller",   LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS),
            ("left_arm_controller",    LEFT_ARM_JOINTS),
            ("right_arm_controller",   RIGHT_ARM_JOINTS),
            ("left_gripper_controller",  ["left_arm_gripper"]),
            ("right_gripper_controller", ["right_arm_gripper"]),
        ]:
            server = ActionServer(
                self,
                FollowJointTrajectory,
                f"/xlerobot/{ctrl_name}/follow_joint_trajectory",
                execute_callback=self._make_fjt_callback(joints),
                goal_callback=lambda goal_req: GoalResponse.ACCEPT,
                cancel_callback=lambda cancel_req: CancelResponse.ACCEPT,
                callback_group=self._cb_group,
            )
            self._fjt_servers[ctrl_name] = server

        self.get_logger().info(f"AlohaXBridge started @ {hz} Hz  bus1={port1}  bus2={port2} (waiting for hardware...)")

    def _connect_loop(self, calib_file: str):
        while rclpy.ok():
            try:
                self._try_connect(calib_file)
                return
            except Exception as e:
                self.get_logger().warn(f"Hardware not ready, retrying in 5s: {e}")
                time.sleep(5.0)

    def _connect_bus_partial(self, bus: FeetechMotorsBus, bus_name: str) -> list[str]:
        try:
            bus.connect(handshake=False)
        except Exception as e:
            self.get_logger().warn(f"{bus_name} connect failed: {e}")
            return []

        try:
            found = bus.broadcast_ping() or {}
        except Exception as e:
            self.get_logger().warn(f"{bus_name} ping failed: {e}")
            found = {}

        active = [name for name, motor in bus.motors.items() if motor.id in found]
        missing = [name for name in bus.motors if name not in active]

        if active:
            self.get_logger().info(
                f"{bus_name} connected on {bus.port} with active motors: {active}"
            )
        else:
            self.get_logger().warn(f"{bus_name} connected on {bus.port}, but no motors responded")

        if missing:
            self.get_logger().warn(f"{bus_name} missing motors ignored: {missing}")

        return active

    def _try_connect(self, calib_file: str):
        for bus in (self.bus1, self.bus2):
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass
        self.bus1 = FeetechMotorsBus(self._port1, BUS1_MOTORS)
        self.bus2 = FeetechMotorsBus(self._port2, BUS2_MOTORS)
        self._active_bus1 = self._connect_bus_partial(self.bus1, "bus1")
        self._active_bus2 = self._connect_bus_partial(self.bus2, "bus2")
        if not self._active_bus1 and not self._active_bus2:
            raise RuntimeError("No active motors found on either bus")
        self._configure_buses()
        self._load_calibration(calib_file)
        self._init_cmd_from_hardware()
        self._ready = bool(self._active_bus1 or self._active_bus2)
        self.get_logger().info("Hardware ready!")

    def _configure_buses(self):
        if self._active_bus1:
            for name in self._active_bus1:
                self.bus1.write("Return_Delay_Time", name, 0)
                self.bus1.write("Maximum_Acceleration", name, 254)
                self.bus1.write("Acceleration", name, 254)
        if self._active_bus2:
            for name in self._active_bus2:
                self.bus2.write("Return_Delay_Time", name, 0)
                self.bus2.write("Maximum_Acceleration", name, 254)
                self.bus2.write("Acceleration", name, 254)

        # bus1: 左臂+升降 - 位置模式
        for name in [n for n in LEFT_ARM_JOINTS + LIFT_JOINTS if n in self._active_bus1]:
            self.bus1.write("Operating_Mode",     name, OperatingMode.POSITION.value)
            self.bus1.write("P_Coefficient",      name, 16)
            self.bus1.write("I_Coefficient",      name, 0)
            self.bus1.write("D_Coefficient",      name, 43)
            self.bus1.write("Unloading_Condition", name, 0x2f)
        
        # bus2: 右臂 - 位置模式
        for name in [n for n in RIGHT_ARM_JOINTS if n in self._active_bus2]:
            self.bus2.write("Operating_Mode",     name, OperatingMode.POSITION.value)
            self.bus2.write("P_Coefficient",      name, 16)
            self.bus2.write("I_Coefficient",      name, 0)
            self.bus2.write("D_Coefficient",      name, 43)
            self.bus2.write("Unloading_Condition", name, 0x2f)
        
        # bus1: 驱动轮 - 速度模式
        for name in [n for n in BASE_WHEELS if n in self._active_bus1]:
            self.bus1.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
            self._cmd[name] = 0.0
            self.bus1.write("Goal_Velocity", name, 0, normalize=False)
        
        if self._active_bus1:
            self.bus1.enable_torque(self._active_bus1)
        if self._active_bus2:
            self.bus2.enable_torque(self._active_bus2)

    def _load_calibration(self, calib_file: str):
        path = pathlib.Path(calib_file).expanduser() if calib_file else pathlib.Path("")
        if not path.is_file():
            self.get_logger().warn("No calibration file – running uncalibrated")
            return
        with open(path) as f:
            raw = yaml.safe_load(f)

        def to_calib(motor_obj: Motor, cfg: dict) -> MotorCalibration:
            zero_position = cfg.get("zero_position", cfg.get("home_raw"))
            return MotorCalibration(
                id=motor_obj.id,
                drive_mode=cfg.get("drive_mode", 0),
                homing_offset=cfg.get("homing_offset", 0),
                range_min=cfg.get("range_min", 0),
                range_max=cfg.get("range_max", 4095),
                zero_position=zero_position,
            )

        calib1 = {k: to_calib(BUS1_MOTORS[k], raw[k]) for k in self._active_bus1 if k in raw}
        calib2 = {k: to_calib(BUS2_MOTORS[k], raw[k]) for k in self._active_bus2 if k in raw}
        if calib1:
            self.bus1.write_calibration(calib1)
        if calib2:
            self.bus2.write_calibration(calib2)
        self.get_logger().info(f"Calibration loaded and written to motors from {path}")

    def _init_cmd_from_hardware(self):
        try:
            active_pos1 = [n for n in LEFT_ARM_JOINTS + LIFT_JOINTS if n in self._active_bus1]
            active_pos2 = [n for n in RIGHT_ARM_JOINTS if n in self._active_bus2]
            with self._write_lock:
                pos1 = self.bus1.sync_read("Present_Position", active_pos1) if active_pos1 else {}
                pos2 = self.bus2.sync_read("Present_Position", active_pos2) if active_pos2 else {}
            for k, v in {**pos1, **pos2}.items():
                self._cmd[k] = float(v)
            for k in BASE_WHEELS:
                self._cmd[k] = 0.0
            
            # 将vertical_lift强制设置为home位置（0.0米）
            # 这样RVIZ初始化时会显示在正确的home位置
            if "vertical_lift" in self._cmd:
                self._cmd["vertical_lift"] = 0.0
            
            self.get_logger().info("Initial cmd loaded from hardware")
        except Exception as e:
            self.get_logger().warn(f"Could not read initial positions: {e}")

    def _cb_gui_cmd(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._cmd and name in ALL_POSITION_JOINTS:
                self._cmd[name] = self._joint_position_to_motor_degrees(name, pos)
            elif name in BASE_WHEELS:
                self._cmd[name] = float(np.clip(math.degrees(pos) / 180.0 * 100.0, -100, 100))

    def _cb_left_cmd(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            full = f"left_arm_{name}" if not name.startswith("left_arm") else name
            if full in self._cmd:
                self._cmd[full] = self._joint_position_to_motor_degrees(full, pos)

    def _cb_right_cmd(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            full = f"right_arm_{name}" if not name.startswith("right_arm") else name
            if full in self._cmd:
                self._cmd[full] = self._joint_position_to_motor_degrees(full, pos)

    def _cb_cmd_vel(self, msg: Twist):
        x     = msg.linear.x
        y     = msg.linear.y
        theta = math.degrees(msg.angular.z)
        self._cmd.update(self._body_to_wheel_pct(x, y, theta))

    def _cb_lift_cmd(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            full = "vertical_lift" if name == "vertical_lift" or name == "lift" else f"vertical_{name}"
            if full in self._cmd:
                self._cmd[full] = self._joint_position_to_motor_degrees(full, pos)

    def _trigger_reconnect(self):
        if self._ready:
            self._ready = False
            self.get_logger().warn("Communication lost, triggering reconnect...")
            threading.Thread(target=self._connect_loop, args=(self._calib_file,), daemon=True).start()

    def _timer_cb(self):
        if not self._ready:
            self._publish_default_joint_states()
            return
        if self._traj_active:
            return
        now = self.get_clock().now().to_msg()
        try:
            active_pos1 = [n for n in LEFT_ARM_JOINTS + LIFT_JOINTS if n in self._active_bus1]
            active_pos2 = [n for n in RIGHT_ARM_JOINTS if n in self._active_bus2]
            active_base = [n for n in BASE_WHEELS if n in self._active_bus1]
            with self._write_lock:
                pos1 = self.bus1.sync_read("Present_Position", active_pos1) if active_pos1 else {}
                pos2 = self.bus2.sync_read("Present_Position", active_pos2) if active_pos2 else {}
                vel1 = self.bus1.sync_read("Present_Velocity", active_base) if active_base else {}
        except Exception as e:
            self.get_logger().warn(f"sync_read error: {e}")
            self._trigger_reconnect()
            return

        js = JointState()
        js.header.stamp = now
        for k in LEFT_ARM_JOINTS:
            js.name.append(URDF_JOINT_MAP[k])
            js.position.append(self._motor_degrees_to_joint_position(k, float(pos1.get(k, self._cmd[k]))))
            js.velocity.append(0.0)
        for k in LIFT_JOINTS:
            js.name.append(URDF_JOINT_MAP[k])
            # vertical_lift 使用命令位置而不是硬件读取的位置
            # 这样可以确保RVIZ显示与命令一致，不受硬件初始位置影响
            js.position.append(self._motor_degrees_to_joint_position(k, float(self._cmd[k])))
            js.velocity.append(0.0)
        for k in RIGHT_ARM_JOINTS:
            js.name.append(URDF_JOINT_MAP[k])
            js.position.append(math.radians(float(pos2.get(k, self._cmd[k]))))
            js.velocity.append(0.0)
        for k in BASE_WHEELS:
            js.name.append(URDF_JOINT_MAP[k])
            js.position.append(0.0)
            js.velocity.append(float(vel1.get(k, 0.0)) * BASE_WHEEL_SIGN.get(k, 1.0))
        self._pub_states.publish(js)

        # 发布左臂状态给 teleop
        left_js = JointState()
        left_js.header.stamp = now
        for k in LEFT_ARM_JOINTS:
            left_js.name.append(URDF_JOINT_MAP[k])
            left_js.position.append(self._motor_degrees_to_joint_position(k, float(pos1.get(k, self._cmd[k]))))
        self._pub_left_arm_state.publish(left_js)

        # 发布右臂状态给 teleop
        right_js = JointState()
        right_js.header.stamp = now
        for k in RIGHT_ARM_JOINTS:
            right_js.name.append(URDF_JOINT_MAP[k])
            right_js.position.append(self._motor_degrees_to_joint_position(k, float(pos2.get(k, self._cmd[k]))))
        self._pub_right_arm_state.publish(right_js)

        # 发布升降状态给 teleop
        lift_js = JointState()
        lift_js.header.stamp = now
        for k in LIFT_JOINTS:
            lift_js.name.append(URDF_JOINT_MAP[k])
            lift_js.position.append(self._motor_degrees_to_joint_position(k, float(self._cmd[k])))
        self._pub_lift_state.publish(lift_js)

        if not self._traj_active:
            self._write_commands()

    def _publish_default_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for name in ALL_POSITION_JOINTS + BASE_WHEELS:
            js.name.append(URDF_JOINT_MAP[name])
            # vertical_lift 默认显示在home位置（0.0米）
            if name == "vertical_lift":
                js.position.append(0.0)
            else:
                js.position.append(0.0)
            js.velocity.append(0.0)
        self._pub_states.publish(js)

    def _write_commands(self):
        with self._write_lock:
            if not self._ready:
                return
            try:
                left_cmd  = {
                    k: self._clip_motor_position(k, float(self._cmd[k]))
                    for k in LEFT_ARM_JOINTS + LIFT_JOINTS
                    if k in self._active_bus1
                }
                right_cmd = {
                    k: self._clip_motor_position(k, float(self._cmd[k]))
                    for k in RIGHT_ARM_JOINTS
                    if k in self._active_bus2
                }
                wheel_cmd = {
                    k: int(round(self._cmd[k] * BASE_WHEEL_SIGN.get(k, 1.0)))
                    for k in BASE_WHEELS
                    if k in self._active_bus1
                }

                if left_cmd or right_cmd:
                    self.get_logger().debug(f"sync_write L={left_cmd}, R={right_cmd}")

                if left_cmd:
                    self.bus1.sync_write("Goal_Position", left_cmd)
                if right_cmd:
                    self.bus2.sync_write("Goal_Position", right_cmd)
                if wheel_cmd:
                    self.bus1.sync_write("Goal_Velocity", wheel_cmd, normalize=False)
            except Exception as e:
                self.get_logger().warn(f"sync_write error: {e}")

    def _clip_motor_position(self, name: str, degrees: float) -> float:
        bus = self.bus1 if name in BUS1_MOTORS else self.bus2
        motor = bus.motors.get(name)
        if motor is None:
            return float(np.clip(degrees, RAW_POSITION_MIN, RAW_POSITION_MAX))

        calibration = bus.calibration.get(name)
        if calibration is None:
            return float(np.clip(degrees, RAW_POSITION_MIN, RAW_POSITION_MAX))

        if motor.norm_mode is MotorNormMode.DEGREES:
            mid = calibration.zero_position
            if mid is None:
                mid = (calibration.range_min + calibration.range_max) / 2
            max_res = bus.model_resolution_table[motor.model] - 1
            min_degrees = (calibration.range_min - mid) * 360.0 / max_res
            max_degrees = (calibration.range_max - mid) * 360.0 / max_res
            lo, hi = sorted((min_degrees, max_degrees))
            return float(np.clip(degrees, lo, hi))

        if motor.norm_mode is MotorNormMode.RANGE_0_100:
            return float(np.clip(degrees, 0.0, 100.0))

        if motor.norm_mode is MotorNormMode.RANGE_M100_100:
            return float(np.clip(degrees, -100.0, 100.0))

        return degrees

    def _body_to_wheel_pct(self, x, y, theta_deg):
        WHEEL_BASE = 0.4
        linear_vel = x
        angular_vel_rad = math.radians(theta_deg)
        
        v_left = linear_vel - angular_vel_rad * WHEEL_BASE / 2
        v_right = linear_vel + angular_vel_rad * WHEEL_BASE / 2
        
        omega_left = v_left / WHEEL_RADIUS
        omega_right = v_right / WHEEL_RADIUS
        
        omega_left_deg = math.degrees(omega_left)
        omega_right_deg = math.degrees(omega_right)
        
        raw_left = abs(omega_left_deg) * STEPS_PER_DEG
        raw_right = abs(omega_right_deg) * STEPS_PER_DEG
        max_raw = max(raw_left, raw_right)
        
        if max_raw > MAX_RAW and max_raw > 0:
            scale = MAX_RAW / max_raw
            omega_left_deg *= scale
            omega_right_deg *= scale
        
        left_pct = np.clip(omega_left_deg / 360.0 * 100.0, -100, 100)
        right_pct = np.clip(omega_right_deg / 360.0 * 100.0, -100, 100)
        
        return {"base_left_wheel": float(left_pct),
                "base_right_wheel": float(right_pct)}

    def _joint_position_to_motor_degrees(self, name: str, position: float) -> float:
        if name == LIFT_JOINT:
            lift_m = float(np.clip(position, 0.0, self.lift_travel_m))
            return LIFT_DIRECTION_SIGN * lift_m * self.lift_motor_degrees_per_meter
        return math.degrees(position)

    def _motor_degrees_to_joint_position(self, name: str, degrees: float) -> float:
        if name == LIFT_JOINT:
            if self.lift_motor_degrees_per_meter == 0.0:
                return 0.0
            return float(np.clip(
                LIFT_DIRECTION_SIGN * degrees / self.lift_motor_degrees_per_meter,
                0.0,
                self.lift_travel_m,
            ))
        return math.radians(degrees)

    def _make_fjt_callback(self, joints: list[str]):
        def _cb(goal_handle):
            self.get_logger().info(f"FJT goal received for joints: {joints}")
            return self._fjt_execute(goal_handle, joints)
        return _cb

    def _fjt_execute(self, goal_handle, joints: list[str]):
        self._traj_active = True

        traj = goal_handle.request.trajectory
        feedback = FollowJointTrajectory.Feedback()
        prev_t = 0.0

        try:
            for point in traj.points:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                for name, pos in zip(traj.joint_names, point.positions):
                    if name in self._cmd:
                        self._cmd[name] = self._joint_position_to_motor_degrees(name, pos)

                self._write_commands()

                t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                dt = max(t - prev_t, 0.02)
                prev_t = t
                time.sleep(dt)

                feedback.joint_names = list(traj.joint_names)
                feedback.desired = point
                goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        finally:
            self._traj_active = False

    def stop_base(self):
        try:
            active_base = [k for k in BASE_WHEELS if k in self._active_bus1]
            if active_base:
                self.bus1.sync_write("Goal_Velocity", {k: 0 for k in active_base},
                                     normalize=False, num_retry=3)
        except Exception:
            pass

    def destroy_node(self):
        if self._ready:
            self.stop_base()
        try:
            self.bus1.disconnect()
        except Exception:
            pass
        try:
            self.bus2.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AlohaXBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
