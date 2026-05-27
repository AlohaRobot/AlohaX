#!/usr/bin/env python3
"""
AlohaX Keyboard Controller Teleoperation - Direct Hardware Control

Hardware Configuration:
- bus1 (/dev/so101_L): 左臂(ID1-6) + 驱动轮(ID7-8) + 升降(ID11)
- bus2 (/dev/so101_R): 右臂(ID1-6)

Controls (Keyboard):
- Arrow Keys (Up/Down/Left/Right): 底盘移动
- Space: 所有舵机回零
- Home: 所有舵机卸载力
- PageUp/PageDown: 垂直升降 (+/-)
- =/-: 速度加/减

Left Arm Controls:
- Z/C: shoulder_pan (左/右)
- S/X: shoulder_lift (前/后)
- D/A: elbow_flex (前/后)
- 2/W: wrist_flex (前/后)
- Q/E: wrist_roll (左/右)
- 3: gripper open

Right Arm Controls:
- M/.: shoulder_pan (左/右)
- K/: shoulder_lift (前/后)
- L/J: elbow_flex (前/后)
- 8/I: wrist_flex (前/后)
- U/O: wrist_roll (左/右)
- 9: gripper open

底盘/机械臂控制: 直接控制Feetech电机，跳过ROS bridge
"""

import sys
import math
import time
import curses
import argparse
import logging
import traceback
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 配置日志
logger = logging.getLogger(__name__)
logger.setLevel(logging.ERROR)
root_logger = logging.getLogger()
root_logger.setLevel(logging.ERROR)

# 添加项目源代码目录到 Python 路径
src_path = Path(__file__).parents[2] / "src" / "alohax_hw_bridge"
sys.path.insert(0, str(src_path))

from alohax_hw_bridge.motors import Motor, MotorNormMode
from alohax_hw_bridge.motors.feetech import FeetechMotorsBus, OperatingMode

# ================== 硬件配置 ==================
# Bus1 配置: 左臂 + 驱动轮 + 垂直升降
BUS1_MOTORS = {
    "left_arm_shoulder_pan":  Motor(1, "sts3215", MotorNormMode.DEGREES),
    "left_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "left_arm_elbow_flex":    Motor(3, "sts3215", MotorNormMode.DEGREES),
    "left_arm_wrist_flex":    Motor(4, "sts3215", MotorNormMode.DEGREES),
    "left_arm_wrist_roll":    Motor(5, "sts3215", MotorNormMode.DEGREES),
    "left_arm_gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    "base_left_wheel":        Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
    "base_right_wheel":       Motor(8, "sts3215", MotorNormMode.RANGE_0_100),
    "vertical_lift":          Motor(11, "sts3215", MotorNormMode.DEGREES),
}

# Bus2 配置: 右臂
BUS2_MOTORS = {
    "right_arm_shoulder_pan":  Motor(1, "sts3215", MotorNormMode.DEGREES),
    "right_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "right_arm_elbow_flex":    Motor(3, "sts3215", MotorNormMode.DEGREES),
    "right_arm_wrist_flex":    Motor(4, "sts3215", MotorNormMode.DEGREES),
    "right_arm_wrist_roll":    Motor(5, "sts3215", MotorNormMode.DEGREES),
    "right_arm_gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

# 电机分组
LEFT_ARM_JOINTS  = [k for k in BUS1_MOTORS if k.startswith("left_arm")]
RIGHT_ARM_JOINTS = [k for k in BUS2_MOTORS if k.startswith("right_arm")]
BASE_WHEELS      = [k for k in BUS1_MOTORS if k.startswith("base")]
LIFT_JOINT       = "vertical_lift"
LIFT_JOINTS      = [LIFT_JOINT]

DEFAULT_PORT1: str = "/dev/so101_L"
DEFAULT_PORT2: str = "/dev/so101_R"
STEPS_PER_DEG = 4096.0 / 360.0
BASE_WHEEL_SIGN = {"base_left_wheel": -1.0, "base_right_wheel": 1.0}
LIFT_DIRECTION_SIGN = -1.0
LIFT_TRAVEL_M = 0.25
LIFT_MOTOR_DEGREES_PER_METER = 144000.0

WHEEL_RADIUS: float = 0.05
WHEEL_BASE: float = 0.30

# 底盘速度4个档位: 1(最小) - 4(最大)
BASE_SPEED_LEVELS = {
    1: {"lin": 0.05, "ang": 20.0},
    2: {"lin": 0.1, "ang": 40.0},
    3: {"lin": 0.15, "ang": 70.0},
    4: {"lin": 0.5, "ang": 572.0},
}

# 机械臂速度4个档位（PID和加速度）：4档为最高速，每降低1档参数减半
ARM_SPEED_LEVELS = {
    1: {"p": 4, "i": 0, "d": 11, "acc": 64, "step": 0.025},
    2: {"p": 8, "i": 0, "d": 22, "acc": 128, "step": 0.05},
    3: {"p": 12, "i": 0, "d": 32, "acc": 192, "step": 0.075},
    4: {"p": 16, "i": 0, "d": 43, "acc": 254, "step": 0.1},
}

# 升降机构速度4个档位（PID、最大速度和步长）：4档为最高速，每降低1档参数减半
LIFT_SPEED_LEVELS = {
    1: {"kp_vel": 75, "v_max": 325, "step": 0.5},
    2: {"kp_vel": 150, "v_max": 650, "step": 1.0},
    3: {"kp_vel": 225, "v_max": 975, "step": 1.5},
    4: {"kp_vel": 300, "v_max": 1300, "step": 2.0},
}

# ================== 校准参数 (来自 alohax_calibration.yaml) ==================
CALIBRATION = {
    "left_arm_shoulder_pan":  {"zero_position": 2047},
    "left_arm_shoulder_lift": {"zero_position": 2047},
    "left_arm_elbow_flex":    {"zero_position": 2047},
    "left_arm_wrist_flex":    {"zero_position": 2047},
    "left_arm_wrist_roll":    {"zero_position": 1000},
    "left_arm_gripper":       {"zero_position": 2048},
    "right_arm_shoulder_pan":  {"zero_position": 2047},
    "right_arm_shoulder_lift": {"zero_position": 2047},
    "right_arm_elbow_flex":    {"zero_position": 2047},
    "right_arm_wrist_flex":    {"zero_position": 2047},
    "right_arm_wrist_roll":    {"zero_position": 1000},
    "right_arm_gripper":       {"zero_position": 2048},
    "vertical_lift":          {"zero_position": 2000},
}

# ================== 电机控制函数 ==================
def degps_to_raw(degps: float) -> int:
    steps_per_deg = 4096.0 / 360.0
    mag = int(round(abs(degps) * steps_per_deg))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


def body_to_wheel_raw(v_cmd: float, omega_cmd_degps: float,
                      wheel_radius: float = WHEEL_RADIUS,
                      wheel_base: float = WHEEL_BASE) -> dict[str, int]:
    omega_radps = omega_cmd_degps * (np.pi / 180.0)
    v_left = v_cmd - omega_radps * wheel_base / 2.0
    v_right = v_cmd + omega_radps * wheel_base / 2.0
    w_left_radps = v_left / wheel_radius
    w_right_radps = v_right / wheel_radius
    w_left_degps = w_left_radps * (180.0 / np.pi)
    w_right_degps = w_right_radps * (180.0 / np.pi)
    return {"base_left_wheel": degps_to_raw(w_left_degps),
            "base_right_wheel": degps_to_raw(w_right_degps)}


def degrees_to_raw(name: str, degrees: float) -> int:
    calib = CALIBRATION.get(name, {})
    zero_pos = calib.get("zero_position", 2047)
    return int(round(degrees * STEPS_PER_DEG + zero_pos))


def raw_to_degrees(name: str, raw: int) -> float:
    calib = CALIBRATION.get(name, {})
    zero_pos = calib.get("zero_position", 2047)
    return (raw - zero_pos) / STEPS_PER_DEG


def joint_position_to_motor_cmd(name: str, position: float) -> float:
    if name == LIFT_JOINT:
        lift_m = float(np.clip(position, -0.30, 0.05))
        lift_raw = LIFT_DIRECTION_SIGN * lift_m * LIFT_MOTOR_DEGREES_PER_METER
        calib = CALIBRATION.get(LIFT_JOINT, {})
        return int(round(lift_raw + calib.get("zero_position", 2000)))
    elif name.endswith("gripper"):
        rad = float(np.clip(position, JOINT_LIMITS["gripper"]["min"], JOINT_LIMITS["gripper"]["max"]))
        calib = CALIBRATION.get(name, {})
        zero = calib.get("zero_position", 2048)
        range_min = calib.get("range_min", 2048)
        range_max = calib.get("range_max", 3396)
        t = (rad - JOINT_LIMITS["gripper"]["min"]) / (JOINT_LIMITS["gripper"]["max"] - JOINT_LIMITS["gripper"]["min"])
        return int(round(range_min + t * (range_max - range_min)))
    else:
        return math.degrees(position)


def motor_cmd_to_joint_position(name: str, cmd: float) -> float:
    if name == LIFT_JOINT:
        calib = CALIBRATION.get(LIFT_JOINT, {})
        zero = calib.get("zero_position", 2000)
        raw = cmd - zero
        lift_m = LIFT_DIRECTION_SIGN * raw / LIFT_MOTOR_DEGREES_PER_METER
        return float(np.clip(lift_m, -0.30, 0.05))
    elif name.endswith("gripper"):
        calib = CALIBRATION.get(name, {})
        range_min = calib.get("range_min", 2048)
        range_max = calib.get("range_max", 3396)
        t = (cmd - range_min) / (range_max - range_min)
        rad = JOINT_LIMITS["gripper"]["min"] + t * (JOINT_LIMITS["gripper"]["max"] - JOINT_LIMITS["gripper"]["min"])
        return float(np.clip(rad, JOINT_LIMITS["gripper"]["min"], JOINT_LIMITS["gripper"]["max"]))
    else:
        degrees = raw_to_degrees(name, int(cmd))
        return math.radians(degrees)


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

# ================== 关节限制 ==================
JOINT_LIMITS = {
    "shoulder_pan":  {"min": -1.996, "max": 2.111},
    "shoulder_lift": {"min": -2.004, "max": 2.028},
    "elbow_flex":    {"min": -0.072, "max": 2.995},
    "wrist_flex":    {"min": -2.238, "max": 1.334},
    "wrist_roll":    {"min": -3.139, "max": 3.140},
    "gripper":       {"min": 0.002,  "max": 2.069},
}

# ================== 键盘按键映射 ==================
KEYBOARD_KEYMAP = {
    # 底盘控制
    'base_forward': curses.KEY_UP,
    'base_backward': curses.KEY_DOWN,
    'base_left': curses.KEY_LEFT,
    'base_right': curses.KEY_RIGHT,
    
    # 升降控制
    'vertical_lift+': curses.KEY_PPAGE,
    'vertical_lift-': curses.KEY_NPAGE,
    
    # 速度控制
    'speed+': ord('='),
    'speed-': ord('-'),
    
    # 特殊功能
    'reset': ord(' '),      # 空格键回零
    'torque_toggle': curses.KEY_HOME,  # Home键切换扭矩
    
    # 左臂控制
    'left_shoulder_pan-': ord('z'),
    'left_shoulder_pan+': ord('c'),
    'left_shoulder_lift-': ord('s'),
    'left_shoulder_lift+': ord('x'),
    'left_elbow_flex-': ord('d'),
    'left_elbow_flex+': ord('a'),
    'left_wrist_flex-': ord('2'),
    'left_wrist_flex+': ord('w'),
    'left_wrist_roll-': ord('q'),
    'left_wrist_roll+': ord('e'),
    'left_gripper+': ord('3'),
    
    # 右臂控制
    'right_shoulder_pan-': ord('m'),
    'right_shoulder_pan+': ord('.'),
    'right_shoulder_lift-': ord('k'),
    'right_shoulder_lift+': ord(','),
    'right_elbow_flex-': ord('l'),
    'right_elbow_flex+': ord('j'),
    'right_wrist_flex-': ord('8'),
    'right_wrist_flex+': ord('i'),
    'right_wrist_roll-': ord('u'),
    'right_wrist_roll+': ord('o'),
    'right_gripper+': ord('9'),
}

# ================== 显示类 ==================
class TeleopDisplay:
    def __init__(self):
        self.left_arm_state = {}
        self.right_arm_state = {}
        self.left_arm_hw_state = {}
        self.right_arm_hw_state = {}
        self.left_arm_current = {}
        self.left_arm_load = {}
        self.right_arm_current = {}
        self.right_arm_load = {}
        self.lift_state = 0.0
        self.lift_hw_state = 0.0
        self.lift_current = None
        self.lift_load = None
        self.left_wheel_state = 0.0
        self.left_wheel_hw_state = 0.0
        self.left_wheel_current = None
        self.left_wheel_load = None
        self.right_wheel_state = 0.0
        self.right_wheel_hw_state = 0.0
        self.right_wheel_current = None
        self.right_wheel_load = None
        self.base_state = "IDLE"
        self.left_gripper_state = 0.0
        self.right_gripper_state = 0.0
        self.base_speed_level = 2
        self.reset_flag = False
        self.torque_status = "ON"
        self.stdscr = None
        self.loop_period_ms = 0.0

    def init_curses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.nodelay(True)
        self.stdscr.keypad(True)  # 启用键盘支持，以便正确读取方向键、PgUp、PgDn、Home等特殊键
        curses.curs_set(0)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(6, curses.COLOR_RED, curses.COLOR_BLACK)

    def cleanup(self):
        if self.stdscr:
            curses.nocbreak()
            curses.curs_set(1)
            self.stdscr.nodelay(False)
            curses.echo()
            curses.endwin()

    def update_left_arm(self, state):
        self.left_arm_state = state

    def update_right_arm(self, state):
        self.right_arm_state = state

    def update_lift(self, position):
        self.lift_state = position

    def update_lift_hw(self, position):
        self.lift_hw_state = position

    def update_base(self, state):
        self.base_state = state

    def update_grippers(self, left_gripper_deg, right_gripper_deg):
        self.left_gripper_state = left_gripper_deg
        self.right_gripper_state = right_gripper_deg

    def update_hw_arm_states(self, left_hw_state, right_hw_state):
        self.left_arm_hw_state = left_hw_state
        self.right_arm_hw_state = right_hw_state

    def update_arm_motor_states(self, left_current, left_load, right_current, right_load,
                                  lift_current=None, lift_load=None,
                                  left_wheel_current=None, left_wheel_load=None,
                                  right_wheel_current=None, right_wheel_load=None):
        self.left_arm_current = left_current
        self.left_arm_load = left_load
        self.right_arm_current = right_current
        self.right_arm_load = right_load
        if lift_current is not None:
            self.lift_current = lift_current
        if lift_load is not None:
            self.lift_load = lift_load
        if left_wheel_current is not None:
            self.left_wheel_current = left_wheel_current
        if left_wheel_load is not None:
            self.left_wheel_load = left_wheel_load
        if right_wheel_current is not None:
            self.right_wheel_current = right_wheel_current
        if right_wheel_load is not None:
            self.right_wheel_load = right_wheel_load
    
    def update_lift_motor_states(self, lift_current, lift_load):
        self.lift_current = lift_current
        self.lift_load = lift_load
    
    def update_wheel_states(self, left_wheel_tgt, right_wheel_tgt,
                           left_wheel_hw, right_wheel_hw,
                           left_wheel_current, left_wheel_load,
                           right_wheel_current, right_wheel_load):
        self.left_wheel_state = left_wheel_tgt
        self.right_wheel_state = right_wheel_tgt
        self.left_wheel_hw_state = left_wheel_hw
        self.right_wheel_hw_state = right_wheel_hw
        self.left_wheel_current = left_wheel_current
        self.left_wheel_load = left_wheel_load
        self.right_wheel_current = right_wheel_current
        self.right_wheel_load = right_wheel_load

    def set_reset_flag(self, flag):
        self.reset_flag = flag

    def update_base_speed_level(self, level):
        self.base_speed_level = level

    def update_torque_status(self, enabled):
        self.torque_status = "ON" if enabled else "OFF"
    
    def update_loop_period(self, period_ms):
        self.loop_period_ms = period_ms

    def display(self):
        if not self.stdscr:
            return
        try:
            height, width = self.stdscr.getmaxyx()
            if height < 24 or width < 100:
                self.stdscr.erase()
                try:
                    self.stdscr.addstr(0, 0, f"Window too small! Need 100x24, got {width}x{height}", curses.color_pair(3) | curses.A_BOLD)
                except:
                    pass
                self.stdscr.noutrefresh()
                curses.doupdate()
                return

            self.stdscr.erase()
            line = 0
            
            def safe_addstr(l, c, text, attr=curses.color_pair(3) | curses.A_BOLD):
                try:
                    if l < height and c + len(text) <= width:
                        self.stdscr.addstr(l, c, text, attr)
                except curses.error:
                    pass
            
            BRIGHT_WHITE = curses.color_pair(3) | curses.A_BOLD
            BRIGHT_GREEN = curses.color_pair(4) | curses.A_BOLD
            BRIGHT_YELLOW = curses.color_pair(5) | curses.A_BOLD
            BRIGHT_RED = curses.color_pair(6) | curses.A_BOLD
            
            safe_addstr(line, 0, "=" * min(100, width), BRIGHT_WHITE)
            line += 1
            title_text = "AlohaX Keyboard Teleop - Display OK!"
            period_text = f"Period: {self.loop_period_ms:.1f}ms"
            safe_addstr(line, 0, title_text, BRIGHT_WHITE)
            period_x = max(0, width - len(period_text) - 2)
            safe_addstr(line, period_x, period_text, BRIGHT_YELLOW)
            line += 1
            safe_addstr(line, 0, "=" * min(100, width), BRIGHT_WHITE)
            line += 1
            
            safe_addstr(line, 0, "[ LEFT ARM ]", BRIGHT_WHITE)
            line += 1
            safe_addstr(line, 0, f"  {'Joint':12}  {'tgt':8} {'hw':8} {'err':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
            line += 1

            def fmt_single_row(label, val, hw, current, load):
                nonlocal line
                if line >= height:
                    return
                val_deg = int(round(math.degrees(val)))
                hw_deg = int(round(math.degrees(hw)))
                err_deg = val_deg - hw_deg
                safe_addstr(line, 0, f"  {label:12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{val_deg:>8d} ", BRIGHT_WHITE)
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{hw_deg:>8d}  ", BRIGHT_GREEN)
                    
                    err_color = BRIGHT_WHITE
                    if abs(err_deg) > 80:
                        err_color = BRIGHT_RED
                    elif abs(err_deg) > 50:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{err_deg:>+8d}", err_color)
                    
                    display_current = None
                    if current is not None:
                        display_current = current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    display_load = None
                    if load is not None:
                        if load >= 1000:
                            display_load = 1000 - load
                        else:
                            display_load = load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    load_color = BRIGHT_WHITE
                    if display_load is not None:
                        if abs(display_load) > 800:
                            load_color = BRIGHT_RED
                        elif abs(display_load) > 500:
                            load_color = BRIGHT_YELLOW
                    
                    if line < height and 46 + 8 <= width:
                        self.stdscr.addstr(line, 46, current_str, cur_color)
                    if line < height and 56 + 8 <= width:
                        self.stdscr.addstr(line, 56, load_str, load_color)
                except curses.error:
                    pass
                line += 1

            joint_map_left = {
                "Pan": "shoulder_pan",
                "Lift": "shoulder_lift", 
                "Elbow": "elbow_flex",
                "WristFlex": "wrist_flex",
                "WristRoll": "wrist_roll",
                "Gripper": "gripper"
            }
            for label, joint_name in joint_map_left.items():
                if label == "Gripper":
                    val = self.left_gripper_state
                else:
                    val = self.left_arm_state.get(joint_name, 0.0)
                hw = self.left_arm_hw_state.get(joint_name, 0.0)
                current = self.left_arm_current.get(joint_name)
                load = self.left_arm_load.get(joint_name)
                fmt_single_row(label, val, hw, current, load)
            line += 1

            if line < height:
                safe_addstr(line, 0, "[ RIGHT ARM ]", BRIGHT_WHITE)
                line += 1
                safe_addstr(line, 0, f"  {'Joint':12}  {'tgt':8} {'hw':8} {'err':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
                line += 1

                joint_map_right = {
                    "Pan": "shoulder_pan",
                    "Lift": "shoulder_lift", 
                    "Elbow": "elbow_flex",
                    "WristFlex": "wrist_flex",
                    "WristRoll": "wrist_roll",
                    "Gripper": "gripper"
                }
                for label, joint_name in joint_map_right.items():
                    if label == "Gripper":
                        val = self.right_gripper_state
                    else:
                        val = self.right_arm_state.get(joint_name, 0.0)
                    hw = self.right_arm_hw_state.get(joint_name, 0.0)
                    current = self.right_arm_current.get(joint_name)
                    load = self.right_arm_load.get(joint_name)
                    fmt_single_row(label, val, hw, current, load)
                line += 1

            if line < height:
                safe_addstr(line, 0, "[ VERTICAL LIFT ]", BRIGHT_WHITE)
                line += 1
                safe_addstr(line, 0, f"  {'Joint':12}  {'tgt(cm)':8} {'hw(cm)':8} {'err(cm)':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
                line += 1
                
                lift_tgt_cm = self.lift_state * 100.0
                lift_hw_cm = self.lift_hw_state * 100.0
                lift_err = lift_tgt_cm - lift_hw_cm
                
                safe_addstr(line, 0, f"  {'Lift':12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{lift_tgt_cm:>8.1f} ", BRIGHT_WHITE)
                
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{lift_hw_cm:>8.1f}  ", BRIGHT_GREEN)
                    
                    err_color = BRIGHT_WHITE
                    if abs(lift_err) > 50:
                        err_color = BRIGHT_RED
                    elif abs(lift_err) > 20:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{lift_err:>+8.1f}", err_color)
                    
                    display_current = None
                    if self.lift_current is not None:
                        display_current = self.lift_current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    display_load = None
                    if self.lift_load is not None:
                        if self.lift_load >= 1000:
                            display_load = 1000 - self.lift_load
                        else:
                            display_load = self.lift_load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    load_color = BRIGHT_WHITE
                    if display_load is not None:
                        if abs(display_load) > 800:
                            load_color = BRIGHT_RED
                        elif abs(display_load) > 500:
                            load_color = BRIGHT_YELLOW
                    
                    if line < height and 46 + 8 <= width:
                        self.stdscr.addstr(line, 46, current_str, cur_color)
                    if line < height and 56 + 8 <= width:
                        self.stdscr.addstr(line, 56, load_str, load_color)
                except curses.error:
                    pass
                line += 2

            if line < height:
                safe_addstr(line, 0, "[ BASE CONTROL ]", BRIGHT_WHITE)
                line += 1
                safe_addstr(line, 0, f"  {'Joint':12}  {'tgt':8} {'hw':8} {'err':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
                line += 1
                
                safe_addstr(line, 0, f"  {'LeftWheel':12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{self.left_wheel_state:>8.0f} ", BRIGHT_WHITE)
                
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{self.left_wheel_hw_state:>8.0f}  ", BRIGHT_GREEN)
                    
                    left_wheel_err = self.left_wheel_state - self.left_wheel_hw_state
                    err_color = BRIGHT_WHITE
                    if abs(left_wheel_err) > 50:
                        err_color = BRIGHT_RED
                    elif abs(left_wheel_err) > 20:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{left_wheel_err:>+8.0f}", err_color)
                    
                    display_current = None
                    if self.left_wheel_current is not None:
                        display_current = self.left_wheel_current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    display_load = None
                    if self.left_wheel_load is not None:
                        if self.left_wheel_load >= 1000:
                            display_load = 1000 - self.left_wheel_load
                        else:
                            display_load = self.left_wheel_load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    load_color = BRIGHT_WHITE
                    if display_load is not None:
                        if abs(display_load) > 800:
                            load_color = BRIGHT_RED
                        elif abs(display_load) > 500:
                            load_color = BRIGHT_YELLOW
                    
                    if line < height and 46 + 8 <= width:
                        self.stdscr.addstr(line, 46, current_str, cur_color)
                    if line < height and 56 + 8 <= width:
                        self.stdscr.addstr(line, 56, load_str, load_color)
                except curses.error:
                    pass
                line += 1
                
                safe_addstr(line, 0, f"  {'RightWheel':12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{self.right_wheel_state:>8.0f} ", BRIGHT_WHITE)
                
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{self.right_wheel_hw_state:>8.0f}  ", BRIGHT_GREEN)
                    
                    right_wheel_err = self.right_wheel_state - self.right_wheel_hw_state
                    err_color = BRIGHT_WHITE
                    if abs(right_wheel_err) > 50:
                        err_color = BRIGHT_RED
                    elif abs(right_wheel_err) > 20:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{right_wheel_err:>+8.0f}", err_color)
                    
                    display_current = None
                    if self.right_wheel_current is not None:
                        display_current = self.right_wheel_current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    display_load = None
                    if self.right_wheel_load is not None:
                        if self.right_wheel_load >= 1000:
                            display_load = 1000 - self.right_wheel_load
                        else:
                            display_load = self.right_wheel_load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    load_color = BRIGHT_WHITE
                    if display_load is not None:
                        if abs(display_load) > 800:
                            load_color = BRIGHT_RED
                        elif abs(display_load) > 500:
                            load_color = BRIGHT_YELLOW
                    
                    if line < height and 46 + 8 <= width:
                        self.stdscr.addstr(line, 46, current_str, cur_color)
                    if line < height and 56 + 8 <= width:
                        self.stdscr.addstr(line, 56, load_str, load_color)
                except curses.error:
                    pass
                line += 1
                
                safe_addstr(line, 0, f"  Status: {self.base_state} | Speed: {self.base_speed_level} (Base/Arms/Lift)", BRIGHT_WHITE)
                line += 2

            if line < height:
                safe_addstr(line, 0, f"[ CONTROLS ] | Arrow=Move | PgUp/PgDn=Lift | +/-=Speed | Space=Reset | Home=Toggle Torque | Torque: {self.torque_status}", BRIGHT_WHITE)

            self.stdscr.noutrefresh()
            curses.doupdate()
        except curses.error as e:
            pass
        except Exception as e:
            pass

# ================== 机械臂控制类 ==================
class SimpleTeleopArm:
    def __init__(self, prefix="left", kp=1, speed_level=2):
        self.prefix = prefix
        self.kp = kp
        self.speed_level = speed_level
        self._update_step()
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }

    def move_to_zero_position(self):
        self.target_positions = self.zero_pos.copy()

    def handle_keys(self, key_state):
        if key_state.get('gripper+'):
            self.target_positions["gripper"] = 1.2
        else:
            self.target_positions["gripper"] = 0.0

        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] -= self.degree_step
        if key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] += self.degree_step
        if key_state.get('shoulder_lift+'):
            self.target_positions["shoulder_lift"] += self.degree_step
        if key_state.get('shoulder_lift-'):
            self.target_positions["shoulder_lift"] -= self.degree_step

        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] -= self.degree_step
        if key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] += self.degree_step

        if key_state.get('elbow_flex+'):
            self.target_positions["elbow_flex"] += self.degree_step
        if key_state.get('elbow_flex-'):
            self.target_positions["elbow_flex"] -= self.degree_step

        if key_state.get('wrist_flex+'):
            self.target_positions["wrist_flex"] += self.degree_step
        if key_state.get('wrist_flex-'):
            self.target_positions["wrist_flex"] -= self.degree_step

        for joint in self.target_positions:
            if joint in JOINT_LIMITS:
                self.target_positions[joint] = clamp(
                    self.target_positions[joint],
                    JOINT_LIMITS[joint]["min"],
                    JOINT_LIMITS[joint]["max"],
                )
    
    def _update_step(self):
        self.degree_step = ARM_SPEED_LEVELS[self.speed_level]["step"]
    
    def update_speed_level(self, level):
        if 1 <= level <= 4:
            self.speed_level = level
            self._update_step()

class ContinuousLiftControl:
    def __init__(self, speed_level=2):
        self._ticks_per_rev = 4096.0
        self._deg_per_tick = 360.0 / self._ticks_per_rev
        self._mm_per_deg = (84.0) / 360.0

        self._last_tick = 0.0
        self._extended_ticks = 0.0
        self._z0_deg = 0.0

        self.target_height_mm = 0.0
        self.min_height_mm = -450.0
        self.max_height_mm = 50.0
        self.descent_floor_mm = -445.0

        self.speed_level = speed_level
        self.on_target_mm = 1.0
        self.dir_sign = -1.0
        self._update_params()

        self.configured = False

    def configure(self, bus, motor_name):
        if self.configured:
            return
        self._bus = bus
        self._motor_name = motor_name
        try:
            self._last_tick = float(self._bus.read("Present_Position", self._motor_name, normalize=False))
        except Exception as e:
            self._last_tick = 0.0
        self._extended_ticks = 0.0
        self.configured = True

    def _update_extended_ticks(self):
        if not self.configured:
            return
        try:
            cur = float(self._bus.read("Present_Position", self._motor_name, normalize=False))
            delta = cur - self._last_tick
            half = self._ticks_per_rev * 0.5
            if delta > +half:
                delta -= self._ticks_per_rev
            elif delta < -half:
                delta += self._ticks_per_rev
            self._extended_ticks += delta
            self._last_tick = cur
        except Exception as e:
            pass

    def _extended_deg(self):
        return self.dir_sign * self._extended_ticks * self._deg_per_tick

    def get_height_mm(self):
        if not self.configured:
            return 0.0
        try:
            self._update_extended_ticks()
        except Exception as e:
            pass
        raw_mm = (self._extended_deg() - self._z0_deg) * self._mm_per_deg
        return raw_mm

    def home(self, use_current=True):
        if not self.configured:
            return
        name = self._motor_name
        home_up_speed = -1000
        home_stall_current_ma = 450
        home_backoff_mm = 50.0
        try:
            self._bus.write("Torque_Enable", name, 1, normalize=False)
            self._bus.write("Lock", name, 1, normalize=False)
            self._bus.write("Goal_Velocity", name, home_up_speed, normalize=False)
        except Exception as e:
            return
        stuck = 0
        try:
            last_tick = int(self._bus.read("Present_Position", name, normalize=False))
        except Exception as e:
            last_tick = 0
        for _ in range(600):
            time.sleep(0.5)
            self._update_extended_ticks()
            now_tick = self._last_tick
            moved = abs(now_tick - last_tick) > 10
            last_tick = now_tick
            cur_ma = 0
            if use_current:
                try:
                    raw_cur_ma = int(self._bus.read("Present_Current", name, normalize=False))
                    cur_ma = raw_cur_ma * 6.5
                except Exception:
                    cur_ma = 0
            if (use_current and cur_ma >= home_stall_current_ma) or (not moved):
                stuck += 1
            else:
                stuck = 0
            if stuck >= 2:
                break
        try:
            self._bus.write("Goal_Velocity", name, 0, normalize=False)
        except Exception as e:
            pass
        time.sleep(0.5)
        try:
            self._bus.write("Goal_Velocity", name, -home_up_speed, normalize=False)
        except Exception as e:
            pass
        time.sleep(3.5)
        try:
            self._bus.write("Goal_Velocity", name, 0, normalize=False)
        except Exception as e:
            pass
        self._update_extended_ticks()
        self._z0_deg = self._extended_deg()
        self.target_height_mm = 0.0

    def set_height_mm(self, target_mm):
        self.target_height_mm = float(np.clip(target_mm, self.min_height_mm, self.max_height_mm))

    def compute_velocity(self):
        if not self.configured:
            return 0
        try:
            cur_mm = self.get_height_mm()
        except Exception as e:
            return 0
        err = self.target_height_mm - cur_mm
        if abs(err) <= self.on_target_mm:
            v_cmd = 0
        else:
            v_cmd = self.kp_vel * err
            if v_cmd > self.v_max:
                v_cmd = self.v_max
            elif v_cmd < -self.v_max:
                v_cmd = -self.v_max
        if v_cmd < 0 and cur_mm <= self.descent_floor_mm:
            v_cmd = 0
        if (cur_mm >= self.max_height_mm and v_cmd > 0) or (cur_mm <= self.min_height_mm and v_cmd < 0):
            v_cmd = 0
        return int(self.dir_sign * v_cmd)

    def move_to_zero_position(self):
        self.target_height_mm = 0.0

    def handle_keys(self, key_state):
        if key_state.get('vertical_lift+'):
            self.target_height_mm = min(self.target_height_mm + self.step_mm, self.max_height_mm)
        if key_state.get('vertical_lift-'):
            self.target_height_mm = max(self.target_height_mm - self.step_mm, self.min_height_mm)
    
    def _update_params(self):
        params = LIFT_SPEED_LEVELS[self.speed_level]
        self.kp_vel = params["kp_vel"]
        self.v_max = params["v_max"]
        self.step_mm = params["step"]
    
    def update_speed_level(self, level):
        if 1 <= level <= 4:
            self.speed_level = level
            self._update_params()

# ================== 键盘状态获取函数 ==================
def get_keyboard_key_state(stdscr):
    state = {
        # 底盘控制
        'base_forward': False,
        'base_backward': False,
        'base_left': False,
        'base_right': False,
        
        # 升降控制
        'vertical_lift+': False,
        'vertical_lift-': False,
        
        # 速度控制
        'speed+': False,
        'speed-': False,
        
        # 特殊功能
        'reset': False,
        'torque_toggle': False,
        
        # 左臂控制
        'left_shoulder_pan-': False,
        'left_shoulder_pan+': False,
        'left_shoulder_lift-': False,
        'left_shoulder_lift+': False,
        'left_elbow_flex-': False,
        'left_elbow_flex+': False,
        'left_wrist_flex-': False,
        'left_wrist_flex+': False,
        'left_wrist_roll-': False,
        'left_wrist_roll+': False,
        'left_gripper+': False,
        
        # 右臂控制
        'right_shoulder_pan-': False,
        'right_shoulder_pan+': False,
        'right_shoulder_lift-': False,
        'right_shoulder_lift+': False,
        'right_elbow_flex-': False,
        'right_elbow_flex+': False,
        'right_wrist_flex-': False,
        'right_wrist_flex+': False,
        'right_wrist_roll-': False,
        'right_wrist_roll+': False,
        'right_gripper+': False,
    }
    
    if stdscr is None:
        return state
    
    try:
        key = stdscr.getch()
        while key != -1:
            for action, key_code in KEYBOARD_KEYMAP.items():
                if key == key_code:
                    state[action] = True
            key = stdscr.getch()
    except Exception:
        pass
    
    return state

# ================== 底盘控制函数 ==================
def get_base_action(key_state, speed_level=2):
    base_state = "IDLE"
    v_cmd = 0.0
    omega_cmd_degps = 0.0
    
    speed_config = BASE_SPEED_LEVELS[speed_level]
    lin_speed = speed_config["lin"]
    ang_speed = speed_config["ang"]
    
    if key_state.get('base_forward'):
        v_cmd = lin_speed
        base_state = "FORWARD"
    elif key_state.get('base_backward'):
        v_cmd = -lin_speed
        base_state = "BACKWARD"
    
    if key_state.get('base_left'):
        omega_cmd_degps = ang_speed
        if base_state == "IDLE":
            base_state = "ROTATE LEFT"
        else:
            base_state += " + ROTATE LEFT"
    elif key_state.get('base_right'):
        omega_cmd_degps = -ang_speed
        if base_state == "IDLE":
            base_state = "ROTATE RIGHT"
        else:
            base_state += " + ROTATE RIGHT"
    
    wheel_cmds = body_to_wheel_raw(v_cmd, omega_cmd_degps)
    return wheel_cmds, base_state

# ================== 主节点类 ==================
class AlohaXTeleopNode(Node):
    def __init__(self, port1=DEFAULT_PORT1, port2=DEFAULT_PORT2, display=None):
        super().__init__('alohax_teleop_keyboard')

        self.port1 = port1
        self.port2 = port2

        self.bus1 = FeetechMotorsBus(port1, BUS1_MOTORS)
        self.bus2 = FeetechMotorsBus(port2, BUS2_MOTORS)
        self.bus1_connected = False
        self.bus2_connected = False
        self.active_bus1 = []
        self.active_bus2 = []

        self.cmd: dict[str, float] = {k: 0.0 for k in {**BUS1_MOTORS, **BUS2_MOTORS}}

        self.last_torque_toggle = False
        self.last_reset_press = False
        self.last_speed_plus_press = False
        self.last_speed_minus_press = False
        self.base_speed_level = 2
        
        self.is_homing = False
        self.normal_speed_level = 2
        self.homing_start_time = None
        self.homing_timeout = 10.0

        self.left_arm = SimpleTeleopArm(prefix="left", speed_level=self.base_speed_level)
        self.right_arm = SimpleTeleopArm(prefix="right", speed_level=self.base_speed_level)
        self.lift = ContinuousLiftControl(speed_level=self.base_speed_level)

        self.display = display
        self.torque_enabled = True
        
        self.last_callback_time = None

        self._connect_hardware()

        self.timer = self.create_timer(0.033, self.timer_callback)
    
    def _connect_hardware(self):
        try:
            self.bus1.connect(handshake=False)
            found1 = self.bus1.broadcast_ping() or {}
            self.active_bus1 = [name for name, motor in BUS1_MOTORS.items() if motor.id in found1]
            if self.active_bus1:
                self.get_logger().info(f"Bus1 connected: {self.active_bus1}")
                self._configure_bus(self.bus1, self.active_bus1, is_bus1=True)
                self.bus1_connected = True
            else:
                self.get_logger().warn("Bus1 connected but no motors found")
        except Exception as e:
            self.get_logger().warn(f"Bus1 connection failed: {e}")
        
        try:
            self.bus2.connect(handshake=False)
            found2 = self.bus2.broadcast_ping() or {}
            self.active_bus2 = [name for name, motor in BUS2_MOTORS.items() if motor.id in found2]
            if self.active_bus2:
                self.get_logger().info(f"Bus2 connected: {self.active_bus2}")
                self._configure_bus(self.bus2, self.active_bus2, is_bus1=False)
                self.bus2_connected = True
            else:
                self.get_logger().warn("Bus2 connected but no motors found")
        except Exception as e:
            self.get_logger().warn(f"Bus2 connection failed: {e}")
        
        if not self.active_bus1 and not self.active_bus2:
            raise RuntimeError("No motors found on either bus")
            
        try:
            self._init_cmd_to_home()
        except Exception as e:
            pass

        if LIFT_JOINT in self.active_bus1:
            self.lift.configure(self.bus1, LIFT_JOINT)
    
    def _set_base_cmd_zero(self):
        for k in BASE_WHEELS:
            self.cmd[k] = 0.0
    
    def _disable_arm_torque(self):
        self.torque_enabled = False
        if self.display:
            self.display.update_torque_status(self.torque_enabled)
        try:
            if self.bus1_connected:
                self.bus1.disable_torque(self.active_bus1)
        except Exception:
            pass
        try:
            if self.bus2_connected:
                self.bus2.disable_torque(self.active_bus2)
        except Exception:
            pass
    
    def _enable_arm_torque(self):
        self.torque_enabled = True
        if self.display:
            self.display.update_torque_status(self.torque_enabled)
        try:
            if self.bus1_connected:
                self.bus1.enable_torque(self.active_bus1)
        except Exception:
            pass
        try:
            if self.bus2_connected:
                self.bus2.enable_torque(self.active_bus2)
        except Exception:
            pass
    
    def _configure_bus(self, bus, active_motors, is_bus1):
        arm_params = ARM_SPEED_LEVELS[self.base_speed_level]
        
        for name in active_motors:
            try:
                bus.write("Return_Delay_Time", name, 0)
                bus.write("Maximum_Acceleration", name, arm_params["acc"])
                bus.write("Acceleration", name, arm_params["acc"])
            except Exception as e:
                pass
        
        position_motors = []
        if is_bus1:
            position_motors = [n for n in LEFT_ARM_JOINTS if n in active_motors]
        else:
            position_motors = [n for n in RIGHT_ARM_JOINTS if n in active_motors]
        
        for name in position_motors:
            try:
                bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
                if not name.endswith("gripper"):
                    bus.write("P_Coefficient", name, arm_params["p"])
                    bus.write("I_Coefficient", name, arm_params["i"])
                    bus.write("D_Coefficient", name, arm_params["d"])
                bus.write("Unloading_Condition", name, 0x2f)
            except Exception as e:
                pass
        
        if is_bus1:
            wheel_motors = [n for n in BASE_WHEELS if n in active_motors]
            for name in wheel_motors:
                try:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception as e:
                    pass

        if is_bus1:
            lift_motors = [n for n in LIFT_JOINTS if n in active_motors]
            for name in lift_motors:
                try:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception as e:
                    pass
        
        try:
            bus.enable_torque(active_motors)
        except Exception as e:
            pass
    
    def _update_motor_speed_params(self):
        arm_params = ARM_SPEED_LEVELS[self.base_speed_level]
        
        if self.bus1_connected:
            left_arm_motors = [n for n in LEFT_ARM_JOINTS if n in self.active_bus1]
            for name in left_arm_motors:
                try:
                    self.bus1.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus1.write("Acceleration", name, arm_params["acc"])
                    if not name.endswith("gripper"):
                        self.bus1.write("P_Coefficient", name, arm_params["p"])
                        self.bus1.write("I_Coefficient", name, arm_params["i"])
                        self.bus1.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
        
        if self.bus2_connected:
            right_arm_motors = [n for n in RIGHT_ARM_JOINTS if n in self.active_bus2]
            for name in right_arm_motors:
                try:
                    self.bus2.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus2.write("Acceleration", name, arm_params["acc"])
                    if not name.endswith("gripper"):
                        self.bus2.write("P_Coefficient", name, arm_params["p"])
                        self.bus2.write("I_Coefficient", name, arm_params["i"])
                        self.bus2.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
    
    def _update_motor_homing_speed_params(self):
        homing_speed_level = max(1, (self.normal_speed_level + 1) // 2)
        
        arm_params = ARM_SPEED_LEVELS[homing_speed_level]
        
        if self.bus1_connected:
            left_arm_motors = [n for n in LEFT_ARM_JOINTS if n in self.active_bus1]
            for name in left_arm_motors:
                try:
                    self.bus1.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus1.write("Acceleration", name, arm_params["acc"])
                    if not name.endswith("gripper"):
                        self.bus1.write("P_Coefficient", name, arm_params["p"])
                        self.bus1.write("I_Coefficient", name, arm_params["i"])
                        self.bus1.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
        
        if self.bus2_connected:
            right_arm_motors = [n for n in RIGHT_ARM_JOINTS if n in self.active_bus2]
            for name in right_arm_motors:
                try:
                    self.bus2.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus2.write("Acceleration", name, arm_params["acc"])
                    if not name.endswith("gripper"):
                        self.bus2.write("P_Coefficient", name, arm_params["p"])
                        self.bus2.write("I_Coefficient", name, arm_params["i"])
                        self.bus2.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
        
        self.lift.update_speed_level(homing_speed_level)
    
    def _check_homing_complete(self) -> bool:
        try:
            left_hw, right_hw, lift_hw, _, _, _ = self._read_hardware_states()
            
            left_complete = True
            for joint, target in self.left_arm.target_positions.items():
                if joint == 'gripper':
                    continue
                current = left_hw.get(joint, 0.0)
                if abs(target - current) > 0.1:
                    left_complete = False
                    break
            
            right_complete = True
            for joint, target in self.right_arm.target_positions.items():
                if joint == 'gripper':
                    continue
                current = right_hw.get(joint, 0.0)
                if abs(target - current) > 0.1:
                    right_complete = False
                    break
            
            lift_complete = True
            try:
                current_lift = self.lift.get_height_mm() / 1000.0
                if abs(self.lift.target_height_mm / 1000.0 - current_lift) > 0.01:
                    lift_complete = False
            except Exception:
                lift_complete = True
            
            return left_complete and right_complete and lift_complete
        except Exception:
            return False
    
    def _init_cmd_to_home(self):
        for k in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            if k.endswith("gripper"):
                calib = CALIBRATION.get(k, {})
                self.cmd[k] = float(calib.get("range_min", 2048))
            else:
                self.cmd[k] = 0.0
        for k in LIFT_JOINTS:
            calib = CALIBRATION.get(k, {})
            self.cmd[k] = float(calib.get("zero_position", 2000))
        for k in BASE_WHEELS:
            self.cmd[k] = 0.0
        self.get_logger().info("All joints initialized to home position")
    
    def _write_commands(self):
        if not (self.bus1_connected or self.bus2_connected):
            return
        
        if not self.torque_enabled:
            return

        try:
            if self.bus1_connected:
                left_cmd = {}
                for k in LEFT_ARM_JOINTS:
                    if k in self.active_bus1:
                        if k.endswith("gripper"):
                            raw_pos = int(round(self.cmd[k]))
                        else:
                            deg = float(self.cmd[k])
                            raw_pos = degrees_to_raw(k, deg)
                        left_cmd[k] = raw_pos
                if left_cmd:
                    self.bus1.sync_write("Goal_Position", left_cmd, normalize=False)

                wheel_cmd = {}
                for k in BASE_WHEELS:
                    if k in self.active_bus1:
                        wheel_cmd[k] = int(round(self.cmd[k] * BASE_WHEEL_SIGN.get(k, 1.0)))
                if wheel_cmd:
                    self.bus1.sync_write("Goal_Velocity", wheel_cmd, normalize=False)

                lift_vel = self.lift.compute_velocity()
                if LIFT_JOINT in self.active_bus1:
                    self.bus1.write("Goal_Velocity", LIFT_JOINT, lift_vel, normalize=False)

            if self.bus2_connected:
                right_cmd = {}
                for k in RIGHT_ARM_JOINTS:
                    if k in self.active_bus2:
                        if k.endswith("gripper"):
                            raw_pos = int(round(self.cmd[k]))
                        else:
                            deg = float(self.cmd[k])
                            raw_pos = degrees_to_raw(k, deg)
                        right_cmd[k] = raw_pos
                if right_cmd:
                    self.bus2.sync_write("Goal_Position", right_cmd, normalize=False)
                    
        except Exception as e:
            pass
    
    def _sync_read_multi_registers(self, bus, motors, start_addr, total_length, register_defs):
        try:
            ids = [bus.motors[motor].id for motor in motors]
            ids_values = {}
            
            bus.sync_reader.clearParam()
            bus.sync_reader.start_address = start_addr
            bus.sync_reader.data_length = total_length
            for id_ in ids:
                bus.sync_reader.addParam(id_)
            
            comm = bus.sync_reader.txRxPacket()
            if not bus._is_comm_success(comm):
                return {}
            
            for motor in motors:
                id_ = bus.motors[motor].id
                motor_data = {}
                
                for name, (offset, length) in register_defs.items():
                    reg_addr = start_addr + offset
                    try:
                        value = bus.sync_reader.getData(id_, reg_addr, length)
                        model = bus._id_to_model(id_)
                        encoding_table = bus.model_encoding_table.get(model, {})
                        if name in encoding_table:
                            value = bus._decode_sign(name, {id_: value})[id_]
                        motor_data[name] = value
                    except Exception:
                        motor_data[name] = 0
                
                ids_values[motor] = motor_data
            
            return ids_values
        except Exception:
            return {}
    
    def _read_hardware_states(self):
        left_hw = {}
        right_hw = {}
        lift_hw = 0.0
        left_wheel_hw = 0.0
        right_wheel_hw = 0.0
        
        left_current = {}
        left_load = {}
        right_current = {}
        right_load = {}
        lift_current = None
        lift_load = None
        left_wheel_current = None
        left_wheel_load = None
        right_wheel_current = None
        right_wheel_load = None

        register_map = {
            "Present_Position": (0, 2),
            "Present_Velocity": (2, 2),
            "Present_Load": (4, 2)
        }
        start_addr = 56
        total_length = 6

        try:
            if self.bus1_connected:
                active_left = [n for n in LEFT_ARM_JOINTS if n in self.active_bus1]
                active_lift = [n for n in LIFT_JOINTS if n in self.active_bus1]
                active_wheels = [n for n in BASE_WHEELS if n in self.active_bus1]
                
                if active_left:
                    multi_reg_data = self._sync_read_multi_registers(
                        self.bus1, active_left, start_addr, total_length, register_map
                    )
                    for k, data in multi_reg_data.items():
                        joint_name = k.replace('left_arm_', '')
                        left_hw[joint_name] = motor_cmd_to_joint_position(k, float(data.get("Present_Position", 0)))
                        left_load[joint_name] = data.get("Present_Load", 0)
                    
                    try:
                        current1 = self.bus1.sync_read("Present_Current", active_left, normalize=False)
                        for k, v in current1.items():
                            joint_name = k.replace('left_arm_', '')
                            left_current[joint_name] = int(v)
                    except Exception:
                        pass
                
                if active_lift:
                    pos_lift = self.bus1.sync_read("Present_Position", active_lift, normalize=False)
                    if LIFT_JOINT in pos_lift:
                        lift_hw = motor_cmd_to_joint_position(LIFT_JOINT, float(pos_lift[LIFT_JOINT]))
                    
                    try:
                        current_lift = self.bus1.sync_read("Present_Current", active_lift, normalize=False)
                        if LIFT_JOINT in current_lift:
                            lift_current = int(current_lift[LIFT_JOINT])
                    except Exception:
                        pass
                    
                    try:
                        load_lift = self.bus1.sync_read("Present_Load", active_lift, normalize=False)
                        if LIFT_JOINT in load_lift:
                            lift_load = int(load_lift[LIFT_JOINT])
                    except Exception:
                        pass
                
                if active_wheels:
                    pos_wheels = self.bus1.sync_read("Present_Position", active_wheels, normalize=False)
                    for k, v in pos_wheels.items():
                        if k == 'base_left_wheel':
                            left_wheel_hw = float(v)
                        elif k == 'base_right_wheel':
                            right_wheel_hw = float(v)
                    
                    try:
                        current_wheels = self.bus1.sync_read("Present_Current", active_wheels, normalize=False)
                        if 'base_left_wheel' in current_wheels:
                            left_wheel_current = int(current_wheels['base_left_wheel'])
                        if 'base_right_wheel' in current_wheels:
                            right_wheel_current = int(current_wheels['base_right_wheel'])
                    except Exception:
                        pass
                    
                    try:
                        load_wheels = self.bus1.sync_read("Present_Load", active_wheels, normalize=False)
                        if 'base_left_wheel' in load_wheels:
                            left_wheel_load = int(load_wheels['base_left_wheel'])
                        if 'base_right_wheel' in load_wheels:
                            right_wheel_load = int(load_wheels['base_right_wheel'])
                    except Exception:
                        pass

            if self.bus2_connected:
                active_right = [n for n in RIGHT_ARM_JOINTS if n in self.active_bus2]
                if active_right:
                    multi_reg_data = self._sync_read_multi_registers(
                        self.bus2, active_right, start_addr, total_length, register_map
                    )
                    for k, data in multi_reg_data.items():
                        joint_name = k.replace('right_arm_', '')
                        right_hw[joint_name] = motor_cmd_to_joint_position(k, float(data.get("Present_Position", 0)))
                        right_load[joint_name] = data.get("Present_Load", 0)
                    
                    try:
                        current2 = self.bus2.sync_read("Present_Current", active_right, normalize=False)
                        for k, v in current2.items():
                            joint_name = k.replace('right_arm_', '')
                            right_current[joint_name] = int(v)
                    except Exception:
                        pass

        except Exception as e:
            pass

        return left_hw, right_hw, lift_hw, left_wheel_hw, right_wheel_hw, \
               (left_current, left_load, right_current, right_load,
                lift_current, lift_load, left_wheel_current, left_wheel_load,
                right_wheel_current, right_wheel_load)
    
    def timer_callback(self):
        try:
            current_time = time.time()
            if self.last_callback_time is not None:
                period_ms = (current_time - self.last_callback_time) * 1000.0
                if self.display:
                    self.display.update_loop_period(period_ms)
            self.last_callback_time = current_time
            
            key_state = {}
            if self.display and self.display.stdscr:
                key_state = get_keyboard_key_state(self.display.stdscr)
            
            torque_toggle_pressed = key_state.get('torque_toggle', False)
            reset_pressed = key_state.get('reset', False)

            if torque_toggle_pressed and not self.last_torque_toggle:
                self.torque_enabled = not self.torque_enabled
                if self.display:
                    self.display.update_torque_status(self.torque_enabled)
                try:
                    if self.bus1_connected:
                        if self.torque_enabled:
                            self.bus1.enable_torque(self.active_bus1)
                        else:
                            self.bus1.disable_torque(self.active_bus1)
                    if self.bus2_connected:
                        if self.torque_enabled:
                            self.bus2.enable_torque(self.active_bus2)
                        else:
                            self.bus2.disable_torque(self.active_bus2)
                except Exception as e:
                    pass
            elif reset_pressed and not self.last_reset_press:
                if not self.is_homing:
                    self.is_homing = True
                    self.normal_speed_level = self.base_speed_level
                    self.homing_start_time = time.time()
                    self.left_arm.move_to_zero_position()
                    self.right_arm.move_to_zero_position()
                    self.lift.move_to_zero_position()
                    self._update_motor_homing_speed_params()
                    if self.display:
                        self.display.set_reset_flag(True)
            elif self.is_homing:
                current_time = time.time()
                homing_complete = self._check_homing_complete()
                homing_timed_out = (current_time - self.homing_start_time) > self.homing_timeout
                
                if homing_complete or homing_timed_out:
                    self.is_homing = False
                    self.base_speed_level = self.normal_speed_level
                    self.left_arm.update_speed_level(self.base_speed_level)
                    self.right_arm.update_speed_level(self.base_speed_level)
                    self.lift.update_speed_level(self.base_speed_level)
                    self._update_motor_speed_params()
                    if self.display:
                        self.display.set_reset_flag(False)
            else:
                if self.display:
                    self.display.set_reset_flag(False)
            self.last_torque_toggle = torque_toggle_pressed
            self.last_reset_press = reset_pressed
            
            speed_plus_pressed = key_state.get('speed+', False)
            speed_minus_pressed = key_state.get('speed-', False)

            if speed_plus_pressed and not self.last_speed_plus_press and not self.is_homing:
                self.base_speed_level = min(4, self.base_speed_level + 1)
                self.left_arm.update_speed_level(self.base_speed_level)
                self.right_arm.update_speed_level(self.base_speed_level)
                self.lift.update_speed_level(self.base_speed_level)
                self._update_motor_speed_params()
                if self.display:
                    self.display.update_base_speed_level(self.base_speed_level)
            self.last_speed_plus_press = speed_plus_pressed

            if speed_minus_pressed and not self.last_speed_minus_press and not self.is_homing:
                self.base_speed_level = max(1, self.base_speed_level - 1)
                self.left_arm.update_speed_level(self.base_speed_level)
                self.right_arm.update_speed_level(self.base_speed_level)
                self.lift.update_speed_level(self.base_speed_level)
                self._update_motor_speed_params()
                if self.display:
                    self.display.update_base_speed_level(self.base_speed_level)
            self.last_speed_minus_press = speed_minus_pressed

            if self.torque_enabled:
                left_arm_key_state = {
                    'shoulder_pan+': key_state.get('left_shoulder_pan+'),
                    'shoulder_pan-': key_state.get('left_shoulder_pan-'),
                    'shoulder_lift+': key_state.get('left_shoulder_lift+'),
                    'shoulder_lift-': key_state.get('left_shoulder_lift-'),
                    'elbow_flex+': key_state.get('left_elbow_flex+'),
                    'elbow_flex-': key_state.get('left_elbow_flex-'),
                    'wrist_flex+': key_state.get('left_wrist_flex+'),
                    'wrist_flex-': key_state.get('left_wrist_flex-'),
                    'wrist_roll+': key_state.get('left_wrist_roll+'),
                    'wrist_roll-': key_state.get('left_wrist_roll-'),
                    'gripper+': key_state.get('left_gripper+'),
                }
                self.left_arm.handle_keys(left_arm_key_state)

                right_arm_key_state = {
                    'shoulder_pan+': key_state.get('right_shoulder_pan+'),
                    'shoulder_pan-': key_state.get('right_shoulder_pan-'),
                    'shoulder_lift+': key_state.get('right_shoulder_lift+'),
                    'shoulder_lift-': key_state.get('right_shoulder_lift-'),
                    'elbow_flex+': key_state.get('right_elbow_flex+'),
                    'elbow_flex-': key_state.get('right_elbow_flex-'),
                    'wrist_flex+': key_state.get('right_wrist_flex+'),
                    'wrist_flex-': key_state.get('right_wrist_flex-'),
                    'wrist_roll+': key_state.get('right_wrist_roll+'),
                    'wrist_roll-': key_state.get('right_wrist_roll-'),
                    'gripper+': key_state.get('right_gripper+'),
                }
                self.right_arm.handle_keys(right_arm_key_state)

                lift_key_state = {
                    'vertical_lift+': key_state.get('vertical_lift+'),
                    'vertical_lift-': key_state.get('vertical_lift-'),
                }
                self.lift.handle_keys(lift_key_state)

                for joint in self.left_arm.target_positions:
                    full_name = f"left_arm_{joint}"
                    pos_rad = self.left_arm.target_positions[joint]
                    self.cmd[full_name] = joint_position_to_motor_cmd(full_name, pos_rad)

                for joint in self.right_arm.target_positions:
                    full_name = f"right_arm_{joint}"
                    pos_rad = self.right_arm.target_positions[joint]
                    self.cmd[full_name] = joint_position_to_motor_cmd(full_name, pos_rad)

                wheel_cmds, base_state = get_base_action(key_state, self.base_speed_level)
                self.cmd["base_left_wheel"] = float(wheel_cmds["base_left_wheel"])
                self.cmd["base_right_wheel"] = float(wheel_cmds["base_right_wheel"])
                if self.display:
                    self.display.update_base(base_state)
            else:
                self._set_base_cmd_zero()

            if self.bus1_connected or self.bus2_connected:
                try:
                    self._write_commands()
                except Exception as e:
                    pass

            left_hw, right_hw, lift_hw, left_wheel_hw, right_wheel_hw = {}, {}, 0.0, 0.0, 0.0
            motor_states = ({}, {}, {}, {}, None, None, None, None, None, None)
            if self.bus1_connected or self.bus2_connected:
                try:
                    left_hw, right_hw, _, left_wheel_hw, right_wheel_hw, motor_states = self._read_hardware_states()
                except Exception as e:
                    pass
                try:
                    lift_hw = self.lift.get_height_mm() / 1000.0
                except Exception as e:
                    pass

            if self.display:
                try:
                    self.display.update_left_arm(self.left_arm.target_positions)
                    self.display.update_right_arm(self.right_arm.target_positions)
                    self.display.update_lift(self.lift.target_height_mm / 1000.0)
                    self.display.update_lift_hw(lift_hw)
                    self.display.update_grippers(
                        self.left_arm.target_positions['gripper'],
                        self.right_arm.target_positions['gripper']
                    )
                    self.display.update_hw_arm_states(left_hw, right_hw)
                    self.display.update_arm_motor_states(*motor_states)
                    lift_current, lift_load, left_wheel_current, left_wheel_load, right_wheel_current, right_wheel_load = \
                        motor_states[4], motor_states[5], motor_states[6], motor_states[7], motor_states[8], motor_states[9]
                    self.display.update_lift_motor_states(lift_current, lift_load)
                    self.display.update_wheel_states(
                        self.cmd["base_left_wheel"], self.cmd["base_right_wheel"],
                        left_wheel_hw, right_wheel_hw,
                        left_wheel_current, left_wheel_load,
                        right_wheel_current, right_wheel_load
                    )
                    self.display.display()
                except Exception as e:
                    pass

        except Exception as e:
            pass

    def cleanup(self, keep_display=True):
        if not keep_display and self.display:
            self.display.cleanup()
        
        try:
            if self.bus1_connected:
                for name in BASE_WHEELS:
                    if name in self.active_bus1:
                        self.bus1.write("Goal_Velocity", name, 0, normalize=False)
        except Exception:
            pass
        
        try:
            if self.bus1_connected:
                self.bus1.disconnect(disable_torque=False)
        except Exception:
            pass
        
        try:
            if self.bus2_connected:
                self.bus2.disconnect(disable_torque=False)
        except Exception:
            pass

# ================== Lift 校准函数 ==================
def run_lift_calibration(port1):
    from alohax_hw_bridge.motors_bus import Motor, MotorNormMode
    from alohax_hw_bridge.motors.feetech.feetech import FeetechMotorsBus
    
    lift_motor = Motor(11, "sts3215", MotorNormMode.DEGREES)
    bus = FeetechMotorsBus(port1, {LIFT_JOINT: lift_motor})
    
    print("\n" + "=" * 50)
    print("LIFT CALIBRATION")
    print("=" * 50)
    print("Lift will move UP first to find hard limit,")
    print("then move DOWN 50mm to reference 0 position.")
    print("=" * 50)
    
    try:
        bus.connect(handshake=False)
        found = bus.broadcast_ping() or {}
        active_lift = [name for name, motor in {LIFT_JOINT: lift_motor}.items() if motor.id in found]
        
        if LIFT_JOINT not in active_lift:
            print(f"[ERROR] Lift motor (id=11) not found on {port1}")
            return False
        
        print(f"[OK] Lift motor found, starting calibration...")
        
        lift_ctrl = ContinuousLiftControl()
        lift_ctrl.configure(bus, LIFT_JOINT)
        lift_ctrl.home(use_current=True)
        
        bus.disconnect(disable_torque=False)
        print("[OK] Calibration completed successfully!")
        print("=" * 50 + "\n")
        return True
        
    except Exception as e:
        print(f"[ERROR] Calibration failed: {e}")
        import traceback
        traceback.print_exc()
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
        return False

# ================== 主函数 ==================
def parse_args():
    parser = argparse.ArgumentParser(description="AlohaX Keyboard Teleoperation - Direct Hardware Control")
    parser.add_argument("--port1", type=str, default=DEFAULT_PORT1, 
                       help=f"Serial port for left bus (default: {DEFAULT_PORT1})")
    parser.add_argument("--port2", type=str, default=DEFAULT_PORT2, 
                       help=f"Serial port for right bus (default: {DEFAULT_PORT2})")
    parser.add_argument("--no-display", action="store_true",
                       help="Disable curses display (use simple text output)")
    parser.add_argument("--no-calibrate", action="store_true",
                       help="Skip lift calibration before starting")
    return parser.parse_args()

def main(args=None):
    ros_initialized = False
    display_initialized = False
    
    import sys
    import os

    cli_args = parse_args()
    print(f"Parsed args: port1={cli_args.port1}, port2={cli_args.port2}, no_display={cli_args.no_display}, no_calibrate={cli_args.no_calibrate}")

    if not cli_args.no_calibrate:
        print("\n[LIFT CALIBRATION]")
        response = input("Do you want to run lift calibration? (y/N): ").strip().lower()
        if response == 'y' or response == 'yes':
            calib_ok = run_lift_calibration(cli_args.port1)
            if not calib_ok:
                response = input("Calibration failed. Continue anyway? (y/N): ").strip().lower()
                if response != 'y' and response != 'yes':
                    print("Exiting...")
                    return
        else:
            print("Skipping lift calibration")

    try:
        print("Initializing ROS...")
        rclpy.init(args=args)
        ros_initialized = True
        print("ROS initialized")

        print("Creating teleop node...")
        try:
            node = AlohaXTeleopNode(port1=cli_args.port1, port2=cli_args.port2, display=None)
        except Exception as e:
            print(f"Failed to create teleop node: {e}")
            import traceback
            traceback.print_exc()
            raise

        print("Initializing display...")
        if cli_args.no_display:
            print("Display disabled (--no-display flag set)")
            display = None
            display_initialized = False
        else:
            try:
                display = TeleopDisplay()
            except Exception as e:
                print(f"Failed to create TeleopDisplay: {e}")
                import traceback
                traceback.print_exc()
                raise
            
            try:
                display.init_curses()
                display_initialized = True
            except Exception as e:
                print(f"\n=== WARNING: Failed to initialize curses ===")
                print(f"  {type(e).__name__}: {e}")
                print(f"  Falling back to --no-display mode.")
                print(f"============================================\n")
                display = None
                display_initialized = False

        if display_initialized:
            node.display = display
            node.display.update_torque_status(node.torque_enabled)
        else:
            print("Starting teleop loop...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except RuntimeError as e:
        print(f"Error: {e}")
    except Exception as e:
        import traceback
        print(f"\nUnexpected error occurred: {e}")
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        
        if display_initialized:
            try:
                display.cleanup()
            except Exception:
                pass
        
        if 'node' in locals():
            try:
                node.cleanup(keep_display=False)
            except Exception:
                pass
        
        if ros_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        
        print("Cleanup complete")

if __name__ == '__main__':
    main()