#!/usr/bin/env python3
"""
AlohaX Joycon Controller Teleoperation - Direct Hardware Control

Hardware Configuration:
- bus1 (/dev/so101_L): 左臂(ID1-6) + 驱动轮(ID7-8) + 升降(ID11)
- bus2 (/dev/so101_R): 右臂(ID1-6)

Controls (Right Joycon):
- Stick: 右臂控制 (左右=wrist_roll, 上下=elbow_flex, 按下+上下=wrist_flex)
- R/ZR + Stick: 右臂肩部控制 (左右=shoulder_pan, 上下=shoulder_lift)
- Y/A/X/B: 底盘移动 (Y=前进, A=后退, X=左转, B=右转)
- Plus: 右臂夹爪

Controls (Left Joycon):
- Stick: 左臂控制 (左右=wrist_roll, 上下=elbow_flex, 按下+上下=wrist_flex)
- L/ZL + Stick: 左臂肩部控制 (左右=shoulder_pan, 上下=shoulder_lift)
- Up/Down: 垂直升降 (+/-)
- Minus: 左臂夹爪

Home: 切换扭矩使能/禁用，Capture: 全局复位
注意: 所有关节位置均以弧度为单位

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
import hid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joyconrobotics import JoyconRobotics
import joyconrobotics.joycon as joycon_core


# 配置日志 - 完全抑制 Joycon 断开时的警告
logger = logging.getLogger(__name__)
logger.setLevel(logging.ERROR)  # 只记录 ERROR 及以上级别

# 配置根 logger 也抑制 WARNING
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
JOYCON_FRAME_TIMEOUT_SEC = 2.0
JOYCON_RECONNECT_INTERVAL_SEC = 1.0


# ================== Joycon 安全补丁 ==================
def _patch_joycon_open():
    if getattr(joycon_core.JoyCon, "_alohax_open_patched", False):
        return

    original_open = joycon_core.JoyCon._open

    def patched_open(self, vendor_id, product_id, serial):
        target_serial = serial if serial is not None else getattr(self, "serial", None)
        try:
            candidates = hid.enumerate(vendor_id, product_id)
            for device in candidates:
                device_serial = device.get("serial_number") or device.get("serial")
                if target_serial is not None and device_serial != target_serial:
                    continue

                path = device.get("path")
                if path:
                    return hid.Device(path=path)

            if hasattr(hid, "Device"):
                return hid.Device(vendor_id, product_id, target_serial)
            if hasattr(hid, "device"):
                _joycon_device = hid.device()
                _joycon_device.open(vendor_id, product_id, target_serial)
                return _joycon_device

            raise Exception("Implementation of hid is not recognized!")
        except Exception:
            raise IOError("joycon connect failed")

    joycon_core.JoyCon._open = patched_open
    joycon_core.JoyCon._alohax_open_patched = True
    joycon_core.JoyCon._alohax_original_open = original_open


_patch_joycon_open()


def _joycon_reconnect(self):
    vendor_id = getattr(self, "vendor_id", None)
    product_id = getattr(self, "product_id", None)
    serial = getattr(self, "serial", None)
    if vendor_id is None or product_id is None:
        return False

    try:
        if hasattr(self, "_joycon_device"):
            try:
                self._joycon_device.close()
            except Exception:
                pass
            try:
                del self._joycon_device
            except Exception:
                pass
    except Exception:
        pass

    try:
        self._joycon_device = self._open(vendor_id, product_id, serial)
        if hasattr(self, "_read_joycon_data"):
            self._read_joycon_data()
        if hasattr(self, "_setup_sensors"):
            self._setup_sensors()
        return True
    except Exception:
        return False


def _safe_update_input_report(self):
    while self.enable == True:
        try:
            report = self._read_input_report()
            while report[0] != 0x30:
                report = self._read_input_report()

            self._input_report = report

            for callback in list(self._input_hooks):
                try:
                    callback(self)
                except Exception:
                    pass
        except Exception:
            if not self.enable:
                break
            if not _joycon_reconnect(self):
                time.sleep(JOYCON_RECONNECT_INTERVAL_SEC)
            else:
                time.sleep(0.05)


joycon_core.JoyCon._update_input_report = _safe_update_input_report

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
# zero_position: 关节在 home 姿态时的 Present_Position 原始值
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
        # Lift limits: -0.30m (-30cm) to 0.05m (+5cm)
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
        # Lift limits: -0.30m (-30cm) to 0.05m (+5cm)
        return float(np.clip(lift_m, -0.30, 0.05))
    elif name.endswith("gripper"):
        calib = CALIBRATION.get(name, {})
        range_min = calib.get("range_min", 2048)
        range_max = calib.get("range_max", 3396)
        t = (cmd - range_min) / (range_max - range_min)
        rad = JOINT_LIMITS["gripper"]["min"] + t * (JOINT_LIMITS["gripper"]["max"] - JOINT_LIMITS["gripper"]["min"])
        return float(np.clip(rad, JOINT_LIMITS["gripper"]["min"], JOINT_LIMITS["gripper"]["max"]))
    else:
        # cmd is raw encoder value, convert to degrees first
        degrees = raw_to_degrees(name, int(cmd))
        return math.radians(degrees)


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

# ================== 按键映射配置 ==================
LEFT_KEYMAP = {
    'x+': 'stick_up', 'x-': 'stick_down',
    'y+': 'stick_right', 'y-': 'stick_left',
    'shoulder_pan+': 'l_right', 'shoulder_pan-': 'l_left',
    'shoulder_lift-': 'l_up', 'shoulder_lift+': 'l_down',
    'wrist_roll+': 'stick_right', 'wrist_roll-': 'stick_left',
    'elbow_flex-': 'stick_up', 'elbow_flex+': 'stick_down',
    'wrist_flex-': 'stick_pressed_up', 'wrist_flex+': 'stick_pressed_down',
    'gripper+': 'zl',
    "vertical_lift+": 'up', "vertical_lift-": 'down',
}

RIGHT_KEYMAP = {
    'x+': 'stick_up', 'x-': 'stick_down',
    'y+': 'stick_right', 'y-': 'stick_left',
    'shoulder_pan+': 'r_right', 'shoulder_pan-': 'r_left',
    'shoulder_lift-': 'r_up', 'shoulder_lift+': 'r_down',
    'wrist_roll+': 'stick_right', 'wrist_roll-': 'stick_left',
    'elbow_flex-': 'stick_up', 'elbow_flex+': 'stick_down',
    'wrist_flex-': 'stick_pressed_up', 'wrist_flex+': 'stick_pressed_down',
    'gripper+': 'zr',
}

JOINT_LIMITS = {
    "shoulder_pan":  {"min": -1.996, "max": 2.111},
    "shoulder_lift": {"min": -2.004, "max": 2.028},
    "elbow_flex":    {"min": -0.072, "max": 2.995},
    "wrist_flex":    {"min": -2.238, "max": 1.334},
    "wrist_roll":    {"min": -3.139, "max": 3.140},
    "gripper":       {"min": 0.002,  "max": 2.069},
}

# ================== 显示类 ==================
class TeleopDisplay:
    def __init__(self):
        self.left_arm_state = {}
        self.right_arm_state = {}
        self.left_arm_hw_state = {}
        self.right_arm_hw_state = {}
        # 新增：每个舵机的电流、负载状态
        self.left_arm_current = {}
        self.left_arm_load = {}
        self.right_arm_current = {}
        self.right_arm_load = {}
        # 新增：垂直升降和底盘左右轮状态
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
        self.torque_status = "ON"  # 添加扭矩状态显示
        self.stdscr = None
        self.loop_period_ms = 0.0  # 新增：主循环周期（毫秒）

    def init_curses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.nodelay(True)
        curses.curs_set(0)
        curses.start_color()
        # 初始化颜色对
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)  # 普通绿色
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK) # 普通黄色
        curses.init_pair(3, curses.COLOR_WHITE, curses.COLOR_BLACK)  # 亮白色
        curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_BLACK)  # 亮绿色
        curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_BLACK) # 亮黄色
        curses.init_pair(6, curses.COLOR_RED, curses.COLOR_BLACK)    # 亮红色

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
        """更新左右臂各电机的电流、负载状态"""
        self.left_arm_current = left_current
        self.left_arm_load = left_load
        self.right_arm_current = right_current
        self.right_arm_load = right_load
        # 新增：垂直升降和底盘左右轮的电流、负载
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
        """更新垂直升降的电流、负载状态"""
        self.lift_current = lift_current
        self.lift_load = lift_load
    
    def update_wheel_states(self, left_wheel_tgt, right_wheel_tgt,
                           left_wheel_hw, right_wheel_hw,
                           left_wheel_current, left_wheel_load,
                           right_wheel_current, right_wheel_load):
        """更新底盘左右轮的状态"""
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
            
            # 安全的 addstr 辅助函数 - 带样式
            def safe_addstr(l, c, text, attr=curses.color_pair(3) | curses.A_BOLD):
                try:
                    if l < height and c + len(text) <= width:
                        self.stdscr.addstr(l, c, text, attr)
                except curses.error:
                    pass
            
            # 定义颜色属性
            BRIGHT_WHITE = curses.color_pair(3) | curses.A_BOLD
            BRIGHT_GREEN = curses.color_pair(4) | curses.A_BOLD
            BRIGHT_YELLOW = curses.color_pair(5) | curses.A_BOLD
            BRIGHT_RED = curses.color_pair(6) | curses.A_BOLD
            
            # 先显示一个简单的标题，确保能看到
            safe_addstr(line, 0, "=" * min(100, width), BRIGHT_WHITE)
            line += 1
            # 显示标题和循环周期
            title_text = "AlohaX Joycon Teleop - Display OK!"
            period_text = f"Period: {self.loop_period_ms:.1f}ms"
            safe_addstr(line, 0, title_text, BRIGHT_WHITE)
            # 在右侧显示循环周期
            period_x = max(0, width - len(period_text) - 2)
            safe_addstr(line, period_x, period_text, BRIGHT_YELLOW)
            line += 1
            safe_addstr(line, 0, "=" * min(100, width), BRIGHT_WHITE)
            line += 1
            
            # Left Arm section
            safe_addstr(line, 0, "[ LEFT ARM ]", BRIGHT_WHITE)
            line += 1
            safe_addstr(line, 0, f"  {'Joint':12}  {'tgt':8} {'hw':8} {'err':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
            line += 1

            def fmt_single_row(label, val, hw, current, load):
                if line >= height:
                    return
                # 将弧度转换为角度
                val_deg = int(round(math.degrees(val)))
                hw_deg = int(round(math.degrees(hw)))
                err_deg = val_deg - hw_deg
                # 显示标签和目标值（亮白色）
                safe_addstr(line, 0, f"  {label:12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{val_deg:>8d} ", BRIGHT_WHITE)
                # 显示 hw（亮绿色）
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{hw_deg:>8d}  ", BRIGHT_GREEN)
                    
                    # 显示 err，根据绝对值判断颜色
                    err_color = BRIGHT_WHITE
                    if abs(err_deg) > 80:
                        err_color = BRIGHT_RED
                    elif abs(err_deg) > 50:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{err_deg:>+8d}", err_color)
                    
                    # 新增：显示电流、负载
                    # 处理 current 值，显示值 = 读取值 * 6.5 (mA)
                    display_current = None
                    if current is not None:
                        display_current = current * 6.5
                        # 只显示整数部分
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    # 根据 current 值判断颜色
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    # 处理 load 值的符号
                    display_load = None
                    if load is not None:
                        if load >= 1000:
                            display_load = 1000 - load
                        else:
                            display_load = load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    # 根据 load 值判断颜色
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

            # Left arm joints
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
            line += 1

            if line < height:
                # Right Arm section
                safe_addstr(line, 0, "[ RIGHT ARM ]", BRIGHT_WHITE)
                line += 1
                safe_addstr(line, 0, f"  {'Joint':12}  {'tgt':8} {'hw':8} {'err':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
                line += 1

                # Right arm joints
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
                line += 1

            if line < height:
                safe_addstr(line, 0, "[ VERTICAL LIFT ]", BRIGHT_WHITE)
                line += 1
                safe_addstr(line, 0, f"  {'Joint':12}  {'tgt(cm)':8} {'hw(cm)':8} {'err(cm)':8}    {'cur(mA)':8} {'load':8}", BRIGHT_WHITE)
                line += 1
                
                lift_tgt_cm = self.lift_state * 100.0
                lift_hw_cm = self.lift_hw_state * 100.0
                lift_err = lift_tgt_cm - lift_hw_cm
                
                # 显示垂直升降行
                safe_addstr(line, 0, f"  {'Lift':12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{lift_tgt_cm:>8.1f} ", BRIGHT_WHITE)
                
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{lift_hw_cm:>8.1f}  ", BRIGHT_GREEN)
                    
                    # 显示 err，根据绝对值判断颜色
                    err_color = BRIGHT_WHITE
                    if abs(lift_err) > 50:
                        err_color = BRIGHT_RED
                    elif abs(lift_err) > 20:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{lift_err:>+8.1f}", err_color)
                    
                    # 处理 current 值，显示值 = 读取值 * 6.5 (mA)
                    display_current = None
                    if self.lift_current is not None:
                        display_current = self.lift_current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    # 根据 current 值判断颜色
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    # 处理 load 值的符号
                    display_load = None
                    if self.lift_load is not None:
                        if self.lift_load >= 1000:
                            display_load = 1000 - self.lift_load
                        else:
                            display_load = self.lift_load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    # 根据 load 值判断颜色
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
                
                # 显示左轮行
                safe_addstr(line, 0, f"  {'LeftWheel':12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{self.left_wheel_state:>8.0f} ", BRIGHT_WHITE)
                
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{self.left_wheel_hw_state:>8.0f}  ", BRIGHT_GREEN)
                    
                    left_wheel_err = self.left_wheel_state - self.left_wheel_hw_state
                    # 显示 err，根据绝对值判断颜色
                    err_color = BRIGHT_WHITE
                    if abs(left_wheel_err) > 50:
                        err_color = BRIGHT_RED
                    elif abs(left_wheel_err) > 20:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{left_wheel_err:>+8.0f}", err_color)
                    
                    # 处理 current 值，显示值 = 读取值 * 6.5 (mA)
                    display_current = None
                    if self.left_wheel_current is not None:
                        display_current = self.left_wheel_current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    # 根据 current 值判断颜色
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    # 处理 load 值的符号
                    display_load = None
                    if self.left_wheel_load is not None:
                        if self.left_wheel_load >= 1000:
                            display_load = 1000 - self.left_wheel_load
                        else:
                            display_load = self.left_wheel_load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    # 根据 load 值判断颜色
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
                
                # 显示右轮行
                safe_addstr(line, 0, f"  {'RightWheel':12} ", BRIGHT_WHITE)
                safe_addstr(line, 14, f"{self.right_wheel_state:>8.0f} ", BRIGHT_WHITE)
                
                try:
                    if line < height and 22 + 8 <= width:
                        self.stdscr.addstr(line, 22, f"{self.right_wheel_hw_state:>8.0f}  ", BRIGHT_GREEN)
                    
                    right_wheel_err = self.right_wheel_state - self.right_wheel_hw_state
                    # 显示 err，根据绝对值判断颜色
                    err_color = BRIGHT_WHITE
                    if abs(right_wheel_err) > 50:
                        err_color = BRIGHT_RED
                    elif abs(right_wheel_err) > 20:
                        err_color = BRIGHT_YELLOW
                    if line < height and 32 + 8 <= width:
                        self.stdscr.addstr(line, 32, f"{right_wheel_err:>+8.0f}", err_color)
                    
                    # 处理 current 值，显示值 = 读取值 * 6.5 (mA)
                    display_current = None
                    if self.right_wheel_current is not None:
                        display_current = self.right_wheel_current * 6.5
                        current_str = f"{int(round(display_current)):>8d}"
                    else:
                        current_str = f"{'-':>8s}"
                    # 根据 current 值判断颜色
                    cur_color = BRIGHT_WHITE
                    if display_current is not None:
                        if display_current > 2000:
                            cur_color = BRIGHT_RED
                        elif display_current > 1000:
                            cur_color = BRIGHT_YELLOW
                    
                    # 处理 load 值的符号
                    display_load = None
                    if self.right_wheel_load is not None:
                        if self.right_wheel_load >= 1000:
                            display_load = 1000 - self.right_wheel_load
                        else:
                            display_load = self.right_wheel_load
                        load_str = f"{display_load:>8d}"
                    else:
                        load_str = f"{'-':>8s}"
                    # 根据 load 值判断颜色
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
                
                # 显示状态信息
                safe_addstr(line, 0, f"  Status: {self.base_state} | Speed: {self.base_speed_level} (Base/Arms/Lift)", BRIGHT_WHITE)
                line += 2

            if line < height:
                safe_addstr(line, 0, f"[ CONTROLS ] | R:X/B/Y/A=Move | L:Up/Dn=Lift | +=Speed | Home=Toggle Torque | Capture=Reset | Torque: {self.torque_status}", BRIGHT_WHITE)

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
        """根据当前速度档位更新移动步长"""
        self.degree_step = ARM_SPEED_LEVELS[self.speed_level]["step"]
    
    def update_speed_level(self, level):
        """更新速度档位"""
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
        """根据当前速度档位更新参数"""
        params = LIFT_SPEED_LEVELS[self.speed_level]
        self.kp_vel = params["kp_vel"]
        self.v_max = params["v_max"]
        self.step_mm = params["step"]
    
    def update_speed_level(self, level):
        """更新速度档位"""
        if 1 <= level <= 4:
            self.speed_level = level
            self._update_params()

# ================== Joycon 辅助类 ==================
from joyconrobotics.device import get_R_id, get_L_id


class FixedAxesJoyconRobotics(JoyconRobotics):
    def __init__(self, device, **kwargs):
        joycon_id = get_R_id() if device == "right" else get_L_id()
        if None in joycon_id:
            raise RuntimeError(
                f"{device.capitalize()} Joycon not found. "
                "Please pair the Joycon via Bluetooth first."
            )
        super().__init__(device, **kwargs)
        if self.joycon.is_right():
            self.joycon_stick_v_0 = 1900
            self.joycon_stick_h_0 = 2100
        else:
            self.joycon_stick_v_0 = 2300
            self.joycon_stick_h_0 = 2000
        self._last_input_frame_time = time.time()
        self.joycon.register_update_hook(self._mark_input_frame_received)
    
    def _mark_input_frame_received(self, *_):
        self._last_input_frame_time = time.time()
    
    def seconds_since_last_input_frame(self):
        return time.time() - self._last_input_frame_time
    
    def is_input_frame_timed_out(self, timeout=JOYCON_FRAME_TIMEOUT_SEC):
        return self.seconds_since_last_input_frame() > timeout

    def get_stick_values(self):
        joycon_stick_v = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        joycon_stick_h = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_left_horizontal()
        return joycon_stick_v, joycon_stick_h
    
    def reset_joycon(self):
        # 覆盖原方法，不打印校准信息
        self.gyro.calibrate()
        import time
        time.sleep(0.1)  # 减少等待时间
        self.gyro.reset_orientation
        self.orientation_sensor.reset_yaw()
    
    def common_update(self):
        # 覆盖原方法，移除按下 +/- 键时调用 reset_joycon() 的逻辑
        # Forward and Backward movement
        if self.if_close_y:
            self.position[1] = self.offset_position_m[1]
        
        if self.lerobot:
            joycon_stick_horizontal = 0.0
            joycon_stick_vertical = 0.0
            if self.joycon.is_right():
                joycon_stick_vertical = self.joycon.get_stick_right_vertical() - self.joycon_stick_v_0
                joycon_stick_horizontal = self.joycon.get_stick_right_horizontal() - self.joycon_stick_h_0
            else:
                joycon_stick_vertical = self.joycon.get_stick_left_vertical() - self.joycon_stick_v_0
                joycon_stick_horizontal = self.joycon.get_stick_left_horizontal() - self.joycon_stick_h_0
        
            if self.if_close_y:
                joycon_stick_horizontal = 0.0
        
            if joycon_stick_vertical > 500:
                self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
            elif joycon_stick_vertical < -500:
                self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
            else:
                self.position[2] = self.position[2]
        
            if joycon_stick_horizontal > 500:
                self.position[0] = self.position[0] + 0.001 * self.dof_speed[0] * 2.0
            elif joycon_stick_horizontal < -500:
                self.position[0] = self.position[0] - 0.001 * self.dof_speed[0] * 2.0
            else:
                self.position[0] = self.position[0]
        else:
            # joycon_stick_horizontal = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_left_horizontal()
            # joycon_stick_vertical = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
            if self.horizontal_stick_mode == "xz":
                joycon_stick_vertical, joycon_stick_horizontal = self.get_stick()

                if self.if_close_y:
                    joycon_stick_horizontal = 0.0

                if joycon_stick_vertical > 2000:
                    self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
                elif joycon_stick_vertical < 1500:
                    self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
                else:
                    self.position[2] = self.position[2]

                if joycon_stick_horizontal > 2000:
                    self.position[0] = self.position[0] - 0.001 * self.dof_speed[0] * 2.0
                elif joycon_stick_horizontal < 1500:
                    self.position[0] = self.position[0] + 0.001 * self.dof_speed[0] * 2.0
                else:
                    self.position[0] = self.position[0]
            elif self.horizontal_stick_mode == "yz":
                joycon_stick_vertical, joycon_stick_horizontal = self.get_stick()

                if self.if_close_y:
                    joycon_stick_horizontal = 0.0

                if joycon_stick_vertical > 2000:
                    self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
                elif joycon_stick_vertical < 1500:
                    self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
                else:
                    self.position[2] = self.position[2]

                if joycon_stick_horizontal > 2000:
                    self.position[1] = self.position[1] - 0.001 * self.dof_speed[1] * 2.0
                elif joycon_stick_horizontal < 1500:
                    self.position[1] = self.position[1] + 0.001 * self.dof_speed[1] * 2.0
                else:
                    self.position[1] = self.position[1]
        
        # Left and Right movement
        if self.pure_xz:
            if not self.lerobot:
                joycon_button_up = self.joycon.get_button_r() if self.joycon.is_right() else self.joycon.get_button_l()
                joycon_button_down = self.joycon.get_button_zr() if self.joycon.is_right() else self.joycon.get_button_zl()
                
                if self.change_down_to_gripper:
                    joycon_button_down = self.joycon.get_button_r_stick() if self.joycon.is_right() else self.joycon.get_button_l_stick()
                    joycon_button_down = self.joycon.get_button_zr() if self.joycon.is_right() else self.joycon.get_button_zl()
                
                if joycon_button_up:
                    self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
                elif joycon_button_down:
                    self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
                else:
                    self.position[2] = self.position[2]
        else:
            if not self.lerobot:
                joycon_button_up = self.joycon.get_button_r() if self.joycon.is_right() else self.joycon.get_button_l()
                joycon_button_down = self.joycon.get_button_zr() if self.joycon.is_right() else self.joycon.get_button_zl()
                
                if self.change_down_to_gripper:
                    joycon_button_down = self.joycon.get_button_r_stick() if self.joycon.is_right() else self.joycon.get_button_l_stick()
                    joycon_button_down = self.joycon.get_button_zr() if self.joycon.is_right() else self.joycon.get_button_zl()
                
                if joycon_button_up:
                    self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
                elif joycon_button_down:
                    self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
                else:
                    self.position[2] = self.position[2]
            
            if self.if_close_y:
                self.position[1] = self.offset_position_m[1]
            else:
                joycon_button_xup = self.joycon.get_button_x() if self.joycon.is_right() else self.joycon.get_button_up()
                joycon_button_xback = self.joycon.get_button_b() if self.joycon.is_right() else self.joycon.get_button_down()
                
                if joycon_button_xup:
                    self.position[1] = self.position[1] - 0.001 * self.dof_speed[1] * 2.0
                elif joycon_button_xback:
                    self.position[1] = self.position[1] + 0.001 * self.dof_speed[1] * 2.0
                else:
                    self.position[1] = self.position[1]
        
            # Pitch and Yaw movement
            if self.joycon.is_right():
                joycon_button_yup = self.joycon.get_button_y()
                joycon_button_yback = self.joycon.get_button_a()
            else:
                joycon_button_yup = self.joycon.get_button_left()
                joycon_button_yback = self.joycon.get_button_right()
            
            if joycon_button_yup:
                self.orientation_rad[1] = self.orientation_rad[1] + 0.01 * self.dof_speed[4]
            elif joycon_button_yback:
                self.orientation_rad[1] = self.orientation_rad[1] - 0.01 * self.dof_speed[4]
            else:
                self.orientation_rad[1] = self.orientation_rad[1]
            
            if self.lerobot:
                if self.pitch_down_double:
                    if self.orientation_rad[1] > self.offset_euler_rad[1] + 0.02:
                        self.orientation_rad[0] = self.orientation_rad[0] - (0.02 * self.dof_speed[3])
                    elif self.orientation_rad[1] < self.offset_euler_rad[1] - 0.02:
                        self.orientation_rad[0] = self.orientation_rad[0] + (0.02 * self.dof_speed[3])
                    else:
                        self.orientation_rad[0] = self.orientation_rad[0]
        
            if self.position[2] > self.offset_position_m[2] + 0.002: 
                self.position[2] = self.position[2] - 0.001 * self.dof_speed[2] * 2.0
            elif self.position[2] < self.offset_position_m[2] - 0.002:
                self.position[2] = self.position[2] + 0.001 * self.dof_speed[2] * 2.0
            else:
                self.position[2] = self.position[2]
            
            if self.orientation_rad[2] > self.offset_euler_rad[2] + 0.02 :
                self.yaw_diff = self.yaw_diff + (0.01 * self.dof_speed[5])  
            elif self.orientation_rad[2] < self.offset_euler_rad[2] - 0.02 :
                self.yaw_diff = self.yaw_diff - (0.01 * self.dof_speed[5])  
            else:
                self.yaw_diff = self.yaw_diff
            
            self.orientation_sensor.set_yaw_diff(self.yaw_diff)
            
            if self.orientation_rad[2] < (0.02 * self.dof_speed[5]) and self.orientation_rad[2] > (-0.02* self.dof_speed[5]):
                self.orientation_sensor.reset_yaw()
                self.yaw_diff = 0.0
                self.orientation_sensor.set_yaw_diff(self.yaw_diff)
        
        # 移除按钮事件处理中的 reset_joycon() 调用
        for event_type, status in self.button.events():
            # 不处理 plus/minus 键的重置逻辑
            if ((self.joycon.is_right() and event_type == 'zr') or (self.joycon.is_left() and event_type == 'zl')) and not self.change_down_to_gripper:
                self.gripper_toggle_button = status
            elif ((self.joycon.is_right() and event_type == 'stick_r_btn') or (self.joycon.is_left() and event_type == 'stick_l_btn')) and self.change_down_to_gripper:
                self.gripper_toggle_button = status
            else: 
                self.reset_button = 0
            
        if self.gripper_toggle_button == 1 :
            if self.gripper_state == self.gripper_open:
                self.gripper_state = self.gripper_close
            else:
                self.gripper_state = self.gripper_open
            self.gripper_toggle_button = 0

        # record
        if self.joycon.is_right():
            if self.next_episode_button == 1:
                self.button_control = 1
            elif self.restart_episode_button == 1:
                self.button_control = -1
            elif self.reset_button == 1:
                self.button_control = 8
            else:
                self.button_control = 0
        
        return self.position, self.gripper_state, self.button_control

def get_joycon_key_state(joycon, keymap):
    state = {}
    try:
        stick_v, stick_h = joycon.get_stick_values()
        stick_v_threshold = 500
        stick_h_threshold = 500
        stick_pressed = joycon.joycon.get_button_r_stick() if joycon.joycon.is_right() else joycon.joycon.get_button_l_stick()
        shoulder_button = joycon.joycon.get_button_r() if joycon.joycon.is_right() else joycon.joycon.get_button_l()
        shoulder_button_z = joycon.joycon.get_button_zr() if joycon.joycon.is_right() else joycon.joycon.get_button_zl()
        shoulder_active = shoulder_button or shoulder_button_z
        
        for action, control in keymap.items():
            if control == 'stick_up':
                state[action] = (not stick_pressed) and (not shoulder_active) and (stick_v - joycon.joycon_stick_v_0 < -stick_v_threshold)
            elif control == 'stick_down':
                state[action] = (not stick_pressed) and (not shoulder_active) and (stick_v - joycon.joycon_stick_v_0 > stick_v_threshold)
            elif control == 'stick_left':
                state[action] = (not stick_pressed) and (not shoulder_active) and (stick_h - joycon.joycon_stick_h_0 < -stick_h_threshold)
            elif control == 'stick_right':
                state[action] = (not stick_pressed) and (not shoulder_active) and (stick_h - joycon.joycon_stick_h_0 > stick_h_threshold)
            elif control == 'stick_pressed_up':
                state[action] = stick_pressed and (not shoulder_active) and (stick_v - joycon.joycon_stick_v_0 < -stick_v_threshold)
            elif control == 'stick_pressed_down':
                state[action] = stick_pressed and (not shoulder_active) and (stick_v - joycon.joycon_stick_v_0 > stick_v_threshold)
            elif control == 'stick_pressed_left':
                state[action] = stick_pressed and (not shoulder_active) and (stick_h - joycon.joycon_stick_h_0 < -stick_h_threshold)
            elif control == 'stick_pressed_right':
                state[action] = stick_pressed and (not shoulder_active) and (stick_h - joycon.joycon_stick_h_0 > stick_h_threshold)
            elif control == 'l_up' or control == 'r_up':
                state[action] = shoulder_active and (stick_v - joycon.joycon_stick_v_0 < -stick_v_threshold)
            elif control == 'l_down' or control == 'r_down':
                state[action] = shoulder_active and (stick_v - joycon.joycon_stick_v_0 > stick_v_threshold)
            elif control == 'l_left' or control == 'r_left':
                state[action] = shoulder_active and (stick_h - joycon.joycon_stick_h_0 < -stick_h_threshold)
            elif control == 'l_right' or control == 'r_right':
                state[action] = shoulder_active and (stick_h - joycon.joycon_stick_h_0 > stick_h_threshold)
            elif control == 'a':
                state[action] = joycon.joycon.get_button_a() if joycon.joycon.is_right() else False
            elif control == 'b':
                state[action] = joycon.joycon.get_button_b() if joycon.joycon.is_right() else False
            elif control == 'x':
                state[action] = joycon.joycon.get_button_x() if joycon.joycon.is_right() else False
            elif control == 'y':
                state[action] = joycon.joycon.get_button_y() if joycon.joycon.is_right() else False
            elif control == 'up':
                state[action] = joycon.joycon.get_button_up() if not joycon.joycon.is_right() else False
            elif control == 'down':
                state[action] = joycon.joycon.get_button_down() if not joycon.joycon.is_right() else False
            elif control == 'left':
                state[action] = joycon.joycon.get_button_left() if not joycon.joycon.is_right() else False
            elif control == 'right':
                state[action] = joycon.joycon.get_button_right() if not joycon.joycon.is_right() else False
            elif control == 'plus':
                state[action] = joycon.joycon.get_button_plus() if joycon.joycon.is_right() else False
            elif control == 'minus':
                state[action] = joycon.joycon.get_button_minus() if not joycon.joycon.is_right() else False
            elif control == 'zl':
                state[action] = joycon.joycon.get_button_zl() if not joycon.joycon.is_right() else False
            elif control == 'zr':
                state[action] = joycon.joycon.get_button_zr() if joycon.joycon.is_right() else False
            else:
                state[action] = False
    except Exception as e:
        # 如果Joycon读取失败，返回空状态
        for action in keymap:
            state[action] = False
    return state

# ================== 底盘控制函数 ==================
def get_base_action(joycon, speed_level=2):
    """获取底盘电机速度指令，直接返回原始电机速度值
    
    按键映射 (Joycon 实物)：
    - X = 前进
    - B = 后退
    - Y = 左转
    - A = 右转
    """
    base_state = "IDLE"
    v_cmd = 0.0
    omega_cmd_degps = 0.0
    
    speed_config = BASE_SPEED_LEVELS[speed_level]
    lin_speed = speed_config["lin"]
    ang_speed = speed_config["ang"]
    
    if joycon.joycon.is_right():
        if joycon.joycon.get_button_x():  # 实物 X = 前进
            v_cmd = lin_speed
            base_state = "FORWARD"
        elif joycon.joycon.get_button_b():  # 实物 B = 后退
            v_cmd = -lin_speed
            base_state = "BACKWARD"
        
        if joycon.joycon.get_button_y():  # 实物 Y = 左转
            omega_cmd_degps = ang_speed
            if base_state == "IDLE":
                base_state = "ROTATE LEFT"
            else:
                base_state += " + ROTATE LEFT"
        elif joycon.joycon.get_button_a():  # 实物 A = 右转
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
        super().__init__('alohax_teleop_joycon')

        self.port1 = port1
        self.port2 = port2

        self.bus1 = FeetechMotorsBus(port1, BUS1_MOTORS)
        self.bus2 = FeetechMotorsBus(port2, BUS2_MOTORS)
        self.bus1_connected = False
        self.bus2_connected = False
        self.active_bus1 = []
        self.active_bus2 = []

        self.cmd: dict[str, float] = {k: 0.0 for k in {**BUS1_MOTORS, **BUS2_MOTORS}}

        self.joycon_right = None
        self.joycon_left = None

        self.last_home_press = False
        self.base_speed_level = 2
        self.last_plus_press = False
        self.last_minus_press = False
        
        # 回零速度控制相关变量
        self.is_homing = False  # 是否正在回零
        self.normal_speed_level = 2  # 保存正常工作时的速度档位
        self.homing_start_time = None  # 回零开始时间
        self.homing_timeout = 10.0  # 回零超时时间(秒)
        
        # Joycon 超时保护相关变量
        self._joycon_timeout_state = {"left": False, "right": False}
        self._lift_frozen_height = None  # 超时时冻结的升降高度
        
        self.left_arm = SimpleTeleopArm(prefix="left", speed_level=self.base_speed_level)
        self.right_arm = SimpleTeleopArm(prefix="right", speed_level=self.base_speed_level)
        self.lift = ContinuousLiftControl(speed_level=self.base_speed_level)

        self.display = display

        self.torque_enabled = True  # 初始状态为扭矩使能
        
        self.last_callback_time = None  # 新增：记录上一次回调的时间

        self._connect_hardware()

        self.timer = self.create_timer(0.033, self.timer_callback)
    
    def _connect_hardware(self):
        """连接两个电机总线并配置电机"""
        # Bus1 连接
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
        
        # Bus2 连接
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
            
        # 初始化所有关节到home位置 (0度 → degrees_to_raw → zero_position)
        try:
            self._init_cmd_to_home()
        except Exception as e:
            pass

        if LIFT_JOINT in self.active_bus1:
            self.lift.configure(self.bus1, LIFT_JOINT)
    
    def _is_joycon_fresh(self, joycon):
        return joycon is not None and not joycon.is_input_frame_timed_out()
    
    def _set_base_cmd_zero(self):
        for k in BASE_WHEELS:
            self.cmd[k] = 0.0
    
    def _freeze_lift_target(self):
        if not self.lift.configured:
            return
        try:
            self._lift_frozen_height = self.lift.get_height_mm()
            self.lift.target_height_mm = self._lift_frozen_height
        except Exception:
            pass
    
    def _clear_all_targets(self):
        # 清零所有目标值，避免重新连接后产生冲击
        # 清零左臂目标位置
        for joint in self.left_arm.target_positions:
            self.left_arm.target_positions[joint] = 0.0
        # 清零右臂目标位置
        for joint in self.right_arm.target_positions:
            self.right_arm.target_positions[joint] = 0.0
        # 升降保持冻结高度，不需要清零
        # 清零所有电机命令
        for key in self.cmd:
            self.cmd[key] = 0.0
    
    def _disable_arm_torque(self):
        # 完全等同于 Home 按键卸载扭矩的操作
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
        # 完全等同于 Home 按键使能扭矩的操作
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
    
    def _update_timeout_display(self, timeout_states):
        if not self.display:
            return
        if timeout_states.get("left") and timeout_states.get("right"):
            self.display.update_base("JOYCON_L/R TIMEOUT")
        elif timeout_states.get("left"):
            self.display.update_base("JOYCON_L TIMEOUT")
        elif timeout_states.get("right"):
            self.display.update_base("JOYCON_R TIMEOUT")
    
    def _update_joycon_watchdogs(self):
        timeout_states = {}
        for side, joycon in (("left", self.joycon_left), ("right", self.joycon_right)):
            if joycon is None:
                timeout_states[side] = False
                continue
            
            timed_out = joycon.is_input_frame_timed_out()
            timeout_states[side] = timed_out
            
            if timed_out and not self._joycon_timeout_state[side]:
                # 只在刚超时时执行一次
                if side == "left":
                    self._freeze_lift_target()
            
            # 注意：这里不更新 _joycon_timeout_state，而是在 timer_callback 中处理完后再更新
            # 这样可以确保刚超时的检测是正确的
        
        return timeout_states
    
    def _configure_bus(self, bus, active_motors, is_bus1):
        """配置单个电机总线"""
        arm_params = ARM_SPEED_LEVELS[self.base_speed_level]
        
        for name in active_motors:
            try:
                bus.write("Return_Delay_Time", name, 0)
                bus.write("Maximum_Acceleration", name, arm_params["acc"])
                bus.write("Acceleration", name, arm_params["acc"])
            except Exception as e:
                pass
        
        # 位置模式电机（仅机械臂，不含升降）
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
        
        # 速度模式电机（仅bus1的驱动轮）
        if is_bus1:
            wheel_motors = [n for n in BASE_WHEELS if n in active_motors]
            for name in wheel_motors:
                try:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception as e:
                    pass

        # 升降电机使用速度模式（支持多圈连续转动）
        if is_bus1:
            lift_motors = [n for n in LIFT_JOINTS if n in active_motors]
            for name in lift_motors:
                try:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception as e:
                    pass
        
        # 使能扭矩
        try:
            bus.enable_torque(active_motors)
        except Exception as e:
            pass
    
    def _update_motor_speed_params(self):
        """根据当前速度档位更新所有电机的PID参数和加速度"""
        arm_params = ARM_SPEED_LEVELS[self.base_speed_level]
        
        # 更新Bus1上的左臂电机
        if self.bus1_connected:
            left_arm_motors = [n for n in LEFT_ARM_JOINTS if n in self.active_bus1]
            for name in left_arm_motors:
                try:
                    self.bus1.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus1.write("Acceleration", name, arm_params["acc"])
                    # 夹爪不需要PID参数
                    if not name.endswith("gripper"):
                        self.bus1.write("P_Coefficient", name, arm_params["p"])
                        self.bus1.write("I_Coefficient", name, arm_params["i"])
                        self.bus1.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
        
        # 更新Bus2上的右臂电机
        if self.bus2_connected:
            right_arm_motors = [n for n in RIGHT_ARM_JOINTS if n in self.active_bus2]
            for name in right_arm_motors:
                try:
                    self.bus2.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus2.write("Acceleration", name, arm_params["acc"])
                    # 夹爪不需要PID参数
                    if not name.endswith("gripper"):
                        self.bus2.write("P_Coefficient", name, arm_params["p"])
                        self.bus2.write("I_Coefficient", name, arm_params["i"])
                        self.bus2.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
    
    def _update_motor_homing_speed_params(self):
        """根据当前速度档位设置回零速度（在当前档位基础上再降低一半）"""
        # 计算回零用的速度档位（向下取整，但至少为1）
        homing_speed_level = max(1, (self.normal_speed_level + 1) // 2)
        
        arm_params = ARM_SPEED_LEVELS[homing_speed_level]
        
        # 更新Bus1上的左臂电机
        if self.bus1_connected:
            left_arm_motors = [n for n in LEFT_ARM_JOINTS if n in self.active_bus1]
            for name in left_arm_motors:
                try:
                    self.bus1.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus1.write("Acceleration", name, arm_params["acc"])
                    # 夹爪不需要PID参数
                    if not name.endswith("gripper"):
                        self.bus1.write("P_Coefficient", name, arm_params["p"])
                        self.bus1.write("I_Coefficient", name, arm_params["i"])
                        self.bus1.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
        
        # 更新Bus2上的右臂电机
        if self.bus2_connected:
            right_arm_motors = [n for n in RIGHT_ARM_JOINTS if n in self.active_bus2]
            for name in right_arm_motors:
                try:
                    self.bus2.write("Maximum_Acceleration", name, arm_params["acc"])
                    self.bus2.write("Acceleration", name, arm_params["acc"])
                    # 夹爪不需要PID参数
                    if not name.endswith("gripper"):
                        self.bus2.write("P_Coefficient", name, arm_params["p"])
                        self.bus2.write("I_Coefficient", name, arm_params["i"])
                        self.bus2.write("D_Coefficient", name, arm_params["d"])
                except Exception:
                    pass
        
        # 更新升降机构的回零速度
        self.lift.update_speed_level(homing_speed_level)
    
    def _check_homing_complete(self) -> bool:
        """检测回零是否完成"""
        try:
            # 检测所有关节是否到达目标位置
            left_hw, right_hw, lift_hw, _, _, _ = self._read_hardware_states()
            
            # 检查左臂
            left_complete = True
            for joint, target in self.left_arm.target_positions.items():
                if joint == 'gripper':
                    continue
                current = left_hw.get(joint, 0.0)
                if abs(target - current) > 0.1:  # 1度以内认为到达
                    left_complete = False
                    break
            
            # 检查右臂
            right_complete = True
            for joint, target in self.right_arm.target_positions.items():
                if joint == 'gripper':
                    continue
                current = right_hw.get(joint, 0.0)
                if abs(target - current) > 0.1:  # 1度以内认为到达
                    right_complete = False
                    break
            
            # 检查升降机构
            lift_complete = True
            try:
                current_lift = self.lift.get_height_mm() / 1000.0
                if abs(self.lift.target_height_mm / 1000.0 - current_lift) > 0.01:  # 1cm以内认为到达
                    lift_complete = False
            except Exception:
                lift_complete = True  # 如果读取失败，认为完成
            
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
        """写入所有电机命令到硬件"""
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
            # 允许负值运行，不输出 warning
            pass
    
    def _sync_read_multi_registers(self, bus, motors, start_addr, total_length, register_defs):
        """
        批量读取连续的寄存器
        
        Args:
            bus: 总线对象
            motors: 要读取的电机列表
            start_addr: 起始地址
            total_length: 要读取的总长度
            register_defs: 寄存器定义字典，格式为 {name: (offset, length)}
        
        Returns:
            字典，格式为 {motor: {name: value}}
        """
        try:
            # 获取电机ID列表
            ids = [bus.motors[motor].id for motor in motors]
            ids_values = {}
            
            # 直接设置 sync_reader 并执行读取
            bus.sync_reader.clearParam()
            bus.sync_reader.start_address = start_addr
            bus.sync_reader.data_length = total_length
            for id_ in ids:
                bus.sync_reader.addParam(id_)
            
            # 发送读取命令
            comm = bus.sync_reader.txRxPacket()
            if not bus._is_comm_success(comm):
                return {}
            
            # 解析每个电机的数据
            for motor in motors:
                id_ = bus.motors[motor].id
                motor_data = {}
                
                for name, (offset, length) in register_defs.items():
                    reg_addr = start_addr + offset
                    try:
                        value = bus.sync_reader.getData(id_, reg_addr, length)
                        # 解码符号（如果需要）
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
        
        # 新增：电机电流、负载
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

        # 定义连续寄存器的映射（从 Present_Position 开始）
        # Present_Position: addr 56, length 2 (offset 0)
        # Present_Velocity: addr 58, length 2 (offset 2)
        # Present_Load: addr 60, length 2 (offset 4)
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
                    # 批量读取 Present_Position, Present_Load 等
                    multi_reg_data = self._sync_read_multi_registers(
                        self.bus1, active_left, start_addr, total_length, register_map
                    )
                    for k, data in multi_reg_data.items():
                        joint_name = k.replace('left_arm_', '')
                        left_hw[joint_name] = motor_cmd_to_joint_position(k, float(data.get("Present_Position", 0)))
                        left_load[joint_name] = data.get("Present_Load", 0)
                    
                    # 仍然单独读取 Present_Current（地址 69 不连续）
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
                    
                    # 读取垂直升降的电流、负载
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
                    
                    # 读取底盘左右轮的电流、负载
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
                    # 批量读取 Present_Position, Present_Load 等
                    multi_reg_data = self._sync_read_multi_registers(
                        self.bus2, active_right, start_addr, total_length, register_map
                    )
                    for k, data in multi_reg_data.items():
                        joint_name = k.replace('right_arm_', '')
                        right_hw[joint_name] = motor_cmd_to_joint_position(k, float(data.get("Present_Position", 0)))
                        right_load[joint_name] = data.get("Present_Load", 0)
                    
                    # 仍然单独读取 Present_Current
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
            # 计算循环周期
            current_time = time.time()
            if self.last_callback_time is not None:
                period_ms = (current_time - self.last_callback_time) * 1000.0
                if self.display:
                    self.display.update_loop_period(period_ms)
            self.last_callback_time = current_time
            
            # 更新 Joycon 看门狗
            timeout_states = self._update_joycon_watchdogs()
            any_timed_out = any(timeout_states.values())
            
            # Home 按键检测
            home_pressed = False
            capture_pressed = False
            if self._is_joycon_fresh(self.joycon_right):
                home_pressed = self.joycon_right.joycon.get_button_home()
            if self._is_joycon_fresh(self.joycon_left):
                capture_pressed = self.joycon_left.joycon.get_button_capture()

            # Home 键：切换扭矩状态（仅在未超时时）
            if home_pressed and not self.last_home_press and not any_timed_out:
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
                    pass  # 隐藏错误日志
            # Capture 键：回零（仅在未超时时）
            elif capture_pressed and not self.last_home_press and not any_timed_out:
                if not self.is_homing:
                    # 开始回零，保存正常速度档位并切换到回零速度
                    self.is_homing = True
                    self.normal_speed_level = self.base_speed_level
                    self.homing_start_time = time.time()
                    self.left_arm.move_to_zero_position()
                    self.right_arm.move_to_zero_position()
                    self.lift.move_to_zero_position()
                    # 应用回零速度（当前档位的一半）
                    self._update_motor_homing_speed_params()
                    if self.display:
                        self.display.set_reset_flag(True)
            # 回零完成检测
            elif self.is_homing:
                current_time = time.time()
                homing_complete = self._check_homing_complete()
                homing_timed_out = (current_time - self.homing_start_time) > self.homing_timeout
                
                if homing_complete or homing_timed_out:
                    # 回零完成或超时，恢复正常速度
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
            self.last_home_press = home_pressed or capture_pressed
            
            # 速度档位控制（仅在非回零状态且未超时时响应）
            plus_pressed = self.joycon_right.joycon.get_button_plus() if self._is_joycon_fresh(self.joycon_right) else False
            minus_pressed = self.joycon_left.joycon.get_button_minus() if self._is_joycon_fresh(self.joycon_left) else False

            if plus_pressed and not self.last_plus_press and not self.is_homing and not any_timed_out:
                self.base_speed_level = min(4, self.base_speed_level + 1)
                # 更新机械臂和升降机构的速度档位
                self.left_arm.update_speed_level(self.base_speed_level)
                self.right_arm.update_speed_level(self.base_speed_level)
                self.lift.update_speed_level(self.base_speed_level)
                # 更新电机的PID参数和加速度
                self._update_motor_speed_params()
                if self.display:
                    self.display.update_base_speed_level(self.base_speed_level)
            self.last_plus_press = plus_pressed

            if minus_pressed and not self.last_minus_press and not self.is_homing and not any_timed_out:
                self.base_speed_level = max(1, self.base_speed_level - 1)
                # 更新机械臂和升降机构的速度档位
                self.left_arm.update_speed_level(self.base_speed_level)
                self.right_arm.update_speed_level(self.base_speed_level)
                self.lift.update_speed_level(self.base_speed_level)
                # 更新电机的PID参数和加速度
                self._update_motor_speed_params()
                if self.display:
                    self.display.update_base_speed_level(self.base_speed_level)
            self.last_minus_press = minus_pressed
            
            if any_timed_out:
                # ================== 超时保护逻辑 ==================
                self._set_base_cmd_zero()
                
                # 检查是否刚进入超时状态
                left_just_timed_out = timeout_states.get("left", False) and not self._joycon_timeout_state.get("left", False)
                right_just_timed_out = timeout_states.get("right", False) and not self._joycon_timeout_state.get("right", False)
                
                if left_just_timed_out or right_just_timed_out:
                    # 刚超时，清零所有目标值，卸载双臂扭矩
                    self._clear_all_targets()
                    self._disable_arm_torque()
                
                # 更新超时显示
                self._update_timeout_display(timeout_states)
            else:
                # ================== 正常控制逻辑 ==================
                # 检查是否刚从超时恢复
                left_just_recovered = not timeout_states.get("left", False) and self._joycon_timeout_state.get("left", False)
                right_just_recovered = not timeout_states.get("right", False) and self._joycon_timeout_state.get("right", False)
                
                if left_just_recovered or right_just_recovered:
                    # 刚恢复，重新使能扭矩
                    self._enable_arm_torque()
                    self._lift_frozen_height = None  # 清除冻结高度
                
                # ================== 左臂和升降控制 ==================
                if self._is_joycon_fresh(self.joycon_left):
                    left_key_state = get_joycon_key_state(self.joycon_left, LEFT_KEYMAP)
                    self.left_arm.handle_keys(left_key_state)
                    self.lift.handle_keys(left_key_state)
                    
                    # 更新命令字典 - 左臂
                    for joint in self.left_arm.target_positions:
                        full_name = f"left_arm_{joint}"
                        pos_rad = self.left_arm.target_positions[joint]
                        self.cmd[full_name] = joint_position_to_motor_cmd(full_name, pos_rad)
                
                # ================== 右臂和底盘控制 ==================
                if self._is_joycon_fresh(self.joycon_right):
                    right_key_state = get_joycon_key_state(self.joycon_right, RIGHT_KEYMAP)
                    self.right_arm.handle_keys(right_key_state)
                    
                    # 更新命令字典 - 右臂
                    for joint in self.right_arm.target_positions:
                        full_name = f"right_arm_{joint}"
                        pos_rad = self.right_arm.target_positions[joint]
                        self.cmd[full_name] = joint_position_to_motor_cmd(full_name, pos_rad)
                    
                    # 底盘直接控制
                    wheel_cmds, base_state = get_base_action(self.joycon_right, self.base_speed_level)
                    self.cmd["base_left_wheel"] = float(wheel_cmds["base_left_wheel"])
                    self.cmd["base_right_wheel"] = float(wheel_cmds["base_right_wheel"])
                    if self.display:
                        self.display.update_base(base_state)
            
            # ================== 写入硬件 ==================
            if self.bus1_connected or self.bus2_connected:
                try:
                    self._write_commands()
                except Exception as e:
                    pass
            
            # ================== 读取硬件状态并更新显示 ==================
            left_hw, right_hw, lift_hw, left_wheel_hw, right_wheel_hw = {}, {}, 0.0, 0.0, 0.0
            motor_states = ({}, {}, {}, {}, None, None, None, None, None, None)  # 默认空值
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
                    # 更新垂直升降和底盘左右轮的状态
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
            
            # 更新 Joycon 超时状态
            for side in ["left", "right"]:
                self._joycon_timeout_state[side] = timeout_states.get(side, False)
            
        except Exception as e:
            # 不打印错误，确保程序继续运行
            pass

    def cleanup(self, keep_display=True):
        # Joycon 清理由 main 函数处理
        if not keep_display and self.display:
            self.display.cleanup()
        
        # 停止所有电机
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
    parser = argparse.ArgumentParser(description="AlohaX Joycon Teleoperation - Direct Hardware Control")
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
    # 标记是否已初始化 ROS
    ros_initialized = False
    # 标记是否已初始化显示
    display_initialized = False
    # 标记是否已连接 Joycon
    joycon_right = None
    joycon_left = None
    # 标记是否需要断开 Joycon
    need_disconnect_right = False
    need_disconnect_left = False
    
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
        # 1. 初始化 ROS
        print("Initializing ROS...")
        rclpy.init(args=args)
        ros_initialized = True
        print("ROS initialized")

        # 2. 解析参数
        # cli_args already parsed above
        
        # 3. 先连接 Joycon（在初始化显示前）
        print("Step 3: Connecting to Joycons...")
        
        # 抑制 Joycon 库的输出
        devnull = open(os.devnull, 'w')
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull
        
        # 连接右 Joycon
        try:
            joycon_right = FixedAxesJoyconRobotics("right")
            need_disconnect_right = True
        except Exception as e:
            joycon_right = None
        
        # 连接左 Joycon
        try:
            joycon_left = FixedAxesJoyconRobotics("left")
            need_disconnect_left = True
        except Exception as e:
            joycon_left = None
        
        # 恢复输出
        sys.stdout = old_stdout
        sys.stderr = old_stderr
        
        # 检查是否有 Joycon 连接
        if not joycon_right and not joycon_left:
            print("\nError: No Joycon connected!")
            print("Please ensure:")
            print("  1. Joycons are paired via Bluetooth")
            print("  2. Hold SYNC button on each Joycon to connect")
            print("  3. Check with: bluetoothctl devices")
            raise RuntimeError("No Joycon connected")
        
        print("Joycons connected successfully")
        
        # 4. 先创建节点（此时不要显示，避免日志破坏界面）
        print("Step 4: Creating teleop node...")
        try:
            node = AlohaXTeleopNode(port1=cli_args.port1, port2=cli_args.port2, display=None)
        except Exception as e:
            print(f"Failed to create teleop node: {e}")
            import traceback
            traceback.print_exc()
            raise
        
        # 5. 将已连接的 Joycon 传递给节点
        if joycon_right:
            node.joycon_right = joycon_right
        if joycon_left:
            node.joycon_left = joycon_left
        
        # 6. 初始化显示（在节点创建之后，避免日志破坏界面）
        print("Step 5: Initializing display...")
        if cli_args.no_display:
            print("Display disabled (--no-display flag set)")
            display = None
            display_initialized = False
            print("Step 5 completed: Display skipped")
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
        
        # 7. 将 display 传递给节点
        if display_initialized:
            node.display = display
            node.display.update_torque_status(node.torque_enabled)
        else:
            print("Starting teleop loop...")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except RuntimeError as e:
        # 预期的错误（如没有 Joycon）
        print(f"Error: {e}")
    except Exception as e:
        import traceback
        print(f"\nUnexpected error occurred: {e}")
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        
        # 1. 断开 Joycon 连接
        if need_disconnect_right and joycon_right:
            print("Disconnecting right Joycon...")
            try:
                joycon_right.disconnect()
                print("Right Joycon disconnected")
            except Exception as e:
                print(f"Failed to disconnect right Joycon: {e}")
        
        if need_disconnect_left and joycon_left:
            print("Disconnecting left Joycon...")
            try:
                joycon_left.disconnect()
                print("Left Joycon disconnected")
            except Exception as e:
                print(f"Failed to disconnect left Joycon: {e}")
        
        # 2. 清理节点
        if 'node' in locals():
            print("Cleaning up node...")
            try:
                node.cleanup(keep_display=False)
                node.destroy_node()
                print("Node cleaned up")
            except Exception as e:
                print(f"Failed to cleanup node: {e}")
        
        # 3. 清理显示
        if display_initialized:
            print("Cleaning up display...")
            try:
                display.cleanup()
                print("Display cleaned up")
            except Exception as e:
                print(f"Failed to cleanup display: {e}")
        
        # 4. 关闭 ROS
        if ros_initialized:
            print("Shutting down ROS...")
            try:
                rclpy.shutdown()
                print("ROS shutdown complete")
            except Exception as e:
                print(f"Failed to shutdown ROS: {e}")
        
        print("Cleanup complete")

if __name__ == '__main__':
    main()
