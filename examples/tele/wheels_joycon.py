#!/usr/bin/env python3
"""
AlohaX Joycon Controller Teleoperation - Wheels and Lift Only

Hardware Configuration:
- bus1 (/dev/so101_L): 驱动轮(ID7-8) + 升降(ID11)

Controls (Right Joycon):
- X: 前进
- B: 后退
- Y: 左转
- A: 右转
- +/-: 速度档位调节

Controls (Left Joycon):
- Up/Down: 垂直升降 (+/-)

Home/Capture: 全局复位

底盘/升降控制: 直接控制Feetech电机，跳过ROS bridge
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
import serial
import serial.tools.list_ports
import threading
from contextlib import nullcontext

# 配置日志
# 文件处理器：记录所有级别
file_handler = logging.FileHandler('wheels_joycon.log')
file_handler.setLevel(logging.DEBUG)

# 控制台处理器：只记录 INFO 及以上，但 WARNING 和 ERROR 不显示
class InfoOnlyFilter(logging.Filter):
    def filter(self, record):
        return record.levelno >= logging.INFO

stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.INFO)
stream_handler.addFilter(InfoOnlyFilter())

# 设置格式
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
stream_handler.setFormatter(formatter)

# 配置根日志记录器
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(stream_handler)


def set_console_logging_enabled(enabled: bool):
    stream_handler.setLevel(logging.INFO if enabled else logging.CRITICAL + 1)


def try_release_serial_port(port_path):
    """尝试释放可能被占用的串口"""
    try:
        logger.info(f"Trying to release serial port: {port_path}")
        # 尝试打开并立即关闭串口，这有时能清除残留的占用
        ser = serial.Serial(
            port=port_path,
            baudrate=1000000,
            timeout=0.1
        )
        ser.close()
        logger.info(f"Successfully released port {port_path}")
        time.sleep(0.2)
        return True
    except serial.SerialException as e:
        if "Permission denied" in str(e):
            logger.warning(f"Permission denied when trying to access {port_path}")
        elif "Device or resource busy" in str(e) or "Port is in use" in str(e):
            logger.warning(f"Port {port_path} is still in use")
        else:
            logger.warning(f"Error accessing {port_path}: {e}")
        return False
    except Exception as e:
        logger.warning(f"Unexpected error when trying to release port: {e}")
        return False


import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joyconrobotics import JoyconRobotics
import joyconrobotics.joycon as joycon_core


def _patch_joycon_open():
    if getattr(joycon_core.JoyCon, "_wheels_open_patched", False):
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
        except Exception as e:
            logger.debug(
                "JoyCon HID open failed (vendor_id=%s, product_id=%s, serial=%s): %s",
                vendor_id,
                product_id,
                target_serial,
                e,
            )
            raise IOError("joycon connect failed") from e

    joycon_core.JoyCon._open = patched_open
    joycon_core.JoyCon._wheels_open_patched = True
    joycon_core.JoyCon._wheels_original_open = original_open


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
        self._read_joycon_data()
        self._setup_sensors()
        logger.info("JoyCon reconnected successfully (vendor_id=%s, product_id=%s)", vendor_id, product_id)
        return True
    except Exception as e:
        logger.warning(
            "JoyCon reconnect failed (vendor_id=%s, product_id=%s, serial=%s): %s",
            vendor_id,
            product_id,
            serial,
            e,
        )
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
                except Exception as callback_error:
                    logger.warning("JoyCon input hook failed: %s", callback_error)
        except Exception as e:
            logger.warning("JoyCon input thread error: %s", e)
            if not self.enable:
                break
            if not _joycon_reconnect(self):
                time.sleep(JOYCON_RECONNECT_INTERVAL_SEC)
            else:
                time.sleep(0.05)


joycon_core.JoyCon._update_input_report = _safe_update_input_report


def log_joycon_hid_devices():
    try:
        devices = hid.enumerate(0x057E, 0)
        if not devices:
            logger.warning("No Joy-Con HID devices found in hid.enumerate()")
            return

        for device in devices:
            logger.info(
                "Joy-Con HID device: path=%s product=%s serial=%s interface=%s",
                device.get("path"),
                device.get("product_string"),
                device.get("serial_number") or device.get("serial"),
                device.get("interface_number"),
            )
    except Exception as e:
        logger.warning("Failed to enumerate Joy-Con HID devices: %s", e)

src_path = Path(__file__).parents[2] / "src" / "alohax_hw_bridge"
sys.path.insert(0, str(src_path))

from alohax_hw_bridge.motors import Motor, MotorNormMode
from alohax_hw_bridge.motors.feetech import FeetechMotorsBus, OperatingMode

BUS1_MOTORS = {
    "base_left_wheel":  Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
    "base_right_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_0_100),
    "vertical_lift":    Motor(11, "sts3215", MotorNormMode.DEGREES),
}

BASE_WHEELS      = [k for k in BUS1_MOTORS if k.startswith("base")]
LIFT_JOINT       = "vertical_lift"
LIFT_JOINTS      = [LIFT_JOINT]

DEFAULT_PORT1: str = "/dev/so101_L"
STEPS_PER_DEG = 4096.0 / 360.0
BASE_WHEEL_SIGN = {"base_left_wheel": -1.0, "base_right_wheel": 1.0}
LIFT_DIRECTION_SIGN = -1.0
LIFT_TRAVEL_M = 0.25
LIFT_MOTOR_DEGREES_PER_METER = 144000.0
JOYCON_FRAME_TIMEOUT_SEC = 2.0
JOYCON_DISCOVERY_TIMEOUT_SEC = 10.0
JOYCON_DISCOVERY_INTERVAL_SEC = 0.5
JOYCON_OPEN_RETRY_COUNT = 10
JOYCON_OPEN_RETRY_INTERVAL_SEC = 0.5
JOYCON_RECONNECT_INTERVAL_SEC = 1.0

WHEEL_RADIUS: float = 0.05
WHEEL_BASE: float = 0.30
BASE_SPEED_LEVELS = {
    1: {"lin": 0.05, "ang": 20.0},
    2: {"lin": 0.1, "ang": 40.0},
    3: {"lin": 0.15, "ang": 70.0},
    4: {"lin": 0.5, "ang": 572.0},
}

CALIBRATION = {
    "vertical_lift": {"zero_position": 2000},
}


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


def motor_cmd_to_joint_position(name: str, cmd: float) -> float:
    if name == LIFT_JOINT:
        calib = CALIBRATION.get(LIFT_JOINT, {})
        zero = calib.get("zero_position", 2000)
        raw = cmd - zero
        lift_m = LIFT_DIRECTION_SIGN * raw / LIFT_MOTOR_DEGREES_PER_METER
        return float(np.clip(lift_m, -0.30, 0.05))
    else:
        degrees = raw_to_degrees(name, int(cmd))
        return math.radians(degrees)


def get_base_action(joycon, speed_level=2):
    base_state = "IDLE"
    v_cmd = 0.0
    omega_cmd_degps = 0.0

    speed_config = BASE_SPEED_LEVELS[speed_level]
    lin_speed = speed_config["lin"]
    ang_speed = speed_config["ang"]

    if joycon.joycon.is_right():
        if joycon.joycon.get_button_x():
            v_cmd = lin_speed
            base_state = "FORWARD"
        elif joycon.joycon.get_button_b():
            v_cmd = -lin_speed
            base_state = "BACKWARD"

        if joycon.joycon.get_button_y():
            omega_cmd_degps = ang_speed
            if base_state == "IDLE":
                base_state = "ROTATE LEFT"
            else:
                base_state += " + ROTATE LEFT"
        elif joycon.joycon.get_button_a():
            omega_cmd_degps = -ang_speed
            if base_state == "IDLE":
                base_state = "ROTATE RIGHT"
            else:
                base_state += " + ROTATE RIGHT"

    wheel_cmds = body_to_wheel_raw(v_cmd, omega_cmd_degps)
    return wheel_cmds, base_state


LEFT_KEYMAP = {
    "vertical_lift+": 'up', "vertical_lift-": 'down',
}


class TeleopDisplay:
    def __init__(self):
        self.lift_state = 0.0
        self.lift_hw_state = 0.0
        self.base_state = "INITIALIZING"
        self.base_speed_level = 2
        self.reset_flag = False
        self.stdscr = None
        self._first_render_logged = False
        self._display_error_logged = False

    def _log_file_only(self, level, message, *args):
        previous_level = stream_handler.level
        stream_handler.setLevel(logging.CRITICAL + 1)
        try:
            logger.log(level, message, *args)
        finally:
            stream_handler.setLevel(previous_level)

    def init_curses(self):
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.nodelay(True)
        self.stdscr.keypad(True)
        curses.curs_set(0)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        self.stdscr.erase()
        self.stdscr.refresh()
        self.display()

    def cleanup(self):
        if self.stdscr:
            curses.nocbreak()
            curses.curs_set(1)
            self.stdscr.nodelay(False)
            curses.echo()
            curses.endwin()

    def update_lift(self, position):
        self.lift_state = position

    def update_lift_hw(self, position):
        self.lift_hw_state = position

    def update_base(self, state):
        self.base_state = state

    def set_reset_flag(self, flag):
        self.reset_flag = flag

    def update_base_speed_level(self, level):
        self.base_speed_level = level

    def display(self):
        if not self.stdscr:
            return
        try:
            height, width = self.stdscr.getmaxyx()
            if height < 15 or width < 60:
                self.stdscr.erase()
                self.stdscr.addstr(0, 0, f"Window too small! Need 60x15, got {width}x{height}")
                self.stdscr.noutrefresh()
                curses.doupdate()
                return

            self.stdscr.erase()
            line = 0

            self.stdscr.addstr(line, 0, "=" * 60)
            line += 1
            self.stdscr.addstr(line, 0, "        AlohaX Wheels & Lift Joycon Control")
            line += 1
            self.stdscr.addstr(line, 0, "=" * 60)
            line += 2

            self.stdscr.addstr(line, 0, "[ VERTICAL LIFT ]")
            line += 1
            self.stdscr.addstr(line, 0, "-" * 30)
            line += 1
            lift_tgt_cm = self.lift_state * 100.0
            lift_hw_cm = self.lift_hw_state * 100.0
            lift_err = lift_tgt_cm - lift_hw_cm
            row = f"  Position: tgt:{lift_tgt_cm:6.1f} cm "
            self.stdscr.addstr(line, 0, row)
            self.stdscr.addstr(line, len(row), f"hw:{lift_hw_cm:6.1f} cm  ", curses.color_pair(1))
            self.stdscr.addstr(line, len(row) + 14, f"err:{lift_err:+6.1f}", curses.color_pair(2))
            line += 2

            self.stdscr.addstr(line, 0, "[ BASE CONTROL ]")
            line += 1
            self.stdscr.addstr(line, 0, "-" * 30)
            line += 1
            self.stdscr.addstr(line, 0, f"  Status:      {self.base_state}")
            line += 1
            self.stdscr.addstr(line, 0, f"  Speed Level: {self.base_speed_level}")
            line += 2

            self.stdscr.addstr(line, 0, "[ CONTROLS ]")
            line += 1
            self.stdscr.addstr(line, 0, "-" * 35)
            line += 1
            self.stdscr.addstr(line, 0, "  Right Joycon: X=Forward, B=Back, Y=Left, A=Right")
            line += 1
            self.stdscr.addstr(line, 0, "  Left Joycon: Up/Down = Lift")
            line += 1
            self.stdscr.addstr(line, 0, "  +/- = Speed Level (1-4)")
            line += 1
            self.stdscr.addstr(line, 0, "  Home/Capture = Reset")
            line += 1

            if self.reset_flag:
                self.stdscr.addstr(line, 0, "*** RESET TRIGGERED ***")
                line += 1

            line += 1
            self.stdscr.addstr(line, 0, "=" * 60)

            self.stdscr.noutrefresh()
            curses.doupdate()
            if not self._first_render_logged:
                self._log_file_only(logging.INFO, "Curses display rendered first frame")
                self._first_render_logged = True
        except curses.error:
            if not self._display_error_logged:
                self._log_file_only(logging.WARNING, "Curses display update failed")
                self._log_file_only(logging.WARNING, traceback.format_exc())
                self._display_error_logged = True
        except Exception:
            if not self._display_error_logged:
                self._log_file_only(logging.ERROR, "Unexpected display error")
                self._log_file_only(logging.ERROR, traceback.format_exc())
                self._display_error_logged = True


class ContinuousLiftControl:
    def __init__(self):
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

        self.kp_vel = 300
        self.v_max = 1300
        self.on_target_mm = 1.0
        self.dir_sign = -1.0
        self.step_mm = 2.0

        self.configured = False
        self._bus = None
        self._motor_name = None
        self._lock = None  # 串口访问锁

    def _read_voltage(self):
        """读取当前电压值，单位是 0.1V"""
        try:
            if self._bus and self._motor_name and self.configured:
                voltage = self._bus.read("Present_Voltage", self._motor_name, normalize=False)
                return voltage
        except Exception as e:
            logger.error(f"Failed to read voltage: {e}")
        return None

    def _handle_read_error(self, operation, error):
        """处理读取错误，特别是电压错误"""
        error_str = str(error).lower()
        if "voltage" in error_str:
            voltage = self._read_voltage()
            if voltage is not None:
                logger.error(f"{operation} failed due to voltage error. Current voltage: {voltage * 0.1}V")
            else:
                logger.error(f"{operation} failed due to voltage error, but could not read voltage")
        else:
            logger.error(f"{operation} failed: {error}")

    def configure(self, bus, motor_name, lock=None):
        if self.configured:
            return
        self._bus = bus
        self._motor_name = motor_name
        self._lock = lock
        try:
            with self._lock if self._lock else nullcontext():
                self._last_tick = float(self._bus.read("Present_Position", self._motor_name, normalize=False))
            self._extended_ticks = 0.0
            self.configured = True
        except Exception as e:
            self._handle_read_error("Configure lift control", e)
            raise

    def _update_extended_ticks(self):
        if not self.configured:
            return
        try:
            with self._lock if self._lock else nullcontext():
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
            self._handle_read_error("Update extended ticks", e)
            raise

    def _extended_deg(self):
        return self.dir_sign * self._extended_ticks * self._deg_per_tick

    def get_height_mm(self):
        if not self.configured:
            return 0.0
        self._update_extended_ticks()
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
            with self._lock if self._lock else nullcontext():
                self._bus.write("Torque_Enable", name, 1, normalize=False)
                self._bus.write("Lock", name, 1, normalize=False)
                self._bus.write("Goal_Velocity", name, home_up_speed, normalize=False)
        except Exception as e:
            logger.error(f"[ContinuousLiftControl.home] Write error: {e}")
            return
        stuck = 0
        try:
            with self._lock if self._lock else nullcontext():
                last_tick = int(self._bus.read("Present_Position", name, normalize=False))
        except Exception as e:
            self._handle_read_error("Read initial position for homing", e)
            return
        for _ in range(600):
            time.sleep(0.5)
            try:
                self._update_extended_ticks()
            except Exception as e:
                self._handle_read_error("Update ticks during homing", e)
                continue
            now_tick = self._last_tick
            moved = abs(now_tick - last_tick) > 10
            last_tick = now_tick
            cur_ma = 0
            if use_current:
                try:
                    with self._lock if self._lock else nullcontext():
                        raw_cur_ma = int(self._bus.read("Present_Current", name, normalize=False))
                    cur_ma = raw_cur_ma * 6.5
                    logger.info(f"[ContinuousLiftControl.home] Present_Current={cur_ma} mA")
                except Exception as e:
                    cur_ma = 0
                    logger.warning(f"[ContinuousLiftControl.home] Failed to read current: {e}")
            if (use_current and cur_ma >= home_stall_current_ma) or (not moved):
                logger.info(f"[ContinuousLiftControl.home] Stalled at current={cur_ma} mA, moved={moved}")
                stuck += 1
            else:
                stuck = 0
            if stuck >= 2:
                break
        try:
            with self._lock if self._lock else nullcontext():
                self._bus.write("Goal_Velocity", name, 0, normalize=False)
        except Exception as e:
            logger.error(f"[ContinuousLiftControl.home] Error stopping motor: {e}")
        time.sleep(0.5)
        try:
            with self._lock if self._lock else nullcontext():
                self._bus.write("Goal_Velocity", name, -home_up_speed, normalize=False)
        except Exception as e:
            logger.error(f"[ContinuousLiftControl.home] Error reversing motor: {e}")
        time.sleep(3.5)
        try:
            with self._lock if self._lock else nullcontext():
                self._bus.write("Goal_Velocity", name, 0, normalize=False)
        except Exception as e:
            logger.error(f"[ContinuousLiftControl.home] Error stopping motor: {e}")
        try:
            self._update_extended_ticks()
        except Exception as e:
            self._handle_read_error("Update ticks after homing", e)
            return
        self._z0_deg = self._extended_deg()
        self.target_height_mm = 0.0
        logger.info(f"[ContinuousLiftControl.home] z0_deg={self._z0_deg:.2f}, height={self.get_height_mm():.2f} mm")
        logger.info("[ContinuousLiftControl.home] Torque kept enabled to prevent drop")

    def set_height_mm(self, target_mm):
        self.target_height_mm = float(np.clip(target_mm, self.min_height_mm, self.max_height_mm))

    def compute_velocity(self):
        if not self.configured:
            return 0
        cur_mm = self.get_height_mm()
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


from joyconrobotics.device import get_R_id, get_L_id


def wait_for_joycon_id(device, timeout_sec=JOYCON_DISCOVERY_TIMEOUT_SEC, interval_sec=JOYCON_DISCOVERY_INTERVAL_SEC):
    deadline = time.monotonic() + timeout_sec
    last_log = 0.0

    while True:
        joycon_id = get_R_id() if device == "right" else get_L_id()
        if None not in joycon_id:
            return joycon_id

        now = time.monotonic()
        if now >= deadline:
            return joycon_id

        if now - last_log >= 1.0:
            logger.info("Waiting for %s Joycon to appear...", device)
            last_log = now

        time.sleep(interval_sec)


class FixedAxesJoyconRobotics:
    def __init__(self, device, **kwargs):
        joycon_id = wait_for_joycon_id(device)
        if None in joycon_id:
            raise RuntimeError(
                f"{device.capitalize()} Joycon not found after {JOYCON_DISCOVERY_TIMEOUT_SEC:.1f}s. "
                "Please pair the Joycon via Bluetooth first."
            )

        self.joycon = joycon_core.JoyCon(*joycon_id)
        self.device = device

        if self.joycon.is_right():
            self.joycon_stick_v_0 = 1900
            self.joycon_stick_h_0 = 2100
        else:
            self.joycon_stick_v_0 = 2300
            self.joycon_stick_h_0 = 2000
        self._last_input_frame_time = time.monotonic()
        self.joycon.register_update_hook(self._mark_input_frame_received)

    def get_stick_values(self):
        joycon_stick_v = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        joycon_stick_h = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_left_horizontal()
        return joycon_stick_v, joycon_stick_h

    def disconnect(self):
        if hasattr(self, "joycon") and self.joycon is not None:
            self.joycon._close()

    def _mark_input_frame_received(self, *_):
        self._last_input_frame_time = time.monotonic()

    def seconds_since_last_input_frame(self):
        return time.monotonic() - self._last_input_frame_time

    def is_input_frame_timed_out(self, timeout=JOYCON_FRAME_TIMEOUT_SEC):
        return self.seconds_since_last_input_frame() > timeout


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
            if control == 'up':
                state[action] = joycon.joycon.get_button_up() if not joycon.joycon.is_right() else False
            elif control == 'down':
                state[action] = joycon.joycon.get_button_down() if not joycon.joycon.is_right() else False
            elif control == 'left':
                state[action] = joycon.joycon.get_button_left() if not joycon.joycon.is_right() else False
            elif control == 'right':
                state[action] = joycon.joycon.get_button_right() if not joycon.joycon.is_right() else False
    except Exception:
        pass
    return state


class AlohaXTeleopNode(Node):
    def __init__(self, port1=DEFAULT_PORT1, display=None):
        super().__init__('wheels_joycon_teleop')

        self.port1 = port1
        self.bus1 = FeetechMotorsBus(port1, BUS1_MOTORS)
        self.bus1_connected = False
        self.active_bus1 = []
        self._serial_lock = threading.Lock()  # 添加串口访问锁

        self.cmd: dict[str, float] = {k: 0.0 for k in BUS1_MOTORS}

        self.joycon_right = None
        self.joycon_left = None

        self.lift = ContinuousLiftControl()
        self.last_home_press = False
        self.base_speed_level = 2
        self.last_plus_press = False
        self.last_minus_press = False
        self._joycon_timeout_state = {"left": False, "right": False}
        self._torque_enabled = False

        self.display = display

        self._connect_hardware()

        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info("AlohaX Wheels & Lift Joycon Teleoperation Node initialized")

    def _read_voltage(self, motor_name):
        """读取指定电机的电压值"""
        try:
            if self.bus1_connected and motor_name in self.active_bus1:
                voltage = self.bus1.read("Present_Voltage", motor_name, normalize=False)
                return voltage
        except Exception as e:
            logger.error(f"Failed to read voltage from {motor_name}: {e}")
        return None

    def _handle_hardware_error(self, operation, error, motor_name=None):
        """处理硬件错误，特别是电压错误"""
        error_str = str(error).lower()
        if "voltage" in error_str:
            if motor_name:
                voltage = self._read_voltage(motor_name)
            else:
                # 尝试读取升降电机的电压
                voltage = self._read_voltage(LIFT_JOINT) if LIFT_JOINT in self.active_bus1 else None
            if voltage is not None:
                logger.error(f"{operation} failed due to voltage error. Current voltage: {voltage * 0.1}V")
            else:
                logger.error(f"{operation} failed due to voltage error, but could not read voltage")
        else:
            logger.error(f"{operation} failed: {error}")

    def _connect_hardware(self):
        try:
            # 尝试释放串口
            try_release_serial_port(self.port1)
            
            self.bus1.connect(handshake=False)
            with self._serial_lock:
                found1 = self.bus1.broadcast_ping() or {}
            self.active_bus1 = [name for name, motor in BUS1_MOTORS.items() if motor.id in found1]
            if self.active_bus1:
                self.get_logger().info(f"Bus1 connected: {self.active_bus1}")
                logger.info(f"Bus1 connected: {self.active_bus1}")
                self._configure_bus(self.bus1, self.active_bus1)
                self.bus1_connected = True
            else:
                warn_msg = "Bus1 connected but no motors found"
                self.get_logger().warn(warn_msg)
                logger.warning(warn_msg)
        except Exception as e:
            error_msg = f"Bus1 connection failed: {e}"
            self.get_logger().warn(error_msg)
            logger.error(error_msg)

        if not self.active_bus1:
            raise RuntimeError("No motors found on bus1")

        try:
            self._init_cmd_to_home()
        except Exception as e:
            warn_msg = f"Could not initialize home position: {e}"
            self.get_logger().warn(warn_msg)
            logger.warning(warn_msg)

        if LIFT_JOINT in self.active_bus1:
            self.lift.configure(self.bus1, LIFT_JOINT, self._serial_lock)

    def _configure_bus(self, bus, active_motors):
        with self._serial_lock:
            for name in active_motors:
                try:
                    bus.write("Return_Delay_Time", name, 0)
                    bus.write("Maximum_Acceleration", name, 254)
                    bus.write("Acceleration", name, 254)
                except Exception as e:
                    warn_msg = f"Failed to configure {name}: {e}"
                    self.get_logger().warn(warn_msg)
                    logger.warning(warn_msg)

            wheel_motors = [n for n in BASE_WHEELS if n in active_motors]
            for name in wheel_motors:
                try:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception as e:
                    warn_msg = f"Failed to set velocity mode for {name}: {e}"
                    self.get_logger().warn(warn_msg)
                    logger.warning(warn_msg)

            lift_motors = [n for n in LIFT_JOINTS if n in active_motors]
            for name in lift_motors:
                try:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception as e:
                    warn_msg = f"Failed to set velocity mode for lift {name}: {e}"
                    self.get_logger().warn(warn_msg)
                    logger.warning(warn_msg)

            try:
                bus.enable_torque(active_motors)
                self._torque_enabled = True
            except Exception as e:
                warn_msg = f"Failed to enable torque: {e}"
                self.get_logger().warn(warn_msg)
                logger.warning(warn_msg)

    def _init_cmd_to_home(self):
        for k in LIFT_JOINTS:
            calib = CALIBRATION.get(k, {})
            self.cmd[k] = float(calib.get("zero_position", 2000))
        for k in BASE_WHEELS:
            self.cmd[k] = 0.0
        self.get_logger().info("All joints initialized to home position")

    def _set_all_torque(self, enabled: bool):
        if not self.bus1_connected:
            return

        if self._torque_enabled == enabled:
            return

        try:
            with self._serial_lock:
                if enabled:
                    self.bus1.enable_torque(self.active_bus1)
                else:
                    for name in self.active_bus1:
                        self.bus1.write("Torque_Enable", name, 0, normalize=False)
            self._torque_enabled = enabled
            logger.info("Torque %s for active bus1 motors", "enabled" if enabled else "disabled")
        except Exception as e:
            self._handle_hardware_error("Set torque state", e)

    def _set_base_cmd_zero(self):
        for k in BASE_WHEELS:
            self.cmd[k] = 0.0

    def _freeze_lift_target(self):
        if not self.lift.configured:
            return
        try:
            self.lift.target_height_mm = self.lift.get_height_mm()
            logger.info("Lift target frozen at current height after timeout")
        except Exception as e:
            self._handle_hardware_error("Freeze lift target", e, LIFT_JOINT)

    def _update_timeout_display(self, timeout_states):
        if not self.display:
            return
        if timeout_states.get("left") and timeout_states.get("right"):
            self.display.update_base("JOYCON_L/R TIMEOUT")
        elif timeout_states.get("left"):
            self.display.update_base("JOYCON_L TIMEOUT")
        elif timeout_states.get("right"):
            self.display.update_base("JOYCON_R TIMEOUT")

    def _is_joycon_fresh(self, joycon):
        return joycon is not None and not joycon.is_input_frame_timed_out()

    def _update_joycon_watchdogs(self):
        timeout_states = {}
        for side, joycon in (("left", self.joycon_left), ("right", self.joycon_right)):
            if joycon is None:
                timeout_states[side] = False
                continue

            timed_out = joycon.is_input_frame_timed_out()
            timeout_states[side] = timed_out

            if timed_out and not self._joycon_timeout_state[side]:
                logger.warning("%s joycon timed out for %.1fs", side.upper(), JOYCON_FRAME_TIMEOUT_SEC)
                if side == "left":
                    self._freeze_lift_target()
            elif not timed_out and self._joycon_timeout_state[side]:
                logger.info("%s joycon input recovered", side.upper())

            self._joycon_timeout_state[side] = timed_out

        any_timed_out = any(timeout_states.values())
        self._set_all_torque(not any_timed_out)
        return timeout_states

    def _write_commands(self):
        if not self.bus1_connected:
            return

        try:
            # 先在锁外计算升降速度，避免 compute_velocity() 内部读串口时和
            # 下面的 sync_write() 发生嵌套加锁/串口争用。
            lift_vel = self.lift.compute_velocity()

            with self._serial_lock:  # 使用锁保护串口访问
                wheel_cmd = {}
                for k in BASE_WHEELS:
                    if k in self.active_bus1:
                        wheel_cmd[k] = int(round(self.cmd[k] * BASE_WHEEL_SIGN.get(k, 1.0)))
                if wheel_cmd:
                    self.bus1.sync_write("Goal_Velocity", wheel_cmd, normalize=False)

                if LIFT_JOINT in self.active_bus1:
                    self.bus1.write("Goal_Velocity", LIFT_JOINT, lift_vel, normalize=False)

        except Exception as e:
            self._handle_hardware_error("Write commands", e)

    def _read_hardware_states(self):
        lift_hw = 0.0

        try:
            if self.bus1_connected:
                active_lift = [n for n in LIFT_JOINTS if n in self.active_bus1]
                if active_lift:
                    pos_lift = self.bus1.sync_read("Present_Position", active_lift, normalize=False)
                    if LIFT_JOINT in pos_lift:
                        lift_hw = motor_cmd_to_joint_position(LIFT_JOINT, float(pos_lift[LIFT_JOINT]))
        except Exception as e:
            self._handle_hardware_error("Read hardware states", e, LIFT_JOINT)

        return lift_hw

    def timer_callback(self):
        try:
            timeout_states = self._update_joycon_watchdogs()

            home_pressed = False
            capture_pressed = False
            if self._is_joycon_fresh(self.joycon_right):
                home_pressed = self.joycon_right.joycon.get_button_home()
            if self._is_joycon_fresh(self.joycon_left):
                capture_pressed = self.joycon_left.joycon.get_button_capture()

            if capture_pressed and not self.last_home_press:
                self.lift.move_to_zero_position()
                if self.display:
                    self.display.set_reset_flag(True)
            else:
                if self.display:
                    self.display.set_reset_flag(False)
            self.last_home_press = home_pressed or capture_pressed

            plus_pressed = self.joycon_right.joycon.get_button_plus() if self._is_joycon_fresh(self.joycon_right) else False
            minus_pressed = self.joycon_left.joycon.get_button_minus() if self._is_joycon_fresh(self.joycon_left) else False

            if plus_pressed and not self.last_plus_press:
                self.base_speed_level = min(4, self.base_speed_level + 1)
                if self.display:
                    self.display.update_base_speed_level(self.base_speed_level)
            self.last_plus_press = plus_pressed

            if minus_pressed and not self.last_minus_press:
                self.base_speed_level = max(1, self.base_speed_level - 1)
                if self.display:
                    self.display.update_base_speed_level(self.base_speed_level)
            self.last_minus_press = minus_pressed

            if self._is_joycon_fresh(self.joycon_left):
                left_key_state = get_joycon_key_state(self.joycon_left, LEFT_KEYMAP)
                self.lift.handle_keys(left_key_state)
            elif timeout_states.get("left"):
                pass

            if self._is_joycon_fresh(self.joycon_right):
                wheel_cmds, base_state = get_base_action(self.joycon_right, self.base_speed_level)
                self.cmd["base_left_wheel"] = float(wheel_cmds["base_left_wheel"])
                self.cmd["base_right_wheel"] = float(wheel_cmds["base_right_wheel"])
                if self.display:
                    self.display.update_base(base_state)

            if any(timeout_states.values()):
                self._set_base_cmd_zero()

            if self.bus1_connected:
                self._write_commands()

            lift_hw = 0.0
            if self.bus1_connected:
                try:
                    # 只调用一次 get_height_mm()，避免重复访问串口
                    lift_hw = self.lift.get_height_mm() / 1000.0
                except Exception as e:
                    self._handle_hardware_error("Get lift height", e, LIFT_JOINT)

            if self.display:
                self._update_timeout_display(timeout_states)
                self.display.update_lift(self.lift.target_height_mm / 1000.0)
                self.display.update_lift_hw(lift_hw)
                self.display.display()

        except Exception as e:
            import traceback
            error_msg = f"Error in timer callback: {e}"
            self.get_logger().error(error_msg)
            logger.error(error_msg)
            logger.error(traceback.format_exc())
            for k in BASE_WHEELS:
                self.cmd[k] = 0.0

    def cleanup(self, keep_display=True):
        logger.info("Starting node cleanup...")
        if not keep_display and self.display:
            try:
                self.display.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up display: {e}")

        try:
            if self.bus1_connected:
                logger.info("Stopping all wheels...")
                with self._serial_lock:
                    for name in BASE_WHEELS:
                        if name in self.active_bus1:
                            try:
                                self.bus1.write("Goal_Velocity", name, 0, normalize=False)
                            except Exception as e:
                                logger.warning(f"Error stopping {name}: {e}")
        except Exception as e:
            logger.warning(f"Error during wheel stopping: {e}")

        try:
            if self.bus1_connected:
                logger.info("Disconnecting bus1...")
                self.bus1.disconnect(disable_torque=False)
                self.bus1_connected = False
                logger.info("Bus1 disconnected successfully")
        except Exception as e:
            logger.warning(f"Error disconnecting bus1: {e}")

        logger.info("Node cleanup complete")


def run_lift_calibration(port1):
    from alohax_hw_bridge.motors_bus import Motor, MotorNormMode
    from alohax_hw_bridge.motors.feetech.feetech import FeetechMotorsBus

    lift_motor = Motor(11, "sts3215", MotorNormMode.DEGREES)
    bus = None

    logger.info("\n" + "=" * 50)
    logger.info("LIFT CALIBRATION")
    logger.info("=" * 50)
    logger.info("Lift will move UP first to find hard limit,")
    logger.info("then move DOWN 50mm to reference 0 position.")
    logger.info("=" * 50 + "\n")

    try:
        # 尝试释放串口
        try_release_serial_port(port1)
        
        bus = FeetechMotorsBus(port1, {LIFT_JOINT: lift_motor})
        bus.connect(handshake=False)
        found = bus.broadcast_ping() or {}
        active_lift = [name for name, motor in {LIFT_JOINT: lift_motor}.items() if motor.id in found]

        if LIFT_JOINT not in active_lift:
            logger.error(f"Lift motor (id=11) not found on {port1}")
            return False

        logger.info("Lift motor found, starting calibration...")

        lift_ctrl = ContinuousLiftControl()
        lift_ctrl.configure(bus, LIFT_JOINT)
        lift_ctrl.home(use_current=True)

        logger.info("Calibration completed successfully!")
        logger.info("=" * 50 + "\n")
        return True

    except Exception as e:
        logger.error(f"Calibration failed: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return False

    finally:
        if bus is not None:
            try:
                logger.info("Disconnecting calibration bus...")
                bus.disconnect(disable_torque=False)
                # 给串口一点时间完全释放
                time.sleep(0.5)
            except Exception as e:
                logger.warning(f"Error disconnecting calibration bus: {e}")


def parse_args():
    parser = argparse.ArgumentParser(description="AlohaX Wheels & Lift Joycon Teleoperation")
    parser.add_argument("--port1", type=str, default=DEFAULT_PORT1,
                       help=f"Serial port for bus (default: {DEFAULT_PORT1})")
    parser.add_argument("--no-display", action="store_true",
                       help="Disable curses display")
    parser.add_argument("--no-calibrate", action="store_true",
                       help="Skip lift calibration before starting")
    return parser.parse_args()


def main(args=None):
    ros_initialized = False
    display_initialized = False
    joycon_right = None
    joycon_left = None
    need_disconnect_right = False
    need_disconnect_left = False
    node = None

    import os

    cli_args = parse_args()
    logger.info(f"Parsed args: port1={cli_args.port1}, no_display={cli_args.no_display}, no_calibrate={cli_args.no_calibrate}")

    if not cli_args.no_calibrate:
        logger.info("\n[LIFT CALIBRATION]")
        response = input("Do you want to run lift calibration? (y/N): ").strip().lower()
        if response == 'y':
            calib_ok = run_lift_calibration(cli_args.port1)
            if not calib_ok:
                response = input("Calibration failed. Continue anyway? (y/N): ").strip().lower()
                if response != 'y' and response != 'yes':
                    logger.info("Exiting...")
                    return
        else:
            logger.info("Skipping lift calibration")

    try:
        logger.info("Initializing ROS...")
        rclpy.init(args=args)
        ros_initialized = True
        logger.info("ROS initialized")

        logger.info("Step 3: Connecting to Joycons...")
        log_joycon_hid_devices()

        devnull = open(os.devnull, 'w')
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull

        try:
            joycon_right = FixedAxesJoyconRobotics("right")
            need_disconnect_right = True
        except Exception as e:
            logger.exception("Failed to connect right Joycon")
            logger.error("Right Joycon error: %s", e)
            joycon_right = None

        try:
            joycon_left = FixedAxesJoyconRobotics("left")
            need_disconnect_left = True
        except Exception as e:
            logger.exception("Failed to connect left Joycon")
            logger.error("Left Joycon error: %s", e)
            joycon_left = None

        sys.stdout = old_stdout
        sys.stderr = old_stderr

        if not joycon_right and not joycon_left:
            logger.error("\nError: No Joycon connected!")
            logger.error("Please ensure Joycons are paired via Bluetooth")
            raise RuntimeError("No Joycon connected")

        logger.info("Joycons connected successfully")

        logger.info("Step 4: Initializing display...")
        if cli_args.no_display:
            logger.info("Display disabled (--no-display flag set)")
            display = None
            display_initialized = False
        else:
            stdin_is_tty = sys.stdin.isatty()
            stdout_is_tty = sys.stdout.isatty()
            stderr_is_tty = sys.stderr.isatty()
            term = os.environ.get("TERM")

            logger.info(
                "Curses pre-check: stdin_isatty=%s, stdout_isatty=%s, stderr_isatty=%s, TERM=%s",
                stdin_is_tty,
                stdout_is_tty,
                stderr_is_tty,
                term,
            )

            try:
                display = TeleopDisplay()
            except Exception as e:
                logger.error(f"Failed to create TeleopDisplay: {e}")
                logger.error(traceback.format_exc())
                raise

            skip_reason = None
            if not stdin_is_tty:
                skip_reason = "stdin is not a TTY"
            elif not stdout_is_tty:
                skip_reason = "stdout is not a TTY"
            elif not term:
                skip_reason = "TERM is not set"
            elif str(term).lower() == "dumb":
                skip_reason = "TERM=dumb is not supported by curses"

            if skip_reason:
                logger.warning("\n=== WARNING: Curses display disabled ===")
                logger.warning("  Reason: %s", skip_reason)
                logger.warning("  Falling back to --no-display mode.")
                display = None
                display_initialized = False
            else:
                try:
                    display.init_curses()
                    display_initialized = True
                    set_console_logging_enabled(False)
                    logger.info("Curses display initialized successfully")
                except Exception as e:
                    logger.warning("\n=== WARNING: Failed to initialize curses ===")
                    logger.warning("  Reason: %s", e)
                    logger.warning("  Stack trace:\n%s", traceback.format_exc())
                    logger.warning("  Falling back to --no-display mode.")
                    display = None
                    display_initialized = False

        if display_initialized:
            rclpy.logging.set_logger_level("wheels_joycon_teleop", rclpy.logging.LoggingSeverity.WARN)

        logger.info("Step 5: Creating teleop node...")
        try:
            node = AlohaXTeleopNode(port1=cli_args.port1, display=display)
        except Exception as e:
            logger.error(f"Failed to create teleop node: {e}")
            logger.error(traceback.format_exc())
            raise

        if joycon_right:
            node.joycon_right = joycon_right
        if joycon_left:
            node.joycon_left = joycon_left

        logger.info("Step 6: Running teleop node... Press Ctrl+C to exit")

        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info("\nShutdown requested...")

    except Exception as e:
        logger.error(f"\nError: {e}")
        logger.error(traceback.format_exc())

    finally:
        set_console_logging_enabled(True)
        logger.info("Cleaning up...")
        if node is not None:
            try:
                logger.info("Cleaning up node...")
                node.cleanup(keep_display=True)
            except Exception as e:
                logger.warning(f"Error during node cleanup: {e}")

        if display_initialized and display:
            display.cleanup()

        if need_disconnect_right and joycon_right:
            try:
                joycon_right.disconnect()
            except Exception:
                pass

        if need_disconnect_left and joycon_left:
            try:
                joycon_left.disconnect()
            except Exception:
                pass

        if ros_initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass

        logger.info("Cleanup complete. Goodbye!")


if __name__ == "__main__":
    main()
