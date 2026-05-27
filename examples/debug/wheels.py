#!/usr/bin/env python3
"""
2-wheel differential drive keyboard teleop demo (Feetech, new LeRobot API style).
- Only `--port` is a CLI argument; everything else is global constants.
- Two rear wheels are driven by differential drive (left/right motors).
- Two front wheels are passive caster wheels.
- Keys: A/D forward/back, W/S differential left/right, Q or ESC to quit.
  Alternative keys: Arrow keys (Left=left wheel backward, Right=right wheel backward,
  Up/Down=differential).

Dependencies:
  pip install pynput numpy
Usage:
  python wheels.py --port /dev/ttyACM0
"""

from __future__ import annotations

import sys
from pathlib import Path

# 添加项目源代码目录到 Python 路径
src_path = Path(__file__).parents[2] / "src" / "alohax_hw_bridge"
sys.path.insert(0, str(src_path))

import argparse
import time

import numpy as np
from pynput import keyboard

from alohax_hw_bridge.motors import Motor, MotorNormMode
from alohax_hw_bridge.motors.feetech import FeetechMotorsBus, OperatingMode

# ------------------------ Global constants (edit here) ------------------------ #
DEFAULT_PORT: str = "/dev/ttyACM0"
MODEL: str = "sts3215"  # Feetech model
LEFT_ID: int = 7  # Left wheel motor ID
RIGHT_ID: int = 8  # Right wheel motor ID
LIN_SPEED: float = 0.2  # Linear speed (m/s)
ANG_SPEED: float = 229.0  # Angular speed (deg/s) - calibrated to match linear speed
WHEEL_RADIUS: float = 0.05  # Wheel radius (m)
WHEEL_BASE: float = 0.30  # Distance between left and right wheels (m)
MAX_RAW: int = 3000  # Raw speed limit (scaled)


def degps_to_raw(degps: float) -> int:
    """Angular speed (deg/s) -> steps/s (-32767..+32767), no sign-bit encoding."""
    steps_per_deg = 4096.0 / 360.0
    mag = int(round(abs(degps) * steps_per_deg))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


def raw_to_degps(raw_speed: int) -> float:
    steps_per_deg = 4096.0 / 360.0
    magnitude = raw_speed & 0x7FFF
    degps = magnitude / steps_per_deg
    return -degps if (raw_speed & 0x8000) else degps


# ------------------------ Kinematics (differential drive) ------------------------ #


def body_to_wheel_raw(
    v_cmd: float,
    omega_cmd_degps: float,
    *,
    wheel_radius: float = WHEEL_RADIUS,
    wheel_base: float = WHEEL_BASE,
    max_raw: int = MAX_RAW,
) -> dict[str, int]:
    """Differential drive kinematics: body velocity -> wheel speed commands.
    Args:
        v_cmd: forward velocity in m/s
        omega_cmd_degps: angular velocity in deg/s (positive = counterclockwise)
    Returns dict: left_wheel/right_wheel with raw speed values.
    """
    omega_radps = omega_cmd_degps * (np.pi / 180.0)

    v_left = v_cmd - omega_radps * wheel_base / 2.0
    v_right = v_cmd + omega_radps * wheel_base / 2.0

    w_left_radps = v_left / wheel_radius
    w_right_radps = v_right / wheel_radius

    w_left_degps = w_left_radps * (180.0 / np.pi)
    w_right_degps = w_right_radps * (180.0 / np.pi)

    raw_left = degps_to_raw(w_left_degps)
    raw_right = degps_to_raw(w_right_degps)

    print(f"raw: left={raw_left}, right={raw_right}")
    return {"left_wheel": raw_left, "right_wheel": raw_right}


def wheel_raw_to_body(
    wheel_raw: dict[str, int],
    *,
    wheel_radius: float = WHEEL_RADIUS,
    wheel_base: float = WHEEL_BASE,
) -> tuple[float, float]:
    """Inverse kinematics: wheel speeds -> body velocity.
    Returns (v_cmd, omega_cmd_degps).
    """
    w_left_degps = raw_to_degps(int(wheel_raw.get("left_wheel", 0)))
    w_right_degps = raw_to_degps(int(wheel_raw.get("right_wheel", 0)))

    w_left_radps = w_left_degps * (np.pi / 180.0)
    w_right_radps = w_right_degps * (np.pi / 180.0)

    v_left = w_left_radps * wheel_radius
    v_right = w_right_radps * wheel_radius

    v_cmd = (v_left + v_right) / 2.0
    omega_radps = (v_right - v_left) / wheel_base
    omega_cmd_degps = omega_radps * (180.0 / np.pi)

    return v_cmd, omega_cmd_degps


# ------------------------ Keyboard teleop ------------------------ #

TELEOP_KEYS = {
    "forward": ["a"],
    "backward": ["d"],
    "differential_left": ["w", "up"],
    "differential_right": ["s", "down"],
    "arrow_left": ["left"],
    "arrow_right": ["right"],
    "quit": "q",
}


class DifferentialDriveTeleop:
    def __init__(self, port: str):
        self.motors = {
            "left_wheel": Motor(id=LEFT_ID, model=MODEL, norm_mode=MotorNormMode.RANGE_0_100),
            "right_wheel": Motor(id=RIGHT_ID, model=MODEL, norm_mode=MotorNormMode.RANGE_0_100),
        }
        self.bus = FeetechMotorsBus(port=port, motors=self.motors)
        self.running = True
        self.pressed = dict.fromkeys(TELEOP_KEYS, False)

        self.lin_speed = float(LIN_SPEED)
        self.ang_speed = float(ANG_SPEED)

    def _on_press(self, key):
        try:
            ch = key.char
        except Exception:
            ch = None
        
        key_str = None
        if ch is not None:
            key_str = ch
        elif key == keyboard.Key.up:
            key_str = "up"
        elif key == keyboard.Key.down:
            key_str = "down"
        elif key == keyboard.Key.left:
            key_str = "left"
        elif key == keyboard.Key.right:
            key_str = "right"
        elif key == keyboard.Key.esc:
            self.running = False
            return
        
        if key_str is not None:
            for action, binds in TELEOP_KEYS.items():
                if action == "quit":
                    continue
                if isinstance(binds, list) and key_str in binds:
                    self.pressed[action] = True
                elif key_str == binds:
                    self.pressed[action] = True

    def _on_release(self, key):
        try:
            ch = key.char
        except Exception:
            ch = None
        
        key_str = None
        if ch is not None:
            key_str = ch
        elif key == keyboard.Key.up:
            key_str = "up"
        elif key == keyboard.Key.down:
            key_str = "down"
        elif key == keyboard.Key.left:
            key_str = "left"
        elif key == keyboard.Key.right:
            key_str = "right"
        
        if key_str is not None:
            for action, binds in TELEOP_KEYS.items():
                if action == "quit":
                    continue
                if isinstance(binds, list) and key_str in binds:
                    self.pressed[action] = False
                elif key_str == binds:
                    self.pressed[action] = False

    def connect(self) -> None:
        self.bus.connect(handshake=False)
        print(f"Connected on port {self.bus.port}")
        for name in self.motors:
            try:
                self.bus.write("Lock", name, 0, normalize=False)
            except Exception:
                pass
            try:
                self.bus.disable_torque(name)
            except Exception:
                pass
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value, normalize=False)
            self.bus.enable_torque(name)
        print("Motors set to VELOCITY mode.")

    def stop(self):
        try:
            for name in self.motors:
                self.bus.write("Goal_Velocity", name, 0, normalize=False)
        except Exception:
            pass

    def close(self):
        try:
            self.bus.disconnect(disable_torque=False)
        except Exception:
            pass

    def run(self):
        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.start()
        try:
            while self.running:
                if self.pressed.get("arrow_left") or self.pressed.get("arrow_right"):
                    if self.pressed.get("arrow_left"):
                        raw_left = degps_to_raw(self.ang_speed)
                        raw_right = 0
                    else:
                        raw_left = 0
                        raw_right = -degps_to_raw(self.ang_speed)
                    wheel_cmds = {"left_wheel": raw_left, "right_wheel": raw_right}
                elif self.pressed.get("differential_left") or self.pressed.get("differential_right"):
                    if self.pressed.get("differential_left"):
                        raw_left = -degps_to_raw(self.ang_speed)
                        raw_right = degps_to_raw(self.ang_speed)
                    else:
                        raw_left = degps_to_raw(self.ang_speed)
                        raw_right = -degps_to_raw(self.ang_speed)
                    wheel_cmds = {"left_wheel": raw_left, "right_wheel": raw_right}
                else:
                    v_cmd = (
                        self.lin_speed
                        if self.pressed.get("forward")
                        else (-self.lin_speed if self.pressed.get("backward") else 0.0)
                    )
                    wheel_cmds = body_to_wheel_raw(v_cmd, 0.0)
                
                for name, val in wheel_cmds.items():
                    self.bus.write("Goal_Velocity", name, val, normalize=False)

                try:
                    currents_raw = {
                        name: self.bus.read("Present_Current", name, normalize=False)
                        for name in self.motors
                    }
                    currents_ma = {name: (currents_raw[name] * 6.5 if currents_raw[name] else 0) for name in currents_raw}
                    ids = {name: self.motors[name].id for name in self.motors}
                    print(
                        f"Current(mA) left(id={ids['left_wheel']})={currents_ma['left_wheel']:.1f} "
                        f"right(id={ids['right_wheel']})={currents_ma['right_wheel']:.1f}"
                    )
                except Exception as exc:
                    print(f"Current read failed: {exc}")

                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
            self.stop()
            self.close()
            print("Teleop stopped.")


# ------------------------ CLI ------------------------ #


def parse_args():
    p = argparse.ArgumentParser(description="Feetech 2-wheel differential drive teleop")
    p.add_argument("--port", type=str, default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    return p.parse_args()


def main():
    args = parse_args()
    teleop = DifferentialDriveTeleop(args.port)
    teleop.connect()
    teleop.run()


if __name__ == "__main__":
    main()
