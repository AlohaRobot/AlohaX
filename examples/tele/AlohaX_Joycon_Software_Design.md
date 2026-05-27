# AlohaX Joycon Teleoperation Software Design

## 1. Overview

This software is AlohaX hardware's Joycon controller direct control program, uses direct hardware control mode (skip ROS bridge), controls left and right arms, base movement, and vertical lift mechanism through two Joycons.

**Main Features**:
- Dual Joycon control (left/right)
- Direct Feetech motor communication
- Real-time status display (including current, load monitoring)
- Configurable speed levels (4 levels, base/arm/lift independent parameters)
- Lift mechanism calibration function
- Joycon timeout protection (auto unload torque)
- Global homing function (Capture button)
- Torque enable/disable toggle (Home button)

---

## 2. Hardware Configuration

### 2.1 Motor Bus Configuration

#### Bus1 (Left bus /dev/so101_L)
| Motor Name | ID | Model | Normalization Mode |
|------------|----|-------|--------------------|
| left_arm_shoulder_pan | 1 | sts3215 | DEGREES |
| left_arm_shoulder_lift | 2 | sts3215 | DEGREES |
| left_arm_elbow_flex | 3 | sts3215 | DEGREES |
| left_arm_wrist_flex | 4 | sts3215 | DEGREES |
| left_arm_wrist_roll | 5 | sts3215 | DEGREES |
| left_arm_gripper | 6 | sts3215 | RANGE_0_100 |
| base_left_wheel | 7 | sts3215 | RANGE_0_100 |
| base_right_wheel | 8 | sts3215 | RANGE_0_100 |
| vertical_lift | 11 | sts3215 | DEGREES |

#### Bus2 (Right bus /dev/so101_R)
| Motor Name | ID | Model | Normalization Mode |
|------------|----|-------|--------------------|
| right_arm_shoulder_pan | 1 | sts3215 | DEGREES |
| right_arm_shoulder_lift | 2 | sts3215 | DEGREES |
| right_arm_elbow_flex | 3 | sts3215 | DEGREES |
| right_arm_wrist_flex | 4 | sts3215 | DEGREES |
| right_arm_wrist_roll | 5 | sts3215 | DEGREES |
| right_arm_gripper | 6 | sts3215 | RANGE_0_100 |

### 2.2 Motor Groups
- **Left arm joints**: left_arm_*
- **Right arm joints**: right_arm_*
- **Base wheels**: base_* (left/right wheel direction sign `BASE_WHEEL_SIGN`: left=-1.0, right=1.0)
- **Lift joint**: vertical_lift (direction sign `LIFT_DIRECTION_SIGN = -1.0`)

---

## 3. Parameter Configuration

### 3.1 Mechanical Parameters
| Parameter Name | Value | Description |
|----------------|-------|-------------|
| STEPS_PER_DEG | 4096/360 | Steps per degree |
| WHEEL_RADIUS | 0.05 | Wheel radius (meters) |
| WHEEL_BASE | 0.30 | Wheel base (meters) |
| LIFT_TRAVEL_M | 0.25 | Lift travel (meters) |
| LIFT_MOTOR_DEGREES_PER_METER | 144000 | Lift motor degrees per meter |
| JOYCON_FRAME_TIMEOUT_SEC | 2.0 | Joycon timeout threshold (seconds) |
| JOYCON_RECONNECT_INTERVAL_SEC | 1.0 | Joycon reconnect interval (seconds) |

### 3.2 Speed Level System

Code defines **three independent speed level sets**, 4 levels each, synchronized switching.

#### Base Speed Levels (BASE_SPEED_LEVELS)
| Level | Linear Speed (m/s) lin | Angular Speed (deg/s) ang |
|-------|-------------------------|----------------------------|
| 1 | 0.05 | 20.0 |
| 2 | 0.10 | 40.0 |
| 3 | 0.15 | 70.0 |
| 4 | 0.50 | 572.0 |

#### Arm Speed Levels (ARM_SPEED_LEVELS)
| Level | P Coeff | I Coeff | D Coeff | Acceleration acc | Step (degrees) |
|-------|---------|---------|---------|------------------|----------------|
| 1 | 4 | 0 | 11 | 64 | 0.025 |
| 2 | 8 | 0 | 22 | 128 | 0.05 |
| 3 | 12 | 0 | 32 | 192 | 0.075 |
| 4 | 16 | 0 | 43 | 254 | 0.1 |

#### Lift Mechanism Speed Levels (LIFT_SPEED_LEVELS)
| Level | kp_vel Velocity Proportional Gain | v_max Max Velocity | step Step (mm) |
|-------|-----------------------------------|--------------------|-----------------|
| 1 | 75 | 325 | 0.5 |
| 2 | 150 | 650 | 1.0 |
| 3 | 225 | 975 | 1.5 |
| 4 | 300 | 1300 | 2.0 |

**Note**: During homing operation, speed level automatically drops to `max(1, (normal_level+1)//2)`, restores original level after completion.

### 3.3 Joint Limits (radians)
| Joint | Min | Max |
|-------|-----|-----|
| shoulder_pan | -1.996 | 2.111 |
| shoulder_lift | -2.004 | 2.028 |
| elbow_flex | -0.072 | 2.995 |
| wrist_flex | -2.238 | 1.334 |
| wrist_roll | -3.139 | 3.140 |
| gripper | 0.002 | 2.069 |

### 3.4 Calibration Parameters (CALIBRATION)

All motors have independent zero_position (Present_Position raw value at home pose):

| Motor Name | zero_position |
|------------|---------------|
| left_arm_shoulder_pan | 2047 |
| left_arm_shoulder_lift | 2047 |
| left_arm_elbow_flex | 2047 |
| left_arm_wrist_flex | 2047 |
| left_arm_wrist_roll | 1000 |
| left_arm_gripper | 2048 |
| right_arm_shoulder_pan | 2047 |
| right_arm_shoulder_lift | 2047 |
| right_arm_elbow_flex | 2047 |
| right_arm_wrist_flex | 2047 |
| right_arm_wrist_roll | 1000 |
| right_arm_gripper | 2048 |
| vertical_lift | 2000 |

**Note**: Gripper conversion uses hardcoded range_min=2048 / range_max=3396 (hardcoded in `joint_position_to_motor_cmd` / `motor_cmd_to_joint_position`, not in CALIBRATION dictionary).

### 3.5 PID Control Parameters (Dynamically Adjustable)

PID parameters switch with speed level, see section 3.2 arm speed level table. Level 4 (highest speed): P=16, I=0, D=43, Acc=254. Gripper does not set PID parameters.

### 3.6 Lift Mechanism Parameters

| Parameter Name | Value | Description |
|----------------|-------|-------------|
| min_height_mm | -450 | Minimum height (millimeters) |
| max_height_mm | 50 | Maximum height (millimeters) |
| descent_floor_mm | -445 | Descent limit (millimeters) |
| kp_vel | Dynamic | Velocity proportional gain (changes with level) |
| v_max | Dynamic | Max velocity (changes with level) |
| on_target_mm | 1.0 | Target threshold (millimeters) |
| step_mm | Dynamic | Step value (changes with level) |
| _ticks_per_rev | 4096 | Encoder ticks per revolution |
| _mm_per_deg | 84/360 | Millimeters per degree |

---

## 4. Main Loop Cycle and Servo Polling Cycle

### 4.1 Main Loop Cycle
```python
self.timer = self.create_timer(0.033, self.timer_callback)
```
- **Main loop cycle**: 33ms (approx 30Hz)
- **Timer callback function**: `timer_callback()`
- **Cycle monitoring**: `last_callback_time` records last callback time, real-time calculates actual cycle and displays

### 4.2 Servo Polling and Communication Cycle

#### Write Cycle
| Hardware Group | Communication Method | Cycle |
|----------------|---------------------|-------|
| Left arm joints | sync_write | 33ms |
| Right arm joints | sync_write | 33ms |
| Base wheels | sync_write | 33ms |
| Vertical lift | write | 33ms |

#### Read Cycle
Reading uses bulk register reading (`_sync_read_multi_registers`), reads Present_Position, Present_Velocity, Present_Load three consecutive registers (address 56-61, length 6 bytes) at once. Additionally reads Present_Current separately (address 69, not consecutive).

| Hardware Group | Communication Method | Cycle | Read Content |
|----------------|---------------------|-------|--------------|
| Left arm joints | sync_read | 33ms | Present_Position / Velocity / Load + Current |
| Right arm joints | sync_read | 33ms | Present_Position / Velocity / Load + Current |
| Vertical lift | sync_read | 33ms | Present_Position + Current + Load |
| Base wheels | sync_read | 33ms | Present_Position + Current + Load |

### 4.3 Communication Configuration
| Parameter Name | Value | Description |
|----------------|-------|-------------|
| Return_Delay_Time | 0 | Return delay time (0μs) |
| Maximum_Acceleration | Dynamic | Maximum acceleration (changes with speed level) |
| Acceleration | Dynamic | Acceleration (changes with speed level) |
| Unloading_Condition | 0x2f | Unloading condition register |

---

## 5. Control Mapping

### 5.1 Button Mapping Constants

Code defines `LEFT_KEYMAP` and `RIGHT_KEYMAP` two dictionaries, maps physical buttons to control actions.

#### Left Joycon Button Mapping (LEFT_KEYMAP)
| Control Action | Physical Button |
|----------------|-----------------|
| elbow_flex-, X- (stick_up) | Stick up |
| elbow_flex+, X+ (stick_down) | Stick down |
| wrist_roll+ (stick_right) | Stick right |
| wrist_roll- (stick_left) | Stick left |
| wrist_flex- (stick_pressed_up) | Press stick + up |
| wrist_flex+ (stick_pressed_down) | Press stick + down |
| shoulder_pan+ (l_right) | L/ZL + stick right |
| shoulder_pan- (l_left) | L/ZL + stick left |
| shoulder_lift- (l_up) | L/ZL + stick up |
| shoulder_lift+ (l_down) | L/ZL + stick down |
| gripper+ (zl) | ZL |
| vertical_lift+ (up) | Up button |
| vertical_lift- (down) | Down button |

#### Right Joycon Button Mapping (RIGHT_KEYMAP)
| Control Action | Physical Button |
|----------------|-----------------|
| elbow_flex- (stick_up) | Stick up |
| elbow_flex+ (stick_down) | Stick down |
| wrist_roll+ (stick_right) | Stick right |
| wrist_roll- (stick_left) | Stick left |
| wrist_flex- (stick_pressed_up) | Press stick + up |
| wrist_flex+ (stick_pressed_down) | Press stick + down |
| shoulder_pan+ (r_right) | R/ZR + stick right |
| shoulder_pan- (r_left) | R/ZR + stick left |
| shoulder_lift- (r_up) | R/ZR + stick up |
| shoulder_lift+ (r_down) | R/ZR + stick down |
| gripper+ (zr) | ZR |

### 5.2 Control Function Summary

#### Right Joycon (Control Right Arm and Base)
| Operation | Function |
|-----------|----------|
| Stick left/right | wrist_roll |
| Stick up/down | elbow_flex |
| Stick pressed + up/down | wrist_flex |
| R/ZR + stick left/right | shoulder_pan |
| R/ZR + stick up/down | shoulder_lift |
| ZR | Right gripper close |
| X | Forward |
| B | Backward |
| Y | Left turn |
| A | Right turn |
| Plus | Speed level +1 (max 4) |
| Home | Toggle torque enable/disable |

#### Left Joycon (Control Left Arm and Lift)
| Operation | Function |
|-----------|----------|
| Stick left/right | wrist_roll |
| Stick up/down | elbow_flex |
| Stick pressed + up/down | wrist_flex |
| L/ZL + stick left/right | shoulder_pan |
| L/ZL + stick up/down | shoulder_lift |
| ZL | Left gripper close |
| Up | Lift up |
| Down | Lift down |
| Minus | Speed level -1 (min 1) |
| Capture | Global homing |

### 5.3 Shared Control
| Operation | Function |
|-----------|----------|
| Right Plus | Speed level +1 (max 4) |
| Left Minus | Speed level -1 (min 1) |
| Right Home | Toggle torque enable/disable |
| Left Capture | Global homing |

---

## 6. Main Classes and Functions

### 6.1 TeleopDisplay Class
**Function**: curses-based real-time status display

**Added display content**:
- Each motor current (mA) and load value
- Vertical lift actual height
- Base left/right wheel status and current/load
- Main loop cycle (ms)
- Torque status (ON/OFF)
- Homing operation flag

**Main methods**:
- `init_curses()`: Initialize curses display (including color pairs)
- `cleanup()`: Cleanup display resources
- `update_left_arm(state)`: Update left arm target position
- `update_right_arm(state)`: Update right arm target position
- `update_lift(state)`: Update lift target position (meters)
- `update_lift_hw(state)`: Update lift actual position (meters)
- `update_base(state)`: Update base status
- `update_grippers(left, right)`: Update gripper status
- `update_hw_arm_states(left, right)`: Update hardware joint positions
- `update_arm_motor_states(...)`: Update all motors current, load
- `update_lift_motor_states(current, load)`: Update lift current and load
- `update_wheel_states(...)`: Update base left/right wheel speed, position, current, load
- `update_torque_status(enabled)`: Update torque status
- `update_base_speed_level(level)`: Update current speed level
- `update_loop_period(ms)`: Update main loop cycle
- `set_reset_flag(flag)`: Set homing flag
- `display()`: Refresh display

### 6.2 SimpleTeleopArm Class
**Function**: Arm joint control

**Constructor**: `__init__(prefix="left", kp=1, speed_level=2)`
- `prefix`: left or right
- `speed_level`: Initial speed level (default 2)

**Main methods**:
- `move_to_zero_position()`: Return to zero position
- `handle_keys(key_state)`: Handle key input to update target position (including JOINT_LIMITS clamping)
- `_update_step()`: Update degree_step based on current speed level
- `update_speed_level(level)`: Update speed level and call `_update_step()`

**Member variables**:
- `degree_step`: Degree increment per step (radians, changes with speed level: 0.025~0.1 degrees)
- `target_positions`: Target position dictionary (radians)
- `zero_pos`: Zero position (all zeros)
- `speed_level`: Current speed level

### 6.3 ContinuousLiftControl Class
**Function**: Vertical lift mechanism continuous control (supports multi-turn continuous rotation)

**Constructor**: `__init__(speed_level=2)`

**Main methods**:
- `configure(bus, motor_name)`: Configure hardware, read initial encoder value
- `home(use_current=True)`: Homing calibration (push up stall → descend 50mm → set zero)
- `set_height_mm(target_mm)`: Set target height (clamped [-450, 50] mm)
- `compute_velocity()`: Calculate velocity command (PID velocity loop, including limit protection)
- `get_height_mm()`: Get current height
- `move_to_zero_position()`: Return to zero position (target_height_mm = 0)
- `handle_keys(key_state)`: Handle key input (Up/Down buttons)
- `_update_extended_ticks()`: Update accumulated turns (handle multi-turn winding)
- `_update_params()`: Update kp_vel/v_max/step_mm based on current speed level
- `update_speed_level(level)`: Update speed level and call `_update_params()`

**Member variables**:
- `_ticks_per_rev` / `_deg_per_tick` / `_mm_per_deg`: Internal parameters
- `min_height_mm` / `max_height_mm` / `descent_floor_mm`: Limits
- `kp_vel` / `v_max` / `step_mm` / `on_target_mm`: Control parameters (changes with speed level)
- `dir_sign = -1.0`: Direction sign

### 6.4 FixedAxesJoyconRobotics Class
**Function**: Joycon input handling (inherits from JoyconRobotics)

**Added features**:
- Input frame timeout detection: Mark latest frame time through register_update_hook
- `seconds_since_last_input_frame()`: Seconds since last input frame
- `is_input_frame_timed_out(timeout=2.0)`: Whether timed out

**Main overrides**:
- `__init__(device)`: Configure stick zero, register input frame hook
- `get_stick_values()`: Get stick values
- `reset_joycon()`: Reset (silent version, reduces wait time)
- `common_update()`: Update (remove auto-reset logic)

**Stick zero configuration**:
- Right Joycon: Vertical=1900, Horizontal=2100
- Left Joycon: Vertical=2300, Horizontal=2000

### 6.5 AlohaXTeleopNode Class
**Function**: Main control node (ROS2 Node)

**Added core features**:
- **Joycon timeout protection**: Input frame timeout 2s → stop base → freeze lift → clear target positions → unload dual arm torque
- **Global homing**: Capture button triggers, all joints return to zero position, uses half speed
- **Torque toggle**: Home button toggles torque enable/disable
- **Dynamic PID**: Real-time update motor PID / acceleration parameters when speed level switches

**Main methods**:
- `__init__(port1, port2, display)`: Initialize (connect hardware, configure motors, create timer)
- `_connect_hardware()`: Connect Bus1/Bus2, scan motors, configure, initialize home position
- `_configure_bus(bus, active_motors, is_bus1)`: Configure single bus motor parameters (mode, PID, acceleration, unloading condition)
- `_init_cmd_to_home()`: Initialize all motor commands to home position
- `_update_motor_speed_params()`: Update all motor PID / acceleration based on speed level
- `_update_motor_homing_speed_params()`: Set homing speed (approx half of level)
- `_check_homing_complete()`: Detect if homing complete (within 1 degree as arrived)
- `_write_commands()`: Write motor commands (sync_write Goal_Position + Goal_Velocity)
- `_read_hardware_states()`: Read hardware states (bulk register read Present_Position/Velocity/Load + Present_Current)
- `_sync_read_multi_registers()`: Bulk read consecutive registers
- `_is_joycon_fresh(joycon)`: Check if Joycon is not timed out
- `_set_base_cmd_zero()`: Clear base command
- `_freeze_lift_target()`: Freeze lift target height
- `_clear_all_targets()`: Clear all target values (timeout recovery anti-shock)
- `_disable_arm_torque()` / `_enable_arm_torque()`: Unload/enable torque
- `_update_joycon_watchdogs()`: Update Joycon watchdogs
- `_update_timeout_display(states)`: Update timeout display
- `timer_callback()`: Main loop callback (including timeout detection, homing, speed control, Joycon handling, hardware communication, display update)
- `cleanup(keep_display=True)`: Cleanup resources

**Key members**:
- `bus1` / `bus2`: FeetechMotorsBus instances
- `bus1_connected` / `bus2_connected`: Bus connection status
- `active_bus1` / `active_bus2`: Scanned active motor list
- `left_arm` / `right_arm`: SimpleTeleopArm instances
- `lift`: ContinuousLiftControl instance
- `joycon_right` / `joycon_left`: FixedAxesJoyconRobotics instances
- `cmd`: Motor command dictionary
- `display`: TeleopDisplay instance
- `torque_enabled`: Torque enable status
- `base_speed_level`: Current speed level (1-4)
- `is_homing`: Whether currently homing
- `normal_speed_level`: Normal working speed level
- `_joycon_timeout_state`: Joycon timeout state dictionary
- `_lift_frozen_height`: Lift height frozen at timeout

### 6.6 Helper Functions

| Function Name | Function |
|---------------|----------|
| `_patch_joycon_open()` | Fix Joycon Bluetooth connection (open hid device through path) |
| `_joycon_reconnect(self)` | Joycon reconnect logic |
| `_safe_update_input_report(self)` | Safe update input report (exception handling) |
| `degps_to_raw(degps)` | Degrees/second to raw velocity value (including sign bit handling) |
| `body_to_wheel_raw(v_cmd, omega_cmd_degps)` | Body velocity to wheel raw velocity |
| `degrees_to_raw(name, degrees)` | Degrees to raw position value (including calibration offset) |
| `raw_to_degrees(name, raw)` | Raw position value to degrees (including calibration offset) |
| `joint_position_to_motor_cmd(name, position)` | Joint position (radians/meters) to motor command |
| `motor_cmd_to_joint_position(name, cmd)` | Motor command to joint position |
| `clamp(value, min_val, max_val)` | Value clamping utility |
| `get_joycon_key_state(joycon, keymap)` | Get Joycon key state based on keymap |
| `get_base_action(joycon, speed_level)` | Get base action (forward/backward/left turn/right turn) |
| `parse_args()` | Parse command line arguments |
| `run_lift_calibration(port1)` | Run lift calibration |
| `main(args)` | Main function |

---

## 7. Workflow

### 7.1 Startup Flow
```
main()
  ├─ Parse command line arguments (parse_args)
  ├─ Lift mechanism calibration (optional, interactive prompt)
  │   └─ run_lift_calibration(port1)
  ├─ Initialize ROS2 (rclpy.init)
  ├─ Connect Joycon (suppress output)
  │   ├─ FixedAxesJoyconRobotics("right")
  │   ├─ FixedAxesJoyconRobotics("left")
  │   └─ Check if Joycon connected
  ├─ Create AlohaXTeleopNode
  │   ├─ Hardware connection (_connect_hardware)
  │   │   ├─ Connect Bus1/Bus2
  │   │   ├─ Scan motors (broadcast_ping)
  │   │   ├─ Configure motors (mode, PID, acceleration, unloading condition)
  │   │   ├─ Initialize to home position
  │   │   └─ Configure lift mechanism (lift.configure)
  │   └─ Create timer (33ms)
  ├─ Pass Joycon to node
  ├─ Initialize display (TeleopDisplay + curses)
  └─ rclpy.spin(node) enter loop
```

### 7.2 Main Loop Flow (timer_callback)
```
33ms cycle execution:
  ├─ Calculate loop cycle (display real-time ms)
  ├─ Update Joycon watchdogs (_update_joycon_watchdogs)
  │
  ├─ Home/Capture button detection
  │   ├─ Home → Toggle torque enable/disable
  │   ├─ Capture → Trigger global homing (is_homing=true)
  │   │   └─ Save normal speed, switch to homing speed (half)
  │   └─ Homing → Detect complete (within 1 degree), restore normal speed after completion
  │
  ├─ Plus/Minus button detection → Speed level adjustment
  │   └─ _update_motor_speed_params() Update PID/acceleration
  │
  ├─ Joycon timeout judgment
  │   ├─ If timed out → Stop base → Freeze lift → Clear targets → Unload torque
  │   └─ If recovered → Re-enable torque
  │
  ├─ Normal control (Joycon not timed out)
  │   ├─ Left Joycon handling
  │   │   ├─ get_joycon_key_state(LEFT_KEYMAP)
  │   │   ├─ Update left arm target position
  │   │   ├─ Update lift target position
  │   │   └─ Update cmd dictionary
  │   └─ Right Joycon handling
  │       ├─ get_joycon_key_state(RIGHT_KEYMAP)
  │       ├─ Update right arm target position
  │       ├─ Get base velocity command
  │       └─ Update cmd dictionary
  │
  ├─ Write hardware (_write_commands)
  │   ├─ Left arm: sync_write Goal_Position (degrees to raw value)
  │   ├─ Base: sync_write Goal_Velocity (including direction sign)
  │   ├─ Lift: write Goal_Velocity (PID velocity loop calculation)
  │   └─ Right arm: sync_write Goal_Position (degrees to raw value)
  │
  ├─ Read hardware states (_read_hardware_states)
  │   ├─ Left arm: bulk read Present_Position/Velocity/Load + Current
  │   ├─ Lift: sync_read Present_Position/Current/Load
  │   ├─ Base: sync_read Present_Position/Current/Load
  │   └─ Right arm: bulk read Present_Position/Velocity/Load + Current
  │
  └─ Update display
      ├─ Target position (left/right arms, lift, gripper)
      ├─ Hardware position (left/right arms, lift, base)
      ├─ Motor states (current, load)
      ├─ Loop cycle, speed level, torque status, homing flag
      └─ display() Refresh
```

### 7.3 Lift Calibration Flow
```
run_lift_calibration(port1)
  ├─ Connect Bus1 (only includes lift motor)
  ├─ Find lift motor (ID=11)
  ├─ Create ContinuousLiftControl instance
  ├─ Call home(use_current=True)
  │   ├─ Enable torque, lock motor
  │   ├─ Move up (Goal_Velocity=-1000)
  │   ├─ Loop detection (max 300s)
  │   │   ├─ Detect if current ≥ 450mA (stall)
  │   │   └─ Or detect if position no longer moves
  │   ├─ Stop
  │   ├─ Move down 50mm (retract)
  │   ├─ Wait for stable
  │   └─ Set as zero (z0_deg)
  └─ Disconnect
```

---

## 8. Motor Mode Configuration

| Motor Type | Operating_Mode | Description |
|------------|----------------|-------------|
| Arm joints | POSITION | Position control mode |
| Base wheels | VELOCITY | Velocity control mode |
| Lift mechanism | VELOCITY | Velocity control mode (multi-turn continuous) |

**Note**: Gripper does not set PID parameters, only sets position command.

---

## 9. Command Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--port1` | /dev/so101_L | Left bus serial port |
| `--port2` | /dev/so101_R | Right bus serial port |
| `--no-display` | - | Disable curses display (degrade to simple text output) |
| `--no-calibrate` | - | Skip lift calibration interactive prompt |

**Interactive behavior**: If not passing `--no-calibrate`, will prompt `Do you want to run lift calibration? (y/N)` at startup.

---

## 10. File Structure

```
alohax_joycon.py
├── Imports and patches (hid, joycon_core, `_patch_joycon_open`)
├── Hardware configuration (BUS1_MOTORS, BUS2_MOTORS, motor groups)
├── Speed level configuration (BASE_SPEED_LEVELS, ARM_SPEED_LEVELS, LIFT_SPEED_LEVELS)
├── Calibration parameters (CALIBRATION, each motor independent zero_position)
├── Motor conversion functions (degps_to_raw, degrees_to_raw, joint_position_to_motor_cmd, etc.)
├── clamp utility function
├── Button mapping configuration (LEFT_KEYMAP, RIGHT_KEYMAP)
├── Joint limit configuration (JOINT_LIMITS)
├── TeleopDisplay class
├── SimpleTeleopArm class
├── ContinuousLiftControl class
├── FixedAxesJoyconRobotics class
├── Button/base helper functions (get_joycon_key_state, get_base_action)
├── AlohaXTeleopNode class (main control node)
├── Lift calibration function (run_lift_calibration)
└── Main functions (parse_args, main)
```

---

## 11. Notes

1. **Unit Convention**:
   - Joint target position: radians (internal calculation)
   - Motor command: degrees (after conversion) / raw encoder value (gripper, lift)
   - Gripper: radians ↔ raw range 2048~3396 linear mapping
   - Raw position: encoder value (0-4095)

2. **Joycon Pairing**: Need to first pair Joycon through Bluetooth. Pairing info can be viewed through `bluetoothctl devices`.

3. **Joycon Connection Patch**: `_patch_joycon_open()` modifies Joycon library's `_open()` method, prioritizes opening through device path obtained from `hid.enumerate()`, solves issue where directly opening through VID/PID fails on some systems.

4. **Timeout Protection**: If Joycon has no input frame for 2 seconds, automatically unload torque and stop all motion.

5. **Calibration**: Recommend running lift calibration after first use or replacing mechanical structure.

6. **Display**: Requires terminal window at least 120x30 characters. If curses initialization fails, automatically degrades to `--no-display` mode.

7. **Homing**:
   - Capture button triggers global homing (all joints return to 0 position)
   - Homing speed approx half of current level speed
   - Homing timeout 10 seconds
   - Automatically restore original speed level after homing complete

8. **Exception Handling**: All exceptions in main loop are caught, ensures program does not exit due to single error. Base stops when error occurs.
