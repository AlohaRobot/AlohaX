# AlohaX Project

Dual-arm mobile robot project based on ROS2 Humble.

## Project Overview

AlohaX is a dual-arm robot system with the following features:
- **Dual-arm system**: Left and right 6-DOF arms (SOARM101)
- **Mobile base**: Dual drive wheel platform
- **Vertical lift**: Additional lift DOF (ID 11)

## Project Structure

```
alohax_ws/
├── src/
│   ├── alohax_urdf/              # URDF robot model and mesh files
│   ├── alohax_hw_bridge/         # Hardware bridge (core)
│   ├── alohax_moveit_config/     # MoveIt motion planning config
│   ├── alohax_bringup/           # Launch scripts
├── docs/                         # Project documentation
│   ├── AlohaX_Robot_Introduction.md
│   ├── AlohaX_BOM.md
│   ├── AlohaX_Assembly_Guide.md
│   ├── Serial_Port_Fix_Method.md
│   └── media/                    # Documentation images
├── examples/                     # Example programs
│   ├── debug/                    # Debug tools
│   │   ├── motors.py             # Motor control debug
│   │   ├── axis.py               # Lift axis control
│   │   ├── wheels.py             # Base control
│   │   ├── README_debug.md       # Debug tool usage guide
│   │   ├── action_scripts/       # Action scripts
│   │   │   ├── go_to_restposition.txt
│   │   │   ├── go_to_midpoint.txt
│   │   │   └── test_dance.txt
│   │   ├── test_cv.py            # Computer vision test
│   │   ├── test_mic.py           # Microphone test
│   │   ├── test_cuda.py          # CUDA test
│   │   ├── test_network.py       # Network test
│   │   ├── test_dataset.py       # Dataset test
│   │   └── test_input.py         # Input test
│   └── tele/                     # Teleoperation programs
│       ├── alohax_xbox.py        # Xbox controller
│       ├── alohax_keyboard.py    # Keyboard control
│       ├── alohax_joycon.py      # Joycon control (recommended)
│       ├── wheels_joycon.py      # Joycon base control
│       └── AlohaX_Joycon_Software_Design.md  # Joycon control detailed design
├── build/                        # Build output
├── install/                      # Install output
└── log/                          # Log output
```

## Hardware Configuration

### Motor Bus Configuration

**Bus1** (`/dev/so101_L`):
- ID 1-6: Left arm joints
- ID 7-8: Drive wheels (velocity mode)
- ID 11: Vertical lift

**Bus2** (`/dev/so101_R`):
- ID 1-6: Right arm joints

### Joint Configuration

Robot includes the following joints:

**Left arm (6 DOF)**:
- `left_arm_shoulder_pan`: Shoulder pan
- `left_arm_shoulder_lift`: Shoulder lift
- `left_arm_elbow_flex`: Elbow flex
- `left_arm_wrist_flex`: Wrist flex
- `left_arm_wrist_roll`: Wrist roll
- `left_arm_gripper`: Gripper

**Right arm (6 DOF)**:
- `right_arm_shoulder_pan`: Shoulder pan
- `right_arm_shoulder_lift`: Shoulder lift
- `right_arm_elbow_flex`: Elbow flex
- `right_arm_wrist_flex`: Wrist flex
- `right_arm_wrist_roll`: Wrist roll
- `right_arm_gripper`: Gripper

**Base**:
- `base_left_wheel`: Left drive wheel
- `base_right_wheel`: Right drive wheel

**Lift**:
- `vertical_lift`: Vertical lift DOF

## Quick Start

### Environment Requirements

- ROS2 Humble
- Python 3.10+
- Feetech servo drivers

### Build Project

```bash
cd /home/wolf/alohax_ws
colcon build --symlink-install
source install/setup.bash
```
```bash
cd /home/seeed/alohax_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Robot

#### 1. Visualize model (without hardware connection)

```bash
ros2 launch alohax_urdf display.launch.py
```

#### 2. Full launch (real hardware)

```bash
ros2 launch alohax_bringup alohax.launch.py
```

#### 3. Specify serial ports

```bash
ros2 launch alohax_bringup alohax.launch.py \
  port1:=/dev/ttyACM0 \
  port2:=/dev/ttyACM1
```

#### 4. Visualize only (without hardware)

```bash
ros2 launch alohax_bringup alohax.launch.py hardware:=false
```

#### 5. Launch parameter description

`alohax.launch.py` supports the following optional parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hardware` | `true` | Whether to connect real hardware |
| `gui` | `false` | Whether to start joint_state_publisher_gui |
| `moveit` | `true` | Whether to start MoveIt |
| `port1` | `/dev/so101_L` | Left serial port |
| `port2` | `/dev/so101_R` | Right serial port |
| `calib_file` | `alohax_calibration.yaml` | Calibration file path |
| `publish_hz` | `50.0` | Joint state publish rate (Hz) |
| `lift_travel_m` | `0.25` | Lift travel (meters) |
| `lift_motor_degrees_per_meter` | `144000.0` | Lift motor degrees per meter |
| `rviz` | `true` | Whether to start RViz2 |

### Launch MoveIt Motion Planning

```bash
ros2 launch alohax_moveit_config move_group.launch.py
```

## Controller Configuration

Project has the following controllers configured:
- `both_arms_controller`: Dual-arm combined control (includes vertical_lift)
- `left_arm_controller`: Left arm independent control (includes vertical_lift)
- `right_arm_controller`: Right arm independent control
- `left_gripper_controller`: Left gripper control
- `right_gripper_controller`: Right gripper control
- `joint_state_broadcaster`: Joint state broadcaster

Config file locations:
- [alohax_controllers.yaml](src/alohax_hw_bridge/config/alohax_controllers.yaml)
- [alohax_calibration.yaml](src/alohax_hw_bridge/config/alohax_calibration.yaml)

## Calibration and Tuning

### Calibration File

Calibration file is at `src/alohax_hw_bridge/config/alohax_calibration.yaml`, contains:
- `homing_offset`: Zero offset
- `range_min`: Minimum position
- `range_max`: Maximum position

### Adjust Calibration Parameters

Adjust parameters in calibration file based on actual hardware:

```yaml
left_arm_shoulder_pan:
  homing_offset: 2084
  range_min: 745
  range_max: 3424
```

## Troubleshooting

### Common Issues

**1. Hardware connection failed**

```bash
# Check serial ports
ls -l /dev/so101_L /dev/so101_R

# Check permissions
sudo chmod 666 /dev/so101_L /dev/so101_R
```

**2. MoveIt Action server not found**

```bash
# Ensure MoveIt is started
ros2 launch alohax_bringup xlerobot.launch.py moveit:=true

# Check Action servers
ros2 action list | grep move
```

**3. Planning failed**

- Check if target position exceeds joint limits
- Increase planning time (modify `PLANNING_TIME`)
- Check if starting state matches actual

## Future Work Recommendations

1. **Improve URDF model**: Add more precise link dimensions, inertia parameters, and collision models
2. **Import meshes**: Place STL/DAE models in `meshes` directory
3. **Calibration tuning**: Adjust calibration file parameters based on actual hardware
4. **Safety limits**: Adjust joint limits based on actual arm mounting position
5. **Testing verification**: Gradually test each joint motion and base drive

## Example Programs

Project includes rich example programs in `examples/` directory:

### Debug Tools (`debug/`)
Provides low-level hardware debugging, direct motor control without ROS:
- `motors.py`: Motor state viewing, single motor control, ID configuration, phase tuning, homing, script execution
- `axis.py`: Vertical lift axis independent control
- `wheels.py`: Base wheel independent control (WSAD/arrow keys)
- `action_scripts/`: Predefined action scripts (homing, midpoint, dance)
- Test scripts: `test_cv.py`, `test_mic.py`, `test_cuda.py`, `test_network.py`, etc.

See [examples/debug/README_debug.md](examples/debug/README_debug.md) for detailed usage.

### Teleoperation Programs (`tele/`)
Provides multiple controller direct control modes (skip ROS bridge):

| Program | Device | Description |
|---------|--------|-------------|
| `alohax_joycon.py` | Joycon | **Recommended**, dual Joycon control left/right arms, base, lift, with real-time status, current monitoring, 4 speed levels, timeout protection |
| `alohax_xbox.py` | Xbox controller | Xbox controller support |
| `alohax_keyboard.py` | Keyboard | Keyboard control |
| `wheels_joycon.py` | Joycon | Simplified, base only |

See [examples/tele/AlohaX_Joycon_Software_Design.md](examples/tele/AlohaX_Joycon_Software_Design.md) for Joycon control detailed design.

### Quick Start Joycon Control
```bash
python3 examples/tele/alohax_joycon.py
```

## Documentation

Project documentation is in `docs/` directory:
- [AlohaX_Robot_Introduction.md](docs/AlohaX_Robot_Introduction.md): Project overview
- [AlohaX_BOM.md](docs/AlohaX_BOM.md): Detailed bill of materials
- [AlohaX_Assembly_Guide.md](docs/AlohaX_Assembly_Guide.md): Complete assembly guide (with images)
- [Serial_Port_Fix_Method.md](docs/Serial_Port_Fix_Method.md): Serial port fix method

## Dependencies

- ROS2 Humble
- MoveIt2
- joint_state_publisher
- joint_state_publisher_gui
- robot_state_publisher
- rviz2
- xacro
- Feetech servo drivers

## License

Apache-2.0

## Maintainer

Wang Feng (wf270@163.com)
