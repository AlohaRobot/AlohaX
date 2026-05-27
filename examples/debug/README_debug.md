# AlohaX Debug Tools Usage Guide

## View All Motor States
```bash
python3 examples/debug/motors.py get_motors_states \
  --port /dev/ttyACM0
```
```bash
python3 examples/debug/motors.py get_motors_states \
  --port /dev/ttyACM1
```

## Control the Mobile Base Only
Use wsad & arrow keys to control the mobile base
```bash
python3 examples/debug/wheels.py \
   --port /dev/ttyACM0
```

## Control the Lift Axis Only
```bash
python3 examples/debug/axis.py \
   --port /dev/ttyACM0
```

## Rotate a Specific Motor by ID
```bash
python3 examples/debug/motors.py move_motor_to_position \
  --id 1 \
  --position 2 \
  --port /dev/ttyACM1
```

## Set a New Motor ID
```bash
python3 examples/debug/motors.py configure_motor_id \
  --id 10 \
  --set_id 8 \
  --port /dev/ttyACM0
```

## Set the Phase of a Specified Servo
```bash
python3 examples/debug/motors.py configure_motor_phase \
  --id 1 \
  --set_phase 12 \
  --port /dev/ttyACM0
```

## Set the Phase for All Servos
```bash
python3 examples/debug/motors.py configure_motor_phase \
  --set_phase 12 \
  --port /dev/ttyACM0
```

## Reset Current Position as the Motor Midpoint
```bash
python3 examples/debug/motors.py reset_motors_to_midpoint \
  --port /dev/ttyACM0

python3 examples/debug/motors.py reset_motors_to_midpoint \
  --port /dev/ttyACM1
```

## Disable Torque for All Arm Motors
```bash
python3 examples/debug/motors.py reset_motors_torque  \
  --port /dev/ttyACM0

python3 examples/debug/motors.py reset_motors_torque  \
  --port /dev/ttyACM1
```

## Execute an Action Script on the Robot Arm
```bash
python3 examples/debug/motors.py move_motors_by_script \
   --script_path action_scripts/test_dance.txt  \
   --port /dev/ttyACM0
```
