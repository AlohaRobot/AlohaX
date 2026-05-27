# AlohaX Assembly Guide

AlohaX is a dual-arm four-wheel humanoid robot based on AlohaMini architecture, combining xlerobot's mobile base design. This guide details the AlohaX assembly process.

---

## Preparation

Before starting AlohaX assembly, ensure you have:
1. Completed procurement of all components (see [AlohaX_BOM.md](AlohaX_BOM.md))
2. Completed printing of all 3D printed parts
3. Prepared necessary tools (screwdriver set, soldering iron, bench vise, etc.)
4. Prepared auxiliary materials such as cable ties, heat shrink tubing, etc.

---

## 1. Body Assembly

### 1.1 Head Camera Installation

Head camera for visual perception, AlohaX is equipped with multiple cameras.

1. **Preparation**
   - Take out USB camera (720p, focal length 2.1mm)
   - Prepare M2×12 Phillips screws

2. **Assembly Steps**
   - Mount camera to head bracket
   - Secure with M2×12 screws
   - Connect camera USB cable, route through pipe

![Head Camera Installation](./media/body_assembly_head_camera.jpg)

### 1.2 Body Main Assembly

Body main part based on AlohaMini design, core component of robot.

1. **Preparation**
   - Take out 3D printed body main frame
   - Prepare 302 glue

2. **Assembly Steps**
   - Cut off excess at bottom, align upper and lower shells of body main part, secure sequentially with 302 glue

![Body Assembly 01](./media/body_assembly_01.jpg)
![Body Assembly 02](./media/body_assembly_02.jpg)

### 1.3 Steel Pipe or Aluminum Pipe Installation (Optional)

Steel pipe for reinforcing body structure and lift mechanism, select 2 steel pipes or aluminum pipes with outer diameter 29mm, thickness no more than 2mm, length 800mm. Too thick will make robot front too heavy.

1. **Preparation**
   - Take out steel pipes
   - Prepare adhesive (epoxy recommended)

2. **Assembly Steps**
   - Insert steel pipes into reserved holes in body main part
   - Secure steel pipes with epoxy, ensure vertical and firm
   - Wait for adhesive to fully cure

![Steel Pipe Installation](./media/body_assembly_steel_pipe.jpg)
![Steel Pipe Installation](./media/body_assembly_pipe.jpg)

### 1.4 Vertical Lift Mechanism Assembly

Vertical lift mechanism is one of AlohaX's core features, can expand working range.

1. **Preparation**
   - Take out lift axis assembly
   - Take out gears and transmission components
   - Prepare servo (ID 11 allocation completed)

2. **Assembly Steps**
   - Mount lift axis to body main part guide rails
   - Install gear set, ensure smooth meshing
   - Connect servo, test lift motion

> **Tip**: Ensure lift motion is smooth, no jamming or abnormal noise.

![Vertical Lift 01](./media/body_vertical_01.jpg)
![Vertical Lift 02](./media/body_vertical_02.jpg)
![Vertical Lift 03](./media/body_vertical_03.jpg)
![Vertical Lift 04](./media/body_vertical_04.jpg)
![Vertical Lift 05](./media/body_vertical_05.jpg)
![Vertical Lift 06](./media/body_vertical_06.jpg)
![Vertical Lift 07](./media/body_vertical_07.jpg)
![Vertical Lift 08](./media/body_vertical_08.jpg)
![Vertical Lift 09](./media/body_vertical_09.png)
> **Important Note**: AlohaX arms are mounted below body, so lift mechanism arm mounting bracket faces downward, opposite to AlohaMini direction.

### 1.5 USB Hub Installation

USB Hub is robot's communication hub, for connecting multiple devices.
USB Hub placed above chest core module, secured with 302 glue or strong double-sided tape, power terminal below, reduces overall wiring length.

![USB Hub Installation](./media/body_assembly_usb_hub_01.jpg)

---

## 2. Base Assembly

Base uses IKEA Raskog cart modification, robot's mobile platform.

### 2.1 Cart Modification

1. **Preparation**
   - Take out IKEA Raskog cart
   - Prepare wood board, screws, and tools

2. **Modification Steps**
   - Retain only bottom basket, remove other shelves, cut Raskog steel pipe at bottom basket
   - Compared to xlerobot, rotate base 90 degrees for more coordinated aspect ratio
   - Cut wood board to same shape as basket interior, install in base basket
   - Secure wood board with self-tapping screws, ensure stable

### 2.2 Drive System Installation

1. **Preparation**
   - Take out 2 servos (ID 7, 8)
   - Take out driving wheels and driven wheels
   - Prepare servo mounting brackets

2. **Assembly Steps**
   - Mount servos to base designated positions
   - Install driving wheels to servo output shafts
   - Install driven wheels to corresponding brackets
   - Connect servo cables to bus servo controller

![Base Assembly 01](./media/base_assembly_01.jpg)
![Base Assembly 02](./media/base_assembly_02.jpg)
![Base Assembly 03](./media/base_assembly_03.jpg)

> **Note**: Uses IKEA Raskog cart as mobile base, retains only bottom basket, and adds wood board reinforcement, solves base softness and motion wobble issues.
xlerobot0.4.0 assembly reference official site:
https://xlerobot.readthedocs.io/en/latest/hardware/getting_started/assemble_2wheel.html

---

## 3. Final Assembly

### 3.1 Slave Arm Installation

AlohaX is equipped with two SO-ARM101 slave arms.

**Slave arm installation reference official guide**: [SO-101 Assembly Guide](https://huggingface.co/docs/lerobot/so101)

> **Note**: Arms directly reuse SO-ARM100/SO-ARM101 design, if you already have two SO-ARM101 arms, you can use them directly.

### 3.2 Body and Base Assembly

Mount assembled body module to base.

1. **Preparation**
   - Ensure body and base are both assembled
   - Prepare angle brackets and M3×18 self-tapping screws

2. **Assembly Steps**
   - Align body module to base front mounting positions
   - Secure body module with angle brackets and self-tapping screws
   - Check if connection is firm, adjust position to ensure vertical

![Body Base Assembly 01](./media/body_base_assembly_01.jpg)
![Body Base Assembly 04](./media/body_base_assembly_04.jpg)

### 3.3 Slave Arm and Mobile Base Assembly

Mount slave arms to body module.

1. **Preparation**
   - Take out assembled slave arms
   - Prepare M2×30 hex socket screws and M3 nuts

2. **Assembly Steps**
   - Use four M2×30 hex socket screws and four M3 nuts to secure arms to both sides of T-bracket
   - Connect 90cm servo cable of servo #11 to another port of left arm's Waveshare control board
   - Route dual arm driver/camera USB cables around shoulders
   - Servo cables recommended to use twisted pairs, improves anti-interference

> **Wiring Tip**: Except for battery to power terminal cables, all other cables fixed to body core, aesthetic and tangle-free.

### 3.4 Wiring

#### 3.4.1 Battery Assembly

Battery is robot's power source.

1. **Preparation**
   - Take out 12V 20A lead-acid battery
   - Prepare battery multi-socket cover

2. **Assembly Steps**
   - Cover battery switch socket cover onto 12V battery, secure firmly with cable ties
   - Connect battery positive and negative to circuit breaker

![Battery Assembly](./media/wiring_battery_assembly_01.jpg)

#### 3.4.2 Circuit Breaker

Circuit breaker for overcurrent protection and emergency stop function.

1. **Preparation**
   - Take out 6A DC circuit breaker

2. **Assembly Steps**
   - Mount circuit breaker to body side, secure with 302 glue or strong double-sided tape
   - Connect input and output power cables
   - Test circuit breaker function, ensure can normally cut power

![Circuit Breaker Installation](./media/wiring_breaker.jpg)

#### 3.4.3 12V Connector

12V connector for connecting battery and various electrical devices.

1. **Preparation**
   - Take out 12V 2-6 terminal block
   - Take out 3 12V cables with plugs, Orin nano power plug interface is 5525, Waveshare control board plug is 5521.

2. **Assembly Steps**
   - Connect terminal block input positive and negative to circuit breaker output terminals with wires.
   - Connect terminal block outputs to Orin nano main controller, left arm, right arm Waveshare control boards respectively.

> **Important Note**: Jetson nano, Raspberry Pi and other 5V powered main controllers, do not connect to 12V! Can take power from USB port on battery cover.

![12V Connector 02](./media/wiring_12V_connector_01.jpg)
![12V Connector 03](./media/wiring_12V_connector_02.jpg)

#### 3.4.4 USB Hub Wiring

USB Hub for connecting all USB devices.

1. **Preparation**
   - Take out USB Hub
   - Prepare USB cables

2. **Assembly Steps**
   - Connect dual arm camera USB cables to USB Hub
   - Connect dual arm controller USB cables to USB Hub
   - Organize cables, excess cables wrapped around robot shoulders, secured with cable ties

![USB Hub Wiring](./media/wiring_usb_hub.jpg)

#### 3.4.5 Astra Pro Camera Wiring

Astra Pro for depth perception (if applicable).

1. **Preparation**
   - Take out Astra Pro camera (if equipped)
   - Prepare USB cable

2. **Assembly Steps**
   - Attach Astra Pro base to robot core with 302 glue, or drill holes outward from core interior, secure Astra Pro base with M6 bolts.
   - Connect USB cable to USB Hub
   - Secure cable, avoid affecting motion

![Astra Pro Wiring](./media/wiring_astra_pro.jpg)

#### 3.4.6 Orin Nano Connection

Main controller is robot's computing core.

1. **Preparation**
   - Take out NVIDIA Orin nano main controller (or Raspberry Pi/PC)

2. **Assembly Steps**
   - Attach main controller to robot body back or lower mounting platform with 302 glue or strong double-sided tape
   - Connect buck converter output to main controller power input (for Raspberry Pi or Jetson nano)
   - Connect USB Hub to main controller
   - Connect other peripherals (such as display, optional)

![Orin Nano Connection](./media/wiring_orin_nano.jpg)

### 3.6 Assembly Completion Check

1. **Mechanical Check**
   - Check if all screws are tightened
   - Check if all moving parts move smoothly
   - Check if cables have wear risk

2. **Electrical Check**
   - Check if all connectors are firm
   - Check if power cable positive and negative are correct
   - Test circuit breaker function

3. **Functional Check**
   - Test if lift mechanism is smooth
   - Test if mobile base can move normally
   - Test if arms can move normally
   - Check if camera can work normally

![Assembly Completion Check 01](./media/assembly_finish_01.jpg)
![Assembly Completion Check 02](./media/assembly_finish_02.jpg)
![Assembly Completion Check 03](./media/assembly_finish_03.jpg)

---

## Safety Notes

- **Charging Safety**: Only connect battery during operation, disconnect entire robot wiring during charging (most chargers output 14.6V, higher than maximum 12.6V input specified for Waveshare control board).
- **Operation Safety**: Ensure surrounding safety during robot operation, prevent personal injury or equipment damage.
- **Emergency Stop Usage**: Use DC circuit breaker to cut power in emergency situations.
- **Long-term Storage**: Disconnect battery connection when not using for long time, prevent battery over-discharge.

---

## FAQ

### Q1: Servo communication abnormal
**A**: Check if servo ID is correctly allocated, check if cable connection is firm, check bus terminal resistor configuration.

### Q2: Lift mechanism jamming
**A**: Check if gear meshing is normal, check if guide rails have foreign objects, check if servo torque is sufficient.

### Q3: USB device disconnection
**A**: Check if USB cable is loose, recommend using twisted pairs to improve anti-interference, check if power supply is stable.

---

## Appendix

### Servo ID Allocation Table

| Servo Position | ID | Description |
|----------------|----|-------------|
| Base left rear wheel | 7 | Driving wheel drive |
| Base right rear wheel | 8 | Driving wheel drive |
| Lift mechanism | 11 | For body core and dual arm lift |
| Left arm joints | 1-6 | Slave arm joints |
| Right arm joints | 1-6 | Slave arm joints |

> **Tip**: Base two wheels and lift mechanism servos are all connected to left arm Waveshare control board, share one control board with left arm. Specific ID allocation may vary based on configuration, please adjust according to actual situation.
