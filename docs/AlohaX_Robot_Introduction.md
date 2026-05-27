# AlohaX Robot Introduction

AlohaX is a dual-arm four-wheel humanoid robot that combines the core advantages of AlohaMini and XLeRobot. It is also currently one of the most cost-effective dual-arm four-wheel humanoid robots, focusing on practical applications in education and research scenarios.

https://github.com/liyiteng/AlohaMini
https://github.com/Vector-Wangel/XLeRobot

## Core Positioning

A low-cost practical dual-arm wheeled robot for education and research fields, capable of completing basic tasks such as picking up and transporting small items. Compared to traditional desktop teaching robots, it is cheaper and more practical. Compared to AlohaMini, it has higher stability, and compared to xlerobot, it has a wider working range.

## Core Design

### Base Solution
Adopts xlerobot0.4.0 core architecture (2 driving wheels + 2 driven wheels), optimal cost-effectiveness and stability. Alternatively, xlerobot0.3.0 or mecanum wheel solution can be selected (3-wheel version has better software compatibility with AlohaMini, supports left-right translation, but slower speed, weaker obstacle crossing ability, requires flat ground).
- IKEA Raskog cart retains only the bottom basket, base rotated 90 degrees, more coordinated aspect ratio, improved passability.
- Base basket reinforced with wood board, solves base softness and motion wobble issues.
- Arms naturally hang outside the body at zero position, more natural posture.

### Body and Dual Arms
Based on AlohaMini body module, key optimizations:
- Body module placed at front of base, convenient for picking up items.
- Arm mounting position lowered below shoulders, lower center of gravity, more natural posture.
- Retains AlohaMini body lift DOF, expands working range (covers ground to tabletop picking/transport scenarios).
- End maximum load 300g, can complete basic tasks such as picking up and transporting small items.

### Electronics and Wiring
- Main controller can choose NVIDIA Orin nano or Raspberry Pi series, or desktop PC or laptop.
- USB Hub placed above chest core module, power terminal below, reduces overall wiring length.
- Dual arm drivers/camera USB cables routed around shoulders, except for battery to power terminal cables, all other cables fixed to body core, aesthetic and tangle-free.
- Servo cables recommended to use twisted pairs, improves anti-interference, reduces packet loss and USB disconnection risks during operation (twisting method, terminal resistor configuration, software anti-interference optimization still in iteration).

## Power and Safety
- Power solution: 12V 20A lead-acid battery + dedicated switch case (safer, lower cost than lithium battery), battery placed at tail for counterweight, ensures center of gravity at robot center.
- Charging note: Only connect battery during operation, disconnect entire robot wiring during charging (most chargers output 14.6V, higher than maximum 12.6V input specified for Waveshare control board).
- Safety protection: Added DC circuit breaker, achieves overcurrent automatic power-off, also serves as emergency stop switch.
- Keep a safe distance during robot operation to prevent personal injury.
- Continuously monitor robot operation status during operation to prevent robot damage and fire hazards.

## Software Compatibility
Developed based on AlohaMini and xlerobot underlying architecture, retains compatibility with both robots' existing software ecosystems, reduces secondary development costs.

## Core Advantages
1. Cost: Price close to or lower than desktop teaching robots, one of the cheapest dual-arm four-wheel humanoid robots currently available.
2. Practicality: Can complete basic tasks such as picking up and transporting small items, suitable for education/research scenarios.
3. Stability: Larger base area, more stable motion, more natural arm posture.
4. Expandability: Retains body lift DOF, working range covers ground to tabletop.

## Acknowledgments
- Thanks to AlohaMini author liyiteng.
- Thanks to xlerobot author vector wang.

### Attachments
- [AlohaX_Assembly_Guide.md](AlohaX_Assembly_Guide.md)
- [AlohaX_BOM.md](AlohaX_BOM.md)
