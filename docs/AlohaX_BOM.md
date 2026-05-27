# AlohaX Bill of Materials

AlohaX hardware includes mobile base and two slave arms. Arms directly reuse SO-ARM100/SO-ARM101 design. Therefore, if you already have two SO-ARM101 arms, you only need to assemble the mobile base. Communication between mobile base, arms, and main controller (can choose NVIDIA Orin nano, Raspberry Pi series, desktop PC or laptop) uses the same bus servo communication system.

*Note: Common tools such as bench vise, screwdriver set can be procured as needed, not listed in table below.*

## Mobile Base

| Item | Model / Description | Quantity | Unit Price (CN) | Purchase (CN) |
|------|---------------------|----------|-----------------|---------------|
| Servo motor | Feetech / 12V 1/345 reduction ratio (STS3215-C018) | 4 | ¥110 | [Taobao](https://item.taobao.com/item.htm?id=996544351583) |
| Wheels | Driving wheel + driven wheel (xlerobot0.4.0 solution) | 2+2 | - | - |
| USB camera | 720p focal length 2.4mm, 36×36mm form factor | 3 | ¥125 | [Taobao](https://item.taobao.com/item.htm?id=666278411821) |
| 3D camera (optional) | Astra Pro | 1 | ¥180 | - |
| IKEA Raskog cart | Raskog cart | 1 | ¥199 | [IKEA](https://www.ikea.com/cn/zh/p/raskog-raskog-rollcontainer-gris-70386300/) |
| Battery | 12V 20A lead-acid battery | 1 | ¥158 | - |
| Battery switch cover | 12V 20A battery dedicated switch case | 1 | ¥70 | - |
| 6A DC circuit breaker | Overcurrent automatic power-off, also serves as emergency stop switch | 1 | 30 | - |
| USB Hub | 7-port | 1 | ¥70 | - |
| Wood board | For base reinforcement | 1 | - | - |
| Bearings | Select as needed | - | ¥3 | [Taobao](https://item.taobao.com/item.htm?id=565418362178) |
| M2×12 Phillips screws | For camera mounting | 12 | - | - |
| M3×12 hex socket screws | Select as needed | - | - | - |
| M3×18 hex socket screws | Select as needed | - | - | - |
| M2×30 hex socket screws | Select as needed | - | - | - |
| M3 hex nuts | Select as needed | - | - | - |
| M3×5×4 heat insert nuts | Select as needed | - | ¥5 | [Taobao](https://item.taobao.com/item.htm?id=809241671998) |
| M4×12 hex socket screws | Select as needed | - | - | - |
| M4×6×5 heat insert nuts | Select as needed | - | ¥4 | [Taobao](https://item.taobao.com/item.htm?id=809241671998) |
| Adhesive | Epoxy (highly recommended) / double-sided tape - for cable securing and structural bonding | 1 | ¥12 | [JD](https://item.jd.com/100141557259.html) |
| Servo extension cable | SCS 3-pin, 90cm | 2 | ¥3 | [Taobao](https://item.taobao.com/item.htm?id=616460581906) |
| SPL-62 terminal block | For power supply | 1 | ¥5 | [Taobao](https://item.taobao.com/item.htm?id=657816423504) |
| USB Type-C cable | Only for testing mobile base | 1 | ¥20 | [Tmall](https://detail.tmall.com/item.htm?id=754024805047) |
| Waveshare bus servo controller (or JUXI Bus Servo Adaptor) | Only for testing mobile base | 1 | ¥27 | [Tmall](https://detail.tmall.com/item.htm?id=738817173460) |
| 3D printed parts | PLA/PETG/ABS (files in project hardware directory) | - | - | - |

## Slave Arm

| Item | Model / Description | Quantity | Unit Price (CN) | Purchase (CN) |
|------|---------------------|----------|-----------------|---------------|
| Servo motor | Feetech / 12V 1/345 reduction ratio (STS3215-C018) | 12 | ¥110 | [Taobao](https://item.taobao.com/item.htm?id=996544351583) |
| Waveshare bus servo controller | For connecting to main controller | 2 | ¥27 | [Tmall](https://detail.tmall.com/item.htm?id=738817173460) |
| USB camera | 720p focal length 3.8mm, 32×32mm form factor | 2 | ¥103 | [Taobao](https://item.taobao.com/item.htm?id=590682120464) |
| 1-to-2 DC splitter cable | 30cm, 5521 interface - for arm power supply | 1 | ¥5 | [Taobao](https://item.taobao.com/item.htm?id=594921965049) |
| DC extension cable | 1.5m, 5521 interface - for arm power supply | 2 | ¥2.50 | [Taobao](https://item.taobao.com/item.htm?id=43628177900) |
| USB Type-C cable | For connecting to main controller | 2 | ¥20 | [Tmall](https://detail.tmall.com/item.htm?id=754024805047) |
| 3D printed parts | PLA/PETG/ABS (files in project hardware directory) | 1 set | - | - |

## Main Controller (Optional)

| Item | Model / Description | Quantity | Unit Price (CN) | Purchase (CN) |
|------|---------------------|----------|-----------------|---------------|
| Computing board | NVIDIA Orin nano / Raspberry Pi 5 (≥2GB RAM) / other SBC / PC | 1 | ¥600+ | [Taobao](https://item.taobao.com/item.htm?id=688878446695) |
| DC converter | 12V → 5V / 5A buck converter | 1 | ¥75 | [Taobao](https://item.taobao.com/item.htm?id=800698078303) |
| Display | 7-inch HD IPS HDMI interface + touch + Type C power (optional) | 1 | ¥291 | [Taobao](https://item.taobao.com/item.htm?id=592070943040) |

## Active Arm (Optional, for Teleoperation)

| Item | Model / Description | Quantity | Unit Price (CN) | Purchase (CN) |
|------|---------------------|----------|-----------------|---------------|
| Servo motor | Feetech / 7.4V 1/147 reduction ratio (STS3215-C046) | 12 | ¥99 | [Taobao](https://item.taobao.com/item.htm?id=996544351583) |
| Waveshare bus servo controller | For connecting to PC | 2 | ¥27 | [Tmall](https://detail.tmall.com/item.htm?id=738817173460) |
| Battery | 5V lithium battery pack (5600mAh, DC 5521 interface) | 1 | ¥30 | [Taobao](https://item.taobao.com/item.htm?id=765749120668) |
| 1-to-2 DC splitter cable | 70cm - for arm power supply | 1 | ¥5 | [Taobao](https://item.taobao.com/item.htm?id=594921965049) |
| USB Type-C cable | For connecting to PC | 2 | ¥20 | [Tmall](https://detail.tmall.com/item.htm?id=754024805047) |
| 3D printed parts | PLA/PETG/ABS (files in project hardware directory) | 1 set | - | - |

> Note: Official SO-ARM100 design uses three different reduction ratios (1/147, 1/191, 1/345) for optimal performance. However, our testing shows that using a single reduction ratio (1/147) can provide excellent user experience and significantly simplify assembly. Table above reflects this simplified configuration.

## Main Component Summary

| Component | Model / Description | Quantity | Unit Price (CN) | Subtotal (CN) |
|-----------|---------------------|----------|-----------------|---------------|
| Servo motor | Feetech STS3215 (12V bus) | 16 | ¥110 | ¥1,760 |
| Motor control board | Waveshare bus servo adapter | 3 | ¥27 | ¥81 |
| USB camera | 720p (3 base + 2 arms) | 5 | ¥111 | ¥555 |
| IKEA cart | Raskog cart | 1 | ¥199 | ¥199 |
| Battery + switch cover | 12V 20A lead-acid battery | 1 | ¥233 | ¥233 |
| 3D printing filament | PLA/PETG (approx 2kg) | 1 set | ¥250 | ¥250 |
| Other | - | 1 | ¥200 | ¥200 |
| **Total (excluding main controller and active arm)** | - | - | - | **¥3,278** |

> **Price Notes**:
> - Camera unit price average (base camera ¥125 + arm camera ¥103) / 5 = ¥111
> - Main controller (NVIDIA Orin nano/Raspberry Pi/PC) not included, can be selected as needed (¥600+)
> - Active arm (for teleoperation) not included, optional configuration
> - 3D printed parts and screws and other consumable prices not included, need to print or procure yourself
