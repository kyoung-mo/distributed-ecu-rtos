# distributed-ecu-rtos

> Distributed ECU-based vehicle control using CAN and RTOS

This project is a personal redesign based on the [CATNIP]([https://github.com/kyoung-mo/CATNIP](https://github.com/kyoung-mo/can-based-autonomous-delivery-car)) team project experience.  
It focuses on a distributed ECU architecture using CAN communication,  
FreeRTOS-based task separation, and ultrasonic sensor-driven autonomous vehicle control.

For detailed system design, see [docs/design.md](docs/design.md)

---

## System Overview

| Node | Board | Role |
|---|---|---|
| Sensor ECU | STM32F103 (MangoM32) | HC-SR04 × 4 measurement → CAN TX |
| Drive ECU | STM32L4 (B-L475E-IOT01A) | Obstacle detection → PID control → Motor |
| Mission ECU | Raspberry Pi 5 | SocketCAN monitoring, mode command |

```
[Sensor ECU]  →  CAN 0x020 (50ms)  →  [Drive ECU]
                                             ↕
                                       CAN 0x100 / 0x200
                                             ↕
                                      [Mission ECU]
```

---

## Key Features

- **FreeRTOS task separation** — sensor acquisition, obstacle decision, PID control, CAN TX run as independent tasks
- **Non-blocking ultrasonic measurement** — timer input capture instead of blocking `_delay_us()`
- **Crosstalk prevention** — sequential trigger with timeout-based outlier rejection
- **CAN-based distributed control** — 3-node single bus at 250 kbps
- **SocketCAN monitoring** — Linux userspace C++ tool for real-time CAN frame logging
- **DBC-based communication spec** — `docs/catnip_v2.dbc` written with Vector CANdb++ Editor

---

## CAN Protocol

| CAN ID | Name | From | To | Period |
|---|---|---|---|---|
| 0x020 | SENSOR_RAW | Sensor ECU | Drive ECU | 50ms |
| 0x010 | DRIVE_CMD | Mission ECU | Drive ECU | Event |
| 0x011 | E_STOP | Mission ECU | Drive ECU | Immediate |
| 0x100 | SPEED_FB | Drive ECU | Mission ECU | 50ms |
| 0x200 | HEARTBEAT | Drive ECU | Mission ECU | 100ms |

Full signal definition → [`docs/catnip_v2.dbc`](docs/catnip_v2.dbc)

---

## Hardware

| Component | Qty | Note |
|---|---|---|
| STM32F103 (MangoM32) | 1 | Sensor ECU |
| STM32L4 (B-L475E-IOT01A) | 1 | Drive ECU |
| Raspberry Pi 5 | 1 | Mission ECU |
| HC-SR04 Ultrasonic Sensor | 4 | Front array |
| SN65HVD230 CAN Transceiver | 2 | STM32 nodes |
| MCP2515 CAN Module | 1 | RPi5 (SPI) |
| L298N Motor Driver | 1 | |
| JGB37-520 Encoder Motor | 4 | |

---

## Getting Started

### Sensor ECU / Drive ECU

1. Open `.ioc` file in STM32CubeIDE
2. Generate code
3. Build and flash

### Mission ECU (RPi5)

```bash
# CAN interface setup
chmod +x mission-ecu/setup.sh
./mission-ecu/setup.sh

# Build
cd mission-ecu
mkdir build && cd build
cmake .. && make

# Run monitor
./can_monitor
```

---

## Repository Structure

```
distributed-ecu-rtos/
├── sensor-ecu/        # STM32F103 — Sensor ECU (FreeRTOS + HC-SR04 + CAN TX)
├── drive-ecu/         # STM32L4   — Drive ECU  (FreeRTOS + PID + obstacle FSM)
├── mission-ecu/       # RPi5      — Mission ECU (SocketCAN monitor, C++)
└── docs/
    ├── design.md      # Full system design document
    ├── catnip_v2.dbc  # CAN signal definition (Vector CANdb++)
    ├── pinmap_sensor_ecu.md
    ├── pinmap_drive_ecu.md
    └── wiring.md
```

---

## Background

This project is developed based on experience from the **CATNIP** team project
(CAN-based Autonomous Navigation and Transport Integration Platform).

CATNIP implemented a delivery robot with line tracing, ArUco markers, MQTT,
and cargo authentication. This project strips away the high-level features
and rebuilds around real-time embedded fundamentals:
FreeRTOS, CAN communication, and sensor-driven control.
