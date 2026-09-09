# Astro-Bio Gripper — Embedded Sub-Controller

> **European Rover Challenge 2026 — Astro-Bio Subtask**  
> Team Indomitus · ESP32-C3 Firmware · PlatformIO + Arduino Core

---

## Table of Contents

- [Mission Overview](#mission-overview)
- [System Architecture](#system-architecture)
- [Directory Structure](#directory-structure)
- [Hardware Setup](#hardware-setup)
- [CAN Bus Protocol Reference](#can-bus-protocol-reference)
- [Finite State Machine](#finite-state-machine)
- [Building & Flashing](#building--flashing)
- [Calibration Guide](#calibration-guide)
- [Writing Main Computer Code](#writing-main-computer-code)
- [Safety Design](#safety-design)
- [Coding Conventions](#coding-conventions)
- [Troubleshooting](#troubleshooting)

---

## Mission Overview

This firmware runs on an **ESP32-C3** microcontroller located inside the gripper of a robotic arm. Its job is to autonomously execute a **liquid extraction and pH measurement sequence** for the Astro-Bio subtask of the European Rover Challenge (ERC).

The ESP32-C3 is a **sub-controller** — it does not make high-level decisions. It receives commands from the rover's **Main Computer** over a **CAN bus**, executes them using local sensors and actuators, and reports results back over the same bus.

### What It Controls

| Component | Purpose |
|-----------|---------|
| **Peristaltic Pump** (via DRV8871) | Extracts liquid from the sample container |
| **Line Sensor** (KY-033) | Detects when the gripper hits the bottom of the container |
| **Water Level Sensor** (T1592) | Monitors how much liquid has been collected |
| **pH Sensor** (DFRobot Gravity V2) | Measures the pH of the extracted liquid |

### What It Communicates

All communication happens over **CAN 2.0A** (standard 11-bit identifiers) at **500 kbit/s**. The ESP32 uses the built-in **TWAI** peripheral with an external CAN transceiver.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      MAIN COMPUTER                           │
│               (ROS2 / Linux / Jetson / etc.)                 │
│                                                              │
│   Sends: CMD_START_POSITIONING, CMD_START_EXTRACTION, ...    │
│   Receives: STATUS_BOTTOM_REACHED, STATUS_PH_READY, ...     │
└──────────────────┬───────────────────────────────────────────┘
                   │  CAN Bus (500 kbit/s, 11-bit IDs)
                   │
┌──────────────────┴───────────────────────────────────────────┐
│                   ESP32-C3 SUB-CONTROLLER                    │
│                   (This Firmware)                            │
│                                                              │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐    │
│  │ CanManager  │  │ SensorArray  │  │ PumpController   │    │
│  │  (TWAI)     │  │  Line/Water  │  │  (DRV8871 PWM)   │    │
│  │             │  │  /pH sensors │  │                   │    │
│  └──────┬──────┘  └──────┬───────┘  └────────┬─────────┘    │
│         │                │                    │              │
│         └────────────────┼────────────────────┘              │
│                          │                                   │
│                 ┌────────┴────────┐                          │
│                 │  StateManager   │                          │
│                 │  (FSM Engine)   │                          │
│                 └─────────────────┘                          │
└──────────────────────────────────────────────────────────────┘
```

The firmware follows a strict **three-layer architecture**:

1. **`include/`** — Global definitions (pins, CAN IDs, state enums). No executable code.
2. **`lib/`** — Hardware Abstraction Layer (HAL). Object-oriented wrappers for each peripheral.
3. **`src/`** — Business logic. The FSM engine and `main.cpp` entry point.

---

## Directory Structure

```
astro-bio_griper/
├── platformio.ini                  ← Build configuration
│
├── include/                        ← Global definitions (NO executable code)
│   ├── Config.h                    ← Pin mappings, PWM, ADC thresholds, timing
│   ├── CanDictionary.h             ← CAN command & status ID enums
│   └── StateDefinitions.h          ← FSM state enums
│
├── lib/                            ← Hardware Abstraction Layer (HAL)
│   ├── CanManager/                 ← ESP32 TWAI (CAN) peripheral driver
│   │   ├── CanManager.h
│   │   └── CanManager.cpp
│   ├── SensorArray/                ← Line, water level, and pH sensor polling
│   │   ├── SensorArray.h
│   │   └── SensorArray.cpp
│   └── PumpController/             ← DRV8871 motor driver via ESP32 LEDC PWM
│       ├── PumpController.h
│       └── PumpController.cpp
│
├── src/                            ← Core business logic
│   ├── main.cpp                    ← Entry point (non-blocking loop)
│   └── StateManager/               ← Finite State Machine
│       ├── StateManager.h
│       └── StateManager.cpp
│
└── test/                           ← PlatformIO test runner (future)
```

Each directory has its own `README.md` with detailed documentation.

---

## Hardware Setup

### Pinout Table

| Function | GPIO | Type | Component | Notes |
|----------|------|------|-----------|-------|
| Pump IN1 | 9 | PWM (LEDC Ch.0) | DRV8871 | Forward direction |
| Pump IN2 | 10 | PWM (LEDC Ch.1) | DRV8871 | Reverse / brake |
| CAN TX | 20 | Digital | TWAI → Transceiver | To CAN transceiver TXD |
| CAN RX | 21 | Digital | TWAI ← Transceiver | From CAN transceiver RXD |
| Line Sensor | 0 | Digital Input | KY-033 | HIGH = black (bottom), LOW = white |
| Water Level | 1 | ADC (12-bit) | T1592 | 0–4095, higher = more water |
| pH Sensor | 2 | ADC (12-bit) | DFRobot Gravity V2 | Analog voltage → pH conversion |

### Wiring Diagram (Logical)

```
ESP32-C3                  DRV8871
  GPIO 9  ──────────────► IN1
  GPIO 10 ──────────────► IN2
  GND     ──────────────► GND

ESP32-C3                  CAN Transceiver (e.g., SN65HVD230)
  GPIO 20 ──────────────► TXD
  GPIO 21 ◄──────────────  RXD
  3.3V    ──────────────► VCC
  GND     ──────────────► GND
                          CANH ──── CAN Bus ────
                          CANL ──── CAN Bus ────

ESP32-C3                  Sensors
  GPIO 0  ◄──────────────  KY-033 DATA (digital)
  GPIO 1  ◄──────────────  T1592 DATA (analog)
  GPIO 2  ◄──────────────  DFRobot pH DATA (analog)
```

---

## CAN Bus Protocol Reference

> **This is the most important section if you are writing code for the Main Computer.**

### Bus Parameters

| Parameter | Value |
|-----------|-------|
| Standard | CAN 2.0A |
| Identifier | 11-bit standard |
| Bitrate | 500 kbit/s |
| Byte order | Little-endian (IEEE 754 float, native ESP32 byte order) |

### ID Allocation Scheme

```
0x000 ─────── Reserved
  │
0x100 ─────── Commands (Main Computer → Gripper ESP32)
  │            0x100  CMD_EMERGENCY_STOP
  │            0x110  CMD_START_POSITIONING
  │            0x120  CMD_START_EXTRACTION
  │            0x130  CMD_START_ACQUISITION
  │            0x140  CMD_RETURN_TO_IDLE
  │
0x200 ─────── Statuses (Gripper ESP32 → Main Computer)
  │            0x200  STATUS_IDLE
  │            0x210  STATUS_BOTTOM_REACHED
  │            0x211  STATUS_OPTIMAL_POSITION
  │            0x220  STATUS_COLLECTION_DONE
  │            0x230  STATUS_PH_READY          ← 4-byte float payload!
  │            0x2F0  STATUS_ERROR
  │            0x2FF  STATUS_HEARTBEAT
  │
0x300 ─────── Reserved (future data frames)
```

### Command Frames (Main Computer → Gripper)

These are frames the **Main Computer sends** to the ESP32. All command frames have **DLC = 0** (no payload data — the command is encoded entirely in the CAN ID).

| CAN ID | Name | DLC | Payload | Description |
|--------|------|-----|---------|-------------|
| `0x100` | `CMD_EMERGENCY_STOP` | 0 | — | Immediately halt pump, transition to ERROR |
| `0x110` | `CMD_START_POSITIONING` | 0 | — | Begin bottom-detection sequence |
| `0x120` | `CMD_START_EXTRACTION` | 0 | — | Start pump, begin water collection |
| `0x130` | `CMD_START_ACQUISITION` | 0 | — | Begin pH measurement and stabilisation |
| `0x140` | `CMD_RETURN_TO_IDLE` | 0 | — | Abort current operation, return to IDLE |

### Status Frames (Gripper → Main Computer)

These are frames the **ESP32 sends back** to the Main Computer.

| CAN ID | Name | DLC | Payload | Description |
|--------|------|-----|---------|-------------|
| `0x200` | `STATUS_IDLE` | 0 | — | System is idle and ready for commands |
| `0x210` | `STATUS_BOTTOM_REACHED` | 0 | — | Line sensor detected bottom — **stop lowering the arm!** |
| `0x211` | `STATUS_OPTIMAL_POSITION` | 0 | — | Arm lifted to optimal height — positioning complete |
| `0x220` | `STATUS_COLLECTION_DONE` | 0 | — | Water level target met, pump stopped |
| `0x230` | `STATUS_PH_READY` | **4** | `float (IEEE 754)` | Stable pH value (see payload format below) |
| `0x2F0` | `STATUS_ERROR` | 0 | — | System entered ERROR state |
| `0x2FF` | `STATUS_HEARTBEAT` | 0 | — | Periodic alive signal (every 1 second) |

### pH Payload Format (`STATUS_PH_READY`, ID `0x230`)

The pH value is transmitted as a **4-byte IEEE 754 single-precision float** in **little-endian** byte order (native ESP32 memory layout).

```
Byte:   [0]    [1]    [2]    [3]
        └──────────────────────┘
         IEEE 754 float (LE)

Example: pH 7.125
  memcpy(payload, &float_value, 4)
  → payload = { 0x00, 0x40, 0xE4, 0x40 }  (little-endian)
```

**To decode on the Main Computer (C/C++):**
```c
float ph_value;
memcpy(&ph_value, can_frame.data, sizeof(float));
// ph_value is now 7.125
```

**To decode on the Main Computer (Python with python-can):**
```python
import struct
ph_value = struct.unpack('<f', bytes(msg.data[:4]))[0]
# ph_value is now 7.125
```

---

## Finite State Machine

### State Diagram

```
                         ┌──────────────────────────────┐
                         │          E-STOP or            │
                         │       CAN Bus Timeout         │
                         │    (pump halted immediately)  │
                         └────────┬─────────────────────┘
                                  │
                                  ▼
    ┌─────────┐              ┌─────────┐
    │         │◄─────────────│  ERROR  │
    │         │  CMD_RETURN  │         │
    │         │  _TO_IDLE    └─────────┘
    │         │                   ▲
    │  IDLE   │───────────────────┤ (from any active state)
    │         │                   │
    │         │    ┌──────────────┤
    │         │    │              │
    └────┬────┘    │              │
         │        │              │
    ┌────┼────────┼──────────────┼─────────────────────┐
    │    │        │              │                      │
    │    ▼        │              │                      │
    │ CMD_START   │ CMD_START    │  CMD_START           │
    │ POSITIONING │ EXTRACTION  │  ACQUISITION         │
    │    │        │              │                      │
    │    ▼        ▼              ▼                      │
    │ ┌───────────┐ ┌──────────┐ ┌────────────┐        │
    │ │POSITIONING│ │EXTRACTION│ │ACQUISITION │        │
    │ │           │ │          │ │            │        │
    │ │ Phase 1:  │ │ Pump ON  │ │ Poll pH    │        │
    │ │ Wait for  │ │ Poll     │ │ variance   │        │
    │ │ bottom    │ │ water    │ │ buffer     │        │
    │ │ (black)   │ │ level    │ │            │        │
    │ │     │     │ │    │     │ │     │      │        │
    │ │     ▼     │ │    ▼     │ │     ▼      │        │
    │ │ Phase 2:  │ │ Target   │ │ Variance   │        │
    │ │ Wait for  │ │ reached  │ │ < thresh   │        │
    │ │ liftoff   │ │ → pump   │ │ → send pH  │        │
    │ │ (white)   │ │   OFF    │ │   float    │        │
    │ └─────┬─────┘ └────┬─────┘ └──────┬─────┘        │
    │       │             │              │              │
    │       └─────────────┼──────────────┘              │
    │                     │                             │
    │                     ▼                             │
    │              Return to IDLE                       │
    └───────────────────────────────────────────────────┘
```

### State Descriptions

#### `IDLE`
- **Pump:** OFF
- **Behaviour:** Waits for CAN commands. Does nothing else.
- **Transitions:** Responds to `CMD_START_POSITIONING`, `CMD_START_EXTRACTION`, `CMD_START_ACQUISITION`.

#### `POSITIONING` (two-phase)
- **Phase 1 — WAIT_FOR_BOTTOM:** Polls the line sensor. When it reads HIGH (black strip = bottom detected), sends `STATUS_BOTTOM_REACHED` to tell the Main Computer to stop lowering the arm.
- **Phase 2 — WAIT_FOR_LIFTOFF:** Waits for the line sensor to return to LOW (white = arm pulled up). Sends `STATUS_OPTIMAL_POSITION`, then returns to IDLE.
- **Can be aborted** with `CMD_RETURN_TO_IDLE`.

#### `EXTRACTION`
- **Pump:** ON at `PUMP_DEFAULT_DUTY` (200/255 ≈ 78%)
- **Behaviour:** Polls the water level ADC. When it reaches `WATER_LEVEL_TARGET` (default 2800/4095), immediately stops the pump and sends `STATUS_COLLECTION_DONE`.
- **Can be aborted** with `CMD_RETURN_TO_IDLE` (pump will stop).

#### `ACQUISITION`
- **Pump:** OFF
- **Behaviour:** The pH sensor is continuously sampled every 200 ms into a 50-sample ring buffer. Each tick checks whether the variance of the buffer has dropped below `PH_VARIANCE_THRESHOLD` (default 0.005). Once stable, the mean pH is transmitted as a 4-byte float via `STATUS_PH_READY`.
- **Can be aborted** with `CMD_RETURN_TO_IDLE`.

#### `ERROR`
- **Pump:** Immediately halted
- **Behaviour:** Entered on E-STOP or CAN bus timeout (with pump running). Only `CMD_RETURN_TO_IDLE` can exit this state.

---

## Building & Flashing

### Prerequisites

- [PlatformIO CLI](https://platformio.org/install/cli) or [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
- USB cable connected to the ESP32-C3

### Build

```bash
pio run
```

Expected output:
```
RAM:   [          ]   4.4% (used 14,316 bytes from 327,680 bytes)
Flash: [==        ]  20.0% (used 261,930 bytes from 1,310,720 bytes)
========================= [SUCCESS] =========================
```

### Flash

```bash
pio run --target upload
```

### Monitor Serial Output

```bash
pio device monitor --baud 115200
```

You should see:
```
========================================
  Astro-Bio Gripper — ERC 2026
  Indomitus Embedded Control
========================================
[SENSOR] Array initialised
[PUMP] Controller initialised — coast mode
[CAN] Bus initialised — 500 kbit/s
[INIT] System ready — entering IDLE
========================================
```

---

## Calibration Guide

Before each competition run, update these values in `include/Config.h`:

### Water Level Threshold

```c
#define WATER_LEVEL_TARGET  2800    // 12-bit ADC (0–4095)
```

**How to calibrate:** Fill the container to the desired level. Read the ADC value from Serial output or via a test sketch. Set this value ~100 counts below the reading to account for noise.

### pH Sensor Calibration

```c
#define PH_SLOPE   (-5.70f)     // Linear calibration slope
#define PH_OFFSET  21.34f       // Linear calibration offset
```

**How to calibrate:**
1. Immerse the sensor in **pH 4.0** buffer solution. Record the ADC voltage.
2. Immerse in **pH 7.0** buffer solution. Record the ADC voltage.
3. Compute: `SLOPE = (4.0 - 7.0) / (V_4 - V_7)` and `OFFSET = 7.0 - SLOPE * V_7`

### pH Stability Threshold

```c
#define PH_VARIANCE_THRESHOLD  0.005f
```

Lower = more strict (takes longer to stabilise). Higher = faster but noisier readings. Start at 0.005 and adjust based on observed sensor noise.

### Pump Duty Cycle

```c
#define PUMP_DEFAULT_DUTY  200    // 0–255 (8-bit PWM)
```

Adjust flow rate to match your tubing diameter and container geometry.

---

## Writing Main Computer Code

> **This section is for the developer writing the rover's Main Computer software that communicates with this ESP32 sub-controller.**

### Interaction Pattern

The Main Computer acts as the **command initiator**. The ESP32 is purely reactive — it only acts when told to, and reports back when done.

```
Main Computer                           ESP32 Gripper
     │                                       │
     │──── CMD_START_POSITIONING (0x110) ────►│
     │                                       │  (polls line sensor)
     │◄──── STATUS_BOTTOM_REACHED (0x210) ───│  ← STOP LOWERING ARM!
     │                                       │
     │      (Main Computer raises arm)       │
     │                                       │
     │◄──── STATUS_OPTIMAL_POSITION (0x211) ─│  ← arm in position
     │                                       │
     │──── CMD_START_EXTRACTION (0x120) ─────►│
     │                                       │  (pump runs, monitors water)
     │◄──── STATUS_COLLECTION_DONE (0x220) ──│  ← water collected
     │                                       │
     │──── CMD_START_ACQUISITION (0x130) ────►│
     │                                       │  (pH stabilises ~10 seconds)
     │◄──── STATUS_PH_READY (0x230) ─────────│  ← pH value in payload!
     │      [4 bytes: IEEE 754 float]        │
     │                                       │
```

### Typical Sequence (Pseudocode)

```python
# 1. POSITIONING — find the bottom of the container
send_can(id=0x110, data=[], dlc=0)           # CMD_START_POSITIONING
wait_for_can(id=0x210)                        # STATUS_BOTTOM_REACHED
stop_arm_motor()                              # ← YOU must stop the arm!
raise_arm(distance=OPTIMAL_OFFSET)            # ← YOU must raise the arm
wait_for_can(id=0x211)                        # STATUS_OPTIMAL_POSITION

# 2. EXTRACTION — collect water
send_can(id=0x120, data=[], dlc=0)           # CMD_START_EXTRACTION
wait_for_can(id=0x220)                        # STATUS_COLLECTION_DONE

# 3. ACQUISITION — measure pH
send_can(id=0x130, data=[], dlc=0)           # CMD_START_ACQUISITION
msg = wait_for_can(id=0x230)                  # STATUS_PH_READY
ph_value = struct.unpack('<f', msg.data[:4])  # decode 4-byte float
print(f"pH = {ph_value}")
```

### Emergency Stop

At **any time**, sending `CMD_EMERGENCY_STOP` (`0x100`) will:
1. Immediately halt the pump
2. Transition the ESP32 to `ERROR` state
3. Send back `STATUS_ERROR` (`0x2F0`)

To recover from ERROR, send `CMD_RETURN_TO_IDLE` (`0x140`).

### Heartbeat Monitoring

The ESP32 sends `STATUS_HEARTBEAT` (`0x2FF`) every **1 second**. If the Main Computer doesn't see a heartbeat for >3 seconds, the ESP32 may have crashed or the bus is disconnected.

**Conversely**, if the ESP32 doesn't receive **any** CAN frame for 3 seconds (`CAN_TIMEOUT_MS`) while the pump is running, it will emergency-stop the pump and enter ERROR state. If your Main Computer is idle, **periodically send any frame** (even a heartbeat) to keep the bus alive.

### Code Examples

#### C++ (Linux SocketCAN)
```cpp
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>

// Send a command
struct can_frame frame;
frame.can_id = 0x120;  // CMD_START_EXTRACTION
frame.can_dlc = 0;
write(can_socket, &frame, sizeof(frame));

// Receive pH value
struct can_frame rx;
read(can_socket, &rx, sizeof(rx));
if (rx.can_id == 0x230 && rx.can_dlc == 4) {
    float ph;
    memcpy(&ph, rx.data, sizeof(float));
    printf("pH = %.3f\n", ph);
}
```

#### Python (python-can)
```python
import can
import struct

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

# Send command
bus.send(can.Message(arbitration_id=0x120, data=[], is_extended_id=False))

# Wait for pH
while True:
    msg = bus.recv(timeout=30.0)
    if msg and msg.arbitration_id == 0x230:
        ph = struct.unpack('<f', bytes(msg.data[:4]))[0]
        print(f"pH = {ph:.3f}")
        break
```

#### ROS2 (C++ with ros2_socketcan)
```cpp
// In your CAN bridge node callback:
void canCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    switch (msg->id) {
        case 0x210: // STATUS_BOTTOM_REACHED
            stopArmMotor();
            break;
        case 0x230: // STATUS_PH_READY
            float ph;
            std::memcpy(&ph, msg->data.data(), sizeof(float));
            publishPhValue(ph);
            break;
    }
}
```

---

## Safety Design

### Fail-Safe Mechanisms

| Trigger | Condition | Action |
|---------|-----------|--------|
| **E-STOP Frame** | CAN ID `0x100` received | Pump halted immediately → ERROR state |
| **CAN Bus Timeout** | No RX frames for 3 seconds **AND** pump is running | Pump halted → ERROR state |
| **Abort Command** | CAN ID `0x140` received in any active state | Pump halted (if running) → IDLE |

### Safety Priority

The `checkSafety()` function runs **before** any state logic on every loop iteration. This means:
1. E-STOP is checked first, every single tick.
2. Bus health is checked second.
3. Only then does the current state handler execute.

### Why Coast Mode?

When the pump stops, both motor driver pins go LOW (coast mode). This is intentional — active braking (both HIGH) would create a current spike through the peristaltic pump tubing, which could damage it or cause pressure buildup. Coast mode allows the motor to spin down naturally.

---

## Coding Conventions

| Rule | Detail |
|------|--------|
| **No magic numbers** | Every constant is `#define`d in `Config.h` |
| **No `delay()`** | Forbidden outside `setup()`. Use `millis()` timers |
| **No dynamic allocation** | No `new`/`malloc` in `loop()`. All objects are static |
| **Pass by reference** | HAL objects are passed by `&` reference, never copied |
| **`#pragma once`** | Used in all headers instead of include guards |
| **Serial logging** | All modules prefix logs: `[CAN]`, `[SENSOR]`, `[PUMP]`, `[FSM]`, `[INIT]` |

---

## Troubleshooting

### `[CAN] Driver install FAILED`
- Check wiring to the CAN transceiver (GPIO 20 → TXD, GPIO 21 → RXD)
- Ensure the transceiver has 3.3V power and shared GND
- Verify CAN bus termination (120Ω resistor at each end of the bus)

### `[CAN] TX FAIL` / Bus not healthy
- Another node must be on the bus for acknowledgement (CAN requires at least 2 nodes)
- Check for bus termination
- Verify bitrate matches on both ends (500 kbit/s)

### pH never stabilises (ACQUISITION state never completes)
- Lower `PH_VARIANCE_THRESHOLD` in `Config.h` (e.g., 0.01 or 0.02)
- Ensure the sensor is fully immersed in the liquid
- Check that the sensor cable is not near motor wires (EMI)
- Wait for thermal equilibrium after inserting the sensor

### Water level never triggers (EXTRACTION state never completes)
- Adjust `WATER_LEVEL_TARGET` in `Config.h` after reading actual ADC values
- Ensure the T1592 probe is properly positioned in the collection vessel

### Pump runs but no flow
- Check tubing connections for kinks or air locks
- Verify motor direction (swap IN1/IN2 wiring or modify `startPump()` to use IN2)
- Increase `PUMP_DEFAULT_DUTY` (try 255 for full power)

---

## License

Internal competition firmware — Team Indomitus, ERC 2026.
