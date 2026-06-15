# `include/` — Global Definitions

This directory contains **header-only definition files** shared across the entire project. These files define constants, pin mappings, CAN IDs, and FSM state enumerations. They contain **no executable code** — only `#define` macros and `enum class` declarations.

All three files use `#pragma once` for include guard protection.

---

## Files

| File | Purpose |
|------|---------|
| `Config.h` | Hardware pin mappings, PWM parameters, ADC thresholds, pH calibration, timing constants |
| `CanDictionary.h` | CAN command and status ID enumerations |
| `StateDefinitions.h` | FSM state and sub-state enumerations |

---

## `Config.h` — Hardware Configuration

This is the **single source of truth** for all hardware-related constants. Every pin number, threshold, timing value, and calibration parameter is defined here. **No magic numbers exist anywhere else in the codebase.**

### Pin Assignments

```
GPIO  0  →  KY-033 Line Sensor (digital input)
GPIO  1  →  T1592 Water Level Sensor (ADC)
GPIO  2  →  DFRobot pH Sensor (ADC)
GPIO  9  →  DRV8871 IN1 (PWM — pump forward)
GPIO 10  →  DRV8871 IN2 (PWM — pump reverse/brake)
GPIO 20  →  CAN TX (TWAI → transceiver)
GPIO 21  →  CAN RX (TWAI ← transceiver)
```

### Constants by Category

#### Serial
| Constant | Default | Description |
|----------|---------|-------------|
| `SERIAL_BAUD_RATE` | 115200 | USB-CDC serial monitor baud rate |

#### Pump Control (DRV8871 + LEDC PWM)
| Constant | Default | Description |
|----------|---------|-------------|
| `PIN_PUMP_IN1` | 9 | Motor driver forward pin |
| `PIN_PUMP_IN2` | 10 | Motor driver reverse pin |
| `PWM_CHANNEL_IN1` | 0 | ESP32 LEDC channel for IN1 |
| `PWM_CHANNEL_IN2` | 1 | ESP32 LEDC channel for IN2 |
| `PWM_FREQUENCY_HZ` | 25000 | 25 kHz — above audible range |
| `PWM_RESOLUTION_BITS` | 8 | 8-bit resolution (0–255) |
| `PUMP_DEFAULT_DUTY` | 200 | Default duty cycle (~78%). **Tune on-site.** |

#### CAN Bus (TWAI)
| Constant | Default | Description |
|----------|---------|-------------|
| `PIN_CAN_TX` | 20 | TWAI transmit pin |
| `PIN_CAN_RX` | 21 | TWAI receive pin |
| `CAN_BITRATE_KBPS` | 500 | Bus speed — must match Main Computer |
| `CAN_RX_QUEUE_LEN` | 10 | TWAI driver internal RX queue depth |
| `CAN_HEARTBEAT_MS` | 1000 | Heartbeat transmission interval (ms) |
| `CAN_TIMEOUT_MS` | 3000 | Bus disconnect timeout (ms). If no frames received for this long **and** the pump is running, the system enters ERROR state. |

#### Line Sensor (KY-033)
| Constant | Default | Description |
|----------|---------|-------------|
| `PIN_LINE_SENSOR` | 0 | Digital input — `HIGH` = black (bottom detected), `LOW` = white |

#### Water Level Sensor (T1592)
| Constant | Default | Description |
|----------|---------|-------------|
| `PIN_WATER_LEVEL` | 1 | ADC input (12-bit, 0–4095) |
| `WATER_LEVEL_TARGET` | 2800 | ADC threshold for "sufficient water collected". **Calibrate before each run.** |

#### pH Sensor (DFRobot Gravity V2)
| Constant | Default | Description |
|----------|---------|-------------|
| `PIN_PH_SENSOR` | 2 | ADC input (12-bit, 0–4095) |
| `PH_SAMPLE_INTERVAL_MS` | 200 | Sampling period (5 Hz) |
| `PH_BUFFER_SIZE` | 50 | Rolling variance window size |
| `PH_VARIANCE_THRESHOLD` | 0.005 | Maximum variance for "stable" declaration |
| `PH_VOLTAGE_REF` | 3.3 | ESP32-C3 ADC reference voltage |
| `PH_ADC_RESOLUTION` | 4095.0 | 12-bit ADC max value |
| `PH_SLOPE` | -5.70 | Linear calibration slope. **Recalibrate with buffer solutions.** |
| `PH_OFFSET` | 21.34 | Linear calibration offset. **Recalibrate with buffer solutions.** |

#### Timing & Safety
| Constant | Default | Description |
|----------|---------|-------------|
| `LOOP_WATCHDOG_MS` | 50 | Reserved for future loop cadence monitoring |
| `ESTOP_DEBOUNCE_MS` | 10 | Reserved for E-STOP debounce logic |

### How to Modify

1. **Changing a pin?** Update the `PIN_*` constant here. No other file needs to change.
2. **Adjusting pump speed?** Change `PUMP_DEFAULT_DUTY` (0–255).
3. **Tuning pH stability?** Adjust `PH_VARIANCE_THRESHOLD`. Lower = stricter, slower. Higher = looser, faster.
4. **Changing CAN speed?** Update `CAN_BITRATE_KBPS` and also change the timing config macro in `CanManager.cpp` (e.g., `TWAI_TIMING_CONFIG_250KBITS()` for 250 kbit/s).

---

## `CanDictionary.h` — CAN Message IDs

Defines two `enum class` types for the CAN protocol:

### `CanCommand` (Main Computer → Gripper)
ID range: `0x100 – 0x1FF`

```cpp
enum class CanCommand : uint16_t {
    CMD_EMERGENCY_STOP      = 0x100,
    CMD_START_POSITIONING   = 0x110,
    CMD_START_EXTRACTION    = 0x120,
    CMD_START_ACQUISITION   = 0x130,
    CMD_RETURN_TO_IDLE      = 0x140,
};
```

### `CanStatus` (Gripper → Main Computer)
ID range: `0x200 – 0x2FF`

```cpp
enum class CanStatus : uint16_t {
    STATUS_IDLE             = 0x200,
    STATUS_BOTTOM_REACHED   = 0x210,
    STATUS_OPTIMAL_POSITION = 0x211,
    STATUS_COLLECTION_DONE  = 0x220,
    STATUS_PH_READY         = 0x230,    // 4-byte float payload
    STATUS_ERROR            = 0x2F0,
    STATUS_HEARTBEAT        = 0x2FF,
};
```

### Adding New CAN IDs

To add a new command or status:
1. Add the entry to the appropriate `enum class` in this file.
2. **Commands:** Handle the new ID in `CanManager::process()` (add to the `switch` statement).
3. **Statuses:** Call `_can.sendStatus(CanStatus::YOUR_NEW_STATUS)` from the appropriate state handler.
4. **With payload:** Use `_can.sendFloat()` or extend `CanManager` with a new method for different payload types.

> **Note for Main Computer developers:** The `uint16_t` underlying type stores the 11-bit CAN identifier value. When sending from the Main Computer, use these hex values directly as the CAN arbitration ID.

---

## `StateDefinitions.h` — FSM State Enumerations

### `SystemState`

The main FSM states:

```cpp
enum class SystemState : uint8_t {
    IDLE,           // Pump OFF, awaiting CAN commands
    POSITIONING,    // Polling line sensor for bottom detection & lift-off
    EXTRACTION,     // Pump ON, polling water level until target reached
    ACQUISITION,    // Polling pH sensor until variance stabilises
    ERROR,          // Fault condition — pump halted, awaiting recovery
};
```

### `PositioningPhase`

Sub-states within the `POSITIONING` state:

```cpp
enum class PositioningPhase : uint8_t {
    WAIT_FOR_BOTTOM,    // Lowering — waiting for line sensor to detect black
    WAIT_FOR_LIFTOFF,   // Bottom was detected — waiting for arm to pull up
};
```

### Adding a New State

1. Add the state to `SystemState` in this file.
2. Create a `handleYourNewState()` method in `StateManager`.
3. Add the case to the `switch` in `StateManager::run()`.
4. Define the transitions (which commands enter/exit your state).
