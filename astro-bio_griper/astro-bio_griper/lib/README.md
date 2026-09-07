# `lib/` — Hardware Abstraction Layer (HAL)

This directory contains the **Hardware Abstraction Layer** — object-oriented C++ wrappers for each hardware peripheral on the ESP32-C3 gripper board. PlatformIO automatically compiles each subdirectory as a separate static library and links it into the firmware.

---

## Architecture Overview

```
lib/
├── CanManager/          ← CAN bus communication (TWAI peripheral)
│   ├── CanManager.h
│   └── CanManager.cpp
├── SensorArray/         ← All sensor polling (line, water level, pH)
│   ├── SensorArray.h
│   └── SensorArray.cpp
└── PumpController/      ← DRV8871 motor driver via ESP32 LEDC PWM
    ├── PumpController.h
    └── PumpController.cpp
```

Each library:
- Has a **default constructor** (no arguments)
- Has a **`begin()`** method called once during `setup()`
- Is instantiated as a **static object** in `main.cpp` (no heap allocation)
- Is passed **by reference** to the `StateManager` FSM

### Dependency Diagram

```
┌────────────┐     ┌──────────────┐     ┌────────────────┐
│ CanManager │     │ SensorArray  │     │ PumpController │
│            │     │              │     │                │
│ Depends on:│     │ Depends on:  │     │ Depends on:    │
│ • Config.h │     │ • Config.h   │     │ • Config.h     │
│ • CanDict  │     │              │     │                │
│ • TWAI API │     │ • Arduino    │     │ • Arduino      │
│            │     │   ADC/GPIO   │     │   LEDC API     │
└────────────┘     └──────────────┘     └────────────────┘
       │                   │                     │
       └───────────────────┼─────────────────────┘
                           │
                    All three passed
                    by reference to
                    StateManager (src/)
```

> **Note:** These libraries include `Config.h` from the project's `include/` directory. This works because `platformio.ini` contains `-I include` in `build_flags`.

---

## CanManager — CAN Bus Communication

### Purpose
Wraps the ESP32 **TWAI** (Two-Wire Automotive Interface) peripheral, which is the ESP32's built-in CAN 2.0A/B controller. Handles bus initialisation, non-blocking frame reception, command extraction, status/data transmission, heartbeat, and bus health monitoring.

### Public API

```cpp
class CanManager {
public:
    bool begin();                           // Initialise TWAI, start bus
    void process();                         // Non-blocking: drain RX, send heartbeat
    
    bool hasEmergencyStop();                // Consumed on read — true once per E-STOP
    bool popCommand(CanCommand& cmd);       // Pop the latest command (consumed on read)
    
    bool sendStatus(CanStatus status);      // TX a status frame (no payload)
    bool sendFloat(CanStatus status, float value);  // TX with 4-byte float payload
    
    bool isBusHealthy() const;              // True if RX activity within CAN_TIMEOUT_MS
};
```

### How `process()` Works

Called every `loop()` iteration. Performs three tasks:

1. **Drain RX queue:** Non-blocking `twai_receive()` with 0 timeout. Processes all pending frames:
   - `CMD_EMERGENCY_STOP` → sets internal E-STOP flag (highest priority)
   - Valid commands → stored in single-slot buffer (latest command wins)
   - Unknown IDs → silently ignored
   
2. **Bus health check:** If no frame has been received for `CAN_TIMEOUT_MS` (3 seconds), marks bus as unhealthy.

3. **Heartbeat TX:** Every `CAN_HEARTBEAT_MS` (1 second), transmits `STATUS_HEARTBEAT`.

### Command Buffer Design

The command buffer is a **single-slot overwrite buffer**, not a FIFO queue. This is intentional:
- CAN commands are sequential operations — only the latest one matters.
- If the Main Computer sends `CMD_START_EXTRACTION` followed immediately by `CMD_RETURN_TO_IDLE`, only the idle command should be processed.
- The E-STOP flag is separate and never overwritten by regular commands.

### Bus Health & Timeout

```
Time ──────────────────────────────────────────────────►
  │                                                     
  │  Last RX frame                                      
  │  ─────┤                                              
  │       │← CAN_TIMEOUT_MS (3s) →│                     
  │       │                        │                     
  │       │    Bus HEALTHY         │    Bus UNHEALTHY    
  │       │                        │    (if pump running 
  │       │                        │     → ERROR state)  
```

The bus timeout only triggers an ERROR state if the pump is currently running. This prevents false alarms during idle periods when the Main Computer may not be actively sending.

### Data Transmission — Float Encoding

The `sendFloat()` method transmits a float as 4 bytes using `memcpy`:

```cpp
uint8_t payload[4];
memcpy(payload, &value, sizeof(float));
// Sends as little-endian IEEE 754 (ESP32 native byte order)
```

**Main Computer decode (C++):**
```cpp
float ph;
memcpy(&ph, frame.data, sizeof(float));
```

**Main Computer decode (Python):**
```python
import struct
ph = struct.unpack('<f', bytes(data[:4]))[0]
```

### Extending CanManager

**Adding a new payload type (e.g., sending uint16_t sensor data):**

```cpp
// In CanManager.h:
bool sendUint16(CanStatus status, uint16_t value);

// In CanManager.cpp:
bool CanManager::sendUint16(CanStatus status, uint16_t value) {
    uint32_t id = static_cast<uint32_t>(status);
    uint8_t payload[2];
    memcpy(payload, &value, sizeof(uint16_t));
    return transmit(id, payload, 2);
}
```

---

## SensorArray — Sensor Polling

### Purpose
Encapsulates all sensor hardware into a single class. Handles GPIO configuration, ADC reading, non-blocking periodic pH sampling, and statistical analysis (rolling variance) for pH stability detection.

### Public API

```cpp
class SensorArray {
public:
    void begin();                           // Configure GPIO and ADC resolution
    void update();                          // Non-blocking periodic pH sampling
    
    bool isBottomDetected() const;          // Line sensor: true when black (bottom)
    uint16_t getWaterLevel() const;         // Water level ADC (0–4095)
    
    bool isPhStable(float& outStableValue) const;  // True when pH variance < threshold
    float getInstantPh() const;             // Latest pH sample (may be noisy)
};
```

### Sensor Details

#### Line Sensor (KY-033)
- **Pin:** GPIO 0 (digital input)
- **Logic:** `HIGH` = black strip detected (bottom reached), `LOW` = white/reflective
- **Mechanism:** A mechanical tube inside the gripper shifts vertically when it hits the bottom of the container, exposing a black strip to the infrared reflective sensor.
- **Read method:** Direct `digitalRead()` — no debouncing needed (mechanical state, not a button).

#### Water Level Sensor (T1592)
- **Pin:** GPIO 1 (12-bit ADC)
- **Logic:** Higher ADC value = more water detected
- **Threshold:** `WATER_LEVEL_TARGET` (default 2800). When the reading reaches this value, water collection is considered complete.
- **Read method:** Direct `analogRead()` on every call — not buffered.

#### pH Sensor (DFRobot Gravity V2)
- **Pin:** GPIO 2 (12-bit ADC)
- **Sampling:** Every `PH_SAMPLE_INTERVAL_MS` (200 ms) — 5 samples per second
- **Buffer:** Ring buffer of `PH_BUFFER_SIZE` (50) readings
- **Stability:** Declared stable when the **variance** of all 50 buffer readings drops below `PH_VARIANCE_THRESHOLD` (0.005)

### pH Processing Pipeline

```
ADC Read (0–4095)
    │
    ▼
adcToPh() conversion
    │  voltage = (raw / 4095.0) × 3.3V
    │  pH = SLOPE × voltage + OFFSET
    │
    ▼
Ring Buffer [50 samples]
    │  Written every 200 ms
    │  Circular — oldest sample overwritten
    │
    ▼
isPhStable() check
    │  Compute variance (two-pass algorithm)
    │  If variance < 0.005 → stable!
    │  Return mean of all 50 samples
    │
    ▼
Float transmitted via CAN
```

### Variance Calculation

Uses a numerically stable **two-pass algorithm**:

```
Pass 1: mean = Σ(samples) / N
Pass 2: variance = Σ(sample - mean)² / N
```

This avoids the catastrophic cancellation that can occur with single-pass `Σx² - (Σx)²/N` formulas, especially important when pH values are clustered around a narrow range.

### Timing — Non-Blocking Sampling

The `update()` method uses `millis()` to enforce the 200 ms sampling interval:

```cpp
void SensorArray::update() {
    if ((millis() - _lastPhSampleMs) >= PH_SAMPLE_INTERVAL_MS) {
        _lastPhSampleMs = millis();
        // ... sample pH and store in buffer
    }
}
```

This means `update()` returns immediately on most calls (no work to do). It only performs an ADC read and buffer write every 200 ms.

### How Long Until pH Stability?

With 50 samples at 200 ms intervals, the buffer fills after:

```
50 × 200 ms = 10 seconds (minimum)
```

In practice, pH sensors take 10–30 seconds to reach thermal equilibrium in a new solution. The variance threshold will typically be met 10–15 seconds after immersion.

---

## PumpController — Motor Driver

### Purpose
Wraps the **DRV8871** H-bridge motor driver using the ESP32's **LEDC** (LED Controller) peripheral for hardware PWM generation. Controls a peristaltic pump for liquid extraction.

### Public API

```cpp
class PumpController {
public:
    void begin();                                       // Configure LEDC channels
    void startPump(uint8_t dutyCycle = PUMP_DEFAULT_DUTY);  // Forward at given duty
    void stopPump();                                    // Coast stop (both pins LOW)
    bool isRunning() const;                             // True if pump is active
};
```

### DRV8871 Truth Table

| IN1 | IN2 | Mode |
|-----|-----|------|
| PWM | LOW | **Forward** (used for extraction) |
| LOW | PWM | Reverse |
| LOW | LOW | **Coast** (used for stop — motor spins down freely) |
| HIGH | HIGH | Brake (fast decay — NOT used, can stress peristaltic tubing) |

### PWM Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Frequency | 25 kHz | Above human hearing range — no audible whine |
| Resolution | 8-bit (0–255) | Sufficient granularity for pump speed control |
| Default duty | 200/255 ≈ 78% | Good flow rate without stressing the tubing |

### Why Coast Instead of Brake?

When `stopPump()` is called, both IN1 and IN2 go LOW (coast mode). Active braking (both HIGH) forces rapid deceleration, which creates:
- Current spikes through the H-bridge
- Pressure transients in the peristaltic tubing
- Potential backflow from compressed tubing segments

Coast mode lets the rotor spin down naturally, which is safer for the pump and tubing.

### Safety Integration

The `PumpController` is a "dumb" actuator — it has no safety logic of its own. All safety decisions are made by `StateManager`:
- **E-STOP:** `StateManager::checkSafety()` calls `_pump.stopPump()` immediately
- **Bus timeout:** Same mechanism if the pump is running
- **State abort:** `CMD_RETURN_TO_IDLE` calls `stopPump()` before transitioning

The `isRunning()` accessor is used by `StateManager::checkSafety()` to determine whether a bus timeout should trigger an emergency stop (only if the pump is active).

---

## Adding a New Hardware Module

To add a new sensor or actuator:

1. **Create a new library directory:** `lib/YourModule/YourModule.h` and `.cpp`
2. **Follow the pattern:**
   - Default constructor
   - `begin()` method for GPIO/peripheral setup
   - Non-blocking `update()` method if periodic work is needed
   - Const accessors for reading state
3. **Add constants to `Config.h`:** Pin numbers, thresholds, timing
4. **Instantiate in `main.cpp`:** As a `static` object, passed by reference to `StateManager`
5. **Integrate with FSM:** Add the reference to `StateManager`'s constructor and use it in state handlers

```
// Example: adding a temperature sensor
lib/
└── TempSensor/
    ├── TempSensor.h
    └── TempSensor.cpp

// Config.h
#define PIN_TEMP_SENSOR  3
#define TEMP_SAMPLE_MS   500

// main.cpp
static TempSensor temp;
temp.begin();   // in setup()
temp.update();  // in loop()
```
