# `src/` — Core Business Logic

This directory contains the application entry point and the Finite State Machine (FSM) engine that orchestrates the entire Astro-Bio extraction sequence.

---

## Files

```
src/
├── main.cpp                ← Entry point: setup() + non-blocking loop()
└── StateManager/
    ├── StateManager.h      ← FSM class declaration
    └── StateManager.cpp    ← FSM state handlers and safety logic
```

---

## `main.cpp` — Entry Point

### Object Instantiation

All hardware objects are created as **static locals** at file scope — no heap allocation:

```cpp
static CanManager     canBus;
static SensorArray    sensors;
static PumpController pump;
static StateManager   fsm(canBus, sensors, pump);
```

The `StateManager` receives **references** to the three HAL objects. This is the only place where these objects are created — they are not copied or moved.

### `setup()` Flow

```
1. Serial.begin(115200)
2. delay(500)                     ← Only delay() in the entire codebase.
                                    Allows USB-CDC enumeration on ESP32-C3.
3. sensors.begin()                ← Configure GPIO, set ADC to 12-bit
4. pump.begin()                   ← Configure LEDC PWM channels, ensure pump is OFF
5. canBus.begin()                 ← Install TWAI driver, start CAN bus at 500 kbit/s
6. Print startup banner
```

### `loop()` — Non-Blocking Main Loop

```cpp
void loop() {
    canBus.process();       // 1. CAN RX/TX housekeeping
    sensors.update();       // 2. Periodic sensor sampling (pH every 200 ms)
    fsm.run();              // 3. FSM state logic + safety check
}
```

This loop runs **as fast as possible** — there are no delays, sleeps, or blocking calls. Typical iteration time is <1 ms, meaning the FSM responds to events within milliseconds.

### Call Sequence Per Loop Iteration

```
loop()
  │
  ├── canBus.process()
  │     ├── Drain all pending CAN RX frames
  │     ├── Set E-STOP flag if received
  │     ├── Store latest command in buffer
  │     ├── Check bus health timeout
  │     └── Send heartbeat if 1s elapsed
  │
  ├── sensors.update()
  │     └── If 200 ms elapsed since last sample:
  │           ├── Read pH ADC
  │           ├── Convert to pH value
  │           └── Store in ring buffer
  │
  └── fsm.run()
        ├── checkSafety()
        │     ├── E-STOP? → pump OFF → ERROR
        │     └── Bus dead + pump running? → pump OFF → ERROR
        │
        └── handleCurrentState()
              ├── IDLE: pop command → transition
              ├── POSITIONING: poll line sensor
              ├── EXTRACTION: poll water level
              ├── ACQUISITION: check pH variance
              └── ERROR: wait for CMD_RETURN_TO_IDLE
```

---

## `StateManager/` — Finite State Machine

### Class Overview

```cpp
class StateManager {
public:
    StateManager(CanManager& can, SensorArray& sensors, PumpController& pump);
    
    void run();                     // Main FSM tick — call every loop()
    SystemState getState() const;   // Read current state
    
private:
    void handleIdle();
    void handlePositioning();
    void handleExtraction();
    void handleAcquisition();
    void handleError();
    bool checkSafety();             // Global safety — runs BEFORE state logic
    void transitionTo(SystemState newState);  // Logged state transition
};
```

### Design Principles

1. **Safety first:** `checkSafety()` runs before any state handler on every tick. An E-STOP or bus timeout will pre-empt any state logic.

2. **Non-blocking:** No state handler blocks. Each one checks a condition, possibly triggers a transition, and returns. The next check happens on the next `loop()` iteration.

3. **Deterministic transitions:** Each state has a clearly defined set of exit conditions. There are no ambiguous or race-prone transitions.

4. **Abort from any state:** Every active state (POSITIONING, EXTRACTION, ACQUISITION) checks for `CMD_RETURN_TO_IDLE` at the start of its handler. This allows the Main Computer to abort any operation at any time.

### State Transition Table

| Current State | Trigger | Action | Next State |
|---------------|---------|--------|------------|
| **IDLE** | `CMD_START_POSITIONING` | Reset positioning phase | POSITIONING |
| **IDLE** | `CMD_START_EXTRACTION` | Start pump at default duty | EXTRACTION |
| **IDLE** | `CMD_START_ACQUISITION` | — | ACQUISITION |
| **POSITIONING** | Line sensor → HIGH (black) | Send `STATUS_BOTTOM_REACHED` | POSITIONING (phase 2) |
| **POSITIONING** | Line sensor → LOW (white) after bottom | Send `STATUS_OPTIMAL_POSITION` | IDLE |
| **POSITIONING** | `CMD_RETURN_TO_IDLE` | — | IDLE |
| **EXTRACTION** | Water level ≥ `WATER_LEVEL_TARGET` | Stop pump, send `STATUS_COLLECTION_DONE` | IDLE |
| **EXTRACTION** | `CMD_RETURN_TO_IDLE` | Stop pump | IDLE |
| **ACQUISITION** | pH variance < threshold | Send `STATUS_PH_READY` (4-byte float) | IDLE |
| **ACQUISITION** | `CMD_RETURN_TO_IDLE` | — | IDLE |
| **ERROR** | `CMD_RETURN_TO_IDLE` | — | IDLE |
| **Any** | `CMD_EMERGENCY_STOP` | Stop pump | ERROR |
| **Any** (pump running) | CAN bus timeout (3s) | Stop pump | ERROR |

### POSITIONING — Two-Phase Sub-State Machine

The POSITIONING state uses `PositioningPhase` to track its internal progress:

```
CMD_START_POSITIONING
        │
        ▼
 ┌─────────────────┐     Line sensor = HIGH     ┌─────────────────┐
 │ WAIT_FOR_BOTTOM  │ ──────────────────────────► │ WAIT_FOR_LIFTOFF │
 │                  │     Send BOTTOM_REACHED     │                  │
 │ (arm lowering)   │                             │ (arm raising)    │
 └─────────────────┘                             └────────┬─────────┘
                                                           │
                                               Line sensor = LOW
                                               Send OPTIMAL_POSITION
                                                           │
                                                           ▼
                                                       → IDLE
```

**Why two phases?** The gripper needs to:
1. Detect the bottom of the container (so the Main Computer knows to stop lowering)
2. Wait for the Main Computer to raise the arm to the optimal extraction height
3. Confirm the arm is raised (sensor returns to white) before declaring success

### EXTRACTION — Pump + Level Monitoring

```
CMD_START_EXTRACTION
        │
        ▼
 ┌──────────────────┐
 │   Pump ON (78%)  │
 │                  │
 │   Every tick:    │
 │   read water ADC │──── level ≥ 2800 ───► stopPump()
 │                  │                        Send COLLECTION_DONE
 │                  │                        → IDLE
 └──────────────────┘
```

### ACQUISITION — pH Stability Detection

```
CMD_START_ACQUISITION
        │
        ▼
 ┌──────────────────────────────────────┐
 │  pH sampled every 200 ms (by        │
 │  SensorArray::update() in loop)      │
 │                                      │
 │  Every tick: isPhStable()?           │
 │                                      │
 │  Need: 50 samples in buffer AND      │
 │        variance < 0.005              │──── YES ───► sendFloat(pH mean)
 │                                      │              → IDLE
 │  Minimum wait: 50 × 200ms = 10s     │
 └──────────────────────────────────────┘
```

### Safety Check Details

```cpp
bool StateManager::checkSafety() {
    // 1. E-STOP has absolute priority
    if (_can.hasEmergencyStop()) {
        _pump.stopPump();
        transitionTo(SystemState::ERROR);
        return true;
    }
    
    // 2. Bus timeout only matters if pump is running
    if (!_can.isBusHealthy() && _pump.isRunning()) {
        _pump.stopPump();
        transitionTo(SystemState::ERROR);
        return true;
    }
    
    return false;  // No safety event — proceed with state logic
}
```

### Serial Logging

All state transitions are logged with the `[FSM]` prefix:

```
[FSM] IDLE → POSITIONING
[FSM] Bottom detected → waiting for lift-off
[FSM] Lift-off confirmed → IDLE
[FSM] IDLE → EXTRACTION
[FSM] Water target reached (ADC=2847) → pump OFF → IDLE
[FSM] IDLE → ACQUISITION
[FSM] pH stable = 6.872 → transmitted → IDLE
[FSM] !!! E-STOP → ERROR
[FSM] !!! CAN bus timeout → pump halted → ERROR
[FSM] Error cleared → IDLE
```

---

## Adding a New State

1. **Define the state** in `include/StateDefinitions.h`:
   ```cpp
   enum class SystemState : uint8_t {
       // ... existing states ...
       YOUR_NEW_STATE,
   };
   ```

2. **Add handler** in `StateManager.h`:
   ```cpp
   private:
       void handleYourNewState();
   ```

3. **Implement handler** in `StateManager.cpp`:
   ```cpp
   void StateManager::handleYourNewState() {
       // Check for abort
       CanCommand cmd;
       if (_can.popCommand(cmd) && cmd == CanCommand::CMD_RETURN_TO_IDLE) {
           transitionTo(SystemState::IDLE);
           _can.sendStatus(CanStatus::STATUS_IDLE);
           return;
       }
       
       // Your logic here...
   }
   ```

4. **Add to dispatch** in `StateManager::run()`:
   ```cpp
   case SystemState::YOUR_NEW_STATE: handleYourNewState(); break;
   ```

5. **Add to logger** in `stateToStr()`:
   ```cpp
   case SystemState::YOUR_NEW_STATE: return "YOUR_NEW_STATE";
   ```

6. **Define transitions** from IDLE (or other states) that enter your new state.

7. **Add CAN IDs** in `CanDictionary.h` if the new state needs new command/status messages.

---

## Interaction with Main Computer

The FSM is designed as a **reactive slave**. It never initiates operations — it only responds to CAN commands from the Main Computer and reports results back.

### Expected Command Sequence

```
Main Computer                         ESP32 FSM
     │                                    │
     │   CMD_START_POSITIONING (0x110)    │
     │ ──────────────────────────────────►│ → POSITIONING
     │                                    │
     │   STATUS_BOTTOM_REACHED (0x210)    │
     │ ◄──────────────────────────────────│ ← stop lowering!
     │                                    │
     │   (raise arm by X mm)              │
     │                                    │
     │   STATUS_OPTIMAL_POSITION (0x211)  │
     │ ◄──────────────────────────────────│ → IDLE
     │                                    │
     │   CMD_START_EXTRACTION (0x120)     │
     │ ──────────────────────────────────►│ → EXTRACTION (pump ON)
     │                                    │
     │   STATUS_COLLECTION_DONE (0x220)   │
     │ ◄──────────────────────────────────│ → IDLE (pump OFF)
     │                                    │
     │   CMD_START_ACQUISITION (0x130)    │
     │ ──────────────────────────────────►│ → ACQUISITION
     │                                    │
     │   STATUS_PH_READY (0x230)          │
     │   [4 bytes: float pH value]        │
     │ ◄──────────────────────────────────│ → IDLE
     │                                    │
```

> **Important for Main Computer developers:** The ESP32 sends `STATUS_IDLE` after completing each operation. Wait for this frame before sending the next command. Sending a command while the ESP32 is in an active state (not IDLE) may result in the command being ignored or causing unexpected behaviour.
