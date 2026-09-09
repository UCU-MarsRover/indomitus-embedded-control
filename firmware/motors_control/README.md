# indomitus-hw-control

Embedded motor control firmware for the Indomitus Mars Rover — ESP32-S3, Damiao CAN brushless motors, 4-wheel swerve drive.

---

## Hardware

| Item | Detail |
|---|---|
| MCU | ESP32-S3 (esp32s3box board) |
| CAN transceiver | Any 3.3 V module (TJA1050, SN65HVD230, etc.) |
| CAN TX pin | GPIO 5 |
| CAN RX pin | GPIO 4 |
| CAN baud rate | 1 Mbps |
| Motor models | DM-J4340-2EC (40:1), DM-J10010L-2EC (10:1) |
| Bus topology | Up to 4 motors daisy-chained on one CAN bus |
| Termination | 120 Ω at each end — use DIP switch 4 on the motor for the built-in resistor |

---

## Project structure

```
include/
  ICanBus.h          CAN transport abstraction (swap the backend here)
  TwaiCanBus.h       ESP32 TWAI implementation of ICanBus
  MotorBase.h        Abstract motor interface + shared types
  DamiaoMotor.h      Damiao driver: config structs, class declaration
src/
  TwaiCanBus.cpp     Only file that includes ESP32-TWAI-CAN.hpp
  DamiaoMotor.cpp    Frame encoding/decoding, register access
  main.cpp           4-motor wired-up example
docs/
  can_bringup_task.md    Protocol notes and bring-up tasks
  CAN_motor_bringup.txt  Reference UART command sketch
  *.pdf                  Motor datasheets
```

---

## Dependencies

Managed by PlatformIO — no manual install needed.

```ini
lib_deps =
    handmade0octopus/ESP32-TWAI-CAN
```

Build with:
```bash
pio run
```

Flash with:
```bash
pio run --target upload
```

Monitor serial output (115200 baud):
```bash
pio device monitor
```

---

## Motor API

### Setup (main.cpp)

```cpp
#include "TwaiCanBus.h"
#include "DamiaoMotor.h"

// Bus must be declared before motors (motors hold a reference to it)
TwaiCanBus bus(5, 4);   // TX=GPIO5, RX=GPIO4

DamiaoMotor motors[4] = {
    DamiaoMotor(1, 0x000, bus, DamiaoConfigs::J4340()),
    DamiaoMotor(2, 0x000, bus, DamiaoConfigs::J4340()),
    DamiaoMotor(3, 0x000, bus, DamiaoConfigs::J4340()),
    DamiaoMotor(4, 0x000, bus, DamiaoConfigs::J4340()),
};

void setup() {
    bus.begin();
    for (auto& m : motors) {
        m.setMode(MotorMode::Velocity);
        delay(5);
        m.enable();
    }
}
```

### Feedback dispatch (loop)

All motors share the same feedback CAN ID (`MST_ID = 0x000`). The motor identity is encoded in byte `D[0] & 0x0F`. Call `acceptFeedback` on every motor for every received frame — each motor filters for its own ID.

```cpp
void loop() {
    CanMsg rx;
    while (bus.recv(rx, 0)) {
        if (rx.id == 0x000 && rx.len >= 8)
            for (auto& m : motors) m.acceptFeedback(rx.data, rx.len);
    }
}
```

### Control

```cpp
// Switch mode
motor.setMode(MotorMode::Velocity);
motor.setMode(MotorMode::PositionVelocity);
motor.setMode(MotorMode::MIT);

// Velocity mode — rad/s on motor shaft
motor.sendVelocity(5.0f);

// Position-Velocity mode — position in rad, velocity cap in rad/s
motor.sendPositionVelocity(3.14f, 10.0f);

// MIT impedance mode — full parameter set
// τ = kp*(p_des - p) + kd*(v_des - v) + tff
// kd must be > 0 when kp > 0, otherwise the motor will oscillate
motor.sendMIT(/*p=*/0.0f, /*v=*/0.0f, /*kp=*/10.0f, /*kd=*/0.5f, /*tff=*/0.0f);

// Lifecycle
motor.enable();
motor.disable();
motor.setZero();   // set current position as software zero
```

### Feedback

```cpp
if (motor.hasFeedback()) {
    float pos = motor.getPosition();   // rad  (motor shaft)
    float vel = motor.getVelocity();   // rad/s
    float tor = motor.getTorque();     // Nm

    uint8_t tMos    = motor.getMosTemp();    // °C — driver FETs
    uint8_t tRotor  = motor.getRotorTemp();  // °C — motor coil

    bool    ok  = motor.isEnabled();
    MotorError e = motor.getError();
}
```

**Convert motor shaft → output shaft:**
```
output_pos = motor.getPosition() / gearRatio   // J4340: /40, J10010L: /10
output_vel = motor.getVelocity() / gearRatio
output_tor = motor.getTorque()   * gearRatio
```

### ID management

```cpp
// Change the motor CAN receive ID (register 8).
// persist=true (default) calls storeToFlash() so it survives reboot.
motor.setEscId(2);
motor.setEscId(2, false);   // change now, do NOT persist

// Change the feedback frame CAN ID (register 7).
// Update MST_ID in main.cpp to match after calling this.
motor.setMstId(0x000);
```

### Register access

```cpp
// Read a register — response arrives as a CAN frame at MST_ID
motor.readRegister(23);    // reg 23 = tMax

// Write a register (takes effect immediately, lost on reboot unless stored)
motor.writeRegister(10, 3);   // reg 10 = CTRL_MODE, 3 = Velocity

// Save ALL current register values to on-board flash
motor.storeToFlash();
```

**Useful register addresses:**

| Reg | Name | Description | R/W |
|---|---|---|---|
| 7 | MST_ID | Feedback frame CAN ID | RW |
| 8 | ESC_ID | Motor CAN receive ID | RW |
| 10 | CTRL_MODE | 1=MIT 2=Pos-Vel 3=Velocity | RW |
| 21 | PMAX | Position encoding range (rad) | RW |
| 22 | VMAX | Velocity encoding range (rad/s) | RW |
| 23 | TMAX | Torque encoding range (Nm) | RW |
| 31 | Deta | Speed loop damping factor (2–10, rec. 4) | RW |
| 80 | p_m | Live motor position | RO |
| 81 | xout | Live output shaft position | RO |

---

## CAN frame reference

### Feedback frame (motor → ESP32)

CAN ID = `MST_ID` (default `0x000`), DLC = 8.

| Byte | Content |
|---|---|
| D[0] | `(ERR << 4) \| ESC_ID` |
| D[1..2] | Position, 16-bit signed fixed-point |
| D[3], D[4] high nibble | Velocity, 12-bit |
| D[4] low nibble, D[5] | Torque, 12-bit |
| D[6] | MOSFET temperature (°C) |
| D[7] | Rotor temperature (°C) |

**ERR codes:**

| Value | Meaning |
|---|---|
| 0x0 | Disabled |
| 0x1 | Enabled (normal) |
| 0x8 | Overvoltage |
| 0x9 | Undervoltage |
| 0xA | Overcurrent |
| 0xB | MOS overtemperature |
| 0xC | Coil overtemperature |
| 0xD | Communication lost |
| 0xE | Overload |

### Control frames (ESP32 → motor)

| Mode | CAN ID | Payload |
|---|---|---|
| MIT | `ESC_ID` | 8 bytes packed (p=16bit, v/kp/kd/tff=12bit each) |
| Position-Velocity | `0x100 + ESC_ID` | 4B float p + 4B float v |
| Velocity | `0x200 + ESC_ID` | 4B float v |
| Enable | `ESC_ID` | `FF FF FF FF FF FF FF FC` |
| Disable | `ESC_ID` | `FF FF FF FF FF FF FF FD` |
| Set zero | `ESC_ID` | `FF FF FF FF FF FF FF FE` |
| Register read | `0x7FF` | `ID_L ID_H 33 RID 00 00 00 00` |
| Register write | `0x7FF` | `ID_L ID_H 55 RID VAL(LE 4B)` |
| Store to flash | `0x7FF` | `ID_L ID_H AA 01 00 00 00 00` |

---

## Motor configs

### DM-J4340-2EC (24 V)

| Parameter | Value |
|---|---|
| Rated voltage | 24 V |
| Rated torque | 9 Nm |
| Peak torque | 27 Nm |
| Rated speed (output) | 36 rpm |
| Gear ratio | 40:1 |
| CAN baud rate | 1 Mbps |
| `tMax` in firmware | **verify via register 23** — set to 10.0 Nm as placeholder |

### DM-J10010L-2EC (24–48 V)

| Parameter | Value |
|---|---|
| Rated voltage | 48 V (supports 24–48 V) |
| Rated torque | 40 Nm |
| Peak torque | 120 Nm |
| Rated speed (output) | 100 rpm @ 48 V |
| Gear ratio | 10:1 |
| CAN baud rate | 1 Mbps |
| `tMax` in firmware | 20.0 Nm |

---

## Protection thresholds (defaults, configurable via registers)

| Protection | Threshold | Action |
|---|---|---|
| Driver overtemperature | 120 °C | Exit enable mode |
| Motor overtemperature | 100 °C (recommended) | Exit enable mode |
| Overvoltage (24 V model) | 32 V | Exit enable mode |
| Overvoltage (48 V model) | 52 V | Exit enable mode |
| Undervoltage | 15 V | Exit enable mode |
| Overcurrent | 9.8 A | Exit enable mode |
| Communication loss | Configurable timeout | Exit enable mode |

---

## Swapping the CAN backend

`TwaiCanBus` is the only concrete implementation of `ICanBus`. To use a different CAN library or hardware:

1. Create `include/MyCanBus.h` and `src/MyCanBus.cpp` implementing `ICanBus`:
   ```cpp
   class MyCanBus : public ICanBus {
   public:
       bool send(const CanMsg& msg) override { /* ... */ }
       bool recv(CanMsg& msg, uint32_t timeout_ms) override { /* ... */ }
   };
   ```
2. Replace `TwaiCanBus bus(...)` with `MyCanBus bus(...)` in `main.cpp`.
3. No changes needed to `MotorBase.h`, `DamiaoMotor.h`, or `DamiaoMotor.cpp`.

---

## Adding a new motor type

Subclass `MotorBase` and implement its pure virtual methods:

```cpp
class MyMotor : public MotorBase {
public:
    MyMotor(uint8_t id, ICanBus& bus) : _id(id), _bus(bus) {}

    uint8_t escId() const override { return _id; }
    bool enable()  override { /* send your enable frame */ }
    // ... implement remaining virtuals
    bool acceptFeedback(const uint8_t data[8], uint8_t dlc) override { /* decode */ }
    const MotorFeedback& feedback() const override { return _fb; }

private:
    uint8_t _id;
    ICanBus& _bus;
    MotorFeedback _fb;
};
```

The convenience getters (`getPosition()`, `getMosTemp()`, `isEnabled()`, etc.) are inherited from `MotorBase` for free.
