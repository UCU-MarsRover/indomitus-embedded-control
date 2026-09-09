# emergency-esp

Emergency (E-stop) node for the rover. ESP32-C3-MINI-1.

Five subsystems, one `lib/` module each with its own API. `src/main.cpp` only
does bring-up ordering and periodic integrity checks — the emergency policy
itself is application code.

## Pin map

| GPIO | Signal           | Direction | Notes                                     |
|------|------------------|-----------|-------------------------------------------|
| 0    | `RADIO_RX`       | in        | UART1, to radio TX                        |
| 1    | `RADIO_TX`       | out       | UART1, to radio RX                        |
| 2    | `JETSON_RESET`   | out       | Pulse HIGH 1s. **No external pulldown**   |
| 3    | `POWER_ON`       | out       | Pulse HIGH 1s                             |
| 4    | `POWER_OFF`      | out       | Idles HIGH, pulses LOW 1s                 |
| 5    | `ESTOP_BUTTON`   | in        | Active high, internal pulldown            |
| 6    | `RADIO_M0`       | out       | Radio mode select. **10k PD**             |
| 7    | `RADIO_M1`       | out       | Radio mode select. **10k PD**             |
| 10   | `CAN_RX`         | in        | TWAI, to transceiver RXD                  |
| 20   | `CAN_TX`         | out       | TWAI, to transceiver TXD. **10k PU to 3V3** |

Free: 8, 9 (strapping — avoid for anything critical), 21 (debug TX).

Three controlled outputs, one job each: cut rover power (5), cut the Jetson off
CAN (4), reset the Jetson (2).

`CAN_TX` is on GPIO20 (U0RXD), an input at reset whose internal pullup holds the
transceiver recessive through boot. GPIO21 is avoided for CAN: it is U0TXD and
emits the bootloader log at every reset, which a transceiver would inject onto
the bus as error frames.

Why these: GPIO 11–17 are the internal SPI flash; 2/8/9 are strapping pins
sampled at reset with boot-time pulls; 18/19 are USB D-/D+. The two gate lines
sit on GPIO 4/5 because those are RTC-capable pads and so can be latched with
`gpio_hold_en()`. The radio avoids the default UART0 pads (20/21) because
**GPIO21 emits the ROM bootloader log at every reset** — that garbage would go
straight into the radio. Full reasoning in [pins.hpp](include/pins.hpp).

## Required external hardware

**Each gate line needs a 10 kΩ pulldown to GND, at the transistor gate.**

This is not optional and firmware cannot substitute for it. From power-on until
`init()` runs (~20–50 ms), and again during every reset, brownout dip, and
watchdog reboot, every ESP32-C3 GPIO is a high-impedance input. A floating gate
in that window is undefined — which for `POWER_CUT` means an unintended rover
shutdown. The resistor is the only thing covering it.

Recommended alongside it: a 100 nF gate-to-GND cap to swallow injected noise,
and the ESP's brownout detector left enabled so a sagging rail resets the chip
into that pulled-down state rather than executing on marginal logic.

### Jetson reset is wired differently

`JETSON_RESET` is the odd one out — **active low, with a 10k pull*up*.** Two
constraints force it:

- `SYS_RESET*` on the reComputer's REC switch header is a **1.8 V open-drain**
  input, asserted by pulling to GND. Driving 3V3 into it can damage the Orin.
- GPIO2 is an ESP32-C3 **strapping pin that must read HIGH at reset**. The 10k
  pulldown a directly-driven gate needs would stop the ESP booting.

An optocoupler the ESP *sinks* solves both, and isolates the two boards:

```
3V3 ──[330R]──▶|── GPIO2        (opto LED, cathode to ESP)
GPIO2 ──[10k]── 3V3              (holds the boot strap high)
opto collector → header pin 8 (SYS_RESET*)
opto emitter   → header pin 7  (header GND)
```

GPIO2 high = idle. GPIO2 low = Jetson resets. Note the failure mode: a fault
holding GPIO2 low across an ESP reset boots the ESP into serial-download mode.
Recoverable, but it's the price of using the last strapping pin.

The REC switch header also exposes `PWR_BTN*` (pin 12) for an orderly shutdown
via a >10 s hold. Not wired — one pin, one job — but it's there if you later
want the Jetson to unmount its SSD before `PowerCut::engage()`.

### Fail-safe direction

Worth being explicit about: because HIGH means "cut", a dead or unpowered
emergency ESP leaves the rover **powered and running**. That is a consequence of
the transistor topology, and it is the opposite of the usual e-stop convention
(where loss of the safety signal stops the machine). It does buy immunity from
nuisance trips. If you ever want loss-of-ESP to mean stop, that requires
inverting the driver in hardware, not a firmware change.

## Modules

| Module | API | Purpose |
|---|---|---|
| [safety_output](lib/safety_output/safety_output.hpp) | `SafetyOutput` | Glitch-free critical output primitive; used by the two below |
| [power_cut](lib/power_cut/power_cut.hpp) | `PowerCut::` | `engage()` / `release()` / `is_engaged()` / `verify()` |
| [jetson_can_cut](lib/jetson_can_cut/jetson_can_cut.hpp) | `JetsonCanCut::` | `isolate()` / `connect()` / `is_isolated()` / `verify()` |
| [jetson_reset](lib/jetson_reset/jetson_reset.hpp) | `JetsonReset::` | `reset()` / `update()` / `is_busy()` — non-blocking 100 ms pulse |
| [estop_button](lib/estop_button/estop_button.hpp) | `EstopButton::` | Debounced `poll()` returning press/release edges |
| [can](lib/can/can_driver.hpp) | `can_init/send/recv` | 1 Mbit TWAI, same API as `light_control` |
| [can](lib/can/can_service.hpp) | `CanService::` | Optional FreeRTOS RX/TX tasks with a frame callback |
| [radio](lib/radio/radio_link.hpp) | `RadioLink::` | CRC-16 framed UART link to the radio adapter |

### How `SafetyOutput` protects the gate lines

Both cut lines are built on it, and it does four things:

- **Glitch-free bring-up** — writes the output latch to 0 *before* enabling the
  pad driver, so switching the driver on can only ever drive 0. It deliberately
  avoids `gpio_reset_pin()`, which resets pads with the internal *pullup*
  enabled.
- **Internal pulldown stays on** alongside the push-pull driver, biasing the pad
  low whenever the driver isn't holding it.
- **Redundant state** — the intended level is stored as a value plus its
  complement; a single-bit upset is detected and resolved by forcing LOW.
- **Periodic repair** — `verify()` rewrites the register unconditionally every
  50 ms, so a corrupted GPIO register or a peripheral that stole the pad is
  fixed within one period. Same-value writes produce no edge.

Never attach LEDC/PWM, ADC, or any other peripheral to GPIO 4 or 5.

## Protocol design note

When you implement the CAN and radio protocols, keep the two directions
asymmetric: make **engaging** a cut cheap (a single command byte) and
**releasing** one expensive (require a magic key, e.g. `0xA5 0x5A`). A corrupted
frame should never be able to un-cut the rover. The radio parser already drops
anything failing CRC, and `RadioLink::ms_since_last_frame()` is there if you want
a link-loss policy — decide that one deliberately, since "radio dropout kills the
rover" is a real tradeoff either way.

An e-stop should also **latch**: releasing the button must not by itself restore
power.

## Build

```sh
pio run -e debug      # verbose logging
pio run -e release    # logging off
pio run -e debug -t upload
```

### CAN bring-up test

`src/main_test_can.cpp` starts the CanService tasks, sends a frame on **0x303**
every second, and logs all bus traffic plus controller counters every 5 s.

```sh
pio run -e test_can -t upload -t monitor
```

Healthy output is `state=1` with TEC/REC pinned at 0. If TEC climbs toward 128
and the node goes bus-off, nothing is ACKing — a CAN node cannot ACK its own
frame, so at least one other active node must be on the bus and both ends need
120 Ω termination.
