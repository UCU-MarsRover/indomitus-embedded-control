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
| 3    | `ESTOP_BUTTON`   | in        | Active high, internal pulldown            |
| 4    | `JETSON_CAN_CUT` | out       | Gate. HIGH = Jetson isolated. **10k PD**  |
| 5    | `POWER_CUT`      | out       | Gate. HIGH = rover dead. **10k PD**       |
| 6    | `CAN_TX`         | out       | TWAI, to transceiver TXD                  |
| 7    | `CAN_RX`         | in        | TWAI, to transceiver RXD                  |

Free: 2, 8, 9 (strapping — avoid for anything critical), 10, 20/21 (debug UART).

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
