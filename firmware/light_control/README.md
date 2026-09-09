# light_control

ESP32 firmware for rover light control.

This module follows the same architecture as `container_control`:

- `src/main.cpp` boots hardware and starts the CAN task
- `lib/can` owns CAN protocol parsing and responses
- `lib/light` owns actual light output control

## CAN interface

Jetson -> ESP32:

- ID `0x300`
- `0x01` spotlight ON
- `0x02` spotlight OFF
- `0x20` spotlight left ON
- `0x21` spotlight left OFF
- `0x22` spotlight right ON
- `0x23` spotlight right OFF
- `0x04` beautiful light ON
- `0x05` beautiful light OFF
- `0x24` beautiful 1 ON
- `0x25` beautiful 1 OFF
- `0x26` beautiful 2 ON
- `0x27` beautiful 2 OFF
- `0x28` beautiful 3 ON
- `0x29` beautiful 3 OFF
- `0x2A` beautiful 4 ON
- `0x2B` beautiful 4 OFF
- `0x06` red light ON
- `0x07` red light OFF
- `0x08` green light ON
- `0x09` green light OFF
- `0x0A` blue light ON
- `0x0B` blue light OFF
- `0x0C` buzzer ON
- `0x0D` buzzer OFF
- `0x2C` tower ON
- `0x2D` tower OFF
- `0x03` traffic light bitmask in `byte 1`
- `0x10` power telemetry ON (both sensors)
- `0x11` power telemetry OFF (both sensors)
- `0x12` power telemetry 1 ON
- `0x13` power telemetry 1 OFF
- `0x14` power telemetry 2 ON
- `0x15` power telemetry 2 OFF

ESP32 -> Jetson:

- ID `0x301`
- `byte 0 = echoed command`
- `byte 1 = 0x00 OK | 0x01 ERROR`

Power telemetry (ESP32 -> Jetson), one frame per enabled sensor every 200 ms:

- ID `0x302` sensor 1 (INA228 at I2C `0x45`)
- ID `0x303` sensor 2 (INA228 at I2C `0x44`)
- `bytes 0-3` current in A (little-endian `float`)
- `bytes 4-7` bus voltage in V (little-endian `float`)

Both sensors start disabled and are enabled independently.

Traffic-light bitmask:

- bit0 red
- bit1 yellow
- bit2 green
- bit3 blue
