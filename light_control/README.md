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
- `0x04` beautiful light ON
- `0x05` beautiful light OFF
- `0x03` traffic light bitmask in `byte 1`

ESP32 -> Jetson:

- ID `0x301`
- `byte 0 = echoed command`
- `byte 1 = 0x00 OK | 0x01 ERROR`

Traffic-light bitmask:

- bit0 red
- bit1 yellow
- bit2 green
- bit3 blue
