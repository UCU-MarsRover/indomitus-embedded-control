# CAN Helper Documentation

## Overview
`include/can_comm.hpp` contains shared TWAI/CAN utilities used by all nodes.
The project now uses one source file (`src/main.cpp`) and three PlatformIO environments (`esp1`, `esp2`, `esp3`) to build firmware for each board.

## PlatformIO Project
- Build all: `pio run`
- Build only one node: `pio run -e esp1` / `pio run -e esp2` / `pio run -e esp3`
- Upload one node: `pio run -e esp1 -t upload` (change env for needed board)
- Serial monitor: `pio device monitor -b 115200`

## Step-by-step run
1. Open terminal in project directory:
   - `cd .../indomitus-embedded-control/can-communication`
2. Check connected serial devices:
   - `pio device list`
3. Flash coordinator firmware to board 1:
   - `pio run -e esp1 -t upload --upload-port /dev/ttyUSB0`
4. Flash worker firmware to board 2:
   - `pio run -e esp2 -t upload --upload-port /dev/ttyUSB1`
5. Flash worker firmware to board 3:
   - `pio run -e esp3 -t upload --upload-port /dev/ttyUSB2`
6. Open serial monitor for any board:
   - `pio device monitor -b 115200 --port /dev/ttyUSB0`

Replace `/dev/ttyUSB0/1/2` with your real ports from `pio device list`.

## Node roles
- `esp1`: coordinator/sender (`NODE_ROLE=1`, `NODE_ID=1`)
- `esp2`: worker (`NODE_ROLE=2`, `NODE_ID=2`)
- `esp3`: worker (`NODE_ROLE=2`, `NODE_ID=3`)

Role and node ID are selected in `platformio.ini` via `build_flags`.

## CAN IDs and payloads
- Request frame ID: `0x120`
- Response frame ID: `0x121`

Request payload (`0x120`, 4 bytes):
- `data[0]` = destination node (`2` or `3`)
- `data[1]` = `a`
- `data[2]` = `b`
- `data[3]` = `seq` (sequence number)

Response payload (`0x121`, 3 bytes):
- `data[0]` = source node (`2` or `3`)
- `data[1]` = `sum = a + b`
- `data[2]` = `seq` (copied from request)

## Runtime flow
1. `esp1` increments `seq` and sends requests to `esp2` and `esp3`.
2. Worker nodes process only frames with matching destination ID.
3. Workers reply with `sum` and same `seq`.
4. `esp1` waits up to 500 ms and prints per-node results.

## Helper functions 
- `can_send(id, data, len, timeout_ms)`:
  - Sends one standard CAN frame.
  - `id`: CAN ID, `data`: pointer to payload bytes, `len`: payload length (`0..8`), `timeout_ms`: TX timeout.
  - Returns `true` when frame was transmitted successfully.
- `can_recv(out, timeout_ms)`:
  - Waits for one CAN frame and writes it to `out` (`id`, `len`, `data[]`).
  - `timeout_ms = 0` means non-blocking poll.
  - Returns `true` when a frame was received before timeout.

### Minimal usage
```cpp
uint8_t tx[2] = {10, 7};
bool tx_ok = can_send(0x120, tx, 2, 200);

CanMsg rx;
bool has_msg = can_recv(rx, 20);
if (has_msg) {
  // use rx.id, rx.len, rx.data[]
}
```
