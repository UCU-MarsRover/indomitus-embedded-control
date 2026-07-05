# Indomitus Rover — ESP Firmware

Firmware for all ESP32/ESP32S3 microcontrollers on the Indomitus rover. Each subsystem's firmware lives in its own folder, built and flashed independently.

## Structure

```
.
├── <subsystem_1>/       # e.g. arm, drill, science-payload, power-board...
├── <subsystem_2>/
├── ...
└── README.md
```

Each folder is a self-contained firmware project (its own build config, dependencies, and README where needed) for one ESP on the rover. There is no shared build system across folders — treat each as independent.

## Adding a new subsystem

1. Create a new folder named after the subsystem.
2. Keep it self-contained (own `platformio.ini` or equivalent, own libraries).
3. Add a row to the subsystem table above.
4. If the board communicates with the main rover computer (ROS2 side), document the interface (serial protocol, topic names, message format) in that folder's own README.
