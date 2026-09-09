# Indomitus Rover — ESP Firmware

Firmware for all ESP32/ESP32S3 microcontrollers on the Indomitus rover. Each subsystem's firmware lives in its own folder, built and flashed independently.

## Structure

```
.
├── firmware/             # one folder per subsystem microcontroller
│   ├── <subsystem_1>/    # e.g. claw_drill, container_control, light_control...
│   ├── <subsystem_2>/
│   └── ...
├── shared/
│   └── libs/             # libraries shared across firmware projects
├── docs/                 # cross-project documentation
└── README.md
```

Each folder under `firmware/` is a self-contained PlatformIO project (its own `platformio.ini`, `src/`, `include/`, `lib/`, and README where needed) for one microcontroller (ESP32/ESP32S3, STM32, ...) on the rover. There is no shared build system across firmware folders — treat each as independent, aside from anything pulled in from `shared/libs`.
