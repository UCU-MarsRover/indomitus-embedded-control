# Indomitus Rover — ESP Firmware

[![PlatformIO](https://img.shields.io/badge/build-PlatformIO-FF7F00?logo=platformio&logoColor=white)](https://platformio.org/)
![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20ESP32S3%20%7C%20ESP32C3-informational)
[![MIT License](https://img.shields.io/badge/license-MIT-blue)](LICENSE)

[![main](https://img.shields.io/github/actions/workflow/status/UCU-MarsRover/indomitus-embedded-control/release.yaml?branch=main&label=main&logo=github)](https://github.com/UCU-MarsRover/indomitus-embedded-control/actions/workflows/release.yaml?query=branch%3Amain)
[![develop](https://img.shields.io/github/actions/workflow/status/UCU-MarsRover/indomitus-embedded-control/release.yaml?branch=develop&label=develop&logo=github)](https://github.com/UCU-MarsRover/indomitus-embedded-control/actions/workflows/release.yaml?query=branch%3Adevelop)
[![main release](https://img.shields.io/github/v/release/UCU-MarsRover/indomitus-embedded-control?label=main&color=green)](https://github.com/UCU-MarsRover/indomitus-embedded-control/releases/latest)
[![develop release](https://img.shields.io/github/v/release/UCU-MarsRover/indomitus-embedded-control?include_prereleases&label=develop)](https://github.com/UCU-MarsRover/indomitus-embedded-control/releases)

Firmware for all microcontrollers on the Indomitus rover. Each subsystem's firmware lives in its own folder, built and flashed independently.

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

Each folder under `firmware/` is a self-contained PlatformIO project (its own `platformio.ini`, `src/`, `include/`, `lib/`, and README where needed) for one microcontroller (ESP32, ESP32S3, ESP32C3, ...) on the rover. There is no shared build system across firmware folders — treat each as independent, aside from anything pulled in from `shared/libs`.
