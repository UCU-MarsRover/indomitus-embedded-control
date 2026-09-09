<h1 align="center">Indomitus — Mars Rover's Microcontrollers</h1>

<p align="center">
  Firmware for all microcontrollers on the Indomitus rover. Each subsystem's firmware lives in its own folder, built and flashed independently.
</p>

<p align="center">
  <a href="https://platformio.org/"><img alt="PlatformIO" src="https://img.shields.io/badge/build-PlatformIO-FF7F00?logo=platformio&logoColor=white"></a>
  <img alt="Platform" src="https://img.shields.io/badge/platform-ESP32%20%7C%20ESP32S3%20%7C%20ESP32C3-informational">
  <a href="LICENSE"><img alt="MIT License" src="https://img.shields.io/badge/license-MIT-blue"></a>
</p>

<p align="center">
  <a href="https://github.com/UCU-MarsRover/indomitus-embedded-control/actions/workflows/release.yaml?query=branch%3Amain"><img alt="main" src="https://img.shields.io/github/actions/workflow/status/UCU-MarsRover/indomitus-embedded-control/release.yaml?branch=main&label=main&logo=github"></a>
  <a href="https://github.com/UCU-MarsRover/indomitus-embedded-control/actions/workflows/release.yaml?query=branch%3Adevelop"><img alt="develop" src="https://img.shields.io/github/actions/workflow/status/UCU-MarsRover/indomitus-embedded-control/release.yaml?branch=develop&label=develop&logo=github"></a>
  <a href="https://github.com/UCU-MarsRover/indomitus-embedded-control/releases/latest"><img alt="main release" src="https://img.shields.io/github/v/release/UCU-MarsRover/indomitus-embedded-control?label=main&color=green"></a>
  <a href="https://github.com/UCU-MarsRover/indomitus-embedded-control/releases"><img alt="develop release" src="https://img.shields.io/github/v/release/UCU-MarsRover/indomitus-embedded-control?include_prereleases&label=develop"></a>
</p>

---

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
