#include <Arduino.h>

#include "Config.h"
#include "CanManager.h"
#include "SensorArray.h"
#include "PumpController.h"
#include "StateManager/StateManager.h"

// =============================================================================
// main.cpp — Astro-Bio Gripper Sub-Controller Entry Point
// =============================================================================
//
// Architecture:
//   loop() is strictly non-blocking. It calls three subsystems in sequence:
//     1. CanManager::process()   — drain CAN RX, send heartbeat
//     2. SensorArray::update()   — sample pH at 200 ms intervals
//     3. StateManager::run()     — FSM tick (reads sensors, sends CAN frames)
//
// =============================================================================

// ---- Hardware Abstraction Layer instances (static allocation) ---------------
static CanManager     canBus;
static SensorArray    sensors;
static PumpController pump;

// ---- Core Business Logic ---------------------------------------------------
static StateManager   fsm(canBus, sensors, pump);

// =============================================================================
// setup() — one-time initialisation
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);     // Allow USB-CDC enumeration on ESP32-C3

    Serial.println(F("========================================"));
    Serial.println(F("  Astro-Bio Gripper — ERC 2026"));
    Serial.println(F("  Indomitus Embedded Control"));
    Serial.println(F("========================================"));

    // Initialise subsystems
    sensors.begin();
    pump.begin();

    if (!canBus.begin()) {
        Serial.println(F("[INIT] CAN init failed — entering ERROR state"));
        // Pump is already off from begin(). FSM starts in IDLE, but we
        // won't receive any commands without CAN, so this is safe.
    }

    Serial.println(F("[INIT] System ready — entering IDLE"));
    Serial.println(F("========================================\n"));
}

// =============================================================================
// loop() — non-blocking main loop
// =============================================================================
void loop() {
    canBus.process();       // 1. CAN RX/TX housekeeping
    sensors.update();       // 2. Periodic sensor sampling
    fsm.run();              // 3. FSM state logic
}