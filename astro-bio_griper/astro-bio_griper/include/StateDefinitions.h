#pragma once

// =============================================================================
// StateDefinitions.h — Finite State Machine state enumerations
// =============================================================================

enum class SystemState : uint8_t {
    IDLE,           // Pump OFF, awaiting CAN commands
    POSITIONING,    // Polling line sensor for bottom detection & lift-off
    EXTRACTION,     // Pump ON, polling water level until target reached
    ACQUISITION,    // Polling pH sensor until variance stabilises
    ERROR,          // Fault condition — pump halted, awaiting recovery
};

// Sub-states for the POSITIONING sequence
enum class PositioningPhase : uint8_t {
    WAIT_FOR_BOTTOM,    // Lowering — waiting for line sensor to detect black
    WAIT_FOR_LIFTOFF,   // Bottom was detected — waiting for arm to pull up (white)
};
