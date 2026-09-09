#pragma once

#include <cstdint>

// =============================================================================
// CanDictionary.h — Standard 11-bit CAN ID definitions
// =============================================================================
//
// ID Allocation (11-bit, 0x000–0x7FF):
//   0x100–0x1FF : Commands  — Main Computer → Gripper
//   0x200–0x2FF : Statuses  — Gripper → Main Computer
//   0x300–0x3FF : Data      — Gripper → Main Computer (sensor payloads)
//
// =============================================================================

// -----------------------------------------------------------------------------
// Commands — received from Main Computer
// -----------------------------------------------------------------------------
enum class CanCommand : uint16_t {
    CMD_EMERGENCY_STOP      = 0x100,    // Immediate halt of all actuators
    CMD_START_POSITIONING   = 0x110,    // Begin bottom-detection sequence
    CMD_START_EXTRACTION    = 0x120,    // Begin pump / water collection
    CMD_START_ACQUISITION   = 0x130,    // Begin pH measurement
    CMD_RETURN_TO_IDLE      = 0x140,    // Abort current task, return to IDLE
};

// -----------------------------------------------------------------------------
// Statuses — transmitted to Main Computer
// -----------------------------------------------------------------------------
enum class CanStatus : uint16_t {
    STATUS_IDLE             = 0x200,    // System is idle and ready
    STATUS_BOTTOM_REACHED   = 0x210,    // Line sensor triggered — stop lowering
    STATUS_OPTIMAL_POSITION = 0x211,    // Arm pulled up — optimal height reached
    STATUS_COLLECTION_DONE  = 0x220,    // Water level target met, pump stopped
    STATUS_PH_READY         = 0x230,    // Stable pH value available (4-byte float payload)
    STATUS_ERROR            = 0x2F0,    // General error report
    STATUS_HEARTBEAT        = 0x2FF,    // Periodic heartbeat / alive signal
};
