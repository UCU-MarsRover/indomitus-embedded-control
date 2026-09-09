#pragma once

#include <Arduino.h>
#include "driver/twai.h"
#include "Config.h"
#include "CanDictionary.h"

// =============================================================================
// CanManager — Hardware Abstraction for ESP32 TWAI (CAN 2.0B) peripheral
// =============================================================================

class CanManager {
public:
    CanManager() = default;

    /// Initialise the TWAI peripheral. Call once in setup().
    /// @return true on successful bus start.
    bool begin();

    /// Non-blocking poll: reads all pending RX frames, handles heartbeat TX.
    /// Call every loop iteration.
    void process();

    // ----- Incoming command inspection -----------------------------------

    /// Returns true exactly once after an E-STOP frame was received.
    bool hasEmergencyStop();

    /// Returns the last received command (consumed on read).
    /// @param[out] cmd  The received command.
    /// @return true if a new command was available.
    bool popCommand(CanCommand& cmd);

    // ----- Outgoing status / data frames ---------------------------------

    /// Transmit a status frame (no payload).
    bool sendStatus(CanStatus status);

    /// Transmit a 4-byte float payload (pH value) under the given status ID.
    bool sendFloat(CanStatus status, float value);

    /// @return true if the bus is considered healthy (recent TX/RX success).
    bool isBusHealthy() const;

private:
    /// Send a raw TWAI frame.
    bool transmit(uint32_t id, const uint8_t* data, uint8_t len);

    // Bus health tracking
    unsigned long _lastRxTime     = 0;
    unsigned long _lastHeartbeat  = 0;
    bool          _busHealthy     = false;

    // Incoming command FIFO (single slot — CAN commands are sequenced)
    bool       _cmdPending    = false;
    CanCommand _pendingCmd    = CanCommand::CMD_RETURN_TO_IDLE;

    bool _estopFlag = false;
};
