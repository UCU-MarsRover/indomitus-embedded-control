#include "CanManager.h"

// =============================================================================
// CanManager.cpp — ESP32 TWAI peripheral driver
// =============================================================================

bool CanManager::begin() {
    // General TWAI configuration — 500 kbit/s, normal mode
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(PIN_CAN_TX),
        static_cast<gpio_num_t>(PIN_CAN_RX),
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = CAN_RX_QUEUE_LEN;

    // Timing configuration — 500 kbit/s
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Accept all frames (no hardware filter — we filter in software)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install and start the TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println(F("[CAN] Driver install FAILED"));
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println(F("[CAN] Bus start FAILED"));
        return false;
    }

    _busHealthy    = true;
    _lastRxTime    = millis();
    _lastHeartbeat = millis();

    Serial.println(F("[CAN] Bus initialised — 500 kbit/s"));
    return true;
}

// -----------------------------------------------------------------------------
// Main loop poll
// -----------------------------------------------------------------------------
void CanManager::process() {
    unsigned long now = millis();

    // ---- Drain RX queue ------------------------------------------------
    twai_message_t rxMsg;
    while (twai_receive(&rxMsg, 0) == ESP_OK) {    // pdMS_TO_TICKS(0) = non-blocking
        _lastRxTime = now;
        _busHealthy = true;

        uint16_t id = static_cast<uint16_t>(rxMsg.identifier);

        // Emergency stop has absolute priority
        if (id == static_cast<uint16_t>(CanCommand::CMD_EMERGENCY_STOP)) {
            _estopFlag = true;
            Serial.println(F("[CAN] << E-STOP received"));
            continue;
        }

        // Store the most recent command (overwrite — latest wins)
        switch (static_cast<CanCommand>(id)) {
            case CanCommand::CMD_START_POSITIONING:
            case CanCommand::CMD_START_EXTRACTION:
            case CanCommand::CMD_START_ACQUISITION:
            case CanCommand::CMD_RETURN_TO_IDLE:
                _pendingCmd = static_cast<CanCommand>(id);
                _cmdPending = true;
                Serial.printf("[CAN] << CMD 0x%03X\n", id);
                break;
            default:
                // Unknown ID — ignore silently
                break;
        }
    }

    // ---- Bus health timeout --------------------------------------------
    if ((now - _lastRxTime) > CAN_TIMEOUT_MS) {
        _busHealthy = false;
    }

    // ---- Periodic heartbeat TX -----------------------------------------
    if ((now - _lastHeartbeat) >= CAN_HEARTBEAT_MS) {
        _lastHeartbeat = now;
        sendStatus(CanStatus::STATUS_HEARTBEAT);
    }
}

// -----------------------------------------------------------------------------
// Incoming helpers
// -----------------------------------------------------------------------------

bool CanManager::hasEmergencyStop() {
    if (_estopFlag) {
        _estopFlag = false;
        return true;
    }
    return false;
}

bool CanManager::popCommand(CanCommand& cmd) {
    if (_cmdPending) {
        cmd = _pendingCmd;
        _cmdPending = false;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// Outgoing frames
// -----------------------------------------------------------------------------

bool CanManager::sendStatus(CanStatus status) {
    uint32_t id = static_cast<uint32_t>(status);
    Serial.printf("[CAN] >> STATUS 0x%03X\n", id);
    return transmit(id, nullptr, 0);
}

bool CanManager::sendFloat(CanStatus status, float value) {
    uint32_t id = static_cast<uint32_t>(status);
    uint8_t payload[4];
    memcpy(payload, &value, sizeof(float));

    Serial.printf("[CAN] >> DATA 0x%03X  pH=%.3f\n", id, value);
    return transmit(id, payload, 4);
}

bool CanManager::isBusHealthy() const {
    return _busHealthy;
}

// -----------------------------------------------------------------------------
// Raw transmit
// -----------------------------------------------------------------------------

bool CanManager::transmit(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier      = id;
    msg.data_length_code = len;
    msg.flags            = 0;   // Standard frame, no RTR

    if (data && len > 0) {
        memcpy(msg.data, data, len);
    }

    esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (result != ESP_OK) {
        Serial.printf("[CAN] TX FAIL id=0x%03X err=%d\n", id, result);
        return false;
    }
    return true;
}
