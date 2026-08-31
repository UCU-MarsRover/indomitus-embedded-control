#pragma once

extern "C" {
#include "driver/gpio.h"
#include "driver/twai.h"
}


/**
 * @brief Simple container for a received CAN frame.
 */
struct CanMsg {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
};

/**
 * @brief Initialize TWAI (CAN) in normal mode with 500 kbit/s bitrate.
 *
 * Configuration:
 * - TX pin: PIN_CAN_TX
 * - RX pin: PIN_CAN_RX
 * - Filter: accept all frames
 */
void can_init();

/**
 * @brief Send a standard CAN frame.
 *
 * @param id CAN identifier (11-bit for standard frame).
 * @param data Pointer to payload bytes.
 * @param len Payload length in bytes (clamped to 8).
 * @param timeout_ms Transmit timeout in milliseconds.
 * @return ESP_OK if message was transmitted successfully, otherwise an error code.
 */
esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len,
                     uint32_t timeout_ms = 200);

/**
 * @brief Receive one CAN frame.
 *
 * @param out Output message struct.
 * @param timeout_ms Receive timeout in milliseconds (0 means non-blocking).
 * @return ESP_OK if frame was received and copied to @p out, otherwise an error code.
 */
esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms = 200);
