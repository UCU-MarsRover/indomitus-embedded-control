#pragma once

#include <stdint.h>

extern "C" {
#include "driver/gpio.h"
#include "driver/twai.h"
}

/**
 * @brief CAN (TWAI) transport for the emergency board.
 *
 * Transport only --- frames in, frames out. Message IDs, payload meaning, and
 * what any command is allowed to do are application policy and live outside
 * this module.
 *
 * Pins come from Pins::CAN_TX / Pins::CAN_RX. Bitrate is 1 Mbit/s, matching the
 * other nodes in this repo. The acceptance filter is open: an emergency node
 * has spare cycles and benefits from seeing bus traffic, so filtering is left
 * to the caller.
 */

/// A received CAN frame.
struct CanMsg {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
};

/**
 * @brief Install and start the TWAI driver at 1 Mbit/s, accepting all frames.
 *
 * Safe to call more than once; any existing driver is uninstalled first.
 *
 * @note Call after PowerCut::init() and JetsonCanCut::init(). The cut lines
 *       must be parked before slower peripherals are brought up.
 */
void can_init();

/**
 * @brief Send a standard (11-bit) CAN frame.
 *
 * @param id         CAN identifier.
 * @param data       Payload bytes.
 * @param len        Payload length, clamped to 8.
 * @param timeout_ms Transmit timeout.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len,
                   uint32_t timeout_ms = 200);

/**
 * @brief Receive one CAN frame.
 *
 * @param out        Populated on success.
 * @param timeout_ms Receive timeout; 0 is non-blocking.
 * @return ESP_OK if a frame was received, otherwise an error code
 *         (ESP_ERR_TIMEOUT when nothing arrived).
 */
esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms = 200);

/**
 * @brief Recover the controller if it has dropped off the bus.
 *
 * Handles the bus-off and stopped states. A node whose only job is emergency
 * handling must not stay silent after a bus fault, so call this periodically
 * (every 200-500 ms) or whenever a send fails.
 *
 * @return true if recovery action was taken.
 */
bool can_recover_if_needed();

/// @return true if the controller is running and not bus-off.
bool can_is_healthy();

/// Raw controller status: error counters, queue depths, state.
esp_err_t can_status(twai_status_info_t& out);
