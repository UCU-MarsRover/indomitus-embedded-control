#pragma once

// #include <Arduino.h>

extern "C" {
#include "driver/gpio.h"
#include "driver/twai.h"
}

#ifndef CAN_TX
#define CAN_TX GPIO_NUM_18
#endif

#ifndef CAN_RX
#define CAN_RX GPIO_NUM_17
#endif

/**
 * @brief Simple container for a received CAN frame.
 */
struct CanMsg {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
};

/**
 * @brief Initialize TWAI (CAN) in normal mode with 1 Mbit/s bitrate.
 *
 * Configuration:
 * - TX pin: CAN_TX
 * - RX pin: CAN_RX
 * - Filter: accept all frames
 */
inline void can_init_1mbs_accept_all() {
    twai_general_config_t g = {};
    g.mode = TWAI_MODE_NORMAL;
    g.tx_io = CAN_TX;
    g.rx_io = CAN_RX;
    g.clkout_io = GPIO_NUM_NC;
    g.bus_off_io = GPIO_NUM_NC;
    g.tx_queue_len = 10;
    g.rx_queue_len = 20;
    g.alerts_enabled = TWAI_ALERT_NONE;
    g.clkout_divider = 0;
    g.intr_flags = ESP_INTR_FLAG_LEVEL1;

    const twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
}

/**
 * @brief Send a standard CAN frame.
 *
 * @param id CAN identifier (11-bit for standard frame).
 * @param data Pointer to payload bytes.
 * @param len Payload length in bytes (clamped to 8).
 * @param timeout_ms Transmit timeout in milliseconds.
 * @return ESP_OK if message was transmitted successfully, otherwise an error code.
 */
inline esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len,
                     uint32_t timeout_ms = 200) {
    if (len > 8) {
        len = 8;
    }

    twai_message_t m = {};
    m.identifier = id;
    m.extd = 0;
    m.rtr = 0;
    m.data_length_code = len;

    for (uint8_t i = 0; i < len; ++i) {
        m.data[i] = data[i];
    }

    return twai_transmit(&m, pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief Receive one CAN frame.
 *
 * @param out Output message struct.
 * @param timeout_ms Receive timeout in milliseconds (0 means non-blocking).
 * @return ESP_OK if frame was received and copied to @p out, otherwise an error code.
 */
inline esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms = 0) {
    twai_message_t rx = {};
    const esp_err_t err = twai_receive(&rx, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        return err;
    }

    out.id = rx.identifier;
    out.len = rx.data_length_code;
    for (uint8_t i = 0; i < out.len; ++i) {
        out.data[i] = rx.data[i];
    }
    return ESP_OK;
}
