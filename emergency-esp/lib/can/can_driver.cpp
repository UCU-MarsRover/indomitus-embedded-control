#include "can_driver.hpp"

#include "pins.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "CAN";

void can_init() {
    twai_driver_uninstall();  // No-op if nothing is installed.

    twai_general_config_t g = {};
    g.mode            = TWAI_MODE_NORMAL;
    g.tx_io           = Pins::CAN_TX;
    g.rx_io           = Pins::CAN_RX;
    g.clkout_io       = GPIO_NUM_NC;
    g.bus_off_io      = GPIO_NUM_NC;
    g.tx_queue_len    = 10;
    g.rx_queue_len    = 20;
    g.alerts_enabled  = TWAI_ALERT_NONE;
    g.clkout_divider  = 0;
    g.intr_flags      = ESP_INTR_FLAG_LEVEL1;

    // 80 MHz APB / (brp 4 * (1 + tseg_1 14 + tseg_2 5)) = 1 Mbit/s.
    // Same timing as the other nodes in this repo.
    const twai_timing_config_t t = {
        .brp             = 4,
        .tseg_1          = 14,
        .tseg_2          = 5,
        .sjw             = 3,
        .triple_sampling = false,
    };

    // Accept everything; the application decides which IDs matter.
    const twai_filter_config_t f = {
        .acceptance_code = 0,
        .acceptance_mask = 0xFFFFFFFF,
        .single_filter   = true,
    };

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "TWAI up @1Mbit tx=GPIO%d rx=GPIO%d",
             (int)Pins::CAN_TX, (int)Pins::CAN_RX);
}

esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len,
                   uint32_t timeout_ms) {
    if (len > 8) {
        len = 8;
    }

    twai_message_t m = {};
    m.identifier      = id;
    m.extd            = 0;
    m.rtr             = 0;
    m.data_length_code = len;

    for (uint8_t i = 0; i < len; ++i) {
        m.data[i] = data[i];
    }

    return twai_transmit(&m, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms) {
    twai_message_t rx = {};
    const esp_err_t err = twai_receive(&rx, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        return err;
    }

    out.id  = rx.identifier;
    out.len = rx.data_length_code;
    for (uint8_t i = 0; i < out.len; ++i) {
        out.data[i] = rx.data[i];
    }
    return ESP_OK;
}

bool can_recover_if_needed() {
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) {
        return false;
    }

    if (s.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "bus-off, initiating recovery");
        twai_initiate_recovery();
        return true;
    }

    if (s.state == TWAI_STATE_STOPPED) {
        ESP_LOGW(TAG, "stopped, restarting");
        twai_start();
        return true;
    }

    return false;
}

bool can_is_healthy() {
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) {
        return false;
    }
    return s.state == TWAI_STATE_RUNNING;
}

esp_err_t can_status(twai_status_info_t& out) {
    return twai_get_status_info(&out);
}
