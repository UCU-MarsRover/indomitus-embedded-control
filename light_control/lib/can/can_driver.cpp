#include "can_driver.hpp"
#include "pins.hpp"

void can_init_1mbs_accept_all() {
    twai_general_config_t general_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)Pins::CAN_TX, (gpio_num_t)Pins::CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&general_config, &timing_config, &filter_config);
    twai_start();
}

esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = len;

    for (uint8_t i = 0; i < len && i < 8; ++i) {
        msg.data[i] = data[i];
    }

    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}

esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms) {
    twai_message_t msg = {};
    const esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        return err;
    }

    out.id = msg.identifier;
    out.len = msg.data_length_code;
    for (uint8_t i = 0; i < out.len && i < 8; ++i) {
        out.data[i] = msg.data[i];
    }
    return ESP_OK;
}
