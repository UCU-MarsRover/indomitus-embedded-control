#include "can_driver.hpp"
#include "pins.hpp"

esp_err_t can_init_1mbs_accept_all() {
    twai_general_config_t general_config = {};
    general_config.mode = TWAI_MODE_NORMAL;
    general_config.tx_io = static_cast<gpio_num_t>(Pins::CAN_TX);
    general_config.rx_io = static_cast<gpio_num_t>(Pins::CAN_RX);
    general_config.clkout_io = GPIO_NUM_NC;
    general_config.bus_off_io = GPIO_NUM_NC;
    general_config.tx_queue_len = 10;
    general_config.rx_queue_len = 20;
    general_config.alerts_enabled =
        TWAI_ALERT_RX_DATA |
        TWAI_ALERT_TX_IDLE |
        TWAI_ALERT_TX_SUCCESS |
        TWAI_ALERT_TX_FAILED |
        TWAI_ALERT_RX_QUEUE_FULL |
        TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_ERR_PASS |
        TWAI_ALERT_BUS_OFF;
    general_config.clkout_divider = 0;
    general_config.intr_flags = ESP_INTR_FLAG_LEVEL1;

    const twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&general_config, &timing_config, &filter_config);
    if (err != ESP_OK) {
        return err;
    }

    return twai_start();
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
