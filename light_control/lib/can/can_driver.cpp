#include "can_driver.hpp"
#include "pins.hpp"

void can_init() {
    twai_driver_uninstall();
    
    twai_general_config_t g = {};
    g.mode = TWAI_MODE_NORMAL;
    g.tx_io = Pins::CAN_TX;
    g.rx_io = Pins::CAN_RX;
    g.clkout_io = GPIO_NUM_NC;
    g.bus_off_io = GPIO_NUM_NC;
    g.tx_queue_len = 10;
    g.rx_queue_len = 20;
    g.alerts_enabled = TWAI_ALERT_NONE;
    g.clkout_divider = 0;
    g.intr_flags = ESP_INTR_FLAG_LEVEL1;

    const twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f = {
        .acceptance_code = 0x300 << 21,
        .acceptance_mask = ~(0x7F0 << 21),
        .single_filter = true,
    };

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
}


esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len,
                     uint32_t timeout_ms) {
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


esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms) {
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
