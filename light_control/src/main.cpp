#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "light_manager.hpp"
#include "current_sensor.hpp"
#include "freertos/queue.h"
#include "pins.hpp"

extern QueueHandle_t can_tx_queue;

void can_full_reinit() {
    twai_stop();
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

    const twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t f = {
        .acceptance_code = 0x300 << 21,
        .acceptance_mask = ~(0x7F0 << 21),
        .single_filter = true,
    };

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
}

void setup() {
    Serial.begin(115200);

    // can_full_reinit();
    
    delay(2000);

    ESP_LOGI("SETUP", "Start Initialization!");

    light_init();
    can_init();
    // current_sensor_init();

    can_tx_queue = xQueueCreate(10, sizeof(CanTxMsg));

    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(light_task, "light_task", 4096, nullptr, 4, nullptr, 1);
    // xTaskCreatePinnedToCore(current_telemetry_task, "current_telemetry_task", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(can_monitor_task, "can_monitor_task", 4096, nullptr, 4, nullptr, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
