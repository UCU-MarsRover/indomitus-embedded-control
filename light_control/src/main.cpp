#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "light_manager.hpp"
#include "current_sensor.hpp"

void setup() {
    Serial.begin(115200);
    
    delay(1000);

    ESP_LOGI("SETUP", "Start Initialization!");

    light_init();
    can_init_1mbs_accept_all();

    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(light_task, "light_task", 2048, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(current_telemetry_task, "current_telemetry_task", 2048, nullptr, 4, nullptr, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
