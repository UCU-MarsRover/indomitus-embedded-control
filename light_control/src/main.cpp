#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "light_manager.hpp"
#include "current_sensor.hpp"
#include "freertos/queue.h"
#include "pins.hpp"


extern QueueHandle_t can_tx_queue;


void setup() {
    Serial.begin(115200);
    
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

#ifdef DEBUG_ENABLED
    xTaskCreatePinnedToCore(can_monitor_task, "can_monitor_task", 4096, nullptr, 4, nullptr, 1);
#endif
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
