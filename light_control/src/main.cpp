#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "light_manager.hpp"

void setup() {
#if DEBUG_ENABLED
    Serial.begin(115200);

    while (!Serial) { delay(10); }
#endif

    light_init();
    can_init_1mbs_accept_all();

    xTaskCreatePinnedToCore(can_task, "can", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
