#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "light_manager.hpp"
#include "light_command.hpp"
#include "pins.hpp"
#include "esp_log.h"

QueueHandle_t light_queue = nullptr;

void light_init() {
    pinMode(Pins::SPOTLIGHT, OUTPUT);
    pinMode(Pins::BEAUTIFUL, OUTPUT);
    pinMode(Pins::RED, OUTPUT);
    pinMode(Pins::YELLOW, OUTPUT);
    pinMode(Pins::GREEN, OUTPUT);
    pinMode(Pins::BLUE, OUTPUT);

    light_set_spotlight(false);
    light_set_beautiful(false);
    light_set_traffic_mask(0x00);

    light_queue = xQueueCreate(16, sizeof(LightCommand));
}

void light_set_spotlight(bool enabled) {
    digitalWrite(Pins::SPOTLIGHT, enabled ? HIGH : LOW);
}

void light_set_beautiful(bool enabled) {
    digitalWrite(Pins::BEAUTIFUL, enabled ? HIGH : LOW);
}

void light_set_traffic_mask(uint8_t mask) {
    digitalWrite(Pins::RED,    (mask & 0x01) ? HIGH : LOW);
    digitalWrite(Pins::YELLOW, (mask & 0x02) ? HIGH : LOW);
    digitalWrite(Pins::GREEN,  (mask & 0x04) ? HIGH : LOW);
    digitalWrite(Pins::BLUE,   (mask & 0x08) ? HIGH : LOW);
}


void light_task(void*) {
    LightCommand lc;
    for (;;) {
        if (xQueueReceive(light_queue, &lc, portMAX_DELAY) == pdTRUE) {
#ifdef DEBUG_ENABLED
            ESP_LOGI("Light_task", "command=%d", lc.cmd);
#endif
            switch (lc.cmd) {
                case LightCmd::SPOTLIGHT_ON:   light_set_spotlight(true);      break;
                case LightCmd::SPOTLIGHT_OFF:  light_set_spotlight(false);     break;
                case LightCmd::BEAUTIFUL_ON:   light_set_beautiful(true);      break;
                case LightCmd::BEAUTIFUL_OFF:  light_set_beautiful(false);     break;
                case LightCmd::TRAFFIC_MASK:   light_set_traffic_mask(lc.value); break;
            }
        }
    }
}
