#include "light_manager.hpp"
#include "pins.hpp"

#include <Arduino.h>

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
