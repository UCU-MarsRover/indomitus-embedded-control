#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "lid_manager.hpp"
#include "weight_sensor.hpp"
#include "shared_state.hpp"
#include "pins.hpp"


#ifdef DEBUG_ENABLED
#include <FastLED.h>

static constexpr uint8_t LED_PIN  = 48;
static constexpr uint8_t NUM_LEDS = 1;
CRGB leds[NUM_LEDS];
#endif // DEBUG_ENABLED


void setup() {
#ifdef DEBUG_ENABLED
    Serial.begin(115200);
    delay(1000);

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Black;
    FastLED.show();

    Serial.println("ESP32-S3 boot");
#endif // DEBUG_ENABLED

    lid_init();
    can_init_1mbs_accept_all();
    weight_sensors_init();

    xTaskCreatePinnedToCore(can_task,           "can",    4096, nullptr,   3, nullptr, 1);
    xTaskCreatePinnedToCore(lid_task,           "lid",    2048, nullptr,   2, nullptr, 0);
    xTaskCreatePinnedToCore(weight_sensors_task,"sensor", 2048, &g_state,  2, nullptr, 0);

#ifdef DEBUG_ENABLED
    leds[0] = CRGB::White; FastLED.show();
    delay(300);
    leds[0] = CRGB::Black; FastLED.show();

    Serial.println("Ready");
#endif // DEBUG_ENABLED
}


void loop() {
    vTaskDelay(portMAX_DELAY);
}
