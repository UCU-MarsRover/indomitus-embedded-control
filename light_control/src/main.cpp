#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "light_manager.hpp"

#define DEBUG_PORT Serial0

#ifdef DEBUG_ENABLED
#include <FastLED.h>

static constexpr uint8_t LED_PIN  = 48;
static constexpr uint8_t NUM_LEDS = 1;
CRGB leds[NUM_LEDS];
#endif // DEBUG_ENABLED

void setup() {
#ifdef DEBUG_ENABLED
    DEBUG_PORT.begin(115200);
    delay(1000);
    DEBUG_PORT.println("light_control: setup entered");

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Black;
    FastLED.show();

    DEBUG_PORT.println("light_control: debug LED initialized");
#endif // DEBUG_ENABLED

#ifdef DEBUG_ENABLED
    DEBUG_PORT.println("light_control: before light_init");
#endif
    light_init();
#ifdef DEBUG_ENABLED
    DEBUG_PORT.println("light_control: after light_init");
    DEBUG_PORT.println("light_control: before can_init");
#endif
    const esp_err_t can_init_err = can_init_1mbs_accept_all();
#ifdef DEBUG_ENABLED
    DEBUG_PORT.print("light_control: can_init result = ");
    DEBUG_PORT.println(can_init_err == ESP_OK ? "ESP_OK" : esp_err_to_name(can_init_err));
#endif
    if (can_init_err != ESP_OK) {
#ifdef DEBUG_ENABLED
        DEBUG_PORT.println("light_control: CAN init failed, stopping setup");
        DEBUG_PORT.flush();
#endif
        return;
    }
#ifdef DEBUG_ENABLED
    DEBUG_PORT.println("light_control: after can_init");
    DEBUG_PORT.println("light_control: before task create");
#endif

    xTaskCreatePinnedToCore(can_task, "can", 4096, nullptr, 3, nullptr, 1);

#ifdef DEBUG_ENABLED
    DEBUG_PORT.println("light_control: after task create");
    leds[0] = CRGB::White; FastLED.show();
    delay(300);
    leds[0] = CRGB::Black; FastLED.show();

    DEBUG_PORT.println("light_control: ready");
    DEBUG_PORT.println("light_control: waiting CAN commands on ID 0x300");
    DEBUG_PORT.flush();
#endif // DEBUG_ENABLED
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
