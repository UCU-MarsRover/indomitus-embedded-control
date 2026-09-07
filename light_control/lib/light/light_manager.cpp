#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "light_manager.hpp"
#include "light_command.hpp"
#include "pins.hpp"
#include "esp_log.h"

extern QueueHandle_t light_queue;

static constexpr gpio_num_t kSpotlightPins[] = {
    Pins::SPOTLIGHT_LEFT,
    Pins::SPOTLIGHT_RIGHT,
};

static constexpr gpio_num_t kBeautifulPins[] = {
    Pins::BEAUTIFUL_1,
    Pins::BEAUTIFUL_2,
    Pins::BEAUTIFUL_3,
    Pins::BEAUTIFUL_4,
};

static TaskHandle_t s_beautiful_task = nullptr;
static bool s_beautiful_animation_enabled = false;

static void set_pin(gpio_num_t pin, bool enabled) {
    digitalWrite(pin, enabled ? HIGH : LOW);
}

static void set_beautiful_all(bool enabled) {
    for (gpio_num_t pin : kBeautifulPins) {
        set_pin(pin, enabled);
    }
}

static void beautiful_animation_task(void*) {
    static constexpr bool kBeautifulPatterns[2][4] = {
        {true,  false, false, true },
        {false, true,  true,  false},
    };

    size_t pattern_index = 0;

    for (;;) {
        if (!s_beautiful_animation_enabled) {
            set_beautiful_all(false);
            s_beautiful_task = nullptr;
            vTaskDelete(nullptr);
        }

        for (size_t i = 0; i < 4; ++i) {
            set_pin(kBeautifulPins[i], kBeautifulPatterns[pattern_index][i]);
        }

        pattern_index = (pattern_index + 1) % 2;
        vTaskDelay(pdMS_TO_TICKS(260));
    }
}

void light_init() {
    pinMode(Pins::BUZZER, OUTPUT);

    for (gpio_num_t pin : kSpotlightPins) {
        pinMode(pin, OUTPUT);
    }

    for (gpio_num_t pin : kBeautifulPins) {
        pinMode(pin, OUTPUT);
    }

    pinMode(Pins::RED, OUTPUT);
    pinMode(Pins::GREEN, OUTPUT);
    pinMode(Pins::BLUE, OUTPUT);

    light_set_spotlight(false);
    light_set_beautiful(false);
    light_set_buzzer(false);
    light_set_traffic_mask(0x00);
}

void light_set_spotlight_left(bool enabled) {
    set_pin(Pins::SPOTLIGHT_LEFT, enabled);
}

void light_set_spotlight_right(bool enabled) {
    set_pin(Pins::SPOTLIGHT_RIGHT, enabled);
}

void light_set_spotlight(bool enabled) {
    light_set_spotlight_left(enabled);
    light_set_spotlight_right(enabled);
}

void light_set_beautiful_1(bool enabled) {
    set_pin(Pins::BEAUTIFUL_1, enabled);
}

void light_set_beautiful_2(bool enabled) {
    set_pin(Pins::BEAUTIFUL_2, enabled);
}

void light_set_beautiful_3(bool enabled) {
    set_pin(Pins::BEAUTIFUL_3, enabled);
}

void light_set_beautiful_4(bool enabled) {
    set_pin(Pins::BEAUTIFUL_4, enabled);
}

void light_set_beautiful(bool enabled) {
    if (!enabled) {
        s_beautiful_animation_enabled = false;
        set_beautiful_all(false);
        return;
    }

    s_beautiful_animation_enabled = true;

    if (s_beautiful_task == nullptr) {
        xTaskCreatePinnedToCore(beautiful_animation_task, "beautiful_anim", 2048, nullptr, 2, &s_beautiful_task, 1);
    }
}

void light_set_buzzer(bool enabled) {
    set_pin(Pins::BUZZER, enabled);
}

void light_set_red(bool enabled) {
    set_pin(Pins::RED, enabled);
}

void light_set_green(bool enabled) {
    set_pin(Pins::GREEN, enabled);
}

void light_set_blue(bool enabled) {
    set_pin(Pins::BLUE, enabled);
}

void light_set_traffic_mask(uint8_t mask) {
    light_set_red((mask & 0x01) != 0);
    light_set_green((mask & 0x02) != 0);
    light_set_blue((mask & 0x04) != 0);
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
                case LightCmd::SPOTLIGHT_LEFT_ON:  light_set_spotlight_left(true);   break;
                case LightCmd::SPOTLIGHT_LEFT_OFF: light_set_spotlight_left(false);  break;
                case LightCmd::SPOTLIGHT_RIGHT_ON: light_set_spotlight_right(true);  break;
                case LightCmd::SPOTLIGHT_RIGHT_OFF: light_set_spotlight_right(false); break;
                case LightCmd::BEAUTIFUL_ON:   light_set_beautiful(true);      break;
                case LightCmd::BEAUTIFUL_OFF:  light_set_beautiful(false);     break;
                case LightCmd::BEAUTIFUL_1_ON: light_set_beautiful_1(true);    break;
                case LightCmd::BEAUTIFUL_1_OFF: light_set_beautiful_1(false);   break;
                case LightCmd::BEAUTIFUL_2_ON: light_set_beautiful_2(true);    break;
                case LightCmd::BEAUTIFUL_2_OFF: light_set_beautiful_2(false);   break;
                case LightCmd::BEAUTIFUL_3_ON: light_set_beautiful_3(true);    break;
                case LightCmd::BEAUTIFUL_3_OFF: light_set_beautiful_3(false);   break;
                case LightCmd::BEAUTIFUL_4_ON: light_set_beautiful_4(true);    break;
                case LightCmd::BEAUTIFUL_4_OFF: light_set_beautiful_4(false);   break;
                case LightCmd::RED_ON:         light_set_red(true);            break;
                case LightCmd::RED_OFF:        light_set_red(false);           break;
                case LightCmd::GREEN_ON:       light_set_green(true);          break;
                case LightCmd::GREEN_OFF:      light_set_green(false);         break;
                case LightCmd::BLUE_ON:        light_set_blue(true);           break;
                case LightCmd::BLUE_OFF:       light_set_blue(false);          break;
                case LightCmd::BUZZER_ON:      light_set_buzzer(true);         break;
                case LightCmd::BUZZER_OFF:     light_set_buzzer(false);        break;
                case LightCmd::TOWER_ON:       light_set_traffic_mask(0x07);   break;
                case LightCmd::TOWER_OFF:      light_set_traffic_mask(0x00);   break;
                case LightCmd::TRAFFIC_MASK:   light_set_traffic_mask(lc.value); break;
            }
        }
    }
}
