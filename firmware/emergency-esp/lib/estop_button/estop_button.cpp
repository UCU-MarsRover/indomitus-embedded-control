#include "estop_button.hpp"

#include "pins.hpp"

extern "C" {
#include "driver/gpio.h"
}

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "ESTOP_BTN";

namespace {

bool     g_stable       = false;  ///< Accepted, debounced level.
bool     g_candidate    = false;  ///< Level currently being timed.
uint32_t g_candidate_at = 0;      ///< When the candidate level first appeared.
uint32_t g_stable_at    = 0;      ///< When the stable level was accepted.
uint32_t g_presses      = 0;

inline uint32_t now_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

}  // namespace

namespace EstopButton {

void init() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << Pins::ESTOP_BUTTON;
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    // Seed from the real level so a button held at boot reads as already
    // pressed, instead of producing a spurious PRESSED edge on the first poll.
    const bool level = gpio_get_level(Pins::ESTOP_BUTTON) != 0;
    g_stable       = level;
    g_candidate    = level;
    g_candidate_at = now_ms();
    g_stable_at    = g_candidate_at;

    ESP_LOGI(TAG, "GPIO%d input, pulldown, initial=%d",
             (int)Pins::ESTOP_BUTTON, (int)level);
}

Event poll() {
    const bool level = gpio_get_level(Pins::ESTOP_BUTTON) != 0;
    const uint32_t t = now_ms();

    if (level != g_candidate) {
        // Level changed: restart the debounce window.
        g_candidate    = level;
        g_candidate_at = t;
        return Event::NONE;
    }

    if (level == g_stable) {
        return Event::NONE;
    }

    if ((t - g_candidate_at) < DEBOUNCE_MS) {
        return Event::NONE;  // Still settling.
    }

    g_stable    = level;
    g_stable_at = t;

    if (level) {
        ++g_presses;
        ESP_LOGW(TAG, "E-stop button PRESSED (#%lu)", (unsigned long)g_presses);
        return Event::PRESSED;
    }

    ESP_LOGI(TAG, "E-stop button released");
    return Event::RELEASED;
}

bool is_pressed() {
    return g_stable;
}

uint32_t time_in_state_ms() {
    return now_ms() - g_stable_at;
}

uint32_t press_count() {
    return g_presses;
}

}  // namespace EstopButton
