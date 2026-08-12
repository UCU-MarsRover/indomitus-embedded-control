#include "jetson_reset.hpp"

#include "pins.hpp"
#include "safety_output.hpp"

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "JETSON_RST";

namespace {

// GPIO2 is RTC-capable, but the pad hold is deliberately NOT used here: this
// line pulses, and a hold left set across an unexpected reset would be one more
// way to strand the Jetson in reset. active_high = false --- idle is HIGH.
SafetyOutput g_out(Pins::JETSON_RESET, /*rtc_hold=*/false, /*active_high=*/false);

uint32_t g_started_at = 0;
uint32_t g_resets     = 0;
uint32_t g_faults     = 0;

inline uint32_t now_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

}  // namespace

namespace JetsonReset {

void init() {
    g_out.init();
}

bool reset() {
    if (g_out.is_active()) {
        ESP_LOGW(TAG, "reset already in progress, ignored");
        return false;
    }

    g_started_at = now_ms();
    g_out.set(true);
    ESP_LOGW(TAG, "asserting SYS_RESET* for %lums", (unsigned long)PULSE_MS);
    return true;
}

void update() {
    if (!g_out.is_active()) {
        return;
    }

    const uint32_t held = now_ms() - g_started_at;

    if (held >= MAX_ASSERT_MS) {
        // Should be unreachable --- PULSE_MS expires long before this. Getting
        // here means the timing state is wrong, so release and record it.
        ++g_faults;
        g_out.set(false);
        ESP_LOGE(TAG, "reset held %lums, past the %lums ceiling --- released",
                 (unsigned long)held, (unsigned long)MAX_ASSERT_MS);
        return;
    }

    if (held >= PULSE_MS) {
        g_out.set(false);
        ++g_resets;
        ESP_LOGI(TAG, "reset pulse done (#%lu)", (unsigned long)g_resets);
    }
}

void abort() {
    if (g_out.is_active()) {
        g_out.set(false);
        ESP_LOGW(TAG, "reset aborted");
    }
}

bool is_busy() {
    return g_out.is_active();
}

uint32_t reset_count() {
    return g_resets;
}

bool verify() {
    return g_out.verify();
}

uint32_t fault_count() {
    return g_faults + g_out.fault_count();
}

}  // namespace JetsonReset
