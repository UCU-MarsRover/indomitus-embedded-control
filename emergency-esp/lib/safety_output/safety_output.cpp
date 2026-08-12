#include "safety_output.hpp"

#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

static const char* TAG = "SAFETY_OUT";

// Sentinel values chosen so that an all-zero or all-one memory corruption never
// looks like a legitimate state.
static constexpr uint32_t STATE_INACTIVE = 0x5A5A5A5Au;
static constexpr uint32_t STATE_ACTIVE   = 0xA5A5A5A5u;

SafetyOutput::SafetyOutput(gpio_num_t pin, bool rtc_hold, bool active_high)
    : pin_(pin),
      rtc_hold_(rtc_hold),
      active_high_(active_high),
      desired_(STATE_INACTIVE),
      desired_inv_(~STATE_INACTIVE),
      faults_(0) {}

bool SafetyOutput::level_for(bool active) const {
    return active_high_ ? active : !active;
}

void SafetyOutput::write_raw(bool level) {
    // The pad may have been latched by a previous hold; release it, write, then
    // re-latch. Holding a pad blocks writes to it, so this order matters.
    if (rtc_hold_) {
        gpio_hold_dis(pin_);
    }

    gpio_set_level(pin_, level ? 1 : 0);

    if (rtc_hold_) {
        gpio_hold_en(pin_);
    }
}

void SafetyOutput::init() {
    const bool idle = level_for(false);

    // A hold left over from a previous boot would block every write below.
    gpio_hold_dis(pin_);

    // Detach any peripheral that may be routed to this pad, so only the simple
    // GPIO output register drives it.
    //
    // Deliberately NOT gpio_reset_pin(): that helper resets a pad with
    // GPIO_PULLUP_ENABLE, which would briefly pull this gate line toward 3V3
    // through the internal ~45k --- exactly the disturbance this class exists
    // to prevent.
    esp_rom_gpio_connect_out_signal(pin_, SIG_GPIO_OUT_IDX, false, false);

    // Write the idle level to the output latch *before* the driver is enabled.
    // gpio_set_level updates the output register even while the pad is still an
    // input, so when gpio_config() below switches the driver on, the only level
    // it can ever present is the idle one. This is what makes bring-up
    // glitch-free.
    gpio_set_level(pin_, idle ? 1 : 0);

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin_;
    cfg.mode         = GPIO_MODE_OUTPUT;
    // The internal pull is kept on alongside the push-pull driver, biased
    // toward the idle level, so the pad is held safe whenever the driver is not
    // actively holding it. Weak (~45k) --- it reinforces the external resistor,
    // it does not replace it.
    cfg.pull_up_en   = idle ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = idle ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    desired_     = STATE_INACTIVE;
    desired_inv_ = ~STATE_INACTIVE;

    write_raw(idle);

    ESP_LOGI(TAG, "GPIO%d parked %s (active_%s, rtc_hold=%d)",
             (int)pin_, idle ? "HIGH" : "LOW",
             active_high_ ? "high" : "low", (int)rtc_hold_);
}

void SafetyOutput::set(bool active) {
    const uint32_t state = active ? STATE_ACTIVE : STATE_INACTIVE;

    // Update the redundant copies before touching hardware, so that a reset
    // landing mid-call leaves verify() able to see the intended state.
    desired_     = state;
    desired_inv_ = ~state;

    write_raw(level_for(active));
}

bool SafetyOutput::is_active() const {
    return desired_ == STATE_ACTIVE;
}

bool SafetyOutput::read_pad() const {
    return gpio_get_level(pin_) != 0;
}

bool SafetyOutput::verify() {
    const uint32_t state = desired_;
    const uint32_t inv   = desired_inv_;

    // Corruption check: the two copies must be exact complements, and the value
    // must be one of the two sentinels. Anything else means memory was damaged,
    // and the only defensible response is to force the safe state.
    const bool consistent = (state == ~inv) &&
                            (state == STATE_INACTIVE || state == STATE_ACTIVE);

    if (!consistent) {
        ++faults_;
        ESP_LOGE(TAG, "GPIO%d state corrupted (0x%08lX/0x%08lX), forcing idle",
                 (int)pin_, (unsigned long)state, (unsigned long)inv);
        desired_     = STATE_INACTIVE;
        desired_inv_ = ~STATE_INACTIVE;
        write_raw(level_for(false));
        return true;
    }

    const bool want = level_for(state == STATE_ACTIVE);
    const bool got  = read_pad();

    // Unconditionally rewrite the register. If some other code reconfigured the
    // pad, this repairs it; if nothing is wrong, it is a harmless same-value
    // write that produces no edge.
    write_raw(want);

    if (got != want) {
        ++faults_;
        ESP_LOGE(TAG, "GPIO%d read back %d, expected %d --- re-asserted",
                 (int)pin_, (int)got, (int)want);
        return true;
    }

    return false;
}

uint32_t SafetyOutput::fault_count() const {
    return faults_;
}
