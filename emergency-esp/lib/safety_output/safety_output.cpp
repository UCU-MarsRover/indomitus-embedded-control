#include "safety_output.hpp"

#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

static const char* TAG = "SAFETY_OUT";

// Sentinel values chosen so that an all-zero or all-one memory corruption never
// looks like a legitimate state.
static constexpr uint32_t STATE_LOW  = 0x5A5A5A5Au;
static constexpr uint32_t STATE_HIGH = 0xA5A5A5A5u;

SafetyOutput::SafetyOutput(gpio_num_t pin, bool rtc_hold)
    : pin_(pin),
      rtc_hold_(rtc_hold),
      desired_(STATE_LOW),
      desired_inv_(~STATE_LOW),
      faults_(0) {}

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

    // Write the output latch to 0 *before* the driver is enabled. gpio_set_level
    // updates the output register even while the pad is still an input, so when
    // gpio_config() below switches the driver on, the only level it can ever
    // present is 0. This is what makes bring-up glitch-free.
    gpio_set_level(pin_, 0);

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin_;
    cfg.mode         = GPIO_MODE_OUTPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    // Kept on alongside the push-pull driver so the pad is biased low whenever
    // the driver is not actively holding it. Weak (~45k) --- it reinforces the
    // external pulldown, it does not replace it.
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    desired_     = STATE_LOW;
    desired_inv_ = ~STATE_LOW;

    write_raw(false);

    ESP_LOGI(TAG, "GPIO%d parked LOW (rtc_hold=%d)", (int)pin_, (int)rtc_hold_);
}

void SafetyOutput::set(bool active) {
    const uint32_t state = active ? STATE_HIGH : STATE_LOW;

    // Update the redundant copies before touching hardware, so that a reset
    // landing mid-call leaves verify() able to see the intended state.
    desired_     = state;
    desired_inv_ = ~state;

    write_raw(active);
}

bool SafetyOutput::is_active() const {
    return desired_ == STATE_HIGH;
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
                            (state == STATE_LOW || state == STATE_HIGH);

    if (!consistent) {
        ++faults_;
        ESP_LOGE(TAG, "GPIO%d state corrupted (0x%08lX/0x%08lX), forcing LOW",
                 (int)pin_, (unsigned long)state, (unsigned long)inv);
        desired_     = STATE_LOW;
        desired_inv_ = ~STATE_LOW;
        write_raw(false);
        return true;
    }

    const bool want = (state == STATE_HIGH);
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
