#pragma once

#include <stdint.h>

extern "C" {
#include "driver/gpio.h"
}

/**
 * @brief A digital output for a line where an unintended HIGH is dangerous.
 *
 * Both critical lines on this board (rover power cut, Jetson CAN cut) close a
 * transistor when HIGH, so any stray pulse has a real-world consequence. This
 * class makes a spurious HIGH as close to impossible as software can get:
 *
 *  - Glitch-free bring-up. The output latch is written to 0 *before* the pad
 *    driver is enabled, so enabling the driver can only ever drive 0. Calling
 *    pinMode()/gpio_config() first would briefly output whatever the latch
 *    happened to hold after reset.
 *  - The internal pulldown stays enabled alongside the push-pull driver, so the
 *    pad is biased low even while the driver is off.
 *  - Redundant state. The desired level is stored twice, as a value and as its
 *    bitwise complement. A single-bit upset in either copy is caught by
 *    verify(), which then forces the line to the safe state.
 *  - Periodic re-assert. verify() rewrites the hardware register even when it
 *    already reads back correct, so a corrupted GPIO register, or a peripheral
 *    that stole the pad, is repaired within one verify period.
 *  - Optional RTC pad hold, which latches the pad at the IO-mux level so the
 *    level survives sleep and peripheral reconfiguration.
 *
 * What software CANNOT do, and the reason each line still needs a 10k external
 * pulldown to GND at the transistor gate: from power-on until init() runs
 * (roughly 20-50 ms), and again during every reset, brownout dip, or watchdog
 * reboot, the pad is a high-impedance input and the gate voltage is undefined.
 * Only the resistor covers that window.
 *
 * @note Never attach LEDC/PWM, ADC, or any other peripheral to a pad owned by
 *       a SafetyOutput.
 */
class SafetyOutput {
public:
    /**
     * @param pin      Pad to drive.
     * @param rtc_hold Latch the pad with gpio_hold_en() after each write. Only
     *                 valid for RTC-capable pads (GPIO0..5 on the ESP32-C3);
     *                 pass false for any other pad.
     */
    SafetyOutput(gpio_num_t pin, bool rtc_hold);

    /**
     * @brief Configure the pad, guaranteeing it never drives HIGH on the way.
     *
     * Call this as the very first thing in setup(), before any other peripheral
     * init. It takes the pad away from whatever it was doing and parks it low.
     * Idempotent.
     */
    void init();

    /**
     * @brief Drive the line.
     * @param active true = HIGH = cut engaged, false = LOW = normal operation.
     */
    void set(bool active);

    /// Last commanded state as understood by this object.
    bool is_active() const;

    /// Read the level the pad is actually at right now.
    bool read_pad() const;

    /**
     * @brief Re-assert the intended level and repair corruption.
     *
     * Call periodically (every 20-100 ms) from a task or from loop(). If the
     * redundant copies disagree the line is forced LOW, which is the safe
     * state, and the fault is reported.
     *
     * @return true if a mismatch or corruption was detected and corrected.
     */
    bool verify();

    /// Number of faults verify() has detected and corrected since boot.
    uint32_t fault_count() const;

private:
    void write_raw(bool level);

    const gpio_num_t pin_;
    const bool rtc_hold_;

    // Redundant storage: desired_ and desired_inv_ must always be complements.
    volatile uint32_t desired_;
    volatile uint32_t desired_inv_;
    volatile uint32_t faults_;
};
