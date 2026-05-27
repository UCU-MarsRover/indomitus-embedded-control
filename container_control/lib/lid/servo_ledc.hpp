// servo_ledc.hpp
// Continuous rotation servo controller (360°)
#pragma once
#include "driver/ledc.h"

class ServoLedc {
    int _channel = -1;
public:
    void attach(int pin, int channel) {
        _channel = channel;
        ledc_timer_config_t timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_14_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 50,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer);

        ledc_channel_config_t ch = {
            .gpio_num   = pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)channel,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = microsecondsToDuty(1500),  // neutral (stopped)
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }

    // Set servo speed via pulse width in microseconds
    // 1500µs   = stopped (neutral)
    // < 1500µs = backward (faster as value decreases)
    // > 1500µs = forward (faster as value increases)
    void writeMicroseconds(int microseconds) {
        if (_channel < 0) return;
        uint32_t duty = microsecondsToDuty(microseconds);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel);
    }

    // Stop the servo (neutral position)
    void stop() {
        writeMicroseconds(1500);
    }

    void detach() {
        if (_channel < 0) return;
        ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)_channel, 0);
        _channel = -1;
    }

private:
    // Convert microseconds to PWM duty cycle
    // 50Hz @ 14-bit resolution: period = 20ms = 20000µs
    // duty = microseconds * 16384 / 20000
    static uint32_t microsecondsToDuty(int microseconds) {
        return (uint32_t)microseconds * 16384 / 20000;
    }
};