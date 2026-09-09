#pragma once

#include <Arduino.h>
#include "Config.h"

// =============================================================================
// PumpController — DRV8871 motor driver abstraction via ESP32 LEDC PWM
// =============================================================================
//
// DRV8871 truth table (H-bridge):
//   IN1  IN2  | Mode
//   PWM  LOW  | Forward
//   LOW  PWM  | Reverse
//   LOW  LOW  | Coast (pump off)
//   HIGH HIGH | Brake  (pump off, fast decay)
//
// We use Forward (IN1=PWM, IN2=LOW) for extraction.
// =============================================================================

class PumpController {
public:
    PumpController() = default;

    /// Configure LEDC channels and GPIO. Call once in setup().
    void begin();

    /// Start the pump in forward direction.
    /// @param dutyCycle 0-255 PWM duty (8-bit resolution).
    void startPump(uint8_t dutyCycle = PUMP_DEFAULT_DUTY);

    /// Immediately halt the pump (coast mode).
    void stopPump();

    /// @return true if the pump is currently running.
    bool isRunning() const;

private:
    bool _running = false;
};
