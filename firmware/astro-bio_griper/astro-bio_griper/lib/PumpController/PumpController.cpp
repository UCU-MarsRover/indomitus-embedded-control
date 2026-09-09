#include "PumpController.h"

// =============================================================================
// PumpController.cpp — DRV8871 motor driver via ESP32 LEDC
// =============================================================================

void PumpController::begin() {
    // Configure LEDC channels for IN1 and IN2
    ledcSetup(PWM_CHANNEL_IN1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(PWM_CHANNEL_IN2, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);

    ledcAttachPin(PIN_PUMP_IN1, PWM_CHANNEL_IN1);
    ledcAttachPin(PIN_PUMP_IN2, PWM_CHANNEL_IN2);

    // Start in coast mode (both LOW)
    ledcWrite(PWM_CHANNEL_IN1, 0);
    ledcWrite(PWM_CHANNEL_IN2, 0);

    _running = false;

    Serial.println(F("[PUMP] Controller initialised — coast mode"));
}

void PumpController::startPump(uint8_t dutyCycle) {
    // Forward: IN1 = PWM, IN2 = LOW
    ledcWrite(PWM_CHANNEL_IN2, 0);
    ledcWrite(PWM_CHANNEL_IN1, dutyCycle);
    _running = true;

    Serial.printf("[PUMP] START duty=%u/255\n", dutyCycle);
}

void PumpController::stopPump() {
    // Coast: both LOW (no braking current — gentler on peristaltic pump)
    ledcWrite(PWM_CHANNEL_IN1, 0);
    ledcWrite(PWM_CHANNEL_IN2, 0);
    _running = false;

    Serial.println(F("[PUMP] STOP"));
}

bool PumpController::isRunning() const {
    return _running;
}
