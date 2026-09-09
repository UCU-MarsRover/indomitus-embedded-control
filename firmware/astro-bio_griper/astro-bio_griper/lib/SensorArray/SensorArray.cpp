#include "SensorArray.h"

// =============================================================================
// SensorArray.cpp — Sensor polling implementation
// =============================================================================

void SensorArray::begin() {
    pinMode(PIN_LINE_SENSOR, INPUT);
    // ADC pins don't need explicit pinMode on ESP32 — they default to analog.
    // Ensure 12-bit resolution (default on ESP32-C3).
    analogReadResolution(12);

    Serial.println(F("[SENSOR] Array initialised"));
}

// -----------------------------------------------------------------------------
// Periodic update — call every loop()
// -----------------------------------------------------------------------------
void SensorArray::update() {
    unsigned long now = millis();

    // pH sampling at fixed interval
    if ((now - _lastPhSampleMs) >= PH_SAMPLE_INTERVAL_MS) {
        _lastPhSampleMs = now;

        uint16_t raw = analogRead(PIN_PH_SENSOR);
        float    ph  = adcToPh(raw);

        // Write into ring buffer
        _phBuffer[_phIndex] = ph;
        _phIndex = (_phIndex + 1) % PH_BUFFER_SIZE;
        if (_phCount < PH_BUFFER_SIZE) {
            _phCount++;
        }
    }
}

// -----------------------------------------------------------------------------
// Line Sensor (KY-033)
// -----------------------------------------------------------------------------

bool SensorArray::isBottomDetected() const {
    // KY-033: LOW when obstacle/reflective surface (white), HIGH when non-reflective (black).
    // When the tube drops and the black strip is exposed → sensor reads HIGH.
    return digitalRead(PIN_LINE_SENSOR) == HIGH;
}

// -----------------------------------------------------------------------------
// Water Level Sensor (T1592)
// -----------------------------------------------------------------------------

uint16_t SensorArray::getWaterLevel() const {
    return static_cast<uint16_t>(analogRead(PIN_WATER_LEVEL));
}

// -----------------------------------------------------------------------------
// pH Sensor (DFRobot Gravity V2)
// -----------------------------------------------------------------------------

bool SensorArray::isPhStable(float& outStableValue) const {
    // Need a full buffer before declaring stability
    if (_phCount < PH_BUFFER_SIZE) {
        return false;
    }

    float variance = computeVariance();
    if (variance < PH_VARIANCE_THRESHOLD) {
        // Compute mean for the stable reading
        float sum = 0.0f;
        for (uint16_t i = 0; i < PH_BUFFER_SIZE; i++) {
            sum += _phBuffer[i];
        }
        outStableValue = sum / static_cast<float>(PH_BUFFER_SIZE);
        return true;
    }
    return false;
}

float SensorArray::getInstantPh() const {
    if (_phCount == 0) return 0.0f;
    // Return the most recently written sample
    uint16_t lastIdx = (_phIndex == 0) ? (PH_BUFFER_SIZE - 1) : (_phIndex - 1);
    return _phBuffer[lastIdx];
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

float SensorArray::adcToPh(uint16_t raw) const {
    float voltage = (static_cast<float>(raw) / PH_ADC_RESOLUTION) * PH_VOLTAGE_REF;
    return PH_SLOPE * voltage + PH_OFFSET;
}

float SensorArray::computeVariance() const {
    // Two-pass numerical stability: compute mean, then variance
    float sum = 0.0f;
    for (uint16_t i = 0; i < PH_BUFFER_SIZE; i++) {
        sum += _phBuffer[i];
    }
    float mean = sum / static_cast<float>(PH_BUFFER_SIZE);

    float sqDiffSum = 0.0f;
    for (uint16_t i = 0; i < PH_BUFFER_SIZE; i++) {
        float diff = _phBuffer[i] - mean;
        sqDiffSum += diff * diff;
    }
    return sqDiffSum / static_cast<float>(PH_BUFFER_SIZE);
}
