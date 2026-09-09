#pragma once

#include <Arduino.h>
#include "Config.h"

// =============================================================================
// SensorArray — Unified sensor polling for Line, Water-Level, and pH sensors
// =============================================================================

class SensorArray {
public:
    SensorArray() = default;

    /// Initialise GPIO modes. Call once in setup().
    void begin();

    /// Non-blocking periodic update. Samples pH at PH_SAMPLE_INTERVAL_MS.
    /// Call every loop iteration.
    void update();

    // ----- Line Sensor (KY-033) ------------------------------------------

    /// @return true when the sensor detects the black strip (bottom reached).
    bool isBottomDetected() const;

    // ----- Water Level Sensor (T1592) ------------------------------------

    /// @return raw 12-bit ADC reading from the water level probe.
    uint16_t getWaterLevel() const;

    // ----- pH Sensor (DFRobot Gravity V2) --------------------------------

    /// Check whether the rolling pH variance has dropped below threshold.
    /// @param[out] outStableValue  The mean pH over the buffer window.
    /// @return true when the last PH_BUFFER_SIZE readings are stable.
    bool isPhStable(float& outStableValue) const;

    /// @return the latest instantaneous pH reading (may be noisy).
    float getInstantPh() const;

private:
    /// Convert raw ADC counts to pH using calibration constants.
    float adcToPh(uint16_t raw) const;

    /// Compute mathematical variance of the pH ring buffer.
    float computeVariance() const;

    // pH ring buffer
    float    _phBuffer[PH_BUFFER_SIZE] = {};
    uint16_t _phIndex    = 0;
    uint16_t _phCount    = 0;       // Samples collected so far (up to BUFFER_SIZE)
    unsigned long _lastPhSampleMs = 0;
};
