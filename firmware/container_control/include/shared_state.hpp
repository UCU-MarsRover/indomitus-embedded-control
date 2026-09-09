#pragma once
#include <atomic>

// Calibration step state machine for the two active sensors.
enum class CalibStep : uint8_t {
    IDLE      = 0,
    TARE_1    = 1,
    TARE_2    = 2,
    WEIGHT_1  = 11,
    WEIGHT_2  = 12,
};

struct SharedState {
    // Live raw/processed values for left and right scales.
    std::atomic<float> weight1{0.0f};
    std::atomic<float> weight2{0.0f};
    std::atomic<float> weight1_g{0.0f};
    std::atomic<float> weight2_g{0.0f};
    std::atomic<bool> weight1_error{false};
    std::atomic<bool> weight2_error{false};

    // Live orientation values updated by the fast IMU task.
    std::atomic<float> roll{0.0f};
    std::atomic<float> pitch{0.0f};
    std::atomic<float> yaw{0.0f};

    // Calibration flow
    std::atomic<uint8_t> calib_step{0};
    std::atomic<float>   calib_known_weight{0.0f};

    // Result of calibration for left/right sensors.
    std::atomic<float>   calib_factor1{1.0f};
    std::atomic<float>   calib_factor2{1.0f};

    std::atomic<uint8_t> apply_calib{0};
};

extern SharedState g_state;
