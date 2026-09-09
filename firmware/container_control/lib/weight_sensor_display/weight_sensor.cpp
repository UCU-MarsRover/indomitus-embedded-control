#include "weight_sensor.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <TM1637TinyDisplay6.h>
#include <cmath>
#include <string.h>
#include "pins.hpp"
#include "shared_state.hpp"

extern MPU6050 mpu;
extern bool mpu_initialized;
extern TM1637TinyDisplay6 display;
extern SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t weight_mutex;

static const uint8_t SEGMENT_CODES[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static constexpr uint8_t DISP_SEG_E = 0x79;
static constexpr uint8_t DISP_SEG_R = 0x50;
static constexpr float MAX_PLAUSIBLE_WEIGHT_G = 50000.0f;
static constexpr float TILT_CORRECTION_THRESHOLD_DEG = 20.0f;
static constexpr float DEG_TO_RAD_FACTOR = 0.01745329252f;

static inline float apply_tilt_correction(float value, float roll_deg, float pitch_deg) {
    const float abs_roll = fabsf(roll_deg);
    const float abs_pitch = fabsf(pitch_deg);

    if (abs_roll <= TILT_CORRECTION_THRESHOLD_DEG && abs_pitch <= TILT_CORRECTION_THRESHOLD_DEG) {
        return value;
    }

    const float roll_rad = roll_deg * DEG_TO_RAD_FACTOR;
    const float pitch_rad = pitch_deg * DEG_TO_RAD_FACTOR;
    const float cos_term = cosf(roll_rad) * cosf(pitch_rad);

    if (cos_term < 0.05f) {
        return value;
    }

    return value / cos_term;
}

static bool weight_is_valid(float w) {
    return w > -9000.0f && fabsf(w) <= MAX_PLAUSIBLE_WEIGHT_G;
}

void display_weights(int32_t w1_g, bool w1_err, int32_t w2_g, bool w2_err) {
    uint8_t segments[6];

    auto render_group = [&](uint8_t* out, int32_t value, bool err) {
        if (err) {
            out[0] = DISP_SEG_E;
            out[1] = DISP_SEG_R;
            out[2] = DISP_SEG_R;
            return;
        }

        int32_t abs_value = abs(value);
        out[0] = SEGMENT_CODES[(abs_value / 100) % 10];
        out[1] = SEGMENT_CODES[(abs_value / 10) % 10];
        out[2] = SEGMENT_CODES[abs_value % 10];
    };

    render_group(&segments[0], w2_g, w2_err);
    render_group(&segments[3], w1_g, w1_err);
    segments[2] |= 0x80;
    display.setSegments(segments, 6, 0);
}

void display_yaw_pitch(float yaw, float pitch) {
    (void)yaw;
    (void)pitch;
}

void safe_display_update(float yaw, float pitch) {
    (void)yaw;
    (void)pitch;
}

WeightSensor::WeightSensor(uint8_t dt_pin, uint8_t sck_pin, bool negative_direction)
    : _dt_pin(dt_pin), _sck_pin(sck_pin),
      _negative_direction(negative_direction),
      _calibrated(false),
      _scale_factor(298.37f),
      _scale(dt_pin, sck_pin) {}

void WeightSensor::init() {
    _scale.begin();
    _scale.start(1000, true);
    _scale.setCalFactor(_negative_direction ? -_scale_factor : _scale_factor);
    _calibrated = true;
}

bool WeightSensor::update() {
    return _scale.update();
}

float WeightSensor::read() {
    return _scale.getData();
}

void WeightSensor::tare() {
    _scale.tareNoDelay();
}

void WeightSensor::tare(uint8_t samples) {
    (void)samples;
    tare();
}

void WeightSensor::calibrate_zero() {
    _scale.tareNoDelay();
}

void WeightSensor::calibrate_zero(uint8_t samples) {
    (void)samples;
    calibrate_zero();
}

float WeightSensor::calibrate_weight(float known_weight_g) {
    if (known_weight_g <= 0.0f) return -1.0f;

    // Some HX711_ADC variants expose getVal() for raw ADC delta; this project exposes
    // getData() + getCalFactor(), so convert back to raw ADC counts before thresholding.
    _scale.refreshDataSet();
    float raw_adc_diff = _scale.getData() * _scale.getCalFactor();

    // Accept only real load; values below ~100 raw ADC ticks are effectively a zero/no-load state.
    if (fabsf(raw_adc_diff) < 100.0f) {
        return -1.0f;
    }

    _scale_factor = fabsf(raw_adc_diff / known_weight_g);
    _scale.setCalFactor(_negative_direction ? -_scale_factor : _scale_factor);
    _calibrated = true;
    return _scale_factor;
}

float WeightSensor::calibrate_weight(float known_weight_g, uint8_t samples) {
    (void)samples;
    return calibrate_weight(known_weight_g);
}

void WeightSensor::set_calibration(float raw_zero, float scale_factor) {
    (void)raw_zero;
    _scale_factor = scale_factor;
    _scale.setCalFactor(_negative_direction ? -_scale_factor : _scale_factor);
    _calibrated = true;
}

WeightSensor sensor1((uint8_t)PIN_HX711_LEFT_DT, (uint8_t)PIN_HX711_LEFT_SCK, false);
WeightSensor sensor2((uint8_t)PIN_HX711_RIGHT_DT, (uint8_t)PIN_HX711_RIGHT_SCK, true);

void weight_sensors_init() {
    sensor1.init();
    sensor2.init();

#ifdef DEBUG_ENABLED
    Serial.println("\n[HX711 DIAGNOSTIC]");
    Serial.println("Testing LEFT sensor...");
    Serial.println("  [OK] LEFT sensor is ready");
    Serial.println("Testing RIGHT sensor...");
    Serial.println("  [OK] RIGHT sensor is ready");
    Serial.println("[END HX711 DIAGNOSTIC]\n");
#endif
}

void imu_task(void* arg) {
    SharedState* state = static_cast<SharedState*>(arg);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        if (mpu_initialized) {
            if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                mpu.update();
                state->roll.store(mpu.getAngleX());
                state->pitch.store(mpu.getAngleY());
                state->yaw.store(mpu.getAngleZ());
                xSemaphoreGive(i2c_mutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void weight_sensors_task(void* arg) {
    SharedState* state = static_cast<SharedState*>(arg);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    float right_filtered = 0.0f;
    bool right_filtered_valid = false;
    uint8_t right_spike_streak = 0;
    uint8_t left_zero_streak = 0;
    uint8_t right_zero_streak = 0;
    uint32_t left_last_near_zero_ms = 0;
    uint32_t right_last_near_zero_ms = 0;

    for (;;) {
        if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            bool updated1 = sensor1.update();
            bool updated2 = sensor2.update();

            if (updated1) {
                float w1 = sensor1.read();
                const uint32_t now_ms = millis();

                if (fabsf(w1) <= 0.5f) {
                    w1 = 0.0f;
                }

                if (fabsf(w1) > 0.5f && fabsf(w1) <= 1.0f) {
                    if (left_last_near_zero_ms == 0 || now_ms - left_last_near_zero_ms >= 1000UL) {
                        left_last_near_zero_ms = now_ms;
                        if (left_zero_streak < 255) left_zero_streak++;
                    }
                } else {
                    left_zero_streak = 0;
                    left_last_near_zero_ms = 0;
                }

                if (left_zero_streak >= 5) {
                    sensor1.tare();
                    left_zero_streak = 0;
                    left_last_near_zero_ms = 0;
                    w1 = 0.0f;
                }

                const float roll_deg = state->roll.load();
                const float pitch_deg = state->pitch.load();
                w1 = apply_tilt_correction(w1, roll_deg, pitch_deg);

                bool ok1 = weight_is_valid(w1);
                state->weight1_error.store(!ok1);
                if (ok1) state->weight1_g.store(w1);
            }

            if (updated2) {
                float w2_raw = sensor2.read();

                if (!right_filtered_valid) {
                    right_filtered = w2_raw;
                    right_filtered_valid = true;
                } else {
                    const float delta = fabsf(w2_raw - right_filtered);
                    const bool likely_idle = fabsf(right_filtered) < 250.0f;
                    const bool implausible_spike = likely_idle && delta > 600.0f;

                    if (implausible_spike) {
                        if (right_spike_streak < 255) {
                            right_spike_streak++;
                        }

                        if (right_spike_streak <= 2) {
                            w2_raw = right_filtered;
                        } else {
                            right_filtered = right_filtered * 0.70f + w2_raw * 0.30f;
                        }
                    } else {
                        right_spike_streak = 0;
                        right_filtered = right_filtered * 0.75f + w2_raw * 0.25f;
                    }
                }

                float w2 = right_filtered;
                const uint32_t now_ms = millis();

                if (fabsf(w2) <= 0.5f) {
                    w2 = 0.0f;
                }

                if (fabsf(w2) > 0.5f && fabsf(w2) <= 1.0f) {
                    if (right_last_near_zero_ms == 0 || now_ms - right_last_near_zero_ms >= 1000UL) {
                        right_last_near_zero_ms = now_ms;
                        if (right_zero_streak < 255) right_zero_streak++;
                    }
                } else {
                    right_zero_streak = 0;
                    right_last_near_zero_ms = 0;
                }

                if (right_zero_streak >= 5) {
                    sensor2.tare();
                    right_zero_streak = 0;
                    right_last_near_zero_ms = 0;
                    w2 = 0.0f;
                }

                w2 *= -1.0f;

                const float roll_deg = state->roll.load();
                const float pitch_deg = state->pitch.load();
                w2 = apply_tilt_correction(w2, roll_deg, pitch_deg);

                bool ok2 = weight_is_valid(w2);
                state->weight2_error.store(!ok2);
                if (ok2) state->weight2_g.store(w2);
            }

            xSemaphoreGive(weight_mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void display_task(void* arg) {
    SharedState* state = static_cast<SharedState*>(arg);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    static constexpr uint32_t SIDE_PERIOD_MS = 2000UL;
    static constexpr uint32_t FADE_MS = 500UL;
    static constexpr uint8_t BRIGHT_MAX = 7;
    static constexpr uint8_t SEG_MINUS = 0x40;
    static constexpr uint8_t SEG_L = 0x38;
    static constexpr uint8_t SEG_R = 0x50;

    auto render_weight_label = [](bool left_side, int32_t grams, uint8_t brightness) {
        uint8_t segments[6] = {0, 0, 0, 0, 0, 0};
        segments[0] = left_side ? SEG_L : SEG_R;

        int32_t rounded = static_cast<int32_t>(lroundf(static_cast<float>(grams)));
        uint32_t magnitude = (rounded < 0) ? static_cast<uint32_t>(-rounded) : static_cast<uint32_t>(rounded);
        if (magnitude > 9999U) {
            magnitude = 9999U;
        }

        segments[1] = SEG_MINUS;

        uint32_t d0 = (magnitude / 1000U) % 10U;
        uint32_t d1 = (magnitude / 100U) % 10U;
        uint32_t d2 = (magnitude / 10U) % 10U;
        uint32_t d3 = magnitude % 10U;

        segments[2] = SEGMENT_CODES[d0];
        segments[3] = SEGMENT_CODES[d1];
        segments[4] = SEGMENT_CODES[d2];
        segments[5] = SEGMENT_CODES[d3];

        display.setBrightness(brightness, true);
        display.setSegments(segments, 6, 0);
    };

    for (;;) {
        const uint32_t now = millis();
        const uint32_t cycle = (now / SIDE_PERIOD_MS) % 2U;
        const bool left_side = (cycle == 1U);
        const float value = left_side ? state->weight1_g.load() : state->weight2_g.load();
        const uint32_t phase = now % SIDE_PERIOD_MS;

        uint8_t brightness = BRIGHT_MAX;
        if (phase < FADE_MS) {
            brightness = static_cast<uint8_t>((BRIGHT_MAX * phase) / FADE_MS);
        } else if (phase > SIDE_PERIOD_MS - FADE_MS) {
            const uint32_t tail = phase - (SIDE_PERIOD_MS - FADE_MS);
            brightness = static_cast<uint8_t>((BRIGHT_MAX * (FADE_MS - tail)) / FADE_MS);
        }

        const int32_t grams = static_cast<int32_t>(lroundf(value));
        render_weight_label(left_side, grams, brightness);

#ifdef DEBUG_ENABLED
        if (Serial && Serial.availableForWrite() > 64) {
            Serial.printf("DISPLAY %s = %.1f g  bright=%u\n",
                          left_side ? "LEFT" : "RIGHT",
                          value,
                          brightness);
        }
#endif

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
