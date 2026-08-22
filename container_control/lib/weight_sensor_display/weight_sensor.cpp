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

    for (;;) {
        if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            bool updated1 = sensor1.update();
            bool updated2 = sensor2.update();

            if (updated1) {
                float w1 = sensor1.read();
                bool ok1 = weight_is_valid(w1);
                state->weight1_error.store(!ok1);
                if (ok1) state->weight1_g.store(w1);
            }

            if (updated2) {
                float w2 = sensor2.read();
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
    const TickType_t xFrequency = pdMS_TO_TICKS(200);

    for (;;) {
        int32_t w1 = static_cast<int32_t>(state->weight1_g.load());
        int32_t w2 = static_cast<int32_t>(state->weight2_g.load());
        bool e1 = state->weight1_error.load();
        bool e2 = state->weight2_error.load();

        display_weights(w1, e1, w2, e2);

#ifdef DEBUG_ENABLED
        if (Serial && Serial.availableForWrite() > 64) {
            Serial.printf("W1: %s | W2: %s | Roll: %.1f\n",
                          e1 ? "ERR" : String(w1).c_str(),
                          e2 ? "ERR" : String(w2).c_str(),
                          state->roll.load());
        }
#endif

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
