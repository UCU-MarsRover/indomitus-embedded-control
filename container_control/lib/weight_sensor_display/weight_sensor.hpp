#pragma once
#include <stdint.h>
#include <HX711_ADC.h>
#include "shared_state.hpp"

class WeightSensor {
public:
    WeightSensor(uint8_t dt_pin, uint8_t sck_pin, bool negative_direction = false);

    void  init();

    void  tare();
    void  tare(uint8_t samples);
    bool  update();
    float read();

    void  calibrate_zero();
    void  calibrate_zero(uint8_t samples);
    float calibrate_weight(float known_weight_g);
    float calibrate_weight(float known_weight_g, uint8_t samples);
    void  set_calibration(float raw_zero, float scale_factor);

    bool  is_calibrated() const { return _calibrated; }
    float get_scale_factor() const { return _scale_factor; }

private:
    uint8_t   _dt_pin;
    uint8_t   _sck_pin;
    bool      _negative_direction;
    bool      _calibrated;
    float     _scale_factor;
    HX711_ADC _scale;
};

extern WeightSensor sensor1;
extern WeightSensor sensor2;

void weight_sensors_init();
void imu_task(void* arg);
void weight_sensors_task(void* arg);
void display_task(void* arg);
void display_weights(int32_t w1_g, bool w1_err, int32_t w2_g, bool w2_err);
void display_yaw_pitch(float yaw, float pitch);
void safe_display_update(float yaw, float pitch);
