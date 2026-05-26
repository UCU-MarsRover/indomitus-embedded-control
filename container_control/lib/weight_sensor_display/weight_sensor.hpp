#pragma once
#include <HX711.h>
#include "shared_state.hpp"

class WeightSensor {
    public:
    WeightSensor(uint8_t dt_pin, uint8_t sck_pin, float calibration = 2280.f);
    void init();
    void tare();
    float read(uint8_t samples = 5);
    void set_calibration(float calibration);
    
    private:
    HX711 _scale;
    uint8_t _dt_pin;
    uint8_t _sck_pin;
    float _calibration;
};

extern WeightSensor sensor1;
extern WeightSensor sensor2;
extern WeightSensor sensor3;

void weight_sensors_init();
void weight_sensors_task(void* arg);  // arg = SharedState*
