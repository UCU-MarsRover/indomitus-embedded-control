#include "weight_sensor.hpp"
#include <Arduino.h>
#include <HX711.h>

#include "pins.hpp"


WeightSensor::WeightSensor(uint8_t dt_pin, uint8_t sck_pin, float calibration)
    : _dt_pin(dt_pin), _sck_pin(sck_pin), _calibration(calibration) {}

void WeightSensor::init() {
    _scale.begin(_dt_pin, _sck_pin);
    _scale.set_scale(_calibration);
    _scale.tare();
}

void WeightSensor::tare() {
    _scale.tare();
}

float WeightSensor::read(uint8_t samples) {
    if (!_scale.is_ready()) {
        return -1.0f;
    }
    return _scale.get_units(samples);
}

void WeightSensor::set_calibration(float calibration) {
    _calibration = calibration;
    _scale.set_scale(calibration);
}




WeightSensor sensor1((uint8_t)PIN_HX711_1_DT, (uint8_t)PIN_HX711_1_SCK);
WeightSensor sensor2((uint8_t)PIN_HX711_2_DT, (uint8_t)PIN_HX711_2_SCK);
WeightSensor sensor3((uint8_t)PIN_HX711_3_DT, (uint8_t)PIN_HX711_3_SCK);


void weight_sensors_init() {
    sensor1.init();
    sensor2.init();
    sensor3.init();
}

void weight_sensors_task(void* arg) {
    SharedState* state = static_cast<SharedState*>(arg);
    for (;;) {
        const float w1 = sensor1.read();
        const float w2 = sensor2.read();
        const float w3 = sensor3.read();

        state->weight1.store(w1);
        state->weight2.store(w2);
        state->weight3.store(w3);

#ifdef DEBUG_ENABLED
        Serial.print("[SENSOR] w1="); Serial.print(w1, 3);
        Serial.print("  w2=");        Serial.print(w2, 3);
        Serial.print("  w3=");        Serial.println(w3, 3);
#endif
        vTaskDelay(pdMS_TO_TICKS(500));  // 2 Hz
    }
}
