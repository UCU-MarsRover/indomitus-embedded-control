#pragma once
#include <atomic>
#include <stddef.h>

// INA228 sensors present on the light ESP I2C bus
constexpr size_t POWER_SENSOR_COUNT = 2;

// Telemetry enable flag per sensor, index 0..POWER_SENSOR_COUNT-1
extern std::atomic<bool> power_telemetry_enabled[POWER_SENSOR_COUNT];

void power_sensor_init();
void power_telemetry_task(void*);

// Returns false when index is out of range
bool power_telemetry_set_enabled(size_t index, bool enabled);
void power_telemetry_set_all_enabled(bool enabled);
