#pragma once
#include <atomic>

extern std::atomic<bool> power_telemetry_enabled;

void power_sensor_init();
void power_telemetry_task(void*);
