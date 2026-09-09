#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"
#include "can_manager.hpp"
#include "power_sensor.hpp"
#include "pins.hpp"
#include <atomic>

static const char* TAG = "INA228";

namespace Can = CanProtocol;

// INA228 conversion constants
static constexpr float BUS_VOLTAGE_LSB = 0.0001953125f; // 195.3125 µV/LSB
static constexpr float CURRENT_LSB     = 0.0000953674f; // A/LSB (Imax=50A, Rshunt=0.0002Ω)
static constexpr uint16_t SHUNT_CAL_VALUE = 250;         //Imax=50A

// INA228 register map
static constexpr uint8_t REG_SHUNT_CAL = 0x02;
static constexpr uint8_t REG_VBUS      = 0x05;
static constexpr uint8_t REG_CURRENT   = 0x07;

struct PowerSensorConfig {
    uint8_t  address;
    uint32_t can_id;
    float    current_lsb;
    uint16_t shunt_cal;
    const char* name;
};

static const PowerSensorConfig SENSORS[POWER_SENSOR_COUNT] = {
    { 0x45, Can::TELEMETRY_ID,   CURRENT_LSB, SHUNT_CAL_VALUE, "INA228#1" },
    { 0x44, Can::TELEMETRY_ID_2, CURRENT_LSB, SHUNT_CAL_VALUE, "INA228#2" },
};

// Each sensor is enabled/disabled on its own via CAN commands
std::atomic<bool> power_telemetry_enabled[POWER_SENSOR_COUNT] = {{false}, {false}};

bool power_telemetry_set_enabled(size_t index, bool enabled) {
    if (index >= POWER_SENSOR_COUNT) {
        return false;
    }

    power_telemetry_enabled[index] = enabled;
    return true;
}

void power_telemetry_set_all_enabled(bool enabled) {
    for (size_t i = 0; i < POWER_SENSOR_COUNT; ++i) {
        power_telemetry_enabled[i] = enabled;
    }
}

static void writeRegister16(uint8_t addr, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

static uint32_t readRegister24(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFFFFFFFF;

    Wire.requestFrom(addr, (uint8_t)3);
    if (Wire.available() != 3) return 0xFFFFFFFF;

    uint32_t value = 0;
    value |= ((uint32_t)Wire.read()) << 16;
    value |= ((uint32_t)Wire.read()) << 8;
    value |= Wire.read();
    return value;
}

void power_sensor_init() {
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    for (size_t i = 0; i < POWER_SENSOR_COUNT; ++i) {
        const PowerSensorConfig& s = SENSORS[i];
        // Write shunt calibration so current register returns meaningful values
        writeRegister16(s.address, REG_SHUNT_CAL, s.shunt_cal);
#ifdef DEBUG_ENABLED
        ESP_LOGD(TAG, "%s initialized at 0x%02X", s.name, s.address);
#endif
    }
}

static void poll_sensor(const PowerSensorConfig& s) {
    float voltage = 0.0f;
    float current = 0.0f;

    uint32_t rawV = readRegister24(s.address, REG_VBUS);
    if (rawV != 0xFFFFFFFF) {
        uint32_t vbus = rawV >> 4;
        voltage = vbus * BUS_VOLTAGE_LSB;
    } else {
#ifdef DEBUG_ENABLED
        ESP_LOGW(TAG, "%s: failed to read bus voltage", s.name);
#endif
    }

    uint32_t rawI = readRegister24(s.address, REG_CURRENT);
    if (rawI != 0xFFFFFFFF) {
        int32_t raw20 = rawI >> 4;
        if (raw20 & 0x80000) raw20 -= 0x100000; // sign extend
        current = raw20 * s.current_lsb;
    } else {
#ifdef DEBUG_ENABLED
        ESP_LOGW(TAG, "%s: failed to read current", s.name);
#endif
    }

    uint8_t payload[8];
    memcpy(payload, &current, sizeof(float));
    memcpy(payload + sizeof(float), &voltage, sizeof(float));
    can_tx_enqueue(s.can_id, payload, sizeof(payload));

    Serial.printf("%s (0x%02X) voltage=%.4f V current=%.4f A\n",
                  s.name, s.address, voltage, current);

#ifdef DEBUG_ENABLED
    ESP_LOGD(TAG, "%s voltage=%.4fV current=%.4fA", s.name, voltage, current);
#endif
}

void power_telemetry_task(void*) {
    const TickType_t period    = pdMS_TO_TICKS(200);
    TickType_t       last_wake = xTaskGetTickCount();

    for (;;) {
        for (size_t i = 0; i < POWER_SENSOR_COUNT; ++i) {
            if (power_telemetry_enabled[i]) {
                poll_sensor(SENSORS[i]);
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
