#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"
#include "can_manager.hpp"
#include "power_sensor.hpp"
#include "pins.hpp"

static const char* TAG = "INA228";

namespace Can = CanProtocol;

// INA228 I2C address and conversion constants
static constexpr uint8_t INA228_ADDR = 0x45;
static constexpr float BUS_VOLTAGE_LSB = 0.0001953125f; // 195.3125 µV/LSB
static constexpr float CURRENT_LSB     = 0.0000953674f; // A/LSB (Imax=50A, Rshunt=0.0002Ω)
static constexpr uint16_t SHUNT_CAL_VALUE = 250;         //Imax=50A

static void writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

static uint32_t readRegister24(uint8_t reg) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFFFFFFFF;

    Wire.requestFrom(INA228_ADDR, (uint8_t)3);
    if (Wire.available() != 3) return 0xFFFFFFFF;

    uint32_t value = 0;
    value |= ((uint32_t)Wire.read()) << 16;
    value |= ((uint32_t)Wire.read()) << 8;
    value |= Wire.read();
    return value;
}

void power_sensor_init() {
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    // Write shunt calibration so current register returns meaningful values
    writeRegister16(0x02, SHUNT_CAL_VALUE);
    ESP_LOGD(TAG, "INA228 initialized at 0x%02X", INA228_ADDR);
}

void power_telemetry_task(void*) {
    const TickType_t period    = pdMS_TO_TICKS(200);
    TickType_t       last_wake = xTaskGetTickCount();

    for (;;) {
        float voltage = 0.0f;
        float current = 0.0f;

        uint32_t rawV = readRegister24(0x05);
        if (rawV != 0xFFFFFFFF) {
            uint32_t vbus = rawV >> 4;
            voltage = vbus * BUS_VOLTAGE_LSB;
        } else {
            ESP_LOGW(TAG, "Failed to read bus voltage");
        }

        uint32_t rawI = readRegister24(0x07);
        if (rawI != 0xFFFFFFFF) {
            int32_t raw20 = rawI >> 4;
            if (raw20 & 0x80000) raw20 -= 0x100000; // sign extend
            current = raw20 * CURRENT_LSB;
        } else {
            ESP_LOGW(TAG, "Failed to read current");
        }

        uint8_t payload[8];
        memcpy(payload, &current, sizeof(float));
        memcpy(payload + sizeof(float), &voltage, sizeof(float));
        can_tx_enqueue(Can::TELEMETRY_ID, payload, sizeof(payload));

        Serial.printf("INA228 voltage=%.4f V current=%.4f A\n", voltage, current);

        ESP_LOGD(TAG, "voltage=%.4fV current=%.4fA", voltage, current);

        vTaskDelayUntil(&last_wake, period);
    }
}