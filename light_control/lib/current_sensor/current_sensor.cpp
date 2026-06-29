#include <Arduino.h>
#include <algorithm>
#include <array>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "can_manager.hpp"
#include "pins.hpp"
#include "esp_log.h"

static const char* TAG = "CURRENT";

namespace Can = CanProtocol;

#define SENSITIVITY  0.040f
#define V_OFFSET     1.65f
#define WINDOW_SIZE  9
#define TRIM_COUNT   2

struct CurrentSensor {
    adc1_channel_t                 channel;
    esp_adc_cal_characteristics_t  chars;
    std::array<float, WINDOW_SIZE> window;
    int                            window_idx;
    bool                           window_full;
};

static CurrentSensor sensor1 = { (adc1_channel_t)Pins::CURRENT_ADC_CHANNEL_1, {}, {}, 0, false };
static CurrentSensor sensor2 = { (adc1_channel_t)Pins::CURRENT_ADC_CHANNEL_2, {}, {}, 0, false };

static void sensor_init(CurrentSensor& s) {
    adc1_config_channel_atten(s.channel, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &s.chars);
}

void current_sensor_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    sensor_init(sensor1);
    sensor_init(sensor2);
}

static float raw_to_current(CurrentSensor& s) {
    int raw = adc1_get_raw(s.channel);
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(raw, &s.chars);
    return (voltage_mv / 1000.0f - V_OFFSET) / SENSITIVITY;
}

static float sensor_read(CurrentSensor& s) {
    s.window[s.window_idx] = raw_to_current(s);
    s.window_idx = (s.window_idx + 1) % WINDOW_SIZE;
    if (s.window_idx == 0) s.window_full = true;

    int count = s.window_full ? WINDOW_SIZE : s.window_idx;
    if (count < (TRIM_COUNT * 2 + 1)) {
        return s.window[(s.window_idx - 1 + WINDOW_SIZE) % WINDOW_SIZE];
    }

    float sorted[WINDOW_SIZE];
    memcpy(sorted, s.window.data(), count * sizeof(float));
    std::sort(sorted, sorted + count);

    float sum = 0.0f;
    for (int i = TRIM_COUNT; i < count - TRIM_COUNT; i++) {
        sum += sorted[i];
    }
    return sum / (count - 2 * TRIM_COUNT);
}

void current_telemetry_task(void*) {
    const TickType_t period    = pdMS_TO_TICKS(100);
    TickType_t       last_wake = xTaskGetTickCount();

    for (;;) {
        float current1 = sensor_read(sensor1);
        float current2 = sensor_read(sensor2);

        uint8_t payload[8];
        memcpy(payload,     &current1, sizeof(float));
        memcpy(payload + 4, &current2, sizeof(float));

        can_tx_enqueue(Can::TELEMETRY_CURRENT_ID, payload, 8);

#ifdef DEBUG_ENABLED
        ESP_LOGI(TAG, "current1=%.3fA current2=%.3fA", current1, current2);
#endif

        vTaskDelayUntil(&last_wake, period);
    }
}