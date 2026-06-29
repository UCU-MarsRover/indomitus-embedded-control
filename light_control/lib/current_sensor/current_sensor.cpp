#include <Arduino.h>
#include <algorithm>
#include <array>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
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
    adc_channel_t     channel;
    adc_cali_handle_t cali_handle;
    std::array<float, WINDOW_SIZE> window;
    int               window_idx;
    bool              window_full;
};

static adc_oneshot_unit_handle_t adc_handle; // спільний для обох сенсорів

static CurrentSensor sensor1 = { Pins::CURRENT_ADC_CHANNEL_1, nullptr, {}, 0, false };
static CurrentSensor sensor2 = { Pins::CURRENT_ADC_CHANNEL_2, nullptr, {}, 0, false };

static void sensor_init(CurrentSensor& s) {
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, s.channel, &chan_cfg);

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &s.cali_handle);
}

void current_sensor_init() {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    sensor_init(sensor1);
    sensor_init(sensor2);
}

static float raw_to_current(CurrentSensor& s) {
    int raw = 0;
    adc_oneshot_read(adc_handle, s.channel, &raw);

    int voltage_mv = 0;
    adc_cali_raw_to_voltage(s.cali_handle, raw, &voltage_mv);

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
    memcpy(sorted, s.window, count * sizeof(float));
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

        // два float в одному CAN пакеті (8 байт)
        uint8_t payload[8];
        memcpy(payload,     &current1, sizeof(float));
        memcpy(payload + 4, &current2, sizeof(float));

        can_tx_enqueue(Can::TELEMETRY_CURRENT_ID, payload, 8);

#if DEBUG_ENABLED
        ESP_LOGI(TAG, "current1=%.3fA current2=%.3fA", current1, current2);
#endif

        vTaskDelayUntil(&last_wake, period);
    }
}