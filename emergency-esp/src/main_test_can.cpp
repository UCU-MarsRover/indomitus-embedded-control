#include <Arduino.h>

#include "can_driver.hpp"
#include "can_service.hpp"
#include "pins.hpp"

#include "esp_log.h"

static const char* TAG = "CAN_TEST";

/// Test frame ID.
static constexpr uint32_t TEST_TX_ID = 0x303;

/// Transmit period.
static constexpr uint32_t TX_INTERVAL_MS = 1000;

/// Status dump period.
static constexpr uint32_t STATUS_INTERVAL_MS = 5000;

static volatile uint32_t g_rx_count = 0;
static volatile uint32_t g_tx_count = 0;

/// Runs in the CanService RX task for every frame on the bus.
static void on_frame(const CanMsg& msg) {
    ++g_rx_count;

    char hex[3 * 8 + 1];
    int n = 0;
    for (uint8_t i = 0; i < msg.len && i < 8; ++i) {
        n += snprintf(hex + n, sizeof(hex) - n, "%02X ", msg.data[i]);
    }
    hex[n > 0 ? n - 1 : 0] = '\0';

    ESP_LOGI(TAG, "RX #%lu id=0x%03lX len=%u data=[%s]",
             (unsigned long)g_rx_count, (unsigned long)msg.id, msg.len, hex);
}

void setup() {
    // USB CDC needs a moment before the first logs are visible on the monitor.
    delay(1500);

    ESP_LOGI(TAG, "=== CAN test: TX 0x%03lX @1Mbit, tx=GPIO%d rx=GPIO%d ===",
             (unsigned long)TEST_TX_ID, (int)Pins::CAN_TX, (int)Pins::CAN_RX);

    can_init();
    CanService::start(on_frame, /*with_monitor=*/false);
}

void loop() {
    static uint32_t last_tx = 0;
    static uint32_t last_status = 0;
    static uint32_t seq = 0;

    const uint32_t now = millis();

    if (now - last_tx >= TX_INTERVAL_MS) {
        last_tx = now;

        uint8_t payload[8];
        payload[0] = (uint8_t)(seq & 0xFF);
        payload[1] = (uint8_t)((seq >> 8) & 0xFF);
        payload[2] = (uint8_t)((seq >> 16) & 0xFF);
        payload[3] = (uint8_t)((seq >> 24) & 0xFF);
        payload[4] = (uint8_t)(now & 0xFF);
        payload[5] = (uint8_t)((now >> 8) & 0xFF);
        payload[6] = (uint8_t)((now >> 16) & 0xFF);
        payload[7] = (uint8_t)((now >> 24) & 0xFF);

        // Queued rather than sent directly, so this also exercises the TX task
        // and its bus-off handling.
        const bool queued = CanService::enqueue(TEST_TX_ID, payload, 8);
        if (queued) {
            ++g_tx_count;
            ESP_LOGI(TAG, "TX #%lu id=0x%03lX seq=%lu",
                     (unsigned long)g_tx_count, (unsigned long)TEST_TX_ID,
                     (unsigned long)seq);
        } else {
            ESP_LOGE(TAG, "TX queue full --- bus is not draining");
        }

        ++seq;
    }

    if (now - last_status >= STATUS_INTERVAL_MS) {
        last_status = now;

        twai_status_info_t s;
        if (can_status(s) == ESP_OK) {
            ESP_LOGW(TAG,
                     "state=%d TEC=%lu REC=%lu bus_err=%lu arb_lost=%lu "
                     "tx_failed=%lu queued=%lu | sent=%lu recv=%lu dropped=%lu",
                     (int)s.state,
                     (unsigned long)s.tx_error_counter,
                     (unsigned long)s.rx_error_counter,
                     (unsigned long)s.bus_error_count,
                     (unsigned long)s.arb_lost_count,
                     (unsigned long)s.tx_failed_count,
                     (unsigned long)s.msgs_to_tx,
                     (unsigned long)g_tx_count,
                     (unsigned long)g_rx_count,
                     (unsigned long)CanService::tx_dropped_count());

            if (s.state != TWAI_STATE_RUNNING) {
                ESP_LOGE(TAG, "controller not RUNNING --- recovering");
            }
        }

        // The TX task recovers from bus-off on its own; this covers the case
        // where nothing is being queued at all.
        can_recover_if_needed();
    }

    delay(10);
}
