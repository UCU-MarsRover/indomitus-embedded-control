#include "can_service.hpp"

#include "esp_log.h"
#include "freertos/task.h"

#include <string.h>

static const char* TAG = "CAN_SVC";

namespace {

struct CanTxMsg {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
};

QueueHandle_t          g_tx_queue = nullptr;
CanService::RxHandler  g_handler  = nullptr;
volatile uint32_t      g_dropped  = 0;

void rx_task(void*) {
    for (;;) {
        CanMsg msg;
        if (can_recv(msg, 1000) != ESP_OK) {
            continue;
        }

        ESP_LOGD(TAG, "RX id=0x%03lX len=%u", (unsigned long)msg.id, msg.len);

        if (g_handler != nullptr) {
            g_handler(msg);
        }
    }
}

void tx_task(void*) {
    CanTxMsg msg;
    for (;;) {
        if (xQueueReceive(g_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // A bus-off controller silently swallows everything, so clear the
        // backlog rather than transmitting stale frames once it recovers.
        twai_status_info_t s;
        if (can_status(s) == ESP_OK && s.state == TWAI_STATE_BUS_OFF) {
            ESP_LOGE(TAG, "bus-off, recovering and flushing TX");
            can_recover_if_needed();
            vTaskDelay(pdMS_TO_TICKS(200));
            twai_clear_transmit_queue();
            xQueueReset(g_tx_queue);
            continue;
        }
        if (s.state == TWAI_STATE_STOPPED) {
            can_recover_if_needed();
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        const esp_err_t err = can_send(msg.id, msg.data, msg.len, 10);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TX failed id=0x%03lX err=%d",
                     (unsigned long)msg.id, (int)err);
        }
    }
}

void monitor_task(void*) {
    for (;;) {
        twai_status_info_t s;
        if (can_status(s) == ESP_OK) {
            ESP_LOGD(TAG,
                     "state=%d TEC=%lu REC=%lu bus_err=%lu arb_lost=%lu "
                     "tx_failed=%lu dropped=%lu",
                     (int)s.state,
                     (unsigned long)s.tx_error_counter,
                     (unsigned long)s.rx_error_counter,
                     (unsigned long)s.bus_error_count,
                     (unsigned long)s.arb_lost_count,
                     (unsigned long)s.tx_failed_count,
                     (unsigned long)g_dropped);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

}  // namespace

namespace CanService {

bool start(RxHandler handler, bool with_monitor) {
    g_handler = handler;

    if (g_tx_queue == nullptr) {
        g_tx_queue = xQueueCreate(TX_QUEUE_LEN, sizeof(CanTxMsg));
        if (g_tx_queue == nullptr) {
            ESP_LOGE(TAG, "TX queue alloc failed");
            return false;
        }
    }

    if (xTaskCreate(rx_task, "can_rx", 3072, nullptr, 6, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "rx task create failed");
        return false;
    }
    if (xTaskCreate(tx_task, "can_tx", 3072, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "tx task create failed");
        return false;
    }
    if (with_monitor) {
        xTaskCreate(monitor_task, "can_mon", 2560, nullptr, 2, nullptr);
    }

    ESP_LOGI(TAG, "tasks started");
    return true;
}

bool enqueue(uint32_t id, const uint8_t* data, uint8_t len) {
    if (g_tx_queue == nullptr) {
        return false;
    }
    if (len > 8) {
        len = 8;
    }

    CanTxMsg msg = {};
    msg.id  = id;
    msg.len = len;
    memcpy(msg.data, data, len);

    if (xQueueSend(g_tx_queue, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        ++g_dropped;
        ESP_LOGW(TAG, "TX queue full, dropped id=0x%03lX", (unsigned long)id);
        return false;
    }
    return true;
}

uint32_t tx_dropped_count() {
    return g_dropped;
}

}  // namespace CanService
