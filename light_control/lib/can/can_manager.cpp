#include "can_manager.hpp"
#include "light_command.hpp"
#include "esp_log.h"
#include "Arduino.h"
#include "freertos/queue.h"

static const char *TAG = "CAN_LOG";
extern QueueHandle_t light_queue;

namespace Can = CanProtocol;

static void send_light_response(uint8_t cmd, uint8_t status) {
    const uint8_t payload[2] = {cmd, status};
    const esp_err_t err = can_send(Can::RESP_ID, payload, 2);
#ifdef DEBUG_ENABLED
    ESP_LOGI(TAG, "TX cmd=0x%02X status=0x%02X result=%s",
             cmd, status, err == ESP_OK ? "OK" : "FAIL");
#endif
}

static void handle_command(const CanMsg& msg) {
    if (msg.id != Can::CMD_ID || msg.len < 1) {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Ignored msg id=0x%03lX len=%d", msg.id, msg.len);
#endif
        return;
    }

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "RX id=0x%03lX data[0]=0x%02X", msg.id, msg.data[0]);
#endif
    LightCommand lc{};
    bool valid = true;

    switch (msg.data[0]) {
        case Can::CMD_SPOTLIGHT_ON:
            lc = {LightCmd::SPOTLIGHT_ON, 0};
            send_light_response(Can::CMD_SPOTLIGHT_ON, Can::STATUS_OK);
            break;
        case Can::CMD_SPOTLIGHT_OFF:
            lc = {LightCmd::SPOTLIGHT_OFF, 0};
            send_light_response(Can::CMD_SPOTLIGHT_OFF, Can::STATUS_OK);
            break;
        case Can::CMD_BEAUTIFUL_LIGHT_ON:
            lc = {LightCmd::BEAUTIFUL_ON, 0};
            send_light_response(Can::CMD_BEAUTIFUL_LIGHT_ON, Can::STATUS_OK);
            break;
        case Can::CMD_BEAUTIFUL_LIGHT_OFF:
            lc = {LightCmd::BEAUTIFUL_OFF, 0};
            send_light_response(Can::CMD_BEAUTIFUL_LIGHT_OFF, Can::STATUS_OK);
            break;
        case Can::CMD_TRAFFIC_LIGHT:
            if (msg.len < 2) {
                send_light_response(Can::CMD_TRAFFIC_LIGHT, Can::STATUS_ERROR);
                valid = false;
            } else {
                lc = {LightCmd::TRAFFIC_MASK, msg.data[1]};
                send_light_response(Can::CMD_TRAFFIC_LIGHT, Can::STATUS_OK);
            }
            break;
        default:
            send_light_response(msg.data[0], Can::STATUS_ERROR);
            valid = false;
            break;
    }

    if (valid) {
        xQueueSend(light_queue, &lc, 0);
    }
}

uint32_t last_command = 0;

void can_rx_task(void*) {
    for (;;) {
        CanMsg msg;
        if (can_recv(msg, 2000) == ESP_OK) {
            handle_command(msg);
        }
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "last_command=%lu", last_command);
#endif
    }
}


void can_tx_enqueue(uint32_t id, const uint8_t* data, uint8_t len) {
    CanTxMsg msg;
    msg.id  = id;
    msg.len = len;
    memcpy(msg.data, data, len);
    
    if (xQueueSend(can_tx_queue, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "CAN TX queue full, dropped id=0x%03lX", id);
#endif
    }
}


void can_tx_task(void*) {
    CanTxMsg msg;
    for (;;) {
        if (xQueueReceive(can_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            const esp_err_t err = can_send(msg.id, msg.data, msg.len);
#if DEBUG_ENABLED
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "CAN TX failed id=0x%03lX err=%d", msg.id, err);
            }
#endif
        }
    }
}
