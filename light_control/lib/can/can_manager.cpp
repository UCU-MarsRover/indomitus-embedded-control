#include "can_manager.hpp"
#include "light_command.hpp"
#include "esp_log.h"
#include "Arduino.h"
#include "freertos/queue.h"
#include "power_sensor.hpp"

static const char *TAG = "CAN_LOG";
extern QueueHandle_t light_queue;
extern QueueHandle_t can_tx_queue;

namespace Can = CanProtocol;

static void send_response(uint8_t cmd, uint8_t status) {
    const uint8_t payload[2] = {cmd, status};
    const esp_err_t err = can_send(Can::RESP_ID, payload, 2);

    ESP_LOGD(TAG, "TX cmd=0x%02X status=0x%02X result=%s",
             cmd, status, err == ESP_OK ? "OK" : "FAIL");
}

static void handle_command(const CanMsg& msg) {
    if (msg.id != Can::CMD_ID || msg.len < 1) {
        ESP_LOGW(TAG, "Ignored msg id=0x%03lX len=%d", msg.id, msg.len);
        return;
    }

    if (msg.data[0] == Can::CMD_TELEMETRY_ENABLE) {
        power_telemetry_enabled = true;
        send_response(Can::CMD_TELEMETRY_ENABLE, Can::STATUS_OK);
        return;
    } else if (msg.data[0] == Can::CMD_TELEMETRY_DISABLE) {
        power_telemetry_enabled = false;
        send_response(Can::CMD_TELEMETRY_DISABLE, Can::STATUS_OK);
        return;
    }

    LightCommand lc{};
    bool valid = true;

    switch (msg.data[0]) {
        case Can::CMD_SPOTLIGHT_ON:
            lc = {LightCmd::SPOTLIGHT_ON, 0};
            send_response(Can::CMD_SPOTLIGHT_ON, Can::STATUS_OK);
            break;
        case Can::CMD_SPOTLIGHT_OFF:
            lc = {LightCmd::SPOTLIGHT_OFF, 0};
            send_response(Can::CMD_SPOTLIGHT_OFF, Can::STATUS_OK);
            break;
        case Can::CMD_BEAUTIFUL_LIGHT_ON:
            lc = {LightCmd::BEAUTIFUL_ON, 0};
            send_response(Can::CMD_BEAUTIFUL_LIGHT_ON, Can::STATUS_OK);
            break;
        case Can::CMD_BEAUTIFUL_LIGHT_OFF:
            lc = {LightCmd::BEAUTIFUL_OFF, 0};
            send_response(Can::CMD_BEAUTIFUL_LIGHT_OFF, Can::STATUS_OK);
            break;
        case Can::CMD_TRAFFIC_LIGHT:
            if (msg.len < 2) {
                send_response(Can::CMD_TRAFFIC_LIGHT, Can::STATUS_ERROR);
                valid = false;
            } else {
                lc = {LightCmd::TRAFFIC_MASK, msg.data[1]};
                send_response(Can::CMD_TRAFFIC_LIGHT, Can::STATUS_OK);
            }
            break;

        default:
            send_response(msg.data[0], Can::STATUS_ERROR);
            valid = false;
            break;
    }

    if (valid) {
        xQueueSend(light_queue, &lc, 0);
    }
}


void can_rx_task(void*) {
    for (;;) {
        CanMsg msg;
        if (can_recv(msg, 2000) == ESP_OK) {
            ESP_LOGD(TAG, "RX id=0x%03lX data[0]=0x%02X", msg.id, msg.data[0]);
            handle_command(msg);
        }
    }
}


void can_tx_enqueue(uint32_t id, const uint8_t* data, uint8_t len) {
    CanTxMsg msg;
    msg.id  = id;
    msg.len = len;
    memcpy(msg.data, data, len);
    
    if (xQueueSend(can_tx_queue, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "CAN TX queue full, dropped id=0x%03lX", id);
    }
}


void can_tx_task(void*) {
    CanTxMsg msg;
    for (;;) {
        if (xQueueReceive(can_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {

            
            twai_status_info_t status_info;
            if (twai_get_status_info(&status_info) == ESP_OK) {
                if (status_info.state == TWAI_STATE_BUS_OFF) {
                    ESP_LOGE(TAG, "CAN Bus-Off! Recovering...");
                    twai_initiate_recovery(); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    twai_clear_transmit_queue();
                    xQueueReset(can_tx_queue);
                    continue;
                } 
                else if (status_info.state == TWAI_STATE_STOPPED) {
                    ESP_LOGW(TAG, "Re-starting TWAI driver...");
                    twai_start();
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }

            const esp_err_t err = can_send(msg.id, msg.data, msg.len, 10);

#if DEBUG_ENABLED
            twai_get_status_info(&status_info);
            ESP_LOGD(TAG, "TEC=%lu REC=%lu msgs_to_tx=%lu", 
                    status_info.tx_error_counter, status_info.rx_error_counter, status_info.msgs_to_tx);
            if (err != ESP_OK) {
                UBaseType_t msgs_cnt = uxQueueMessagesWaiting(can_tx_queue);
                ESP_LOGE(TAG, "CAN TX failed id=0x%03lX err=%d len=%u", msg.id, err, (unsigned int)msgs_cnt);
            }
#endif
        }
    }
}

void can_monitor_task(void*) {
    for (;;) {
        twai_status_info_t s;
        if (twai_get_status_info(&s) == ESP_OK) {
            ESP_LOGD("CAN_MON", "state=%d TEC=%lu REC=%lu bus_err=%lu arb_lost=%lu tx_failed=%lu",
                     s.state, s.tx_error_counter, s.rx_error_counter,
                     s.bus_error_count, s.arb_lost_count, s.tx_failed_count);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
