#include "can_manager.hpp"
#include "weight_sensor.hpp"
#include <cstring>
#ifdef DEBUG_ENABLED
#include <Arduino.h>
#endif

extern SemaphoreHandle_t weight_mutex;

using namespace CanProtocol;

#ifdef DEBUG_ENABLED
static void log_can_rx(const CanMsg& msg) {
    Serial.print("[CAN RX] id=0x"); Serial.print(msg.id, HEX);
    Serial.print("  len="); Serial.print(msg.len);
    Serial.print("  data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
}
#endif

static void send_weight(float left, float right) {
    uint8_t left_payload[4];
    uint8_t right_payload[4];
    memcpy(left_payload, &left, 4);
    memcpy(right_payload, &right, 4);

    const esp_err_t err_left = can_send(LEFT_WEIGHT_RESP_ID, left_payload, 4);
    const esp_err_t err_right = can_send(RIGHT_WEIGHT_RESP_ID, right_payload, 4);

#ifdef DEBUG_ENABLED
    Serial.print("[CAN] TX left="); Serial.print(left, 3);
    Serial.print(" right="); Serial.print(right, 3);
    Serial.print(" result=");
    Serial.println((err_left == ESP_OK && err_right == ESP_OK) ? "OK" : "FAIL");
#endif
}

static void handle_get_weight() {
    send_weight(
        g_state.weight1_g.load(),
        g_state.weight2_g.load()
    );
}

static void handle_command(const CanMsg& msg) {
#ifdef DEBUG_ENABLED
    log_can_rx(msg);
#endif
    if (msg.id != CMD_ID || msg.len < 1) return;

    switch (msg.data[0]) {
        case CMD_GET_WEIGHT:
            handle_get_weight();
            break;

        case CMD_TARE_LEFT:
            if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                sensor1.tare();
                xSemaphoreGive(weight_mutex);
            }
            break;

        case CMD_TARE_RIGHT:
            if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                sensor2.tare();
                xSemaphoreGive(weight_mutex);
            }
            break;

        default:
#ifdef DEBUG_ENABLED
            Serial.print("[CAN] unknown cmd=0x"); Serial.println(msg.data[0], HEX);
#endif
            break;
    }
}

void can_task(void*) {
    for (;;) {
        CanMsg msg;
        if (can_recv(msg, 20) == ESP_OK) {
            handle_command(msg);
        }
    }
}
