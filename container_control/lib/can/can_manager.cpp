#include "can_manager.hpp"
#include "lid_manager.hpp"
#ifdef DEBUG_ENABLED
#include <Arduino.h>
#endif // DEBUG_ENABLED

using namespace CanProtocol;

#ifdef DEBUG_ENABLED
static void log_can_rx(const CanMsg& msg) {
    Serial.print("[CAN RX] id=0x"); Serial.print(msg.id, HEX);
    Serial.print("  len=");         Serial.print(msg.len);
    Serial.print("  data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
}
#endif // DEBUG_ENABLED

static void send_lid_response(uint8_t cmd, uint8_t status) {
    const uint8_t payload[2] = {cmd, status};
    const esp_err_t err = can_send(LID_RESP_ID, payload, 2);
#ifdef DEBUG_ENABLED
    Serial.print("TX lid cmd=0x");    Serial.print(cmd, HEX);
    Serial.print(" status=0x");       Serial.print(status, HEX);
    Serial.print(" result=");         Serial.println(err == ESP_OK ? "OK" : "FAIL");
#endif // DEBUG_ENABLED
}

// static void send_weight(float value) {
//     uint8_t payload[4];
//     memcpy(payload, &value, 4);
//     const esp_err_t err = can_send(WEIGHT_RESP_ID, payload, 4);
// #ifdef DEBUG_ENABLED
//     Serial.print("TX weight="); Serial.print(value, 3);
//     Serial.print(" result=");   Serial.println(err == ESP_OK ? "OK" : "FAIL");
// #endif // DEBUG_ENABLED
// }

static void send_weight(float w1, float w2, float w3) {
    uint8_t payload1[8];
    memcpy(payload1,     &w1, 4);
    memcpy(payload1 + 4, &w2, 4);
    const esp_err_t err1 = can_send(WEIGHT_RESP_ID_12, payload1, 8);

    uint8_t payload2[4];
    memcpy(payload2, &w3, 4);
    const esp_err_t err2 = can_send(WEIGHT_RESP_ID_3, payload2, 4);

#ifdef DEBUG_ENABLED
    Serial.print("TX w1="); Serial.print(w1, 3);
    Serial.print(" w2=");   Serial.print(w2, 3);
    Serial.print(" w3=");   Serial.print(w3, 3);
    Serial.print(" result=");
    Serial.println((err1 == ESP_OK && err2 == ESP_OK) ? "OK" : "FAIL");
#endif
}

static void handle_get_weight() {
    send_weight(
        g_state.weight1.load(),
        g_state.weight2.load(),
        g_state.weight3.load()
    );
}

static void handle_command(const CanMsg& msg) {
#ifdef DEBUG_ENABLED
    log_can_rx(msg);
#endif // DEBUG_ENABLED
    if (msg.id != CMD_ID || msg.len < 1) return;

    switch (msg.data[0]) {
        case CMD_OPEN:        lid_handle_open(send_lid_response);        break;
        case CMD_CLOSE:       lid_handle_close(send_lid_response);       break;
        case CMD_STOP:        lid_handle_stop(send_lid_response);        break;
        case CMD_POLL_STATUS: lid_handle_poll(send_lid_response);        break;
        case CMD_GET_WEIGHT:  handle_get_weight();                       break;
        default:
#ifdef DEBUG_ENABLED
            Serial.print("CMD: unknown 0x"); Serial.println(msg.data[0], HEX);
#endif // DEBUG_ENABLED
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
