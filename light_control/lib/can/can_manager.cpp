#include "can_manager.hpp"
#include "light_manager.hpp"

#ifdef DEBUG_ENABLED
#include <Arduino.h>
#endif

using namespace CanProtocol;

#ifdef DEBUG_ENABLED
static void log_can_rx(const CanMsg& msg) {
    Serial.print("[CAN RX] id=0x"); Serial.print(msg.id, HEX);
    Serial.print(" len=");          Serial.print(msg.len);
    Serial.print(" data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
}
#endif

static void send_light_response(uint8_t cmd, uint8_t status) {
    const uint8_t payload[2] = {cmd, status};
    const esp_err_t err = can_send(RESP_ID, payload, 2);
#ifdef DEBUG_ENABLED
    Serial.print("TX light cmd=0x"); Serial.print(cmd, HEX);
    Serial.print(" status=0x");      Serial.print(status, HEX);
    Serial.print(" result=");        Serial.println(err == ESP_OK ? "OK" : "FAIL");
#endif
}

static void handle_command(const CanMsg& msg) {
#ifdef DEBUG_ENABLED
    log_can_rx(msg);
#endif
    if (msg.id != CMD_ID || msg.len < 1) return;

    switch (msg.data[0]) {
        case CMD_SPOTLIGHT_ON:
            light_set_spotlight(true);
            send_light_response(CMD_SPOTLIGHT_ON, STATUS_OK);
            break;

        case CMD_SPOTLIGHT_OFF:
            light_set_spotlight(false);
            send_light_response(CMD_SPOTLIGHT_OFF, STATUS_OK);
            break;

        case CMD_TRAFFIC_LIGHT:
            if (msg.len < 2) {
                send_light_response(CMD_TRAFFIC_LIGHT, STATUS_ERROR);
            } else {
                light_set_traffic_mask(msg.data[1]);
                send_light_response(CMD_TRAFFIC_LIGHT, STATUS_OK);
            }
            break;

        default:
#ifdef DEBUG_ENABLED
            Serial.print("CMD: unknown 0x"); Serial.println(msg.data[0], HEX);
#endif
            send_light_response(msg.data[0], STATUS_ERROR);
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
