#include "can_manager.hpp"
#include "light_manager.hpp"
#include "esp_log.h"
#include "Arduino.h"


uint32_t last_command = 0;
static const char *TAG = "CAN_LOG";


using namespace CanProtocol;


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
    if (msg.id != CMD_ID || msg.len < 1) {
        Serial.print("[CAN] Ignored msg id=0x"); Serial.print(msg.id, HEX);
        Serial.print(" len="); Serial.println(msg.len);
        return;
    }

    Serial.print("[CAN RX] id=0x"); Serial.print(msg.id, HEX);
    Serial.print(" data[0]=0x");    Serial.println(msg.data[0], HEX);

    switch (msg.data[0]) {
        case CMD_SPOTLIGHT_ON:
            last_command = 1;
            light_set_spotlight(true);
            send_light_response(CMD_SPOTLIGHT_ON, STATUS_OK);
            break;

        case CMD_SPOTLIGHT_OFF:
            last_command = 2;
            light_set_spotlight(false);
            send_light_response(CMD_SPOTLIGHT_OFF, STATUS_OK);
            break;

        case CMD_BEAUTIFUL_LIGHT_ON:
            last_command = 3;
            light_set_beautiful(true);
            send_light_response(CMD_BEAUTIFUL_LIGHT_ON, STATUS_OK);
            break;

        case CMD_BEAUTIFUL_LIGHT_OFF:
            last_command = 4;
            light_set_beautiful(false);
            send_light_response(CMD_BEAUTIFUL_LIGHT_OFF, STATUS_OK);
            break;

        case CMD_TRAFFIC_LIGHT:
            last_command = 5;
            if (msg.len < 2) {
                send_light_response(CMD_TRAFFIC_LIGHT, STATUS_ERROR);
            } else {
                light_set_traffic_mask(msg.data[1]);
                send_light_response(CMD_TRAFFIC_LIGHT, STATUS_OK);
            }
            break;

        default:
            last_command = 6;
            send_light_response(msg.data[0], STATUS_ERROR);
            break;
    }
}

void can_task(void*) {
    uint32_t loop_counter = 0;
    for (;;) {
        CanMsg msg;
        if (can_recv(msg, 2000) == ESP_OK) {
            handle_command(msg);
        }

        Serial.print("[CAN] last_command=");
        Serial.print(last_command);
        Serial.print(" (");
        Serial.print(last_command);
        Serial.println(")");
    }
}
