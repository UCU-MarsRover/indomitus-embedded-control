#include "can_manager.hpp"
#include "light_manager.hpp"

#ifdef DEBUG_ENABLED
#include <Arduino.h>
#define DEBUG_PORT Serial0
#endif

using namespace CanProtocol;

#ifdef DEBUG_ENABLED
static void log_can_rx(const CanMsg& msg) {
    DEBUG_PORT.print("[CAN RX] id=0x"); DEBUG_PORT.print(msg.id, HEX);
    DEBUG_PORT.print(" len=");          DEBUG_PORT.print(msg.len);
    DEBUG_PORT.print(" data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) DEBUG_PORT.print("0");
        DEBUG_PORT.print(msg.data[i], HEX);
        DEBUG_PORT.print(" ");
    }
    DEBUG_PORT.println("]");
}
#endif

#ifdef DEBUG_ENABLED
static void log_alerts(uint32_t alerts) {
    if (alerts == 0) {
        return;
    }

    DEBUG_PORT.print("[TWAI ALERTS] 0x");
    DEBUG_PORT.println(alerts, HEX);

    if (alerts & TWAI_ALERT_RX_DATA) {
        DEBUG_PORT.println("  - RX_DATA");
    }
    if (alerts & TWAI_ALERT_TX_IDLE) {
        DEBUG_PORT.println("  - TX_IDLE");
    }
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
        DEBUG_PORT.println("  - TX_SUCCESS");
    }
    if (alerts & TWAI_ALERT_TX_FAILED) {
        DEBUG_PORT.println("  - TX_FAILED");
    }
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
        DEBUG_PORT.println("  - RX_QUEUE_FULL");
    }
    if (alerts & TWAI_ALERT_BUS_ERROR) {
        DEBUG_PORT.println("  - BUS_ERROR");
    }
    if (alerts & TWAI_ALERT_ERR_PASS) {
        DEBUG_PORT.println("  - ERR_PASS");
    }
    if (alerts & TWAI_ALERT_BUS_OFF) {
        DEBUG_PORT.println("  - BUS_OFF");
    }
}
#endif

#ifdef DEBUG_ENABLED
static void log_status_info() {
    twai_status_info_t status = {};
    if (twai_get_status_info(&status) != ESP_OK) {
        DEBUG_PORT.println("[TWAI STATUS] failed to read status");
        return;
    }

    DEBUG_PORT.print("[TWAI STATUS] state=");
    switch (status.state) {
        case TWAI_STATE_STOPPED:
            DEBUG_PORT.print("STOPPED");
            break;
        case TWAI_STATE_RUNNING:
            DEBUG_PORT.print("RUNNING");
            break;
        case TWAI_STATE_BUS_OFF:
            DEBUG_PORT.print("BUS_OFF");
            break;
        case TWAI_STATE_RECOVERING:
            DEBUG_PORT.print("RECOVERING");
            break;
        default:
            DEBUG_PORT.print("UNKNOWN");
            break;
    }

    DEBUG_PORT.print(" tx_err=");
    DEBUG_PORT.print(status.tx_error_counter);
    DEBUG_PORT.print(" rx_err=");
    DEBUG_PORT.print(status.rx_error_counter);
    DEBUG_PORT.print(" msgs_to_tx=");
    DEBUG_PORT.print(status.msgs_to_tx);
    DEBUG_PORT.print(" msgs_to_rx=");
    DEBUG_PORT.print(status.msgs_to_rx);
    DEBUG_PORT.print(" tx_failed=");
    DEBUG_PORT.print(status.tx_failed_count);
    DEBUG_PORT.print(" rx_missed=");
    DEBUG_PORT.print(status.rx_missed_count);
    DEBUG_PORT.print(" rx_overrun=");
    DEBUG_PORT.print(status.rx_overrun_count);
    DEBUG_PORT.print(" arb_lost=");
    DEBUG_PORT.print(status.arb_lost_count);
    DEBUG_PORT.print(" bus_err=");
    DEBUG_PORT.println(status.bus_error_count);
}
#endif

static void send_light_response(uint8_t cmd, uint8_t status) {
    const uint8_t payload[2] = {cmd, status};
    const esp_err_t err = can_send(RESP_ID, payload, 2);
#ifdef DEBUG_ENABLED
    DEBUG_PORT.print("TX light cmd=0x"); DEBUG_PORT.print(cmd, HEX);
    DEBUG_PORT.print(" status=0x");      DEBUG_PORT.print(status, HEX);
    DEBUG_PORT.print(" result=");        DEBUG_PORT.println(err == ESP_OK ? "OK" : "FAIL");
#endif
}

#ifdef DEBUG_ENABLED
static void log_traffic_mask(uint8_t mask) {
    DEBUG_PORT.print("TRAFFIC mask=0x"); DEBUG_PORT.print(mask, HEX);
    DEBUG_PORT.print(" R="); DEBUG_PORT.print((mask & 0x01) ? 1 : 0);
    DEBUG_PORT.print(" Y="); DEBUG_PORT.print((mask & 0x02) ? 1 : 0);
    DEBUG_PORT.print(" G="); DEBUG_PORT.print((mask & 0x04) ? 1 : 0);
    DEBUG_PORT.print(" B="); DEBUG_PORT.println((mask & 0x08) ? 1 : 0);
}
#endif

static void handle_command(const CanMsg& msg) {
#ifdef DEBUG_ENABLED
    log_can_rx(msg);
#endif
    if (msg.id != CMD_ID || msg.len < 1) return;

    switch (msg.data[0]) {
        case CMD_SPOTLIGHT_ON:
#ifdef DEBUG_ENABLED
            DEBUG_PORT.println("CMD_SPOTLIGHT_ON");
#endif
            light_set_spotlight(true);
            send_light_response(CMD_SPOTLIGHT_ON, STATUS_OK);
            break;

        case CMD_SPOTLIGHT_OFF:
#ifdef DEBUG_ENABLED
            DEBUG_PORT.println("CMD_SPOTLIGHT_OFF");
#endif
            light_set_spotlight(false);
            send_light_response(CMD_SPOTLIGHT_OFF, STATUS_OK);
            break;

        case CMD_BEAUTIFUL_LIGHT_ON:
#ifdef DEBUG_ENABLED
            DEBUG_PORT.println("CMD_BEAUTIFUL_LIGHT_ON");
#endif
            light_set_beautiful(true);
            send_light_response(CMD_BEAUTIFUL_LIGHT_ON, STATUS_OK);
            break;

        case CMD_BEAUTIFUL_LIGHT_OFF:
#ifdef DEBUG_ENABLED
            DEBUG_PORT.println("CMD_BEAUTIFUL_LIGHT_OFF");
#endif
            light_set_beautiful(false);
            send_light_response(CMD_BEAUTIFUL_LIGHT_OFF, STATUS_OK);
            break;

        case CMD_TRAFFIC_LIGHT:
            if (msg.len < 2) {
#ifdef DEBUG_ENABLED
                DEBUG_PORT.println("CMD_TRAFFIC_LIGHT invalid payload");
#endif
                send_light_response(CMD_TRAFFIC_LIGHT, STATUS_ERROR);
            } else {
#ifdef DEBUG_ENABLED
                DEBUG_PORT.println("CMD_TRAFFIC_LIGHT");
                log_traffic_mask(msg.data[1]);
#endif
                light_set_traffic_mask(msg.data[1]);
                send_light_response(CMD_TRAFFIC_LIGHT, STATUS_OK);
            }
            break;

        default:
#ifdef DEBUG_ENABLED
            DEBUG_PORT.print("CMD: unknown 0x"); DEBUG_PORT.println(msg.data[0], HEX);
#endif
            send_light_response(msg.data[0], STATUS_ERROR);
            break;
    }
}

void can_task(void*) {
    uint32_t loop_counter = 0;
    for (;;) {
#ifdef DEBUG_ENABLED
        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            log_alerts(alerts);
        }

        if ((loop_counter++ % 100) == 0) {
            log_status_info();
        }
#endif
        CanMsg msg;
        if (can_recv(msg, 20) == ESP_OK) {
            handle_command(msg);
        }
    }
}
