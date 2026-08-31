#pragma once

#include "can_driver.hpp"
#include "freertos/queue.h"

extern uint32_t last_command;

namespace CanProtocol {
    constexpr uint32_t CMD_ID      = 0x300;
    constexpr uint32_t RESP_ID     = 0x301;
    constexpr uint32_t TELEMETRY_ID = 0x302;

    constexpr uint8_t CMD_SPOTLIGHT_ON  = 0x01;
    constexpr uint8_t CMD_SPOTLIGHT_OFF = 0x02;
    constexpr uint8_t CMD_TRAFFIC_LIGHT = 0x03;
    constexpr uint8_t CMD_BEAUTIFUL_LIGHT_ON  = 0x04;
    constexpr uint8_t CMD_BEAUTIFUL_LIGHT_OFF = 0x05;
    constexpr uint8_t CMD_RED_LIGHT_ON   = 0x06;
    constexpr uint8_t CMD_RED_LIGHT_OFF  = 0x07;
    constexpr uint8_t CMD_GREEN_LIGHT_ON = 0x08;
    constexpr uint8_t CMD_GREEN_LIGHT_OFF = 0x09;
    constexpr uint8_t CMD_BLUE_LIGHT_ON  = 0x0A;
    constexpr uint8_t CMD_BLUE_LIGHT_OFF = 0x0B;
    constexpr uint8_t CMD_BUZZER_ON      = 0x0C;
    constexpr uint8_t CMD_BUZZER_OFF     = 0x0D;
    constexpr uint8_t CMD_SPOTLIGHT_LEFT_ON   = 0x20;
    constexpr uint8_t CMD_SPOTLIGHT_LEFT_OFF  = 0x21;
    constexpr uint8_t CMD_SPOTLIGHT_RIGHT_ON  = 0x22;
    constexpr uint8_t CMD_SPOTLIGHT_RIGHT_OFF = 0x23;
    constexpr uint8_t CMD_BEAUTIFUL_1_ON      = 0x24;
    constexpr uint8_t CMD_BEAUTIFUL_1_OFF     = 0x25;
    constexpr uint8_t CMD_BEAUTIFUL_2_ON      = 0x26;
    constexpr uint8_t CMD_BEAUTIFUL_2_OFF     = 0x27;
    constexpr uint8_t CMD_BEAUTIFUL_3_ON      = 0x28;
    constexpr uint8_t CMD_BEAUTIFUL_3_OFF     = 0x29;
    constexpr uint8_t CMD_BEAUTIFUL_4_ON      = 0x2A;
    constexpr uint8_t CMD_BEAUTIFUL_4_OFF     = 0x2B;
    constexpr uint8_t CMD_TOWER_ON            = 0x2C;
    constexpr uint8_t CMD_TOWER_OFF           = 0x2D;

    constexpr uint8_t CMD_TELEMETRY_ENABLE = 0x10;
    constexpr uint8_t CMD_TELEMETRY_DISABLE = 0x11;

    constexpr uint8_t STATUS_OK    = 0x00;
    constexpr uint8_t STATUS_ERROR = 0x01;
}

struct CanTxMsg {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};

void can_rx_task(void*);

extern QueueHandle_t can_tx_queue;
void can_tx_enqueue(uint32_t id, const uint8_t* data, uint8_t len);
void can_tx_task(void*);
void can_monitor_task(void*);
