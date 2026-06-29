#pragma once

#include "can_driver.hpp"
#include "freertos/queue.h"

extern uint32_t last_command;

namespace CanProtocol {
    constexpr uint32_t CMD_ID      = 0x300;
    constexpr uint32_t RESP_ID     = 0x301;
    constexpr uint32_t TELEMETRY_CURRENT_ID = 0x302;

    constexpr uint8_t CMD_SPOTLIGHT_ON  = 0x01;
    constexpr uint8_t CMD_SPOTLIGHT_OFF = 0x02;
    constexpr uint8_t CMD_TRAFFIC_LIGHT = 0x03;
    constexpr uint8_t CMD_BEAUTIFUL_LIGHT_ON  = 0x04;
    constexpr uint8_t CMD_BEAUTIFUL_LIGHT_OFF = 0x05;

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
