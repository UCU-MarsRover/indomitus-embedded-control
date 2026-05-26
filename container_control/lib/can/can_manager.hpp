#pragma once
#include "can_driver.hpp"
#include "shared_state.hpp"
#include "can_manager.hpp"
#include "lid_manager.hpp"

namespace CanProtocol {
    constexpr uint32_t CMD_ID         = 0x200;
    constexpr uint32_t LID_RESP_ID    = 0x201;
    constexpr uint32_t WEIGHT_RESP_ID_12= 0x202;  // w1 + w2
    constexpr uint32_t WEIGHT_RESP_ID_3 = 0x203;  // w3

    constexpr uint8_t CMD_OPEN        = 0x01;
    constexpr uint8_t CMD_CLOSE       = 0x02;
    constexpr uint8_t CMD_STOP        = 0x08;
    constexpr uint8_t CMD_POLL_STATUS = 0x04;
    constexpr uint8_t CMD_GET_WEIGHT  = 0x10;

    constexpr uint8_t STATUS_ACK         = 0x00;
    constexpr uint8_t STATUS_IN_PROGRESS = 0x01;
    constexpr uint8_t STATUS_DONE        = 0x02;
    constexpr uint8_t STATUS_ERROR       = 0x03;
}


static void log_can_rx(const CanMsg& msg);

static void send_lid_response(uint8_t cmd, uint8_t status);
static void send_weight(float value);

static void handle_get_weight();
static void handle_command(const CanMsg& msg);

void can_task(void*);
