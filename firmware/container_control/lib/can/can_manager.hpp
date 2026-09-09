#pragma once
#include "can_driver.hpp"
#include "shared_state.hpp"

namespace CanProtocol {
    constexpr uint32_t CMD_ID             = 0x320;
    constexpr uint32_t LEFT_WEIGHT_RESP_ID = 0x322;
    constexpr uint32_t RIGHT_WEIGHT_RESP_ID = 0x323;

    constexpr uint8_t CMD_GET_WEIGHT = 0x10;
    constexpr uint8_t CMD_TARE_LEFT  = 0x11;
    constexpr uint8_t CMD_TARE_RIGHT = 0x12;
}

static void log_can_rx(const CanMsg& msg);
static void send_weight(float left, float right);
static void handle_get_weight();
static void handle_command(const CanMsg& msg);

void can_task(void*);
