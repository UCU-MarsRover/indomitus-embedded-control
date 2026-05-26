#pragma once

#include "can_driver.hpp"

namespace CanProtocol {
    constexpr uint32_t CMD_ID      = 0x300;
    constexpr uint32_t RESP_ID     = 0x301;

    constexpr uint8_t CMD_SPOTLIGHT_ON  = 0x01;
    constexpr uint8_t CMD_SPOTLIGHT_OFF = 0x02;
    constexpr uint8_t CMD_TRAFFIC_LIGHT = 0x03;

    constexpr uint8_t STATUS_OK    = 0x00;
    constexpr uint8_t STATUS_ERROR = 0x01;
}

void can_task(void*);
