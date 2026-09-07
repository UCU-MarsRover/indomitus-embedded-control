#pragma once

#include <stdint.h>
extern "C" {
#include "driver/gpio.h"
}

namespace Pins {
    // match light_control layout where possible
    constexpr gpio_num_t CAN_TX = GPIO_NUM_20;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_21;

    // simple LED pin for jaw sampling gripper
    constexpr uint8_t LED = 8; // use GPIO 8 as requested
}
