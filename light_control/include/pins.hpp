#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/adc.h"

namespace Pins {
    constexpr gpio_num_t CAN_TX = GPIO_NUM_17;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_18;

    constexpr uint8_t SPOTLIGHT = 40;
    constexpr uint8_t BEAUTIFUL = 39;
    constexpr uint8_t RED = 1;
    constexpr uint8_t YELLOW = 2;
    constexpr uint8_t GREEN = 42;
    constexpr uint8_t BLUE = 41;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
}
