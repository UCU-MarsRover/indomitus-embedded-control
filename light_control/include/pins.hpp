#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/adc.h"

namespace Pins {
    constexpr gpio_num_t CAN_TX = GPIO_NUM_15;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_16;

    constexpr gpio_num_t SPOTLIGHT_LEFT = GPIO_NUM_4;
    constexpr gpio_num_t SPOTLIGHT_RIGHT = GPIO_NUM_5;

    constexpr gpio_num_t BEAUTIFUL_1 = GPIO_NUM_39;
    constexpr gpio_num_t BEAUTIFUL_2 = GPIO_NUM_38;
    constexpr gpio_num_t BEAUTIFUL_3 = GPIO_NUM_10;
    constexpr gpio_num_t BEAUTIFUL_4 = GPIO_NUM_48;

    constexpr gpio_num_t RED = GPIO_NUM_2;
    constexpr gpio_num_t GREEN = GPIO_NUM_42;
    constexpr gpio_num_t BLUE = GPIO_NUM_41;
    constexpr gpio_num_t BUZZER = GPIO_NUM_40;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
}
