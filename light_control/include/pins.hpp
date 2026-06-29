#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/adc.h"

namespace Pins {
    constexpr gpio_num_t CAN_TX = GPIO_NUM_7;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_8;

    constexpr uint8_t SPOTLIGHT = 40;
    constexpr uint8_t BEAUTIFUL = 39;
    constexpr uint8_t RED = 1;
    constexpr uint8_t YELLOW = 2;
    constexpr uint8_t GREEN = 42;
    constexpr uint8_t BLUE = 41;

    constexpr adc_channel_t CURRENT_ADC_CHANNEL_1 = ADC_CHANNEL_9; // GPIO10
    constexpr adc_channel_t CURRENT_ADC_CHANNEL_2 = ADC_CHANNEL_5; // GPIO6
}
