#pragma once
#include "driver/gpio.h"

// SERVO
static constexpr uint8_t PIN_SERVO = 10;

// CAN (TWAI)
static constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_37;
static constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_38;

// HX711 #1
static constexpr gpio_num_t PIN_HX711_1_DT  = GPIO_NUM_5;
static constexpr gpio_num_t PIN_HX711_1_SCK = GPIO_NUM_4;

// HX711 #2
static constexpr gpio_num_t PIN_HX711_2_DT  = GPIO_NUM_15;
static constexpr gpio_num_t PIN_HX711_2_SCK = GPIO_NUM_7;

// HX711 #3
static constexpr gpio_num_t PIN_HX711_3_DT  = GPIO_NUM_18;
static constexpr gpio_num_t PIN_HX711_3_SCK = GPIO_NUM_17;
