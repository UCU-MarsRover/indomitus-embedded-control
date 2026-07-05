#pragma once
#include "driver/gpio.h"



// SERVO MOTORS
static constexpr uint8_t PIN_SERVO_RIGHT_FAR   = 1;
static constexpr uint8_t PIN_SERVO_RIGHT_NEAR  = 2;
static constexpr uint8_t PIN_SERVO_LEFT_FAR    = 42;
static constexpr uint8_t PIN_SERVO_LEFT_NEAR   = 41;

// CAN (TWAI)
static constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_37;
static constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_38;

// HX711 #1 (Left)
static constexpr gpio_num_t PIN_HX711_1_DT  = GPIO_NUM_12;
static constexpr gpio_num_t PIN_HX711_1_SCK = GPIO_NUM_11;

// HX711 #2 (Center)
static constexpr gpio_num_t PIN_HX711_2_DT  = GPIO_NUM_14;
static constexpr gpio_num_t PIN_HX711_2_SCK = GPIO_NUM_13;

// HX711 #3 (Right)
static constexpr gpio_num_t PIN_HX711_3_DT  = GPIO_NUM_10;
static constexpr gpio_num_t PIN_HX711_3_SCK = GPIO_NUM_9;

static constexpr gpio_num_t PIN_6DGT_SCK = GPIO_NUM_19;
static constexpr gpio_num_t PIN_6DGT_DIO = GPIO_NUM_20;
