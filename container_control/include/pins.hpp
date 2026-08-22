#pragma once
#include "driver/gpio.h"

// ESP32-C3 Super Mini pin map for the active two HX711 load cells.

// CAN (TWAI) - kept for library functionality, not initialized in main()
static constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_7;
static constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_10;

// I2C bus for IMU
static constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_4;
static constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_3;
static constexpr gpio_num_t PIN_IMU_SCL = GPIO_NUM_4;
static constexpr gpio_num_t PIN_IMU_SDA = GPIO_NUM_3;

// TM1637 display moved to free UART pins RX/TX: GPIO21 / GPIO20
static constexpr gpio_num_t PIN_DISPLAY_CLK = GPIO_NUM_21;
static constexpr gpio_num_t PIN_DISPLAY_DIO = GPIO_NUM_20;

// HX711 Left sensor
static constexpr gpio_num_t PIN_HX711_LEFT_DT  = GPIO_NUM_1;
static constexpr gpio_num_t PIN_HX711_LEFT_SCK = GPIO_NUM_0;

// HX711 Right sensor
static constexpr gpio_num_t PIN_HX711_RIGHT_DT  = GPIO_NUM_6;
static constexpr gpio_num_t PIN_HX711_RIGHT_SCK = GPIO_NUM_5;

// Legacy / unused pins intentionally removed from the active build.
