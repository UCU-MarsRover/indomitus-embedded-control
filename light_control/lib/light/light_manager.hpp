#pragma once
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "light_command.hpp"

extern QueueHandle_t light_queue;

void light_init();
void light_set_spotlight(bool enabled);
void light_set_spotlight_left(bool enabled);
void light_set_spotlight_right(bool enabled);
void light_set_beautiful(bool enabled);
void light_set_beautiful_1(bool enabled);
void light_set_beautiful_2(bool enabled);
void light_set_beautiful_3(bool enabled);
void light_set_beautiful_4(bool enabled);
void light_set_buzzer(bool enabled);
void light_set_red(bool enabled);
void light_set_green(bool enabled);
void light_set_blue(bool enabled);
void light_set_traffic_mask(uint8_t mask);

void light_task(void*);
