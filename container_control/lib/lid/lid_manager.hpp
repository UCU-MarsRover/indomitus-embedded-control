#pragma once
#include <Arduino.h>
#include <cstdint>

using LidResponseFn = void(*)(uint8_t cmd, uint8_t status);

void lid_init();

void lid_handle_open(LidResponseFn respond);
void lid_handle_close(LidResponseFn respond);
void lid_handle_stop(LidResponseFn respond);
void lid_handle_poll(LidResponseFn respond);
void lid_task(void* arg);