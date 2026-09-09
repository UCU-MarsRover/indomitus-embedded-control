#pragma once

extern "C" {
#include "driver/gpio.h"
#include "driver/twai.h"
}

struct CanMsg {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
};

void can_init();
esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len, uint32_t timeout_ms = 200);
esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms = 200);
