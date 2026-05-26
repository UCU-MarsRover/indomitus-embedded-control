#pragma once

#include <Arduino.h>
#include <driver/twai.h>

struct CanMsg {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t data[8] = {0};
};

void can_init_1mbs_accept_all();
esp_err_t can_send(uint32_t id, const uint8_t* data, uint8_t len);
esp_err_t can_recv(CanMsg& out, uint32_t timeout_ms);
