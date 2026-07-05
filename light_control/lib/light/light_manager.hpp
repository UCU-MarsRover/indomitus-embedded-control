#pragma once

#include <stdint.h>

void light_init();
void light_set_spotlight(bool enabled);
void light_set_beautiful(bool enabled);
void light_set_traffic_mask(uint8_t mask);
