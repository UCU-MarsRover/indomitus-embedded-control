#pragma once
#include <cstdint>

enum class LightCmd : uint8_t {
    SPOTLIGHT_ON,
    SPOTLIGHT_OFF,
    BEAUTIFUL_ON,
    BEAUTIFUL_OFF,
    TRAFFIC_MASK,
};

struct LightCommand {
    LightCmd cmd;
    uint8_t  value; // використовується тільки для TRAFFIC_MASK
};