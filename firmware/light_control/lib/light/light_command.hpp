#pragma once
#include <cstdint>

enum class LightCmd : uint8_t {
    SPOTLIGHT_ON,
    SPOTLIGHT_OFF,
    SPOTLIGHT_LEFT_ON,
    SPOTLIGHT_LEFT_OFF,
    SPOTLIGHT_RIGHT_ON,
    SPOTLIGHT_RIGHT_OFF,
    BEAUTIFUL_ON,
    BEAUTIFUL_OFF,
    BEAUTIFUL_1_ON,
    BEAUTIFUL_1_OFF,
    BEAUTIFUL_2_ON,
    BEAUTIFUL_2_OFF,
    BEAUTIFUL_3_ON,
    BEAUTIFUL_3_OFF,
    BEAUTIFUL_4_ON,
    BEAUTIFUL_4_OFF,
    RED_ON,
    RED_OFF,
    GREEN_ON,
    GREEN_OFF,
    BLUE_ON,
    BLUE_OFF,
    BUZZER_ON,
    BUZZER_OFF,
    TOWER_ON,
    TOWER_OFF,
    TRAFFIC_MASK,
};

struct LightCommand {
    LightCmd cmd;
    uint8_t  value;
};