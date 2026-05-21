#pragma once

#include <atomic>

struct SharedState {
    std::atomic<float> weight{0.0f};

};

extern SharedState g_state;