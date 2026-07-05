#pragma once

#include <atomic>

struct SharedState {
    std::atomic<float> weight1{0.0f};
    std::atomic<float> weight2{0.0f};
    std::atomic<float> weight3{0.0f};
};

extern SharedState g_state;