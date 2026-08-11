#include "jetson_can_cut.hpp"

#include "pins.hpp"
#include "safety_output.hpp"

namespace {
// GPIO4 is RTC-capable on the C3, so the pad hold is available.
SafetyOutput g_out(Pins::JETSON_CAN_CUT, /*rtc_hold=*/true);
}  // namespace

namespace JetsonCanCut {

void init() {
    g_out.init();
}

void isolate() {
    g_out.set(true);
}

void connect() {
    g_out.set(false);
}

bool is_isolated() {
    return g_out.is_active();
}

bool verify() {
    return g_out.verify();
}

uint32_t fault_count() {
    return g_out.fault_count();
}

}  // namespace JetsonCanCut
