#include "power_cut.hpp"

#include "pins.hpp"
#include "safety_output.hpp"

namespace {
// GPIO5 is RTC-capable on the C3, so the pad hold is available.
SafetyOutput g_out(Pins::POWER_CUT, /*rtc_hold=*/true);
}  // namespace

namespace PowerCut {

void init() {
    g_out.init();
}

void engage() {
    g_out.set(true);
}

void release() {
    g_out.set(false);
}

bool is_engaged() {
    return g_out.is_active();
}

bool verify() {
    return g_out.verify();
}

uint32_t fault_count() {
    return g_out.fault_count();
}

}  // namespace PowerCut
