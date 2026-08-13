/**
 * @file main.cpp
 * @brief Emergency (E-stop) node --- bring-up skeleton plus the radio e-stop
 *        policy.
 *
 * Bring-up responsibilities (unconditional, not policy):
 *
 *   1. Bring-up ORDER. The two transistor-gate lines are parked LOW before any
 *      other peripheral is touched, which keeps the window where those pads are
 *      undriven as short as the chip allows.
 *   2. The periodic verify() calls that let SafetyOutput repair a corrupted pad.
 *
 * Radio e-stop policy (see RadioProto below and README.md "Radio protocol"):
 *
 *   - Ground station sends a single 0x01 heartbeat byte every 2 s.
 *   - The rover is stopped (PowerCut::engage()) whenever any of these is true,
 *     re-evaluated every loop iteration:
 *       - no valid heartbeat for RadioProto::HEARTBEAT_TIMEOUT_MS
 *       - the last radio command byte received was 0x00 (stop)
 *       - the local e-stop button is currently held down
 *   - Not latched: as soon as none of those are true --- heartbeats resumed,
 *     a heartbeat byte arrived after a stop byte, and the local button is
 *     released --- power is restored automatically. No separate rearm step.
 *
 * CAN protocol is still unimplemented (TODO below) --- out of scope for the
 * radio e-stop feature this file currently wires up.
 */

#include <Arduino.h>

#include "can_driver.hpp"
#include "estop_button.hpp"
#include "jetson_can_cut.hpp"
#include "jetson_reset.hpp"
#include "power_cut.hpp"
#include "radio_link.hpp"

#include "esp_log.h"

static const char* TAG = "MAIN";

/// How often the gate lines are re-asserted and checked.
static constexpr uint32_t VERIFY_INTERVAL_MS = 50;
/// How often the CAN controller is checked for bus-off.
static constexpr uint32_t CAN_HEALTH_INTERVAL_MS = 250;

/// Payload bytes the ground station is expected to send over RadioLink.
namespace RadioProto {
constexpr uint8_t HEARTBEAT = 0x01;
constexpr uint8_t STOP      = 0x00;

/// No heartbeat this long (from boot, or from the last one seen) stops the
/// rover. Matches the spec: >4 s of silence stops the rover.
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 4000;
}  // namespace RadioProto

/// Last time a valid 0x01 heartbeat frame was seen. Seeded at boot so the
/// rover requires a live link within HEARTBEAT_TIMEOUT_MS of power-up, not
/// just after the first frame ever arrives.
static uint32_t g_last_heartbeat_ms = 0;
/// True from the moment a 0x00 stop frame arrives until the next 0x01
/// heartbeat frame clears it. Not a latch --- a resumed heartbeat is enough.
static bool g_radio_stop = false;
/// Tracks the previous loop's combined stop state, purely so transitions can
/// be logged once instead of every single iteration.
static bool g_was_stopped = false;

void setup() {
    // ---- Critical outputs first, always -----------------------------------
    // Nothing above these two lines may block or take time. Both pads float
    // until init() runs; the external 10k pulldowns are what hold the gates low
    // until this point.
    PowerCut::init();
    JetsonCanCut::init();
    JetsonReset::init();

    // ---- Everything else ---------------------------------------------------
    EstopButton::init();
    can_init();
    RadioLink::init();

    g_last_heartbeat_ms = millis();

    ESP_LOGI(TAG, "emergency node ready");
}

void loop() {
    static uint32_t last_verify = 0;
    static uint32_t last_can_health = 0;
    const uint32_t now = millis();

    // ---- Button ------------------------------------------------------------
    // No edge handling needed here: EstopButton::is_pressed() below is read
    // live as part of the combined stop condition every iteration.
    EstopButton::poll();

    // ---- CAN ---------------------------------------------------------------
    CanMsg msg;
    while (can_recv(msg, 0) == ESP_OK) {
        // TODO: your protocol.
    }

    // ---- Radio ---------------------------------------------------------------
    RadioLink::Frame frame;
    while (RadioLink::receive(frame)) {
        if (frame.len == 1 && frame.data[0] == RadioProto::STOP) {
            g_radio_stop = true;
        } else if (frame.len == 1 && frame.data[0] == RadioProto::HEARTBEAT) {
            g_last_heartbeat_ms = now;
            g_radio_stop        = false;
        }
        // Anything else is an unrecognised payload and is silently ignored.
    }

    // ---- Combined stop condition ---------------------------------------------
    // Re-evaluated fresh every iteration: nothing here latches, so power is
    // restored automatically the moment every condition below clears.
    const bool radio_timeout =
        (now - g_last_heartbeat_ms) >= RadioProto::HEARTBEAT_TIMEOUT_MS;
    const bool should_stop =
        EstopButton::is_pressed() || g_radio_stop || radio_timeout;

    if (should_stop && !g_was_stopped) {
        const char* reason = EstopButton::is_pressed() ? "local button"
                            : g_radio_stop              ? "radio stop command"
                                                         : "radio heartbeat timeout";
        ESP_LOGW(TAG, "E-STOP engaged: %s", reason);
    } else if (!should_stop && g_was_stopped) {
        ESP_LOGI(TAG, "E-STOP cleared, power restored");
    }
    g_was_stopped = should_stop;

    if (should_stop) {
        PowerCut::engage();
    } else {
        PowerCut::release();
    }

    // ---- Timed pulses ------------------------------------------------------
    // Releases the Jetson reset line when its pulse is up. Cheap, and must run
    // every iteration so a reset is never held longer than intended.
    JetsonReset::update();

    // ---- Periodic integrity ------------------------------------------------
    if (now - last_verify >= VERIFY_INTERVAL_MS) {
        last_verify = now;
        // Re-assert the three controlled lines and repair the pad if anything
        // disturbed it. Keep these running no matter what state the
        // application is in.
        PowerCut::verify();
        JetsonCanCut::verify();
        JetsonReset::verify();
    }

    if (now - last_can_health >= CAN_HEALTH_INTERVAL_MS) {
        last_can_health = now;
        can_recover_if_needed();
    }

    delay(EstopButton::POLL_INTERVAL_MS);
}
