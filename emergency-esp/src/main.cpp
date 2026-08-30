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
 *   - Ground station sends a single 0x00 byte to stop, 0x01 to release.
 *   - The rover is stopped (PowerCut::engage()) whenever either is true,
 *     re-evaluated every loop iteration:
 *       - the last radio command byte received was 0x00 (stop)
 *       - the local e-stop button is currently held down
 *   - Not latched: the moment both clear --- a 0x01 arrives after a 0x00, and
 *     the local button is released --- power is restored automatically.
 *   - A single 0x02 byte triggers JetsonReset::reset(): a one-shot,
 *     non-blocking pulse on the Jetson's SYS_RESET* line. Unlike STOP/RUN this
 *     is edge-triggered, not a held state --- one 0x02 is one reset pulse.
 *
 * Sync handshake: any received frame that isn't exactly one byte of 0x00,
 * 0x01, or 0x02 is treated as a plain-text sync/test message and echoed
 * straight back (see the ground station's e32-e-stop-gs project, which sends
 * "hello from ground station" on repeat until it sees its own message
 * echoed back). Purely a link-check; it has no effect on the stop condition.
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
constexpr uint8_t STOP          = 0x00;
constexpr uint8_t RUN           = 0x01;
constexpr uint8_t REBOOT_JETSON = 0x02;
}  // namespace RadioProto

/// True from the moment a 0x00 stop frame arrives until a 0x01 run frame
/// clears it. Not a latch --- the next 0x01 is enough, no rearm step.
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
        } else if (frame.len == 1 && frame.data[0] == RadioProto::RUN) {
            g_radio_stop = false;
        } else if (frame.len == 1 && frame.data[0] == RadioProto::REBOOT_JETSON) {
            if (!JetsonReset::reset()) {
                ESP_LOGW(TAG, "radio: reboot jetson ignored, pulse already in progress");
            }
        } else {
            // Not a control byte: treat it as a sync/test message and echo it
            // straight back, e.g. the ground station's boot-time
            // "hello from ground station" handshake.
            ESP_LOGI(TAG, "radio: echoing %u-byte message: %.*s",
                     (unsigned)frame.len, (int)frame.len,
                     (const char*)frame.data);
            RadioLink::send(frame.data, frame.len);
        }
    }

    // ---- Combined stop condition ---------------------------------------------
    // Re-evaluated fresh every iteration: nothing here latches, so power is
    // restored automatically the moment both conditions below clear.
    const bool should_stop = EstopButton::is_pressed() || g_radio_stop;

    if (should_stop && !g_was_stopped) {
        const char* reason =
            EstopButton::is_pressed() ? "local button" : "radio stop command";
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
