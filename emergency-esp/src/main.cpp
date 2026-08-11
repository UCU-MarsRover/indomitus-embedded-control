/**
 * @file main.cpp
 * @brief Emergency (E-stop) node --- bring-up skeleton.
 *
 * This file deliberately contains no emergency policy. It only establishes the
 * things the modules cannot do for themselves:
 *
 *   1. Bring-up ORDER. The two transistor-gate lines are parked LOW before any
 *      other peripheral is touched, which keeps the window where those pads are
 *      undriven as short as the chip allows.
 *   2. The periodic verify() calls that let SafetyOutput repair a corrupted pad.
 *
 * High-level behaviour --- what a button press does, which CAN IDs mean what,
 * whether radio silence should trigger anything, latching and rearm rules ---
 * goes in the marked sections below or in your own module.
 */

#include <Arduino.h>

#include "can_driver.hpp"
#include "estop_button.hpp"
#include "jetson_can_cut.hpp"
#include "power_cut.hpp"
#include "radio_link.hpp"

#include "esp_log.h"

static const char* TAG = "MAIN";

/// How often the gate lines are re-asserted and checked.
static constexpr uint32_t VERIFY_INTERVAL_MS = 50;
/// How often the CAN controller is checked for bus-off.
static constexpr uint32_t CAN_HEALTH_INTERVAL_MS = 250;

void setup() {
    // ---- Critical outputs first, always -----------------------------------
    // Nothing above these two lines may block or take time. Both pads float
    // until init() runs; the external 10k pulldowns are what hold the gates low
    // until this point.
    PowerCut::init();
    JetsonCanCut::init();

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
    const EstopButton::Event ev = EstopButton::poll();
    if (ev == EstopButton::Event::PRESSED) {
        // TODO: your policy. e.g. PowerCut::engage();
    } else if (ev == EstopButton::Event::RELEASED) {
        // TODO: your policy. Note that releasing the button should almost
        // certainly NOT release the cut on its own --- an e-stop normally
        // latches until something deliberately rearms it.
    }

    // ---- CAN ---------------------------------------------------------------
    CanMsg msg;
    while (can_recv(msg, 0) == ESP_OK) {
        // TODO: your protocol. Suggestion: make engaging a cut cheap (a single
        // command byte) and releasing one expensive (require a magic key), so
        // a corrupted frame can never un-cut the rover.
    }

    // ---- Radio -------------------------------------------------------------
    RadioLink::Frame frame;
    while (RadioLink::receive(frame)) {
        // TODO: your protocol.
    }
    // RadioLink::ms_since_last_frame() is available if you want a link-loss
    // policy. Whether losing the radio should cut power is a real decision:
    // it protects a runaway rover, but it also means every radio dropout kills
    // the machine. Pick deliberately.

    // ---- Periodic integrity ------------------------------------------------
    if (now - last_verify >= VERIFY_INTERVAL_MS) {
        last_verify = now;
        // Re-assert both gate lines and repair the pad if anything disturbed
        // it. Keep these running no matter what state the application is in.
        PowerCut::verify();
        JetsonCanCut::verify();
    }

    if (now - last_can_health >= CAN_HEALTH_INTERVAL_MS) {
        last_can_health = now;
        can_recover_if_needed();
    }

    delay(EstopButton::POLL_INTERVAL_MS);
}
