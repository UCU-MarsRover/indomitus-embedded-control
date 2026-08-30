/**
 * @file main.cpp
 * @brief Emergency (E-stop) node.
 *
 * The output layer is lifted verbatim from main_test_hw_commands.cpp --- the
 * same Channel struct, the same plain pinMode/digitalWrite, the same 1 s pulse.
 * That is the version proven on the physical board, so it is kept as-is rather
 * than rebuilt on an abstraction.
 *
 *   GPIO2  JETSON_RESET  idle LOW,  pulses HIGH 1s
 *   GPIO3  POWER_ON      idle LOW,  pulses HIGH 1s
 *   GPIO4  POWER_OFF     idle HIGH, pulses LOW  1s
 *   GPIO5  ESTOP_BUTTON  input, active high, internal pulldown
 *
 * All three outputs are MOMENTARY. The power system latches its own state, so
 * this node presses buttons rather than holding a cut line. Nothing here is a
 * held level, and no pulse can outlive its window.
 *
 * ---------------------------------------------------------------------------
 * Command sources
 * ---------------------------------------------------------------------------
 * Three, all reaching the same three channels:
 *
 *  1. LoRa control bytes --- a one-byte frame from the ground station
 *     (e32-e-stop-gs):
 *
 *         0x00  STOP    pulse POWER_OFF
 *         0x01  RUN     pulse POWER_ON
 *         0x02  REBOOT  pulse JETSON_RESET
 *
 *  2. LoRa text --- any frame that is not exactly one byte is echoed straight
 *     back (the ground station's sync handshake: it repeats
 *     "hello from ground station" until it sees its own message returned) and
 *     then also tried as a text command, so ON / OFF / REBOOT / STATUS work
 *     over the air as well.
 *
 *  3. USB serial text --- the same words typed into the monitor. Kept
 *     deliberately: it is the exact path that was verified working, so if
 *     something misbehaves you can type OFF over USB and immediately tell
 *     whether the fault is in the output layer or in the radio link.
 *
 * ---------------------------------------------------------------------------
 * Stop policy
 * ---------------------------------------------------------------------------
 * Because the hardware takes presses rather than a held level, stopping is
 * EDGE-triggered:
 *
 *   - Pressing the local button pulses POWER_OFF once.
 *   - RELEASING the button does nothing. The rover does not restart by itself;
 *     that needs an explicit RUN. An e-stop that un-stops when you let go is
 *     not an e-stop.
 *   - RUN is refused while the button is still held down, so the ground
 *     station cannot start a rover that somebody is standing over with the
 *     button pressed.
 */

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

#include "can_driver.hpp"
#include "estop_button.hpp"
#include "pins.hpp"
#include "radio_link.hpp"

/// How long each line stays flipped away from idle before returning.
static constexpr uint32_t PULSE_MS = 1000;

/// How often the radio link is drained.
static constexpr uint32_t RADIO_POLL_INTERVAL_MS = 10;

/// How often the CAN controller is checked for bus-off.
static constexpr uint32_t CAN_HEALTH_INTERVAL_MS = 250;

/// One-byte control frames from the ground station.
namespace RadioProto {
constexpr uint8_t STOP          = 0x00;
constexpr uint8_t RUN           = 0x01;
constexpr uint8_t REBOOT_JETSON = 0x02;
}  // namespace RadioProto

// ---- Output channels ------------------------------------------------------
// Same structure as main_test_hw_commands.cpp.

struct Channel {
    const uint8_t pin;
    const char* name;
    const bool idle_high;  ///< Rest level. Asserted level is the opposite.
    uint32_t started_at;
    bool active;

    Channel(uint8_t pin_, const char* name_, bool idle_high_)
        : pin(pin_), name(name_), idle_high(idle_high_), started_at(0),
          active(false) {}
};

static Channel g_reboot((uint8_t)Pins::JETSON_RESET, "REBOOT",
                        /*idle_high=*/false);
static Channel g_power_on((uint8_t)Pins::POWER_ON, "ON", /*idle_high=*/false);
static Channel g_power_off((uint8_t)Pins::POWER_OFF, "OFF",
                           /*idle_high=*/true);

static bool g_radio_ready = false;

/// Writes one line to USB serial and, if the radio is up, sends the same text
/// as a single radio frame --- so both interfaces always show the same thing.
static void out(const char* fmt, ...) {
    char buf[96];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.println(buf);

    if (g_radio_ready) {
        const size_t len = strnlen(buf, RadioLink::MAX_PAYLOAD);
        RadioLink::send((const uint8_t*)buf, (uint8_t)len);
    }
}

static void channel_init(Channel& c) {
    pinMode(c.pin, OUTPUT);
    digitalWrite(c.pin, c.idle_high ? HIGH : LOW);
}

static bool channel_trigger(Channel& c) {
    if (c.active) {
        out("ERR: %s pulse already running, ignored", c.name);
        return false;
    }
    c.active     = true;
    c.started_at = millis();
    const bool asserted_high = !c.idle_high;
    digitalWrite(c.pin, asserted_high ? HIGH : LOW);
    out("OK: %s -> GPIO%d %s for %lums", c.name, (int)c.pin,
        asserted_high ? "HIGH" : "LOW", (unsigned long)PULSE_MS);
    return true;
}

/// Non-blocking: call every loop() iteration.
static void channel_update(Channel& c) {
    if (!c.active) {
        return;
    }
    if (millis() - c.started_at >= PULSE_MS) {
        digitalWrite(c.pin, c.idle_high ? HIGH : LOW);
        c.active = false;
        out("...: %s -> GPIO%d back to %s (pulse done)", c.name, (int)c.pin,
            c.idle_high ? "HIGH" : "LOW");
    }
}

// ---- Actions --------------------------------------------------------------

static void do_stop(const char* reason) {
    out("E-STOP (%s)", reason);
    channel_trigger(g_power_off);
}

static void do_run(const char* reason) {
    // Refuse to start while somebody is holding the button down.
    if (EstopButton::is_pressed()) {
        out("ERR: RUN refused (%s) --- e-stop button is held", reason);
        return;
    }
    out("RUN (%s)", reason);
    channel_trigger(g_power_on);
}

static void do_reboot(const char* reason) {
    out("REBOOT JETSON (%s)", reason);
    channel_trigger(g_reboot);
}

static void print_status() {
    out("STATUS: GPIO%d(reboot)=%s GPIO%d(on)=%s GPIO%d(off)=%s button=%s",
        (int)g_reboot.pin, digitalRead(g_reboot.pin) ? "HIGH" : "LOW",
        (int)g_power_on.pin, digitalRead(g_power_on.pin) ? "HIGH" : "LOW",
        (int)g_power_off.pin, digitalRead(g_power_off.pin) ? "HIGH" : "LOW",
        EstopButton::is_pressed() ? "PRESSED" : "released");
}

// ---- Text command parsing -------------------------------------------------

/// @param source Label used only in log lines, e.g. "UART" or "LoRa".
/// @param quiet  True for text arriving over the radio, where an unrecognised
///               string is almost certainly the sync handshake rather than a
///               typo --- so it is ignored instead of answered with an error.
static void handle_text(const String& raw, const char* source, bool quiet) {
    String cmd = raw;
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() == 0) {
        return;
    }

    if (cmd == "OFF" || cmd == "F" || cmd == "STOP") {
        do_stop(source);
    } else if (cmd == "ON" || cmd == "N" || cmd == "START") {
        do_run(source);
    } else if (cmd == "REBOOT" || cmd == "R") {
        do_reboot(source);
    } else if (cmd == "STATUS" || cmd == "S") {
        print_status();
    } else if (!quiet) {
        out("ERR: unknown command \"%s\"", cmd.c_str());
        out("commands: ON / OFF / REBOOT / STATUS  (or N / F / R / S)");
    }
}

// ---- USB serial line reader ----------------------------------------------

static String g_uart_line;

static void poll_uart() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_uart_line.length() > 0) {
                handle_text(g_uart_line, "UART", /*quiet=*/false);
                g_uart_line = "";
            }
        } else {
            g_uart_line += c;
        }
    }
}

// ---- Radio frame reader ---------------------------------------------------

static void poll_radio() {
    RadioLink::Frame frame;
    while (RadioLink::receive(frame)) {
        if (frame.len == 1) {
            switch (frame.data[0]) {
                case RadioProto::STOP:
                    do_stop("LoRa");
                    break;
                case RadioProto::RUN:
                    do_run("LoRa");
                    break;
                case RadioProto::REBOOT_JETSON:
                    do_reboot("LoRa");
                    break;
                default:
                    out("ERR: unknown control byte 0x%02X", frame.data[0]);
                    break;
            }
            continue;
        }

        // Multi-byte frame. Echo it back first --- the ground station's sync
        // handshake waits to see its own message returned --- then try it as a
        // text command so ON / OFF / REBOOT / STATUS work over the air too.
        RadioLink::send(frame.data, frame.len);

        char buf[RadioLink::MAX_PAYLOAD + 1];
        const uint8_t n = frame.len < RadioLink::MAX_PAYLOAD
                              ? frame.len
                              : RadioLink::MAX_PAYLOAD;
        memcpy(buf, frame.data, n);
        buf[n] = '\0';
        Serial.printf("> [LoRa] %s\n", buf);

        handle_text(String(buf), "LoRa", /*quiet=*/true);
    }
}

// ---- Button ---------------------------------------------------------------

static void poll_button() {
    // Only the press edge acts. Releasing deliberately does nothing: restarting
    // the rover must be an explicit command, never a side effect of letting go.
    if (EstopButton::poll() == EstopButton::Event::PRESSED) {
        do_stop("local button");
    }
}

void setup() {
    Serial.begin(115200);

    // Park the three command lines before anything slower is brought up.
    channel_init(g_reboot);
    channel_init(g_power_on);
    channel_init(g_power_off);

    EstopButton::init();

    // USB CDC needs a moment before the first output is visible on the monitor.
    delay(1500);

    can_init();
    RadioLink::init();
    g_radio_ready = true;

    Serial.println("=== emergency node ready ===");
    print_status();
}

void loop() {
    static uint32_t last_radio_poll = 0;
    static uint32_t last_can_health = 0;
    const uint32_t now = millis();

    poll_button();
    poll_uart();

    if (now - last_radio_poll >= RADIO_POLL_INTERVAL_MS) {
        last_radio_poll = now;
        poll_radio();
    }

    // ---- CAN ---------------------------------------------------------------
    CanMsg msg;
    while (can_recv(msg, 0) == ESP_OK) {
        // TODO: CAN protocol. Keep it asymmetric like the radio one: stopping
        // cheap, starting guarded, so a corrupted frame can never start the
        // rover.
    }

    if (now - last_can_health >= CAN_HEALTH_INTERVAL_MS) {
        last_can_health = now;
        can_recover_if_needed();
    }

    // Releases each line when its pulse is up. Must run every iteration.
    channel_update(g_reboot);
    channel_update(g_power_on);
    channel_update(g_power_off);

    delay(EstopButton::POLL_INTERVAL_MS);
}
