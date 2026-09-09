/**
 * @file main_test_hw_commands.cpp
 * @brief Bring-up test for three transistor-gate command pulses, driven
 *        identically from two sources: plain-text commands typed into the
 *        USB serial monitor, and the same text sent as a payload over the
 *        LoRa radio link (lib/radio, the E32 module on UART1).
 *
 * Every response --- the echo of what was received, the OK/ERR
 * acknowledgement, STATUS --- is written to both USB serial and the radio
 * link, regardless of which one the command arrived on. That is what makes
 * the two interfaces "identical": whichever one you use, both show the same
 * traffic.
 *
 * All three lines behave the same way: idle at their rest level, flip to the
 * opposite level for PULSE_MS on command, then return to idle automatically.
 * Nothing latches.
 *
 *   GPIO2  reboot jetson   <-- "REBOOT" / "R"        idle LOW,  pulses HIGH 1s
 *   GPIO3  hardware on     <-- "ON" / "N" / "START"   idle LOW,  pulses HIGH 1s
 *   GPIO4  hardware stop   <-- "OFF" / "F" / "STOP"   idle HIGH, pulses LOW  1s
 *   ---    query state     <-- "STATUS" / "S"
 *
 * @warning GPIO2 is an ESP32-C3 strapping pin and must read HIGH during the
 *          ROM boot window (see include/pins.hpp for the full reasoning
 *          already established for this board's JETSON_RESET line). This
 *          file only drives GPIO2 LOW *after* setup() runs, so it is safe as
 *          long as no external pulldown is wired to this pad --- if the final
 *          circuit needs one, put the reboot pulse on a different GPIO
 *          instead.
 *
 * Own platformio env (test_hw_commands) --- only one main() may be linked at
 * a time, same as main_test_can.cpp.
 */

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

#include "radio_link.hpp"

/// How long each line stays flipped away from idle before returning.
static constexpr uint32_t PULSE_MS = 1000;

/// How often the radio link is drained. Call receive() often enough that a
/// burst of frames doesn't back up in the UART driver's buffer.
static constexpr uint32_t RADIO_POLL_INTERVAL_MS = 10;

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

static Channel g_reboot(2, "REBOOT", /*idle_high=*/false);
static Channel g_turn_on(3, "ON", /*idle_high=*/false);
static Channel g_turn_off(4, "STOP", /*idle_high=*/true);

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

static void channel_trigger(Channel& c) {
    if (c.active) {
        out("ERR: %s pulse already running, ignored", c.name);
        return;
    }
    c.active     = true;
    c.started_at = millis();
    const bool asserted_high = !c.idle_high;
    digitalWrite(c.pin, asserted_high ? HIGH : LOW);
    out("OK: %s -> GPIO%d %s for %lums", c.name, (int)c.pin,
        asserted_high ? "HIGH" : "LOW", (unsigned long)PULSE_MS);
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

static void print_status() {
    out("STATUS: GPIO%d(reboot)=%s  GPIO%d(on)=%s  GPIO%d(stop)=%s",
        (int)g_reboot.pin, digitalRead(g_reboot.pin) ? "HIGH" : "LOW",
        (int)g_turn_on.pin, digitalRead(g_turn_on.pin) ? "HIGH" : "LOW",
        (int)g_turn_off.pin, digitalRead(g_turn_off.pin) ? "HIGH" : "LOW");
}

static void print_help() {
    out("commands: ON / OFF / REBOOT / STATUS  (or N / F / R / S)");
}

/// @param source Label used only in the echo line, e.g. "UART" or "LoRa".
static void handle_command(const String& raw, const char* source) {
    // Echo exactly what was received, before any interpretation --- so a typo
    // or a mangled line is visible immediately, on both interfaces.
    out("> [%s] received: \"%s\"", source, raw.c_str());

    String cmd = raw;
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "REBOOT" || cmd == "R") {
        channel_trigger(g_reboot);
    } else if (cmd == "ON" || cmd == "N" || cmd == "START") {
        channel_trigger(g_turn_on);
    } else if (cmd == "OFF" || cmd == "F" || cmd == "STOP") {
        channel_trigger(g_turn_off);
    } else if (cmd == "STATUS" || cmd == "S") {
        print_status();
    } else if (cmd.length() > 0) {
        out("ERR: unknown command \"%s\"", cmd.c_str());
        print_help();
    }
}

// ---- USB serial line reader ----------------------------------------------
static String g_uart_line;

static void poll_uart() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_uart_line.length() > 0) {
                handle_command(g_uart_line, "UART");
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
        char buf[RadioLink::MAX_PAYLOAD + 1];
        memcpy(buf, frame.data, frame.len);
        buf[frame.len] = '\0';
        handle_command(String(buf), "LoRa");
    }
}

void setup() {
    Serial.begin(115200);

    // USB CDC needs a moment before the first output is visible on the monitor.
    delay(1500);

    channel_init(g_reboot);
    channel_init(g_turn_on);
    channel_init(g_turn_off);

    RadioLink::init();
    g_radio_ready = true;

    Serial.println("=== HW command UART+LoRa test ===");
    Serial.printf(
        "GPIO%d=reboot jetson (idle LOW, pulse HIGH 1s), "
        "GPIO%d=turn on (idle LOW, pulse HIGH 1s), "
        "GPIO%d=stop (idle HIGH, pulse LOW 1s)\n",
        (int)g_reboot.pin, (int)g_turn_on.pin, (int)g_turn_off.pin);
    print_help();
    print_status();
}

void loop() {
    static uint32_t last_radio_poll = 0;

    poll_uart();

    const uint32_t now = millis();
    if (now - last_radio_poll >= RADIO_POLL_INTERVAL_MS) {
        last_radio_poll = now;
        poll_radio();
    }

    channel_update(g_reboot);
    channel_update(g_turn_on);
    channel_update(g_turn_off);
}
