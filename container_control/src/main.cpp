#include <Arduino.h>
#include <FastLED.h>
#include "can_comm.hpp"

// CAN IDs
static constexpr uint32_t CMD_ID  = 0x200;
static constexpr uint32_t RESP_ID = 0x201;

// Commands from Jetson
static constexpr uint8_t CMD_OPEN      = 0x01;
static constexpr uint8_t CMD_CLOSE     = 0x02;
static constexpr uint8_t CMD_WORK_DONE = 0x03;

// Status codes sent back
static constexpr uint8_t STATUS_OK   = 0x00;
static constexpr uint8_t STATUS_FAIL = 0x01;

// RGB LED (WS2812)
static constexpr uint8_t LED_PIN  = 48;
static constexpr uint8_t NUM_LEDS = 1;
CRGB leds[NUM_LEDS];

static void led_set(CRGB color) {
    leds[0] = color;
    FastLED.show();
}

// ---------------------------------------------------------------------------
// Hardware stubs
// ---------------------------------------------------------------------------
static bool do_open_lid() {
    Serial.println("[ACT] Opening lid...");
    led_set(CRGB::Green);
    Serial.println("[LED] GREEN");
    delay(500);
    return true;
}

static bool do_close_lid() {
    Serial.println("[ACT] Closing lid...");
    led_set(CRGB::Red);
    Serial.println("[LED] RED");
    delay(500);
    return true;
}

static bool do_work_done_ack() {
    Serial.println("[ACT] Work-done ack received");
    for (int i = 0; i < 3; i++) {
        led_set(CRGB::Blue);
        delay(150);
        led_set(CRGB::Black);
        delay(150);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Send response back to Jetson
// ---------------------------------------------------------------------------
static void send_response(uint8_t cmd, bool ok) {
    const uint8_t payload[2] = {cmd, ok ? STATUS_OK : STATUS_FAIL};
    const bool sent = can_send(RESP_ID, payload, 2);
    Serial.print("TX RESP  cmd=0x");
    Serial.print(cmd, HEX);
    Serial.print("  status=");
    Serial.print(ok ? "OK" : "FAIL");
    Serial.print("  can_send=");
    Serial.println(sent ? "OK" : "FAIL");
}

// ---------------------------------------------------------------------------
// Handle one incoming CAN frame
// ---------------------------------------------------------------------------
static void handle_command(const CanMsg &msg) {
    Serial.print("[CAN RX] id=0x");
    Serial.print(msg.id, HEX);
    Serial.print("  len=");
    Serial.print(msg.len);
    Serial.print("  data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");

    if (msg.id != CMD_ID || msg.len < 1) return;

    const uint8_t cmd = msg.data[0];
    bool ok = false;

    switch (cmd) {
        case CMD_OPEN:
            Serial.println("CMD: OPEN LID");
            ok = do_open_lid();
            break;
        case CMD_CLOSE:
            Serial.println("CMD: CLOSE LID");
            ok = do_close_lid();
            break;
        case CMD_WORK_DONE:
            Serial.println("CMD: WORK DONE");
            ok = do_work_done_ack();
            break;
        default:
            Serial.print("CMD: unknown 0x");
            Serial.println(cmd, HEX);
            led_set(CRGB::Orange);
            ok = false;
            break;
    }

    send_response(cmd, ok);
}

// ---------------------------------------------------------------------------
// Arduino lifecycle
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000); // замість while (!Serial) — не блокує без монітора

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    led_set(CRGB::Black);

    Serial.println("ESP32-S3 container node boot");
    Serial.println("LED initialized");

    can_init_1mbs_accept_all();
    Serial.println("CAN started at 1 Mbps");

    // Startup blink — білий колір означає готовність
    led_set(CRGB::White);
    delay(300);
    led_set(CRGB::Black);
}

void loop() {
    CanMsg msg;
    if (can_recv(msg, 100)) {
        handle_command(msg);
    }
}