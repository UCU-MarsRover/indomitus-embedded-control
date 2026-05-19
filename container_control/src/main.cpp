#include <Arduino.h>
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

// Вбудований світлодіод ESP32-S3
static constexpr uint8_t BUILTIN_LED_PIN = 48; // GPIO48 на більшості плат ESP32-S3

// ---------------------------------------------------------------------------
// Hardware stubs — replace with real GPIO / actuator logic
// ---------------------------------------------------------------------------
static bool do_open_lid() {
    Serial.println("[ACT] Opening lid...");
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Увімкнути LED
    Serial.println("[LED] ON");
    delay(500);
    return true;
}

static bool do_close_lid() {
    Serial.println("[ACT] Closing lid...");
    digitalWrite(BUILTIN_LED_PIN, LOW); // Вимкнути LED
    Serial.println("[LED] OFF");
    delay(500);
    return true;
}

static bool do_work_done_ack() {
    Serial.println("[ACT] Work-done ack received");
    // Моргнути 3 рази на підтвердження
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUILTIN_LED_PIN, HIGH);
        delay(150);
        digitalWrite(BUILTIN_LED_PIN, LOW);
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
    // === DEBUG: логуємо БУДЬ-ЯКИЙ фрейм ===
    Serial.print("[CAN RX] id=0x");
    Serial.print(msg.id, HEX);
    Serial.print("  len=");
    Serial.print(msg.len);
    Serial.print("  data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0"); // leading zero
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
    // === кінець DEBUG ===
    if (msg.id != CMD_ID || msg.len < 1) {
        return;
    }

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
    while (!Serial);

    Serial.println("ESP32-S3 container node boot");

    // Ініціалізація LED
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, LOW);
    Serial.println("LED pin initialized");

    can_init_1mbs_accept_all();
    Serial.println("CAN started at 1 Mbps");
}

void loop() {
    Serial.println("loop tick");
    CanMsg msg;
    // if (can_recv(msg, 100)) {
    //     handle_command(msg);
    // }
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    delay(150);
    digitalWrite(BUILTIN_LED_PIN, LOW);
    delay(150);
}