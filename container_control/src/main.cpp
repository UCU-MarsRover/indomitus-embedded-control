#include <Arduino.h>
#include <FastLED.h>
#include "can_comm.hpp"

// ---------------------------------------------------------------------------
// CAN IDs
// ---------------------------------------------------------------------------
static constexpr uint32_t CMD_ID         = 0x200;  // PC -> ESP32
static constexpr uint32_t LID_RESP_ID        = 0x201;  // ESP32 -> PC (lid)
static constexpr uint32_t WEIGHT_RESP_ID = 0x202;  // ESP32 -> PC (weight)

// ---------------------------------------------------------------------------
// Commands (PC -> ESP32, byte 0)
// ---------------------------------------------------------------------------
static constexpr uint8_t CMD_OPEN        = 0x01;
static constexpr uint8_t CMD_CLOSE       = 0x02;
static constexpr uint8_t CMD_STOP        = 0x08;
static constexpr uint8_t CMD_POLL_STATUS = 0x04;
static constexpr uint8_t CMD_GET_WEIGHT  = 0x10;

// ---------------------------------------------------------------------------
// Status codes (ESP32 -> PC, byte 1 у LID_RESP_ID)
// ---------------------------------------------------------------------------
static constexpr uint8_t STATUS_ACK         = 0x00;
static constexpr uint8_t STATUS_IN_PROGRESS = 0x01;
static constexpr uint8_t STATUS_DONE        = 0x02;
static constexpr uint8_t STATUS_ERROR       = 0x03;

// ---------------------------------------------------------------------------
// RGB LED (WS2812)
// ---------------------------------------------------------------------------
static constexpr uint8_t LED_PIN  = 48;
static constexpr uint8_t NUM_LEDS = 1;
CRGB leds[NUM_LEDS];

static void led_set(CRGB color) {
    leds[0] = color;
    FastLED.show();
}

// ---------------------------------------------------------------------------
// Lid state machine
// ---------------------------------------------------------------------------
enum class LidState : uint8_t {
    IDLE,
    OPENING,
    CLOSING,
    DONE_OPEN,
    DONE_CLOSED,
    ERROR,
};

static LidState  lid_state    = LidState::IDLE;
static uint8_t   last_cmd     = 0x00;
static uint32_t  action_start = 0;

static constexpr uint32_t LID_ACTION_MS = 5000;

// send current lid status based on state machine
static uint8_t lid_current_status() {
    switch (lid_state) {
        case LidState::OPENING:
        case LidState::CLOSING:     return STATUS_IN_PROGRESS;
        case LidState::DONE_OPEN:
        case LidState::DONE_CLOSED: return STATUS_DONE;
        case LidState::ERROR:       return STATUS_ERROR;
        default:                    return STATUS_DONE;  // IDLE
    }
}

// non-blocking update of lid state machine; should be called frequently in the main loop
static void lid_update() {
    if (lid_state != LidState::OPENING && lid_state != LidState::CLOSING) return;

    if (millis() - action_start >= LID_ACTION_MS) {
        if (lid_state == LidState::OPENING) {
            lid_state = LidState::DONE_OPEN;
            led_set(CRGB::Green);
            Serial.println("[LID] Opened — DONE");
        } else {
            lid_state = LidState::DONE_CLOSED;
            led_set(CRGB::Red);
            Serial.println("[LID] Closed — DONE");
        }
    }
}

// ---------------------------------------------------------------------------
// CAN send helpers
// ---------------------------------------------------------------------------
static void send_lid_response(uint8_t cmd, uint8_t status) {
    const uint8_t payload[2] = {cmd, status};
    const esp_err_t sent = can_send(LID_RESP_ID, payload, 2);
    Serial.print("TX  id=0x"); Serial.print(LID_RESP_ID, HEX);
    Serial.print("  cmd=0x"); Serial.print(cmd, HEX);
    Serial.print("  status=0x"); Serial.print(status, HEX);
    Serial.print("  can_send="); Serial.println(sent == ESP_OK ? "OK" : "FAIL");
}

static void send_weight(float value) {
    uint8_t payload[4];
    memcpy(payload, &value, 4);  // float32 little-endian
    const esp_err_t sent = can_send(WEIGHT_RESP_ID, payload, 4);
    Serial.print("TX  id=0x"); Serial.print(WEIGHT_RESP_ID, HEX);
    Serial.print("  weight="); Serial.print(value, 3);
    Serial.print("  can_send="); Serial.println(sent == ESP_OK ? "OK" : "FAIL");
}

// ---------------------------------------------------------------------------
// Hardware stubs — should be replaced with real implementations after connecting the hardware
// ---------------------------------------------------------------------------
static void hw_start_open() {
    // TODO: start motor to open lid
    Serial.println("[HW] start open");
    led_set(CRGB::Yellow);  // yellow means "in progress"
}

static void hw_start_close() {
    // TODO: start motor to close lid
    Serial.println("[HW] start close");
    led_set(CRGB::Yellow);
}

static void hw_stop() {
    // TODO: stop motors immediately
    Serial.println("[HW] stop");
    led_set(CRGB::Blue);
}

static float hw_read_weight() {
    // TODO: read real weight sensor
    return 0.0f;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------
static void handle_open() {
    Serial.println("CMD: OPEN LID");

    if (lid_state == LidState::OPENING) {
        // send acknowledge that we're already opening
        send_lid_response(CMD_OPEN, STATUS_IN_PROGRESS);
        return;
    }

    last_cmd     = CMD_OPEN;
    lid_state    = LidState::OPENING;
    action_start = millis();

    hw_start_open();
    send_lid_response(CMD_OPEN, STATUS_ACK);
}

static void handle_close() {
    Serial.println("CMD: CLOSE LID");

    if (lid_state == LidState::CLOSING) {
        send_lid_response(CMD_CLOSE, STATUS_IN_PROGRESS);
        return;
    }

    last_cmd     = CMD_CLOSE;
    lid_state    = LidState::CLOSING;
    action_start = millis();

    hw_start_close();
    send_lid_response(CMD_CLOSE, STATUS_ACK);
}

static void handle_stop() {
    Serial.println("CMD: STOP LID");

    if (lid_state != LidState::OPENING && lid_state != LidState::CLOSING) {
        send_lid_response(CMD_STOP, STATUS_DONE);  // already stopped
        return;
    }

    hw_stop();
    lid_state = LidState::ERROR;
    led_set(CRGB::Blue);
    send_lid_response(CMD_STOP, STATUS_DONE);
}

static void handle_poll_status() {
    const uint8_t status = lid_current_status();
    send_lid_response(last_cmd, status);
    Serial.print("CMD: POLL STATUS → 0x"); Serial.println(status, HEX);
}

static void handle_get_weight() {
    Serial.println("CMD: GET WEIGHT");
    const float weight = hw_read_weight();
    send_weight(weight);
}

// ---------------------------------------------------------------------------
// CAN receive + dispatch
// ---------------------------------------------------------------------------
static void log_can_rx(const CanMsg &msg) {
    Serial.print("[CAN RX] id=0x"); Serial.print(msg.id, HEX);
    Serial.print("  len="); Serial.print(msg.len);
    Serial.print("  data=[ ");
    for (uint8_t i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
}

static void handle_command(const CanMsg &msg) {
    log_can_rx(msg);

    if (msg.id != CMD_ID || msg.len < 1) return;

    switch (msg.data[0]) {
        case CMD_OPEN:        handle_open();        break;
        case CMD_CLOSE:       handle_close();       break;
        case CMD_STOP:        handle_stop();        break;
        case CMD_POLL_STATUS: handle_poll_status(); break;
        case CMD_GET_WEIGHT:  handle_get_weight();  break;
        default:
            Serial.print("CMD: unknown 0x"); Serial.println(msg.data[0], HEX);
            led_set(CRGB::Orange);
            break;
    }
}





void setup() {
    Serial.begin(115200);
    delay(1000);

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    led_set(CRGB::Black);

    Serial.println("ESP32-S3 container node boot");

    can_init_1mbs_accept_all();
    Serial.println("CAN started at 1 Mbps");

    // startup blink
    led_set(CRGB::White);
    delay(300);
    led_set(CRGB::Black);

    Serial.println("Ready");
}

void loop() {
    // non-blocking update of lid state machine
    lid_update();

    CanMsg msg;
    if (can_recv(msg, 10)) {
        handle_command(msg);
    }
}