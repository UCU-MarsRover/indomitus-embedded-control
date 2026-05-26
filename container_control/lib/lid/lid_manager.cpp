#include "lid_manager.hpp"
#include "can_manager.hpp"
#include <ESP32Servo.h>

using namespace CanProtocol;


// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------
#ifdef DEBUG_ENABLED
#include <FastLED.h>

extern CRGB leds[];

static void led_set(CRGB color) {
    leds[0] = color;
    FastLED.show();
}
#endif // DEBUG_ENABLED

// ---------------------------------------------------------------------------
// Hardware stubs
// ---------------------------------------------------------------------------

static Servo lid_servo;

static constexpr int SERVO_OPEN_DEG  = 90;
static constexpr int SERVO_CLOSE_DEG = 0;

void lid_init() {
    lid_servo.attach(PIN_SERVO);
    lid_servo.write(SERVO_CLOSE_DEG);
}

static void hw_start_open() {
    lid_servo.write(SERVO_OPEN_DEG);
#ifdef DEBUG_ENABLED
    Serial.println("[HW] start open");
    led_set(CRGB::Yellow);
#endif
}

static void hw_start_close() {
    lid_servo.write(SERVO_CLOSE_DEG);
#ifdef DEBUG_ENABLED
    Serial.println("[HW] start close");
    led_set(CRGB::Yellow);
#endif
}

static void hw_stop() {
    lid_servo.detach();
#ifdef DEBUG_ENABLED
    Serial.println("[HW] stop");
    led_set(CRGB::Blue);
#endif
}


// ---------------------------------------------------------------------------
// Lid state machine
// ---------------------------------------------------------------------------
enum class LidState : uint8_t {
    IDLE, OPENING, CLOSING, DONE_OPEN, DONE_CLOSED, ERROR
};

static LidState  lid_state    = LidState::IDLE;
static uint8_t   last_cmd     = 0x00;
static uint32_t  action_start = 0;
static constexpr uint32_t LID_ACTION_MS = 5000;

static uint8_t lid_current_status() {
    switch (lid_state) {
        case LidState::OPENING:
        case LidState::CLOSING:     return STATUS_IN_PROGRESS;
        case LidState::DONE_OPEN:
        case LidState::DONE_CLOSED: return STATUS_DONE;
        case LidState::ERROR:       return STATUS_ERROR;
        default:                    return STATUS_DONE;
    }
}

static void lid_update() {
    if (lid_state != LidState::OPENING && lid_state != LidState::CLOSING) return;
    if (millis() - action_start >= LID_ACTION_MS) {
        if (lid_state == LidState::OPENING) {
            lid_state = LidState::DONE_OPEN;
#ifdef DEBUG_ENABLED
            led_set(CRGB::Green);
            Serial.println("[LID] Opened");
#endif // DEBUG_ENABLED
        } else {
            lid_state = LidState::DONE_CLOSED;
#ifdef DEBUG_ENABLED
            led_set(CRGB::Red);
            Serial.println("[LID] Closed");
#endif // DEBUG_ENABLED
        }
    }
}

void lid_handle_open(LidResponseFn respond) {
    if (lid_state == LidState::OPENING) { respond(CMD_OPEN, STATUS_IN_PROGRESS); return; }
    last_cmd = CMD_OPEN; lid_state = LidState::OPENING; action_start = millis();
    hw_start_open();
    respond(CMD_OPEN, STATUS_ACK);
}

void lid_handle_close(LidResponseFn respond) {
    if (lid_state == LidState::CLOSING) { respond(CMD_CLOSE, STATUS_IN_PROGRESS); return; }
    last_cmd = CMD_CLOSE; lid_state = LidState::CLOSING; action_start = millis();
    hw_start_close();
    respond(CMD_CLOSE, STATUS_ACK);
}

void lid_handle_stop(LidResponseFn respond) {
    if (lid_state != LidState::OPENING && lid_state != LidState::CLOSING) {
        respond(CMD_STOP, STATUS_DONE); return;
    }
    lid_state = LidState::ERROR;
    hw_stop();
    respond(CMD_STOP, STATUS_DONE);
}

void lid_handle_poll(LidResponseFn respond) {
    respond(last_cmd, lid_current_status());
}

void lid_task(void*) {
    for (;;) {
        lid_update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
