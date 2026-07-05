#include "lid_manager.hpp"
#include "can_manager.hpp"
#include "servo_ledc.hpp"

using namespace CanProtocol;


// ---------------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------------
#ifdef DEBUG_ENABLED
// #include <FastLED.h>

// extern CRGB leds[];

// static void led_set(CRGB color) {
//     leds[0] = color;
//     FastLED.show();
// }
#endif // DEBUG_ENABLED

// ---------------------------------------------------------------------------
// Hardware stubs
// ---------------------------------------------------------------------------

static ServoLedc lid_servo_right_far;
static ServoLedc lid_servo_right_near;
static ServoLedc lid_servo_left_far;
static ServoLedc lid_servo_left_near;

// Continuous rotation servos: 1500µs = stop, >1500 = forward, <1500 = backward
static constexpr int SERVO_SPEED_CW  = 1400;  // clockwise (forward)
static constexpr int SERVO_SPEED_CCW = 1600;  // counter-clockwise (backward)

static constexpr uint32_t LID_ACTION_MS = 1200; // time for 270° rotation

void lid_init() {
    lid_servo_right_far.attach(PIN_SERVO_RIGHT_FAR, LEDC_CHANNEL_0);
    lid_servo_right_near.attach(PIN_SERVO_RIGHT_NEAR, LEDC_CHANNEL_1);
    lid_servo_left_far.attach(PIN_SERVO_LEFT_FAR, LEDC_CHANNEL_2);
    lid_servo_left_near.attach(PIN_SERVO_LEFT_NEAR, LEDC_CHANNEL_3);

    // Initialize to neutral (stopped)
    lid_servo_right_far.stop();
    lid_servo_right_near.stop();
    lid_servo_left_far.stop();
    lid_servo_left_near.stop();
}

static void hw_start_open() {
    lid_servo_right_far.writeMicroseconds(SERVO_SPEED_CW);
    lid_servo_right_near.writeMicroseconds(SERVO_SPEED_CW);
    lid_servo_left_far.writeMicroseconds(SERVO_SPEED_CCW);
    lid_servo_left_near.writeMicroseconds(SERVO_SPEED_CCW);
#ifdef DEBUG_ENABLED
    Serial.println("[HW] start open");
    // led_set(CRGB::Yellow);
#endif
}

static void hw_start_close() {
    lid_servo_right_far.writeMicroseconds(SERVO_SPEED_CCW);
    lid_servo_right_near.writeMicroseconds(SERVO_SPEED_CCW);
    lid_servo_left_far.writeMicroseconds(SERVO_SPEED_CW);
    lid_servo_left_near.writeMicroseconds(SERVO_SPEED_CW);
#ifdef DEBUG_ENABLED
    Serial.println("[HW] start close");
    // led_set(CRGB::Yellow);
#endif
}

static void hw_stop() {
    lid_servo_right_far.writeMicroseconds(1500);
    lid_servo_right_near.writeMicroseconds(1500);
    lid_servo_left_far.writeMicroseconds(1500);
    lid_servo_left_near.writeMicroseconds(1500);
#ifdef DEBUG_ENABLED
    Serial.println("[HW] stop");
    // led_set(CRGB::Blue);
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
            // led_set(CRGB::Green);
            Serial.println("[LID] Opened");
#endif // DEBUG_ENABLED
        } else {
            lid_state = LidState::DONE_CLOSED;
#ifdef DEBUG_ENABLED
            // led_set(CRGB::Red);
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
