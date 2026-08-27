/* ============================================================================
 *  CLAW + DRILL END-EFFECTOR FIRMWARE — ESP32-C3 Supermini
 * ============================================================================
 *  HARDWARE
 *    CAN_RX ......... GPIO0   (SN65HVD230 R)
 *    CAN_TX ......... GPIO1   (SN65HVD230 D)
 *    STEP ........... GPIO5   (TMC2209, NEMA17 claw)
 *    DIR ............ GPIO6   (TMC2209)
 *    N_EN ........... GPIO7   (TMC2209 enable, active LOW)
 *      TMC2209 UART unused, MS1/MS2 shorted to GND -> fixed microstep addr/mode.
 *    RPWM ........... GPIO21  (BTS7960, drill forward)
 *    LPWM ........... GPIO20  (BTS7960, drill reverse)
 *      BTS7960 R_EN/L_EN wired to 5V (always enabled). R_IS/L_IS unused.
 *    LOCK ........... GPIO10  (IRLB3813 transistor control)
 *
 *  OVERVIEW
 *    Claw (stepper), drill (DC motor), and transistor lock switch are independent,
 *    non-blocking state machines updated every loop() iteration - no delay() is ever
 *    used, so CAN/Serial stay responsive while either motor is moving.
 *
 *  COMMANDS
 *    CLOSE      <steps>  - step claw forward (closing) by <steps> steps
 *    OPEN       <steps>  - step claw reverse (opening) by <steps> steps
 *    STOP_STEP          - abort any stepping in progress immediately
 *    DOWN       <ms>     - spin drill forward for <ms> milliseconds
 *    UP         <ms>     - spin drill reverse for <ms> milliseconds
 *    STOP_DRILL         - abort drill motion immediately
 *    LOCK                - set GPIO10 HIGH (1) to enable transistor
 *    UNLOCK              - set GPIO10 LOW (0) to disable transistor
 *
 *  Every command is printed to Serial as "received" the instant it arrives,
 *  and again as "complete" the instant its motion finishes (including when
 *  finished early by STOP_STEP/STOP_DRILL, or immediately for LOCK/UNLOCK).
 *  CAN and Serial share identical command semantics; CAN is always polled and
 *  processed before Serial each loop() iteration, so CAN has priority when both
 *  arrive at once.
 *
 *  ----------------------------------------------------------------------
 *  CAN API (11-bit standard IDs, classic CAN, little-endian payloads)
 *  ----------------------------------------------------------------------
 *  COMMAND FRAME -> ID 0x200 (CAN_ID_CMD)
 *    byte0    = command id (table below)
 *    byte1..4 = uint32 parameter, LE (steps for CLOSE/OPEN, ms for UP/DOWN;
 *               omitted/ignored for STOP_STEP / STOP_DRILL / LOCK / UNLOCK)
 *
 *    id  name         param        DLC
 *    --  -----------  -----------  ---
 *    1   CLOSE        steps        5
 *    2   OPEN         steps        5
 *    3   STOP_STEP    -            1
 *    4   DOWN         ms           5
 *    5   UP           ms           5
 *    6   STOP_DRILL   -            1
 *    7   LOCK         -            1
 *    8   UNLOCK       -            1
 *
 *  EVENT FRAME <- ID 0x201 (CAN_ID_EVENT), sent by this board
 *    byte0 = command id echoed
 *    byte1 = event: 0 = RECEIVED, 1 = COMPLETE, 2 = UNKNOWN_CMD
 *
 *  ----------------------------------------------------------------------
 *  SERIAL API (mirrors CAN 1:1, one command per line)
 *  ----------------------------------------------------------------------
 *    CLOSE <steps>
 *    OPEN <steps>
 *    STOP_STEP
 *    DOWN <ms>
 *    UP <ms>
 *    STOP_DRILL
 *    LOCK
 *    UNLOCK
 *  Set ENABLE_SERIAL to 0 to compile out ALL serial I/O.
 *
 *  ----------------------------------------------------------------------
 *  LIMITS / SAFETY
 *  ----------------------------------------------------------------------
 *    - CLOSE/OPEN step count is clamped to [0, MAX_CLAW_STEPS_PER_CMD].
 *    - UP/DOWN duration is clamped to [0, MAX_DRILL_RUN_MS].
 *    - Claw and drill are fully independent; issuing a new CLOSE/OPEN while
 *      one is in progress replaces it (old motion stops, new one starts).
 *      Same for DOWN/UP.
 *    - STEP_EN (N_EN) is only driven LOW (driver enabled) while the claw is
 *      actively stepping, and HIGH (disabled) at rest, to avoid needless
 *      motor heating/holding current.
 *    - LOCK and UNLOCK immediately drive GPIO10 output state (HIGH = 1, LOW = 0).
 * ============================================================================
 */

#include <Arduino.h>
#include "driver/twai.h"

// =============================================================================
#define ENABLE_SERIAL 0
#if ENABLE_SERIAL
  #define DBG_BEGIN(x)     Serial.begin(x)
  #define DBG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_BEGIN(x)
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif

// =============================================================================
//  HARDCODED ACTION CONSTANTS (Adjust your default values here)
// =============================================================================
const uint32_t CLAW_CLOSE_STEPS = 2000;  // Steps for CLOSE command
const uint32_t CLAW_OPEN_STEPS  = 2000;  // Steps for OPEN command
const uint32_t DRILL_DOWN_MS    = 3000;  // Milliseconds for DOWN command
const uint32_t DRILL_UP_MS      = 3000;  // Milliseconds for UP command

// =============================================================================
//  PINS
// =============================================================================
const gpio_num_t PIN_CAN_RX = GPIO_NUM_0;
const gpio_num_t PIN_CAN_TX = GPIO_NUM_1;
const uint8_t PIN_STEP    = 5;
const uint8_t PIN_DIR     = 6;
const uint8_t PIN_STEP_EN = 7;   // active LOW
const uint8_t PIN_RPWM    = 21;  // drill forward
const uint8_t PIN_LPWM    = 20;  // drill reverse
const uint8_t PIN_LOCK    = 10;  // IRLB3813 transistor control pin

// =============================================================================
//  CONSTANTS / LIMITS
// =============================================================================
const bool STEP_EN_ACTIVE_LOW   = true;
const bool DIR_FORWARD_LEVEL    = LOW;   // CLOSE direction
const bool DIR_REVERSE_LEVEL    = HIGH;  // OPEN direction

const uint32_t STEP_PULSE_HIGH_US = 10;   // STEP pulse HIGH time
const uint32_t STEP_INTERVAL_US   = 800;  // time between step pulses

const uint32_t DRILL_PWM_FREQ_HZ    = 20000;
const uint8_t  DRILL_PWM_RESOLUTION = 8;    // bits
const uint8_t  DRILL_PWM_DUTY       = 255;  // fixed run speed (0-255)

const unsigned long SERIAL_BAUD = 115200;

// =============================================================================
//  CAN CONFIG
// =============================================================================
const uint32_t CAN_ID_CMD   = 0x01E;
const uint32_t CAN_ID_EVENT = 0x01F;

enum CmdId : uint8_t {
  CMD_CLOSE      = 1,
  CMD_OPEN       = 2,
  CMD_STOP_STEP  = 3,
  CMD_DOWN       = 4,
  CMD_UP         = 5,
  CMD_STOP_DRILL = 6,
  CMD_LOCK       = 7,
  CMD_UNLOCK     = 8
};

enum EventId : uint8_t {
  EVT_RECEIVED    = 0,
  EVT_COMPLETE    = 1,
  EVT_UNKNOWN_CMD = 2
};

// =============================================================================
//  STATE VARIABLES
// =============================================================================
volatile bool clawActive = false;
uint32_t      clawStepsRemaining = 0;
unsigned long clawLastStepMicros = 0;
bool          clawStepHigh = false;
uint8_t       clawActiveCmd = 0;

volatile bool drillActive = false;
unsigned long drillEndMillis = 0;
uint8_t       drillActiveCmd = 0;

// =============================================================================
//  FORWARD DECLS
// =============================================================================
void sendEvent(uint8_t cmd, EventId evt);
void reportReceived(uint8_t cmd);
void reportComplete(uint8_t cmd);

// =============================================================================
//  CLAW CONTROL
// =============================================================================
void clawSetEnabled(bool enabled) {
  bool level = STEP_EN_ACTIVE_LOW ? !enabled : enabled;
  digitalWrite(PIN_STEP_EN, level ? HIGH : LOW);
}

void clawStart(uint8_t cmd, uint32_t steps) {
  digitalWrite(PIN_DIR, (cmd == CMD_CLOSE) ? DIR_FORWARD_LEVEL : DIR_REVERSE_LEVEL);
  clawStepsRemaining = steps;
  clawActiveCmd = cmd;
  clawStepHigh = false;
  clawLastStepMicros = micros();
  clawActive = (steps > 0);
  clawSetEnabled(clawActive);
  if (!clawActive) reportComplete(cmd);
}

void clawStop(bool announceComplete) {
  bool wasActive = clawActive;
  clawActive = false;
  clawStepsRemaining = 0;
  digitalWrite(PIN_STEP, LOW);
  clawSetEnabled(false);
  if (wasActive && announceComplete) reportComplete(clawActiveCmd);
}

void clawUpdate() {
  if (!clawActive) return;
  unsigned long now = micros();
  if (!clawStepHigh) {
    if (now - clawLastStepMicros >= STEP_INTERVAL_US) {
      digitalWrite(PIN_STEP, HIGH);
      clawStepHigh = true;
      clawLastStepMicros = now;
    }
  } else {
    if (now - clawLastStepMicros >= STEP_PULSE_HIGH_US) {
      digitalWrite(PIN_STEP, LOW);
      clawStepHigh = false;
      clawLastStepMicros = now;
      clawStepsRemaining--;
      if (clawStepsRemaining == 0) {
        clawStop(true);
      }
    }
  }
}

// =============================================================================
//  DRILL CONTROL
// =============================================================================
void drillStart(uint8_t cmd, uint32_t ms) {
  if (cmd == CMD_DOWN) {
    ledcWrite(PIN_RPWM, DRILL_PWM_DUTY);
    ledcWrite(PIN_LPWM, 0);
  } else {
    ledcWrite(PIN_RPWM, 0);
    ledcWrite(PIN_LPWM, DRILL_PWM_DUTY);
  }
  drillActiveCmd = cmd;
  drillEndMillis = millis() + ms;
  drillActive = (ms > 0);
  if (!drillActive) {
    ledcWrite(PIN_RPWM, 0);
    ledcWrite(PIN_LPWM, 0);
    reportComplete(cmd);
  }
}

void drillStop(bool announceComplete) {
  bool wasActive = drillActive;
  drillActive = false;
  ledcWrite(PIN_RPWM, 0);
  ledcWrite(PIN_LPWM, 0);
  if (wasActive && announceComplete) reportComplete(drillActiveCmd);
}

void drillUpdate() {
  if (!drillActive) return;
  if ((long)(millis() - drillEndMillis) >= 0) {
    drillStop(true);
  }
}

// =============================================================================
//  LOCK CONTROL (IRLB3813 Transistor)
// =============================================================================
void lockSet(uint8_t cmd, bool state) {
  digitalWrite(PIN_LOCK, state ? HIGH : LOW);
  reportComplete(cmd);
}

// =============================================================================
//  REPORTING
// =============================================================================
const char* cmdName(uint8_t cmd) {
  switch (cmd) {
    case CMD_CLOSE:      return "CLOSE";
    case CMD_OPEN:       return "OPEN";
    case CMD_STOP_STEP:  return "STOP_STEP";
    case CMD_DOWN:       return "DOWN";
    case CMD_UP:         return "UP";
    case CMD_STOP_DRILL: return "STOP_DRILL";
    case CMD_LOCK:       return "LOCK";
    case CMD_UNLOCK:     return "UNLOCK";
    default:             return "UNKNOWN";
  }
}

void reportReceived(uint8_t cmd) {
  DBG_PRINT("RECEIVED "); DBG_PRINTLN(cmdName(cmd));
  sendEvent(cmd, EVT_RECEIVED);
}

void reportComplete(uint8_t cmd) {
  DBG_PRINT("COMPLETE "); DBG_PRINTLN(cmdName(cmd));
  sendEvent(cmd, EVT_COMPLETE);
}

// =============================================================================
//  COMMAND DISPATCH
// =============================================================================
void dispatchCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_CLOSE:
      reportReceived(cmd);
      clawStart(CMD_CLOSE, CLAW_CLOSE_STEPS);
      break;
    case CMD_OPEN:
      reportReceived(cmd);
      clawStart(CMD_OPEN, CLAW_OPEN_STEPS);
      break;
    case CMD_STOP_STEP:
      reportReceived(cmd);
      clawStop(true);
      break;
    case CMD_DOWN:
      reportReceived(cmd);
      drillStart(CMD_DOWN, DRILL_DOWN_MS);
      break;
    case CMD_UP:
      reportReceived(cmd);
      drillStart(CMD_UP, DRILL_UP_MS);
      break;
    case CMD_STOP_DRILL:
      reportReceived(cmd);
      drillStop(true);
      break;
    case CMD_LOCK:
      reportReceived(cmd);
      lockSet(CMD_LOCK, true);
      break;
    case CMD_UNLOCK:
      reportReceived(cmd);
      lockSet(CMD_UNLOCK, false);
      break;
    default:
      sendEvent(cmd, EVT_UNKNOWN_CMD);
      DBG_PRINT("UNKNOWN CMD "); DBG_PRINTLN(cmd);
      break;
  }
}

// =============================================================================
//  CAN (TWAI) LAYER
// =============================================================================
void canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) { DBG_PRINTLN("CAN install failed"); return; }
  if (twai_start() != ESP_OK) { DBG_PRINTLN("CAN start failed"); return; }
  DBG_PRINTLN("CAN started @1Mbps");
}

void sendEvent(uint8_t cmd, EventId evt) {
  twai_message_t msg = {};
  msg.identifier = CAN_ID_EVENT;
  msg.data_length_code = 2;
  msg.data[0] = cmd;
  msg.data[1] = (uint8_t)evt;
  twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void pollCan() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.identifier == CAN_ID_CMD && msg.data_length_code >= 1) {
      dispatchCommand(msg.data[0]);
    }
  }
}

// =============================================================================
//  SERIAL LAYER
// =============================================================================
#if ENABLE_SERIAL
void handleSerialLine(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  if (line == "CLOSE")           dispatchCommand(CMD_CLOSE);
  else if (line == "OPEN")       dispatchCommand(CMD_OPEN);
  else if (line == "STOP_STEP")  dispatchCommand(CMD_STOP_STEP);
  else if (line == "DOWN")       dispatchCommand(CMD_DOWN);
  else if (line == "UP")         dispatchCommand(CMD_UP);
  else if (line == "STOP_DRILL") dispatchCommand(CMD_STOP_DRILL);
  else if (line == "LOCK")       dispatchCommand(CMD_LOCK);
  else if (line == "UNLOCK")     dispatchCommand(CMD_UNLOCK);
  else { DBG_PRINT("ERR unknown command: "); DBG_PRINTLN(line); }
}

void pollSerial() {
  static String lineBuf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length() > 0) { 
        handleSerialLine(lineBuf); 
        lineBuf = ""; 
      }
    } else {
      lineBuf += c;
    }
  }
}
#endif

// =============================================================================
//  SETUP / LOOP
// =============================================================================
void setup() {
  DBG_BEGIN(SERIAL_BAUD);
  DBG_PRINTLN("Claw+drill firmware starting...");

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  clawSetEnabled(false);

  pinMode(PIN_LOCK, OUTPUT);
  digitalWrite(PIN_LOCK, LOW);

  ledcAttach(PIN_RPWM, DRILL_PWM_FREQ_HZ, DRILL_PWM_RESOLUTION);
  ledcAttach(PIN_LPWM, DRILL_PWM_FREQ_HZ, DRILL_PWM_RESOLUTION);
  ledcWrite(PIN_RPWM, 0);
  ledcWrite(PIN_LPWM, 0);

  canInit();
  DBG_PRINTLN("Ready.");
}

void loop() {
  pollCan();
#if ENABLE_SERIAL
  pollSerial();
#endif
  clawUpdate();
  drillUpdate();
}