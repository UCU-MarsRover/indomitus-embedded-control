/* ============================================================================
 *  SAFE GRIPPER END-EFFECTOR FIRMWARE  —  ESP32-C3 Supermini
 * ============================================================================
 *
 *  HARDWARE
 *  --------
 *    Servo (gripper actuator) ..... GPIO7
 *    Right load cell (HX711) DAT .. GPIO3
 *    Right load cell (HX711) SCK .. GPIO4
 *    Left  load cell (HX711) DAT .. GPIO21
 *    Left  load cell (HX711) SCK .. GPIO20
 *    CAN (TWAI) TX ................ GPIO1   -> SN65HVD230 "D" (TXD)
 *    CAN (TWAI) RX ................ GPIO0   -> SN65HVD230 "R" (RXD)
 *
 *    NOTE: GPIO0 is an ESP32-C3 boot-strapping pin. Make sure nothing on the
 *    CAN bus pulls it low during power-on/reset, or the board may fail to
 *    boot into normal run mode.
 *
 *    TIP: HX711 boards usually have a RATE pin that selects 10 SPS (pin low
 *    / floating) or 80 SPS (pin tied to VCC). Wiring RATE high on both load
 *    cells gives the control loop fresher data more often and makes the
 *    step-gating described below react faster. Not required, just faster.
 *
 *  OVERVIEW
 *  --------
 *  This firmware drives a single servo-actuated 2-finger gripper equipped
 *  with one load cell per finger (max 1 kg / 1000 g rated each). It exposes
 *  a small command/response API over CAN (primary) and a mirrored text
 *  command API over USB Serial (secondary, for bench testing). All motion
 *  is governed by the "safe" motion rules described below, which are
 *  enforced identically regardless of whether the command arrived via CAN
 *  or Serial.
 *
 *  There is one physical axis: the servo angle, constrained to
 *  [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE]. "Spread" (finger opening, in mm) is
 *  a linear function of that same angle (see MM_TO_ANGLE_COEFF below) and
 *  is just a convenience unit — internally everything is angle-based.
 *
 *  Convention used throughout this firmware:
 *    - INCREASING angle  = CLOSING the gripper  (fingers move together,
 *                            grip load on the sensors normally goes UP)
 *    - DECREASING angle  = OPENING the gripper (fingers move apart,
 *                            grip load normally goes DOWN / negative)
 *  If your mechanical linkage is reversed, flip SERVO_MIN_ANGLE/MAX usage
 *  in safeOpen()/safeClose() or invert MM_TO_ANGLE_COEFF.
 *
 *  ----------------------------------------------------------------------
 *  SERVO RESOLUTION: MICROSECONDS, NOT DEGREES  (important — see below)
 *  ----------------------------------------------------------------------
 *  The Arduino Servo API's write(int) only accepts WHOLE degrees. Since
 *  this firmware's step sizes are fractions of a degree (as small as
 *  SERVO_STEP_DEG_MIN = 0.02 deg), calling write() would silently throw
 *  almost all of that precision away: the commanded angle would accumulate
 *  internally but the servo wouldn't actually move until the accumulated
 *  float crossed a whole-degree boundary, at which point it would jump a
 *  full degree at once. That quantization is enough by itself to look like
 *  jittery/twitchy motion, especially once step sizes are under ~1 degree.
 *  To get real sub-degree resolution, writeServo() below computes a pulse
 *  width in MICROSECONDS directly from the float angle and calls
 *  gripperServo.writeMicroseconds() instead of write(). This assumes your
 *  servo's usable travel maps linearly across SERVO_PULSE_MIN_US..
 *  SERVO_PULSE_MAX_US over a 0-180 degree range (the same range implied by
 *  gripperServo.attach(PIN_SERVO, 500, 2500) in setup() — adjust both
 *  together if your servo's datasheet says otherwise). This gives roughly
 *  (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180 = ~11 us per degree, so
 *  effective resolution is roughly a tenth of a degree — far finer than
 *  the mechanical backlash of most hobby servos/linkages, which is
 *  ultimately what limits how smooth the visible motion can be.
 *
 *  ----------------------------------------------------------------------
 *  SAFETY MODEL  (all motion, from any source, goes through this)
 *  ----------------------------------------------------------------------
 *  The control loop is polled every SERVO_STEP_INTERVAL_MS, but it only
 *  ever MOVES the servo when at least one load cell has produced a FRESH
 *  reading since the last step ("sample-gated actuation" — see below). A
 *  step size is chosen from a tiered speed profile driven by how much load
 *  is currently felt ("load-based tiered stepping" — see below), which is
 *  what makes closing/opening/holding move briskly through open air but
 *  stop precisely instead of overshooting once contact happens.
 *
 *  0) SAMPLE-GATED ACTUATION  (fixes overshoot caused by stale readings)
 *     HX711 modules only produce a new conversion every ~12.5 ms (80 SPS)
 *     to ~100 ms (10 SPS). If the servo were stepped on a fixed timer
 *     faster than that, it would take several steps "blind" on the same
 *     old load value before the next real reading arrived — this is what
 *     caused closing to stop too late and holding to open too far. To fix
 *     this, every control tick checks HX711 is_ready() for each sensor;
 *     the safety/mode state machine (and therefore the servo) only
 *     advances on a tick where at least one sensor actually delivered a
 *     new value. Ticks with no fresh data do nothing.
 *
 *  1) HARD ANGLE LIMITS
 *     The commanded angle is always clamped to
 *     [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE]. No command can move outside this
 *     range.
 *
 *  2) LOAD-BASED TIERED STEPPING WHILE ACTIVELY MOVING TOWARD A TARGET
 *     Used by Safe Set Angle / Safe Set Spread / Safe Open / Safe Close,
 *     and by the global emergency overload recovery (rule 3). Step size
 *     is chosen from three tiers based on the load felt in the direction
 *     of travel:
 *       - load <= LOAD_FAST_ZONE_G: step = SERVO_STEP_DEG_FAST — little/no
 *         resistance yet, move briskly.
 *       - load further out than LOAD_APPROACH_MARGIN_G from the relevant
 *         stop threshold: step = SERVO_STEP_DEG_CRUISE — some resistance
 *         is felt, move cautiously.
 *       - within LOAD_APPROACH_MARGIN_G of the stop threshold: step eases
 *         smoothly from SERVO_STEP_DEG_CRUISE down to SERVO_STEP_DEG_MIN
 *         as the threshold is approached, and motion fully stops the
 *         instant the threshold is crossed.
 *     This is what makes Safe Close behave like "close briskly until you
 *     feel the object, slowing right down right before contact," instead
 *     of crawling the whole way or slamming into the stop at full speed.
 *     Motion also stops if the target is abandoned (angle held where it
 *     stopped).
 *
 *  3) GLOBAL EMERGENCY OVERLOAD PROTECTION (always active, top priority)
 *     Independent of what mode the gripper is in (including idle and
 *     hold), every actuated tick checks:
 *       - if either sensor > LOAD_EMERGENCY_HIGH_G: the angle is stepped
 *         DOWN (opened), ignoring whatever mode/target was active, using
 *         the same tiered stepping as rule (2) as it approaches
 *         LOAD_STEP_STOP_HIGH_G, which is the release point.
 *       - if either sensor < LOAD_EMERGENCY_LOW_G: the angle is stepped UP
 *         the same way until back at LOAD_STEP_STOP_LOW_G or higher.
 *     This can interrupt and override Safe Set Angle/Spread, Open, Close,
 *     or Hold. Once relieved, whatever mode was active resumes on the next
 *     actuated tick.
 *
 *     IMPORTANT — margin between LOAD_HOLD_TARGET_G and
 *     LOAD_EMERGENCY_HIGH_G: the emergency check looks at the WORSE of the
 *     two sensors (max), but Safe Hold's target/deadband is on their
 *     AVERAGE. A real two-finger grasp is rarely perfectly balanced, so
 *     Hold can sit "on target" on average while one finger already carries
 *     noticeably more than the other. If LOAD_EMERGENCY_HIGH_G is too
 *     close to LOAD_HOLD_TARGET_G, that normal imbalance is enough to trip
 *     the emergency check, which yanks the angle open, drops the average
 *     below Hold's deadband, and Hold immediately drives back up into the
 *     same imbalance again — a fast open/close fight that looks like
 *     twitching. Keep at least ~150-200 g of headroom between
 *     LOAD_HOLD_TARGET_G and LOAD_EMERGENCY_HIGH_G (and remember the load
 *     cells are only rated to LOAD_CELL_RATED_MAX_G) so ordinary grip
 *     imbalance doesn't false-trigger the emergency path. There is no way
 *     for this single-actuator gripper to correct per-finger imbalance
 *     directly (one servo drives both fingers via one linkage) — the
 *     margin is what keeps normal imbalance from being mistaken for an
 *     overload.
 *
 *  4) SAFE OPEN / SAFE CLOSE
 *     One-shot actions. Safe Open drives the angle down toward
 *     SERVO_MIN_ANGLE; Safe Close drives it up toward SERVO_MAX_ANGLE.
 *     Each walks step-by-step exactly like rule (2) above (sample-gated,
 *     tiered) and stops the first time it either (a) reaches the
 *     mechanical limit, or (b) trips the relevant soft-stop load
 *     threshold.
 *
 *  5) SAFE HOLD  (proportional, not fixed-step)
 *     Start Safe Hold enters a mode that continuously servos the angle to
 *     keep grip load near LOAD_HOLD_TARGET_G, using a small deadband
 *     (LOAD_HOLD_TOLERANCE_G) to avoid jitter. Unlike open/close/move, the
 *     step size here is PROPORTIONAL to how far the average load is from
 *     the target (error * HOLD_STEP_GAIN, clamped between
 *     SERVO_STEP_DEG_MIN and SERVO_STEP_DEG_HOLD_MAX):
 *       - average load far above target  -> a larger corrective step down
 *       - average load just above target -> a tiny step down
 *       - average load far below target  -> a larger corrective step up
 *       - average load just below target -> a tiny step up
 *       - within the deadband            -> no movement
 *     Hold intentionally uses its own, smaller step ceiling
 *     (SERVO_STEP_DEG_HOLD_MAX) rather than SERVO_STEP_DEG_FAST: Hold is
 *     fine force-control around an already-settled grip, not point-to-
 *     point travel, so it should not take large jumps. This still only
 *     actuates on sample-gated ticks (rule 0), so it settles onto the
 *     target load smoothly instead of overshooting past it and dropping
 *     the object. The global emergency overload rule (3) still applies on
 *     top of this and can override an individual tick — see the margin
 *     note under rule (3) above.
 *
 *  ----------------------------------------------------------------------
 *  SPREAD <-> ANGLE CONVERSION
 *  ----------------------------------------------------------------------
 *    angle  = ANGLE_AT_ZERO_SPREAD + spread_mm * MM_TO_ANGLE_COEFF
 *    spread = (angle - ANGLE_AT_ZERO_SPREAD) / MM_TO_ANGLE_COEFF
 *  MM_TO_ANGLE_COEFF (deg/mm) and ANGLE_AT_ZERO_SPREAD are placeholders —
 *  measure your linkage and update the constants below. Safe Set Spread
 *  simply converts mm -> angle with this formula and then behaves exactly
 *  like Safe Set Angle (full safety rules 1-3 apply).
 *
 *  ----------------------------------------------------------------------
 *  CAN API   (11-bit standard IDs, classic CAN, up to 8 data bytes)
 *  ----------------------------------------------------------------------
 *  All multi-byte fields are little-endian, signed 16-bit (int16_t), unless
 *  noted otherwise. Angles and spreads are transmitted as fixed-point,
 *  value*10 (i.e. 0.1 degree / 0.1 mm resolution). Loads are plain grams.
 *
 *  COMMAND FRAME   -> ID 0x01A (CAN_ID_CMD), sent by the control panel
 *    byte0        = command id (see table)
 *    byte1..2     = int16 parameter, LE (only for commands that need one)
 *    (unused bytes may be omitted / zero)
 *
 *    Command id  Name                    Parameter               DLC
 *    ----------  ----------------------  ----------------------  ---
 *      1         SAFE_SET_SPREAD         spread_mm * 10 (int16)   3
 *      2         SAFE_SET_ANGLE          angle_deg * 10 (int16)   3
 *      3         SAFE_OPEN               none                     1
 *      4         SAFE_CLOSE              none                     1
 *      5         START_SAFE_HOLD         none                     1
 *      6         STOP_SAFE_HOLD          none                     1
 *      7         READ_LOAD_SENSORS       none                     1
 *      8         READ_ANGLE              none                     1
 *      9         READ_SPREAD             none                     1
 *     10         READ_GRIPPER_INFO       none                     1
 *
 *  RESPONSE FRAMES, all sent by the gripper on ID 0x01B (CAN_ID_ACK) —
 *  only two CAN IDs are used in total (0x01A in, 0x01B out). Since the
 *  protocol is strictly request/response (one command, one reply), the
 *  requester knows which layout to expect from the command it just sent:
 *
 *    Reply to an ACTION command (1-6)
 *      byte0 = command id echoed
 *      byte1 = status: 0 = OK, 1 = REJECTED (parameter out of range),
 *                      2 = UNKNOWN COMMAND
 *
 *    Reply to READ_LOAD_SENSORS (cmd 7)
 *      byte0..1 = right load, grams (int16)
 *      byte2..3 = left  load, grams (int16)
 *
 *    Reply to READ_ANGLE (cmd 8)
 *      byte0..1 = current TARGET angle * 10 (int16, degrees)
 *
 *    Reply to READ_SPREAD (cmd 9)
 *      byte0..1 = current TARGET spread * 10 (int16, mm)
 *
 *    Reply to READ_GRIPPER_INFO (cmd 10)
 *      byte0..1 = right load, grams (int16)
 *      byte2..3 = left  load, grams (int16)
 *      byte4..5 = target angle  * 10 (int16, degrees)
 *      byte6..7 = target spread * 10 (int16, mm)
 *
 *  CAN always takes priority over Serial: CAN messages are polled and
 *  processed before Serial input in every loop() iteration.
 *
 *  ----------------------------------------------------------------------
 *  SERIAL API  (secondary / bench-test interface, mirrors CAN 1:1)
 *  ----------------------------------------------------------------------
 *  One command per line, space-separated, case-insensitive. Set
 *  ENABLE_SERIAL to 0 below to compile out ALL serial I/O.
 *
 *    SETSPREAD <mm>      e.g. "SETSPREAD 40.0"
 *    SETANGLE <deg>      e.g. "SETANGLE 120"
 *    OPEN
 *    CLOSE
 *    HOLD                (start safe hold)
 *    UNHOLD              (stop safe hold)
 *    READLOAD
 *    READANGLE
 *    READSPREAD
 *    READINFO
 *
 *  In addition, whenever serial is enabled, the firmware prints the full
 *  gripper info line every SERIAL_REPORT_INTERVAL_MS (250 ms default)
 *  unprompted, for live monitoring.
 *
 *  ----------------------------------------------------------------------
 *  LOAD-BASED TIERED STEPPING, AND WHY FORWARD MOVEMENT IS RATE-LIMITED
 *  ----------------------------------------------------------------------
 *  A single fixed step size can't satisfy both "fast when there's nothing
 *  to feel" and "precise when a stop is close" at once. So step size is
 *  chosen per actuated tick from travelStep(), based on how much load is
 *  currently felt in the direction of travel:
 *    1) load <= LOAD_FAST_ZONE_G          -> SERVO_STEP_DEG_FAST  (brisk)
 *    2) load further than LOAD_APPROACH_MARGIN_G from the stop threshold
 *                                          -> SERVO_STEP_DEG_CRUISE (cautious)
 *    3) within LOAD_APPROACH_MARGIN_G of the stop threshold
 *                                          -> eases CRUISE -> MIN (precise)
 *  On top of that, forward movement (continuing to approach a
 *  target/hold point) is additionally rate-limited to at most one step
 *  every SERVO_ACTUATION_MIN_INTERVAL_MS, regardless of how often fresh
 *  samples arrive — this gives the mechanism time to physically settle
 *  and the next load reading time to reflect that settled state, instead
 *  of stacking several steps on top of each other before the load has
 *  caught up. At the default 10 SPS HX711 rate (~100 ms/sample) this
 *  rate limit rarely binds since sample-gating alone already paces things
 *  to ~100 ms; it mainly matters if you switch to 80 SPS.
 *  IMPORTANT: only forward movement is rate-limited. The stop condition
 *  itself (crossing LOAD_STEP_STOP_HIGH_G / LOW_G, or the emergency
 *  thresholds) is still checked and applied on every single fresh sample,
 *  with no delay — stopping is never throttled, only continuing to move
 *  is. This is what keeps the "stop" precise while still moving briskly
 *  on the way there.
 *
 *  ----------------------------------------------------------------------
 *  TUNING GUIDE
 *  ----------------------------------------------------------------------
 *    - Motion through open air still feels slow  -> raise
 *      SERVO_STEP_DEG_FAST and/or LOAD_FAST_ZONE_G, or lower
 *      SERVO_ACTUATION_MIN_INTERVAL_MS, or wire the HX711 RATE pin high
 *      for 80 SPS (faster fresh samples).
 *    - Overshoots on close/open near contact       -> lower
 *      SERVO_STEP_DEG_CRUISE and/or SERVO_STEP_DEG_MIN, and/or raise
 *      LOAD_APPROACH_MARGIN_G (starts easing sooner), and/or raise
 *      SERVO_ACTUATION_MIN_INTERVAL_MS.
 *    - Motion/holding TWITCHES (rapid little back-and-forth) -> this is
 *      almost always one of two things: (a) LOAD_HOLD_TARGET_G is too
 *      close to LOAD_EMERGENCY_HIGH_G, so ordinary per-finger imbalance
 *      trips the emergency path and fights Hold's average-based
 *      correction (see the margin note under safety rule 3 above) - widen
 *      the gap between them; or (b) step sizes below ~1 degree are being
 *      quantized away by the servo's whole-degree write() call - make
 *      sure writeServo() is using writeMicroseconds() (see "SERVO
 *      RESOLUTION" note above), not write().
 *    - Hold oscillates around the target (not sharp twitching, more of a
 *      slow wobble)                                 -> lower HOLD_STEP_GAIN,
 *      or widen LOAD_HOLD_TOLERANCE_G.
 *    - Hold reacts too slowly to slipping object     -> raise HOLD_STEP_GAIN,
 *      or raise SERVO_STEP_DEG_HOLD_MAX (its clamp ceiling).
 *
 *  ----------------------------------------------------------------------
 *  All limits/timings/pins below are declared as named constants so they
 *  can be retuned quickly without touching logic.
 * ============================================================================
 */
#include <Arduino.h>
#include <ESP32Servo.h>
#include <HX711.h>
#include "driver/twai.h"

// =============================================================================
//  MASTER SERIAL ENABLE / DISABLE  (set to 0 to compile out ALL serial comms)
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
//  PIN DEFINITIONS
// =============================================================================
const uint8_t PIN_SERVO             = 7;
const uint8_t PIN_HX711_RIGHT_DAT   = 3;
const uint8_t PIN_HX711_RIGHT_SCK   = 4;
const uint8_t PIN_HX711_LEFT_DAT    = 21;
const uint8_t PIN_HX711_LEFT_SCK    = 20;
const gpio_num_t PIN_CAN_TX         = GPIO_NUM_1;
const gpio_num_t PIN_CAN_RX         = GPIO_NUM_0;

// =============================================================================
//  SERVO ANGLE & PULSE WIDTH LIMITS  (sub-degree microsecond resolution)
// =============================================================================
const int   SERVO_MIN_ANGLE       = 0;
const int   SERVO_MAX_ANGLE       = 86;
const int   SERVO_PULSE_MIN_US    = 500;
const int   SERVO_PULSE_MAX_US    = 2500;
const float SERVO_PULSE_RANGE_DEG = 180.0f;

// =============================================================================
//  LOAD CELL CALIBRATION & RATING  (10kg rated)
// =============================================================================
const float RIGHT_CALIBRATION_FACTOR = 208.38f;
const float LEFT_CALIBRATION_FACTOR  = 208.38f;
const float LOAD_CELL_RATED_MAX_G    = 10000.0f;  // 10 kg rated max per sensor

// =============================================================================
//  LOAD SAFETY THRESHOLDS  (grams — scaled for 10kg capacity)
// =============================================================================
const float LOAD_STEP_STOP_HIGH_G  = 1500.0f;  // soft stop while closing (~2.5 kg target grip)
const float LOAD_STEP_STOP_LOW_G   = -1500.0f; // soft stop while opening
const float LOAD_EMERGENCY_HIGH_G  = 2000.0f;  // hard safety ceiling (2,000 g headroom above target)
const float LOAD_EMERGENCY_LOW_G   = -2000.0f; // hard floor safety override
const float LOAD_HOLD_TARGET_G     = 1500.0f;  // target grip force in Hold mode
const float LOAD_HOLD_TOLERANCE_G  = 300.0f;   // deadband (+/- 100 g) around hold target

// =============================================================================
//  MOTION STEPPING  (optimized for 80 Hz sampling rate ~12.5ms)
// =============================================================================
const float SERVO_STEP_DEG_FAST     = 5.0f;   // degrees per tick in open air (~320 deg/sec max velocity)
const float SERVO_STEP_DEG_CRUISE   = 0.4f;   // cautious stepping once contact force is felt
const float SERVO_STEP_DEG_MIN      = 0.02f;  // fine precision step right at touch/target
const float LOAD_FAST_ZONE_G        = 100.0f;  // load below which FAST speed is used (above noise floor)
const float LOAD_APPROACH_MARGIN_G  = 200.0f;  // gram range over which step size eases CRUISE -> MIN

// Polling and Actuation Timing for 80 Hz HX711 (1 sample every ~12.5 ms)
const unsigned long SERVO_STEP_INTERVAL_MS          = 10; // check for fresh samples every 10 ms
const unsigned long SERVO_ACTUATION_MIN_INTERVAL_MS = 12; // minimum delay between forward steps (12 ms = 80 Hz)

// Hold mode proportional correction
const float SERVO_STEP_DEG_HOLD_MAX = 1.0f;    // max corrective step per 12.5ms sample during hold
const float HOLD_STEP_GAIN          = 0.03f; // proportional gain (deg/g) scaled for higher load range

// Slew rate filter for 10kg cell @ 80Hz (rejects power spikes up to 1,500 g/sample)
const float LOAD_MAX_SLEW_G_PER_SAMPLE = 1500.0f;

// =============================================================================
//  SPREAD <-> ANGLE CONVERSION
// =============================================================================
const float MM_TO_ANGLE_COEFF     = 1.0f;                 // degrees of servo travel per mm spread
const float ANGLE_AT_ZERO_SPREAD  = (float)SERVO_MIN_ANGLE; // angle at 0 mm spread

// =============================================================================
//  SERIAL & CAN TIMING
// =============================================================================
const unsigned long SERIAL_BAUD              = 115200;
const unsigned long SERIAL_REPORT_INTERVAL_MS = 100; // faster live stream (10 Hz output)

const uint32_t CAN_ID_CMD = 0x01A;
const uint32_t CAN_ID_ACK = 0x01B;

enum CmdId : uint8_t {
  CMD_SAFE_SET_SPREAD   = 1,
  CMD_SAFE_SET_ANGLE    = 2,
  CMD_SAFE_OPEN         = 3,
  CMD_SAFE_CLOSE        = 4,
  CMD_START_SAFE_HOLD   = 5,
  CMD_STOP_SAFE_HOLD    = 6,
  CMD_READ_LOAD_SENSORS = 7,
  CMD_READ_ANGLE        = 8,
  CMD_READ_SPREAD        = 9,
  CMD_READ_GRIPPER_INFO = 10
};

enum AckStatus : uint8_t {
  ACK_OK             = 0,
  ACK_REJECTED_RANGE = 1,
  ACK_UNKNOWN_CMD    = 2
};

// =============================================================================
//  GRIPPER STATE & HARDWARE OBJECTS
// =============================================================================
enum GripperMode : uint8_t {
  MODE_IDLE = 0,
  MODE_MOVE_TO_TARGET,
  MODE_OPEN,
  MODE_CLOSE,
  MODE_HOLD
};

Servo   gripperServo;
HX711   rightScale;
HX711   leftScale;

GripperMode gripperMode = MODE_IDLE;

float currentAngle = (float)SERVO_MIN_ANGLE;
float targetAngle  = (float)SERVO_MIN_ANGLE;

float lastRightLoad_g = 0.0f;
float lastLeftLoad_g  = 0.0f;

int8_t emergencyDir = 0; // 0 = normal, -1 = opening (relieving high load), +1 = closing

unsigned long lastActuationMs = 0;

// =============================================================================
//  HELPERS
// =============================================================================
float spreadToAngle(float spread_mm) {
  float angle = ANGLE_AT_ZERO_SPREAD + spread_mm * MM_TO_ANGLE_COEFF;
  return constrain(angle, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
}

float angleToSpread(float angle_deg) {
  return (angle_deg - ANGLE_AT_ZERO_SPREAD) / MM_TO_ANGLE_COEFF;
}

void writeServo(float angle_deg) {
  angle_deg = constrain(angle_deg, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
  float us = SERVO_PULSE_MIN_US +
             (angle_deg / SERVO_PULSE_RANGE_DEG) * (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
  gripperServo.writeMicroseconds((int)lroundf(us));
}

// Returns true if at least one scale delivered a fresh reading this check cycle
bool updateLoadReadings() {
  bool fresh = false;
  if (rightScale.is_ready()) {
    float raw = rightScale.get_units(1);
    float delta = constrain(raw - lastRightLoad_g, -LOAD_MAX_SLEW_G_PER_SAMPLE, LOAD_MAX_SLEW_G_PER_SAMPLE);
    lastRightLoad_g += delta;
    fresh = true;
  }
  if (leftScale.is_ready()) {
    float raw = leftScale.get_units(1);
    float delta = constrain(raw - lastLeftLoad_g, -LOAD_MAX_SLEW_G_PER_SAMPLE, LOAD_MAX_SLEW_G_PER_SAMPLE);
    lastLeftLoad_g += delta;
    fresh = true;
  }
  return fresh;
}

float travelStep(float load_g, float stopThreshold_g) {
  if (load_g <= LOAD_FAST_ZONE_G) return SERVO_STEP_DEG_FAST;
  float remaining = fabsf(stopThreshold_g - load_g);
  if (remaining <= LOAD_APPROACH_MARGIN_G) {
    float t = remaining / LOAD_APPROACH_MARGIN_G;
    return SERVO_STEP_DEG_MIN + t * (SERVO_STEP_DEG_CRUISE - SERVO_STEP_DEG_MIN);
  }
  return SERVO_STEP_DEG_CRUISE;
}

// =============================================================================
//  CORE SAFE MOTION CONTROL LOOP
// =============================================================================
void controlStep() {
  bool fresh = updateLoadReadings();
  if (!fresh) return; // Sample-gated actuation: only move when fresh sensor data arrives

  float maxLoad = max(lastRightLoad_g, lastLeftLoad_g);
  float minLoad = min(lastRightLoad_g, lastLeftLoad_g);

  bool canActuate = (millis() - lastActuationMs) >= SERVO_ACTUATION_MIN_INTERVAL_MS;

  // ---- 1) Global emergency overload protection (Highest Priority) ----------
  if (emergencyDir == 0) {
    if (maxLoad > LOAD_EMERGENCY_HIGH_G) {
      emergencyDir = -1; // force open
      if (gripperMode == MODE_MOVE_TO_TARGET || gripperMode == MODE_OPEN || gripperMode == MODE_CLOSE) {
        gripperMode = MODE_IDLE;
      }
    } else if (minLoad < LOAD_EMERGENCY_LOW_G) {
      emergencyDir = +1; // force close
      if (gripperMode == MODE_MOVE_TO_TARGET || gripperMode == MODE_OPEN || gripperMode == MODE_CLOSE) {
        gripperMode = MODE_IDLE;
      }
    }
  }

  if (emergencyDir == -1) {
    if (maxLoad <= LOAD_STEP_STOP_HIGH_G) {
      emergencyDir = 0; // overload relieved
    } else {
      if (canActuate) {
        float step = travelStep(maxLoad, LOAD_STEP_STOP_HIGH_G);
        currentAngle = max(currentAngle - step, (float)SERVO_MIN_ANGLE);
        writeServo(currentAngle);
        lastActuationMs = millis();
        targetAngle = currentAngle;
      }
      return;
    }
  } else if (emergencyDir == +1) {
    if (minLoad >= LOAD_STEP_STOP_LOW_G) {
      emergencyDir = 0;
    } else {
      if (canActuate) {
        float step = travelStep(-minLoad, -LOAD_STEP_STOP_LOW_G);
        currentAngle = min(currentAngle + step, (float)SERVO_MAX_ANGLE);
        writeServo(currentAngle);
        lastActuationMs = millis();
        targetAngle = currentAngle;
      }
      return;
    }
  }

  // ---- 2) Mode-specific motion control --------------------------------------
  switch (gripperMode) {

    case MODE_IDLE:
      break;

    case MODE_MOVE_TO_TARGET:
    case MODE_OPEN:
    case MODE_CLOSE: {
      if (currentAngle < targetAngle) {
        // closing
        if (maxLoad > LOAD_STEP_STOP_HIGH_G) {
          gripperMode = MODE_IDLE;    // instantaneous stop
          targetAngle = currentAngle;
        } else if (canActuate) {
          float step = travelStep(maxLoad, LOAD_STEP_STOP_HIGH_G);
          currentAngle = min(currentAngle + step, targetAngle);
          writeServo(currentAngle);
          lastActuationMs = millis();
        }
      } else if (currentAngle > targetAngle) {
        // opening
        if (minLoad < LOAD_STEP_STOP_LOW_G) {
          gripperMode = MODE_IDLE;
          targetAngle = currentAngle;
        } else if (canActuate) {
          float step = travelStep(-minLoad, -LOAD_STEP_STOP_LOW_G);
          currentAngle = max(currentAngle - step, targetAngle);
          writeServo(currentAngle);
          lastActuationMs = millis();
        }
      } else {
        gripperMode = MODE_IDLE;
      }
      return;
    }

    case MODE_HOLD: {
      float avgLoad = (lastRightLoad_g + lastLeftLoad_g) / 2.0f;
      float error = avgLoad - LOAD_HOLD_TARGET_G;
      if (!canActuate) return;

      if (error > LOAD_HOLD_TOLERANCE_G) {
        float step = constrain(error * HOLD_STEP_GAIN, SERVO_STEP_DEG_MIN, SERVO_STEP_DEG_HOLD_MAX);
        currentAngle = max(currentAngle - step, (float)SERVO_MIN_ANGLE);
        writeServo(currentAngle);
        lastActuationMs = millis();
      } else if (error < -LOAD_HOLD_TOLERANCE_G) {
        float step = constrain(-error * HOLD_STEP_GAIN, SERVO_STEP_DEG_MIN, SERVO_STEP_DEG_HOLD_MAX);
        currentAngle = min(currentAngle + step, (float)SERVO_MAX_ANGLE);
        writeServo(currentAngle);
        lastActuationMs = millis();
      }
      targetAngle = currentAngle;
      return;
    }
  }
}

// =============================================================================
//  COMMAND HANDLING
// =============================================================================
AckStatus dispatchActionCommand(uint8_t cmd, bool haveParam, float paramValue) {
  switch (cmd) {

    case CMD_SAFE_SET_SPREAD: {
      if (!haveParam) return ACK_REJECTED_RANGE;
      float angle = spreadToAngle(paramValue);
      targetAngle = angle;
      gripperMode = MODE_MOVE_TO_TARGET;
      return ACK_OK;
    }

    case CMD_SAFE_SET_ANGLE: {
      if (!haveParam) return ACK_REJECTED_RANGE;
      paramValue = constrain(paramValue, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
      targetAngle = paramValue;
      gripperMode = MODE_MOVE_TO_TARGET;
      return ACK_OK;
    }

    case CMD_SAFE_OPEN:
      targetAngle = (float)SERVO_MIN_ANGLE;
      gripperMode = MODE_OPEN;
      return ACK_OK;

    case CMD_SAFE_CLOSE:
      targetAngle = (float)SERVO_MAX_ANGLE;
      gripperMode = MODE_CLOSE;
      return ACK_OK;

    case CMD_START_SAFE_HOLD:
      gripperMode = MODE_HOLD;
      targetAngle = currentAngle;
      return ACK_OK;

    case CMD_STOP_SAFE_HOLD:
      if (gripperMode == MODE_HOLD) {
        gripperMode = MODE_IDLE;
        targetAngle = currentAngle;
      }
      return ACK_OK;

    default:
      return ACK_UNKNOWN_CMD;
  }
}

// =============================================================================
//  CAN (TWAI) DRIVER & MESSAGING
// =============================================================================
void canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    DBG_PRINTLN("CAN: driver install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    DBG_PRINTLN("CAN: start failed");
    return;
  }
  DBG_PRINTLN("CAN: started @500kbps");
}

void canSend(uint32_t id, const uint8_t *data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;
  for (uint8_t i = 0; i < len && i < 8; i++) msg.data[i] = data[i];
  twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void canSendAck(uint8_t cmd, AckStatus status) {
  uint8_t d[2] = { cmd, (uint8_t)status };
  canSend(CAN_ID_ACK, d, 2);
}

void canSendLoad() {
  int16_t r = (int16_t)lroundf(lastRightLoad_g);
  int16_t l = (int16_t)lroundf(lastLeftLoad_g);
  uint8_t d[4];
  memcpy(&d[0], &r, 2);
  memcpy(&d[2], &l, 2);
  canSend(CAN_ID_ACK, d, 4);
}

void canSendAngle() {
  int16_t a = (int16_t)lroundf(targetAngle * 10.0f);
  uint8_t d[2];
  memcpy(&d[0], &a, 2);
  canSend(CAN_ID_ACK, d, 2);
}

void canSendSpread() {
  int16_t s = (int16_t)lroundf(angleToSpread(targetAngle) * 10.0f);
  uint8_t d[2];
  memcpy(&d[0], &s, 2);
  canSend(CAN_ID_ACK, d, 2);
}

void canSendInfo() {
  int16_t r = (int16_t)lroundf(lastRightLoad_g);
  int16_t l = (int16_t)lroundf(lastLeftLoad_g);
  int16_t a = (int16_t)lroundf(targetAngle * 10.0f);
  int16_t s = (int16_t)lroundf(angleToSpread(targetAngle) * 10.0f);
  uint8_t d[8];
  memcpy(&d[0], &r, 2);
  memcpy(&d[2], &l, 2);
  memcpy(&d[4], &a, 2);
  memcpy(&d[6], &s, 2);
  canSend(CAN_ID_ACK, d, 8);
}

void handleCanCommand(const twai_message_t &msg) {
  if (msg.data_length_code < 1) return;
  uint8_t cmd = msg.data[0];

  bool haveParam = msg.data_length_code >= 3;
  int16_t rawParam = 0;
  if (haveParam) memcpy(&rawParam, &msg.data[1], 2);

  switch (cmd) {
    case CMD_SAFE_SET_SPREAD: {
      float mm = rawParam / 10.0f;
      canSendAck(cmd, dispatchActionCommand(cmd, true, mm));
      break;
    }
    case CMD_SAFE_SET_ANGLE: {
      float deg = rawParam / 10.0f;
      canSendAck(cmd, dispatchActionCommand(cmd, true, deg));
      break;
    }
    case CMD_SAFE_OPEN:
    case CMD_SAFE_CLOSE:
    case CMD_START_SAFE_HOLD:
    case CMD_STOP_SAFE_HOLD:
      canSendAck(cmd, dispatchActionCommand(cmd, false, 0));
      break;

    case CMD_READ_LOAD_SENSORS:
      canSendLoad();
      break;
    case CMD_READ_ANGLE:
      canSendAngle();
      break;
    case CMD_READ_SPREAD:
      canSendSpread();
      break;
    case CMD_READ_GRIPPER_INFO:
      canSendInfo();
      break;

    default:
      canSendAck(cmd, ACK_UNKNOWN_CMD);
      break;
  }
}

void pollCan() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.identifier == CAN_ID_CMD) {
      handleCanCommand(msg);
    }
  }
}

// =============================================================================
//  SERIAL CLI INTERFACE
// =============================================================================
#if ENABLE_SERIAL

void printGripperInfo() {
  DBG_PRINT("R="); DBG_PRINT(lastRightLoad_g, 1);
  DBG_PRINT("g L="); DBG_PRINT(lastLeftLoad_g, 1);
  DBG_PRINT("g TargetAngle="); DBG_PRINT(targetAngle, 1);
  DBG_PRINT("deg TargetSpread="); DBG_PRINT(angleToSpread(targetAngle), 1);
  DBG_PRINT("mm Mode=");
  switch (gripperMode) {
    case MODE_IDLE:           DBG_PRINTLN("IDLE"); break;
    case MODE_MOVE_TO_TARGET: DBG_PRINTLN("MOVE"); break;
    case MODE_OPEN:           DBG_PRINTLN("OPEN"); break;
    case MODE_CLOSE:          DBG_PRINTLN("CLOSE"); break;
    case MODE_HOLD:           DBG_PRINTLN("HOLD"); break;
  }
}

String toUpperStr(String s) {
  s.toUpperCase();
  return s;
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String cmdStr = toUpperStr(sp < 0 ? line : line.substring(0, sp));
  String argStr = sp < 0 ? "" : line.substring(sp + 1);
  argStr.trim();

  if (cmdStr == "SETSPREAD") {
    float mm = argStr.toFloat();
    dispatchActionCommand(CMD_SAFE_SET_SPREAD, true, mm);
    DBG_PRINTLN("OK SETSPREAD");
  } else if (cmdStr == "SETANGLE") {
    float deg = argStr.toFloat();
    dispatchActionCommand(CMD_SAFE_SET_ANGLE, true, deg);
    DBG_PRINTLN("OK SETANGLE");
  } else if (cmdStr == "OPEN") {
    dispatchActionCommand(CMD_SAFE_OPEN, false, 0);
    DBG_PRINTLN("OK OPEN");
  } else if (cmdStr == "CLOSE") {
    dispatchActionCommand(CMD_SAFE_CLOSE, false, 0);
    DBG_PRINTLN("OK CLOSE");
  } else if (cmdStr == "HOLD") {
    dispatchActionCommand(CMD_START_SAFE_HOLD, false, 0);
    DBG_PRINTLN("OK HOLD");
  } else if (cmdStr == "UNHOLD") {
    dispatchActionCommand(CMD_STOP_SAFE_HOLD, false, 0);
    DBG_PRINTLN("OK UNHOLD");
  } else if (cmdStr == "READLOAD") {
    DBG_PRINT("RIGHT="); DBG_PRINT(lastRightLoad_g, 1);
    DBG_PRINT("g LEFT="); DBG_PRINT(lastLeftLoad_g, 1); DBG_PRINTLN("g");
  } else if (cmdStr == "READANGLE") {
    DBG_PRINT("ANGLE="); DBG_PRINTLN(targetAngle, 1);
  } else if (cmdStr == "READSPREAD") {
    DBG_PRINT("SPREAD="); DBG_PRINTLN(angleToSpread(targetAngle), 1);
  } else if (cmdStr == "READINFO") {
    printGripperInfo();
  } else {
    DBG_PRINT("ERR unknown command: "); DBG_PRINTLN(cmdStr);
  }
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

#endif // ENABLE_SERIAL

// =============================================================================
//  SETUP & LOOP
// =============================================================================
void setup() {
  DBG_BEGIN(SERIAL_BAUD);
  DBG_PRINTLN("Safe gripper firmware starting (10kg / 80Hz mode)...");

  ESP32PWM::allocateTimer(0);
  gripperServo.setPeriodHertz(50);
  gripperServo.attach(PIN_SERVO, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  currentAngle = (float)SERVO_MIN_ANGLE;
  targetAngle  = (float)SERVO_MIN_ANGLE;
  writeServo(currentAngle);

  rightScale.begin(PIN_HX711_RIGHT_DAT, PIN_HX711_RIGHT_SCK);
  rightScale.set_scale(RIGHT_CALIBRATION_FACTOR);
  rightScale.tare();

  leftScale.begin(PIN_HX711_LEFT_DAT, PIN_HX711_LEFT_SCK);
  leftScale.set_scale(LEFT_CALIBRATION_FACTOR);
  leftScale.tare();

  canInit();

  DBG_PRINTLN("Ready.");
}

void loop() {
  static unsigned long lastPollMs   = 0;
  static unsigned long lastReportMs = 0;
  unsigned long now = millis();

  // Process CAN commands at highest priority
  pollCan();

#if ENABLE_SERIAL
  pollSerial();
#endif

  // Control loop runs every 10ms to match 80 Hz HX711 sampling rate (~12.5 ms)
  if (now - lastPollMs >= SERVO_STEP_INTERVAL_MS) {
    lastPollMs = now;
    controlStep(); // Sample-gated logic moves only on fresh HX711 reads
  }

#if ENABLE_SERIAL
  if (now - lastReportMs >= SERIAL_REPORT_INTERVAL_MS) {
    lastReportMs = now;
    printGripperInfo();
  }
#endif
}
