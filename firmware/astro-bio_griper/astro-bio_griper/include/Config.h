#pragma once

// =============================================================================
// Config.h — Astro-Bio Gripper Sub-Controller
// Centralised hardware configuration for ESP32-C3
// =============================================================================

// -----------------------------------------------------------------------------
// Serial
// -----------------------------------------------------------------------------
#define SERIAL_BAUD_RATE        115200

// -----------------------------------------------------------------------------
// Pump Control — DRV8871 Motor Driver
// -----------------------------------------------------------------------------
#define PIN_PUMP_IN1            9
#define PIN_PUMP_IN2            10

// LEDC PWM configuration (ESP32 hardware PWM)
#define PWM_CHANNEL_IN1         0
#define PWM_CHANNEL_IN2         1
#define PWM_FREQUENCY_HZ        25000   // 25 kHz — above audible range
#define PWM_RESOLUTION_BITS     8       // 0-255 duty cycle

#define PUMP_DEFAULT_DUTY       200     // ~78% duty — tunable at competition

// -----------------------------------------------------------------------------
// CAN Bus Communication — TWAI / External Transceiver
// -----------------------------------------------------------------------------
#define PIN_CAN_TX              20
#define PIN_CAN_RX              21
#define CAN_BITRATE_KBPS        500     // Standard 500 kbit/s CAN bus

// TWAI alert flags for bus-off / error detection
#define CAN_RX_QUEUE_LEN        10
#define CAN_HEARTBEAT_MS        1000    // Bus health heartbeat interval
#define CAN_TIMEOUT_MS          3000    // Bus disconnect timeout

// -----------------------------------------------------------------------------
// Line Sensor — KY-033 (Bottom Detection)
// -----------------------------------------------------------------------------
#define PIN_LINE_SENSOR         0       // Digital — HIGH/LOW

// -----------------------------------------------------------------------------
// Water Level Sensor — T1592
// -----------------------------------------------------------------------------
#define PIN_WATER_LEVEL         1       // ADC input

// Target ADC reading indicating sufficient water collected.
// 12-bit ADC (0-4095). Tune on-site before runs.
#define WATER_LEVEL_TARGET      2800

// -----------------------------------------------------------------------------
// pH Sensor — DFRobot Gravity V2
// -----------------------------------------------------------------------------
#define PIN_PH_SENSOR           2       // ADC input

// pH rolling variance buffer
#define PH_SAMPLE_INTERVAL_MS   200     // Sample every 200 ms
#define PH_BUFFER_SIZE          50      // Rolling window of 50 readings
#define PH_VARIANCE_THRESHOLD   0.005f  // Stability threshold (variance)

// pH calibration constants (linear: pH = SLOPE * voltage + OFFSET)
// Default values for DFRobot Gravity V2 at 25 °C — recalibrate on-site!
#define PH_VOLTAGE_REF          3.3f    // ESP32-C3 ADC reference voltage
#define PH_ADC_RESOLUTION       4095.0f // 12-bit ADC
#define PH_SLOPE                (-5.70f)
#define PH_OFFSET               21.34f

// -----------------------------------------------------------------------------
// Timing & Safety
// -----------------------------------------------------------------------------
#define LOOP_WATCHDOG_MS        50      // Minimum loop cadence for diagnostics
#define ESTOP_DEBOUNCE_MS       10      // E-stop CAN frame debounce
