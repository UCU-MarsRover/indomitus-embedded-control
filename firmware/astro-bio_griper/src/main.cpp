// #include <Arduino.h>
// #include "can_driver.hpp"
// #include "pins.hpp"

// void setup() {
//     pinMode(Pins::LED, OUTPUT);
//     digitalWrite(Pins::LED, LOW);

//     can_init();
// }

// void loop() {
   

//     CanMsg msg;
//     if (can_recv(msg, 1000) == ESP_OK) {
//         if (msg.id == 0x28 && msg.len >= 1) {
//             if (msg.data[0] == 0x01) {
//                 digitalWrite(Pins::LED, HIGH);
//                 const uint8_t resp[2] = {0x01, 0x00};
//                 can_send(0x29, resp, 2);
//             } else if (msg.data[0] == 0x02) {
//                 digitalWrite(Pins::LED, LOW);
//                 const uint8_t resp[2] = {0x02, 0x00};
//                 can_send(0x29, resp, 2);
//             } else {
//                 const uint8_t resp[2] = {msg.data[0], 0x01};
//                 can_send(0x29, resp, 2);
//             }
//         }
//     }
// }

/*
  test1_pump_sensor_can.ino
  Pump + sensor controller for ESP32-C3 (Arduino Core), controlled over CAN (TWAI)

  - Built-in TWAI controller (needs external CAN transceiver, e.g. SN65HVD230)
    wired to CAN_TX_PIN / CAN_RX_PIN
  - Device address: 602 -> interpreted as CANopen-style node ID
      RX (commands into device):  CAN ID 0x602
      TX (responses from device): CAN ID 0x582
    (Change CAN_ID_CMD / CAN_ID_RESP below if you meant something else.)
  - Bitrate: 500 kbps (change TWAI_TIMING_CONFIG_500KBITS() if different)

  Commands (1 byte, data[0]):
    0x01  CMD_START_PUMP        -> start pump, runs until water sensor triggers
    0x02  CMD_STOP_PUMP         -> force stop pump immediately
    0x03  CMD_READ_WATER_SENSOR -> reply with current water sensor state
    0x04  CMD_READ_PH_SENSOR    -> reply with placeholder (not implemented yet)

  Response frame (sent on CAN_ID_RESP):
    data[0] = echoed command
    data[1] = value / status
      For START_PUMP ack : 0x01 = accepted, 0x00 = already pumping (ignored)
      For STOP_PUMP  ack : 0x01 = stopped
      For READ_WATER_SENSOR : 0/1 = raw pin state
      For READ_PH_SENSOR    : 0xFF = not implemented (placeholder)
    An unsolicited frame with data[0] = 0x03 (READ_WATER_SENSOR) is also sent
    automatically the moment the pump auto-stops because the sensor triggered,
    so the bus knows the fill finished without having to poll.
*/

/*
  test1_pump_sensor_can.ino
  Pump + sensor controller for ESP32-C3 (Arduino Core), controlled over CAN (TWAI)

  - Built-in TWAI controller (needs external CAN transceiver, e.g. SN65HVD230)
    wired to CAN_TX_PIN / CAN_RX_PIN
  - Device address: CAN ID 0x28
      RX (commands into device):  CAN ID 0x28
      TX (responses from device): CAN ID 0x28
  - Bitrate: 1 Mbps

  Commands (1 byte, data[0]):
    0x01  CMD_START_PUMP        -> start pump, runs until water sensor triggers
    0x02  CMD_STOP_PUMP         -> force stop pump immediately
    0x03  CMD_READ_WATER_SENSOR -> reply with current water sensor state
    0x04  CMD_READ_PH_SENSOR    -> reply with placeholder (not implemented yet)

  Response frame (sent on CAN_ID_RESP):
    data[0] = echoed command
    data[1] = value / status
      For START_PUMP ack : 0x01 = accepted, 0x00 = already pumping (ignored)
      For STOP_PUMP  ack : 0x01 = stopped
      For READ_WATER_SENSOR : 0/1 = raw pin state
      For READ_PH_SENSOR    : 0xFF = not implemented (placeholder)
*/

#include <Arduino.h>
#include <Preferences.h>
#include "driver/twai.h"

// ---------------- Pin definitions ----------------
#define CAN_TX_PIN 20
#define CAN_RX_PIN 21
#define MOTOR_IN1_PIN 1
#define MOTOR_IN2_PIN 3
#define PH_SENSOR_PIN 0
#define LIQUID_SENSOR_PIN 10

// ---------------- CAN addressing ----------------
#define CAN_ID_CMD  0x28
#define CAN_ID_RESP 0x28

// ---------------- Command bytes ----------------
#define CMD_START_PUMP         0x01
#define CMD_STOP_PUMP          0x02
#define CMD_READ_WATER_SENSOR  0x03
#define CMD_READ_PH_SENSOR     0x04

// ---------------- Config ----------------
#define SENSOR_TRIGGERED_STATE HIGH
#define PUMP_SPEED 255

// ---------------- NVS Preferences (Flash) ----------------
Preferences prefs;

// Calibration points by default: voltage mV <-> pH
float p1_v  = 1500.0f;
float p1_ph = 7.00f;
float p2_v  = 2030.0f;
float p2_ph = 4.00f;

// ---------------- State ----------------
enum PumpState { PUMP_IDLE, PUMP_RUNNING };
PumpState pumpState = PUMP_IDLE;

// ---------------- Forward declarations ----------------
void startPump();
void stopPump(bool sensorTriggered);
void handleCommand(const twai_message_t &msg);
void sendResponse(uint8_t cmd, uint8_t value);
void pollCAN();
void updatePump();
void loadCalibration();
float getSensorVoltageMilliVolts();
float getMeasuredPH();
void calibrateCustom(float target_ph);
void shiftCustom(float target_ph);
uint8_t readPHSensor();
void pollSerial();

// ================================================================
// SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(LIQUID_SENSOR_PIN, INPUT);

  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);

  loadCalibration();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
  }

  Serial.println("\n==================================================");
  Serial.println("  ASTROBIO GRIPPER — UNIVERSAL CUSTOM pH SYSTEM");
  Serial.println("==================================================");
  Serial.println("Команди для терміналу:");
  Serial.println("  'read'       -> Показати поточну напругу і pH");
  Serial.println("  'cal 6.86'   -> Відкалібрувати під кастомний pH");
  Serial.println("  'shift 7.01' -> Змістити графік під 1 кастомну рідину");
  Serial.println("  'status'     -> Показати збережені у Flash точки");
  Serial.println("  'reset'      -> Скинути до заводських (7.0 та 4.0)");
  Serial.println("==================================================\n");
}

void loop() {
  pollCAN();
  updatePump();
  pollSerial();
}

// ================================================================
// pH LOGIC (Кастомне калібрування будь-яких величин)
// ================================================================
void loadCalibration() {
  prefs.begin("ph_custom", true);
  p1_v  = prefs.getFloat("p1_v", 1500.0f);
  p1_ph = prefs.getFloat("p1_ph", 7.00f);
  p2_v  = prefs.getFloat("p2_v", 2030.0f);
  p2_ph = prefs.getFloat("p2_ph", 4.00f);
  prefs.end();
}

float getSensorVoltageMilliVolts() {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogReadMilliVolts(PH_SENSOR_PIN);
    delay(5);
  }
  return sum / 20.0f;
}

float getMeasuredPH() {
  float v = getSensorVoltageMilliVolts();

  if (fabsf(p2_v - p1_v) < 1.0f) return 7.0f;

  float slope = (p2_ph - p1_ph) / (p2_v - p1_v);
  float ph = p1_ph + (v - p1_v) * slope;

  if (ph < 0.0f) ph = 0.0f;
  if (ph > 14.0f) ph = 14.0f;
  return ph;
}

void calibrateCustom(float target_ph) {
  float current_v = getSensorVoltageMilliVolts();

  if (fabsf(target_ph - p1_ph) <= fabsf(target_ph - p2_ph)) {
    p1_v = current_v;
    p1_ph = target_ph;
    Serial.printf("[CALIB] Оновлено ТОЧКУ 1: %.1f mV ===> pH %.2f\n", p1_v, p1_ph);
  } else {
    p2_v = current_v;
    p2_ph = target_ph;
    Serial.printf("[CALIB] Оновлено ТОЧКУ 2: %.1f mV ===> pH %.2f\n", p2_v, p2_ph);
  }

  prefs.begin("ph_custom", false);
  prefs.putFloat("p1_v", p1_v);
  prefs.putFloat("p1_ph", p1_ph);
  prefs.putFloat("p2_v", p2_v);
  prefs.putFloat("p2_ph", p2_ph);
  prefs.end();

  Serial.println("[NVS] Нові кастомні коефіцієнти успішно збережені у Flash!");
}

void shiftCustom(float target_ph) {
  float current_ph = getMeasuredPH();
  float diff = target_ph - current_ph;

  p1_ph += diff;
  p2_ph += diff;

  prefs.begin("ph_custom", false);
  prefs.putFloat("p1_ph", p1_ph);
  prefs.putFloat("p2_ph", p2_ph);
  prefs.end();

  Serial.printf("[SHIFT] Графік зсунуто на %.2f pH. Поточний pH підігнано під %.2f\n", diff, target_ph);
}

void pollSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "read") {
      float v = getSensorVoltageMilliVolts();
      float ph = getMeasuredPH();
      Serial.printf("[READ] Напруга: %.1f mV | Точний pH: %.2f\n", v, ph);
    }
    else if (input.startsWith("cal ")) {
      float target_ph = input.substring(4).toFloat();
      calibrateCustom(target_ph);
    }
    else if (input.startsWith("shift ")) {
      float target_ph = input.substring(6).toFloat();
      shiftCustom(target_ph);
    }
    else if (input == "status") {
      Serial.printf("[STATUS] Точка 1: %.1f mV = pH %.2f\n", p1_v, p1_ph);
      Serial.printf("[STATUS] Точка 2: %.1f mV = pH %.2f\n", p2_v, p2_ph);
    }
    else if (input == "reset") {
      prefs.begin("ph_custom", false);
      prefs.putFloat("p1_v", 1500.0f);
      prefs.putFloat("p1_ph", 7.00f);
      prefs.putFloat("p2_v", 2030.0f);
      prefs.putFloat("p2_ph", 4.00f);
      prefs.end();
      loadCalibration();
      Serial.println("[RESET] Скинуто до стандартних 7.00 та 4.00!");
    }
  }
}

uint8_t readPHSensor() {
  float ph = getMeasuredPH();
  return (uint8_t)(ph * 10.0f);
}

// ================================================================
// CAN & PUMP LOGIC
// ================================================================
void pollCAN() {
  twai_message_t rxMsg;
  if (twai_receive(&rxMsg, 0) == ESP_OK) {
    if (!rxMsg.extd && rxMsg.identifier == CAN_ID_CMD && rxMsg.data_length_code >= 1) {
      handleCommand(rxMsg);
    }
  }
}

void handleCommand(const twai_message_t &msg) {
  uint8_t cmd = msg.data[0];

  switch (cmd) {
    case CMD_START_PUMP:
      if (pumpState == PUMP_IDLE) {
        startPump();
        sendResponse(CMD_START_PUMP, 0x01);
      } else {
        sendResponse(CMD_START_PUMP, 0x00);
      }
      break;

    case CMD_STOP_PUMP:
      stopPump(false);
      sendResponse(CMD_STOP_PUMP, 0x01);
      break;

    case CMD_READ_WATER_SENSOR:
      sendResponse(CMD_READ_WATER_SENSOR, (uint8_t)digitalRead(LIQUID_SENSOR_PIN));
      break;

    case CMD_READ_PH_SENSOR:
      sendResponse(CMD_READ_PH_SENSOR, readPHSensor());
      break;

    default:
      break;
  }
}

void sendResponse(uint8_t cmd, uint8_t value) {
  twai_message_t message = {};
  message.identifier = CAN_ID_RESP;
  message.extd = 0;
  message.data_length_code = 2;
  message.data[0] = cmd;
  message.data[1] = value;

  twai_transmit(&message, pdMS_TO_TICKS(10));
}

void startPump() {
  digitalWrite(MOTOR_IN2_PIN, LOW);
  analogWrite(MOTOR_IN1_PIN, PUMP_SPEED);
  pumpState = PUMP_RUNNING;
}

void stopPump(bool sensorTriggered) {
  analogWrite(MOTOR_IN1_PIN, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  pumpState = PUMP_IDLE;

  if (sensorTriggered) {
    sendResponse(CMD_READ_WATER_SENSOR, (uint8_t)digitalRead(LIQUID_SENSOR_PIN));
  }
}

void updatePump() {
  if (pumpState != PUMP_RUNNING) return;

  if (digitalRead(LIQUID_SENSOR_PIN) == SENSOR_TRIGGERED_STATE) {
    stopPump(true);
  }
}

