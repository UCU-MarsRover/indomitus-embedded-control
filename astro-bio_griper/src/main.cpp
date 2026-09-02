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
#include <EEPROM.h>
#include <DFRobot_PH.h>
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

// ---------------- pH config ----------------
DFRobot_PH ph;
constexpr size_t PH_EEPROM_SIZE = 32;
constexpr float PH_TEMPERATURE_C = 25.0f;

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
float getMeasuredPH();
uint8_t readPHSensor();
void pollSerial();

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(LIQUID_SENSOR_PIN, INPUT);

  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  EEPROM.begin(PH_EEPROM_SIZE);
  ph.begin();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
  }

  Serial.println("\n=== ASTROBIO GRIPPER READY ===");
  Serial.println("Команди для USB-терміналу:");
  Serial.println("  'read'       -> Поточні показники (mV та pH)");
  Serial.println("  'enterph'    -> Увійти в режим калібрування");
  Serial.println("  'calph'      -> Зберегти калібрувальну точку (4.0/7.0)");
  Serial.println("  'exitph'     -> Вийти і зафіксувати у Flash");
}

void loop() {
  pollCAN();
  updatePump();
  pollSerial();
}

float getSensorVoltageMilliVolts() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogReadMilliVolts(PH_SENSOR_PIN);
    delay(5);
  }
  return sum / 10.0f;
}

float getMeasuredPH() {
  float current_mv = getSensorVoltageMilliVolts();
  float ph_value = ph.readPH(current_mv, PH_TEMPERATURE_C);

  if (ph_value < 0.0f) ph_value = 0.0f;
  if (ph_value > 14.0f) ph_value = 14.0f;
  return ph_value;
}

void pollSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      return;
    }

    if (input == "read") {
      float voltage = getSensorVoltageMilliVolts();
      float ph_val = getMeasuredPH();
      Serial.printf("[LOG] Напруга: %.1f mV | Обчислений pH: %.2f\n", voltage, ph_val);
    } else {
      char cmdBuffer[16];
      input.toCharArray(cmdBuffer, sizeof(cmdBuffer));

      float voltage = getSensorVoltageMilliVolts();
      Serial.printf("[CALIB] Виконання команди: %s\n", cmdBuffer);

      ph.calibration(voltage, PH_TEMPERATURE_C, cmdBuffer);

      // On ESP32, commit pushes emulated EEPROM changes from RAM to Flash.
      if (input.equalsIgnoreCase("exitph")) {
        EEPROM.commit();
        Serial.println("[FLASH] Збережено у Flash-пам'ять. Калібрування переживе знеструмлення.");
      }
    }
  }
}

uint8_t readPHSensor() {
  float ph = getMeasuredPH();
  return (uint8_t)(ph * 10.0f);
}

void pollCAN() {
  twai_message_t rxMsg;

  if (twai_receive(&rxMsg, 0) == ESP_OK) {
    if (!rxMsg.extd &&
        rxMsg.identifier == CAN_ID_CMD &&
        rxMsg.data_length_code >= 1) {
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
  if (pumpState != PUMP_RUNNING) {
    return;
  }

  if (digitalRead(LIQUID_SENSOR_PIN) == SENSOR_TRIGGERED_STATE) {
    stopPump(true);
  }
}
