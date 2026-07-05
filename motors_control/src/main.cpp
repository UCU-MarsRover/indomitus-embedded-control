#include <Arduino.h>
#include "TwaiCanBus.h"
#include "DamiaoMotor.h"

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------
static constexpr int8_t   CAN_TX_PIN = 5;
static constexpr int8_t   CAN_RX_PIN = 4;

// Feedback frames from all motors arrive on this CAN ID (MST_ID register 7).
// If you change a motor's MST_ID via setMstId(), update this constant too.
static constexpr uint16_t MST_ID = 0x000;

// ---------------------------------------------------------------------------
// CAN bus — must be declared before motors (motors hold a reference to it)
// ---------------------------------------------------------------------------
TwaiCanBus bus(CAN_TX_PIN, CAN_RX_PIN);

// ---------------------------------------------------------------------------
// Motors — ESC IDs 1..4, all sharing one bus
// Adjust DamiaoConfigs::J4340() / J10010L() per wheel as needed.
// ---------------------------------------------------------------------------
DamiaoMotor motors[4] = {
    DamiaoMotor(1, MST_ID, bus, DamiaoConfigs::J4340()),   // front-left
    DamiaoMotor(2, MST_ID, bus, DamiaoConfigs::J4340()),   // front-right
    DamiaoMotor(3, MST_ID, bus, DamiaoConfigs::J4340()),   // rear-left
    DamiaoMotor(4, MST_ID, bus, DamiaoConfigs::J4340()),   // rear-right
};
static constexpr int NUM_MOTORS = 4;

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1500);

    if (!bus.begin()) {
        Serial.println("[CAN] FAILED — check transceiver wiring and termination");
        while (true) delay(1000);
    }
    Serial.println("[CAN] 1 Mbps OK");

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setMode(MotorMode::Velocity);
        delay(5);
        motors[i].enable();
        delay(5);
    }
    Serial.println("[Motors] enabled");
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    // 1) Drain RX queue and dispatch feedback frames to the correct motor.
    //    All motors respond on the same CAN ID (MST_ID); the motor ID is
    //    encoded in D[0] & 0x0F.  acceptFeedback() handles the filtering.
    CanMsg rx;
    while (bus.recv(rx, 0)) {
        if (rx.id == MST_ID && rx.len >= 8) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i].acceptFeedback(rx.data, rx.len);
            }
        }
    }

    // 2) Send velocity commands every 10 ms.
    static uint32_t lastCmd = 0;
    if (millis() - lastCmd >= 10) {
        lastCmd = millis();

        // Example: drive all wheels at 5 rad/s (motor shaft).
        // Replace with your control logic.
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].sendVelocity(5.0f);
        }
    }

    // 3) Print feedback from all motors every 500 ms.
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();

        for (int i = 0; i < NUM_MOTORS; i++) {
            if (!motors[i].hasFeedback()) continue;

            Serial.printf("[M%d] pos=%+.3f rad  vel=%+.3f rad/s  tor=%+.3f Nm"
                          "  T_mos=%u°C  T_rotor=%u°C  err=0x%X  enabled=%d\n",
                          motors[i].escId(),
                          motors[i].getPosition(),
                          motors[i].getVelocity(),
                          motors[i].getTorque(),
                          motors[i].getMosTemp(),
                          motors[i].getRotorTemp(),
                          static_cast<uint8_t>(motors[i].getError()),
                          motors[i].isEnabled());
        }
    }
}
