#include <Arduino.h>
#include "servo_ledc.hpp"
#include "pins.hpp"

static ServoLedc lid_servo_right_far;

void setup() {
    Serial.begin(115200);
    
    lid_servo_right_far.attach(PIN_SERVO_RIGHT_FAR, 0);
    
    Serial.println("Closing (0 deg)...");
    lid_servo_right_far.writeMicroseconds(1400); // повільно в одну сторону
}

void loop() {
    if (Serial.available()) {
        int ms = 1000;
        // Serial.print("Running for ms: ");
        // Serial.println(ms);

        lid_servo_right_far.writeMicroseconds(1400);
        delay(ms);
        lid_servo_right_far.writeMicroseconds(1600); // стоп
        delay(ms);
        lid_servo_right_far.writeMicroseconds(1500);
        delay(1000);
    }
}