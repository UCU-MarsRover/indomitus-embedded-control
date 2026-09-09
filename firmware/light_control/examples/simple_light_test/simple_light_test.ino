#include <Arduino.h>

namespace Pins {
  constexpr uint8_t kSpotlight = 18;
  constexpr uint8_t kRed = 19;
  constexpr uint8_t kYellow = 20;
  constexpr uint8_t kGreen = 21;
  constexpr uint8_t kBlue = 22;
}

void setTraffic(bool red, bool yellow, bool green, bool blue) {
  digitalWrite(Pins::kRed, red ? HIGH : LOW);
  digitalWrite(Pins::kYellow, yellow ? HIGH : LOW);
  digitalWrite(Pins::kGreen, green ? HIGH : LOW);
  digitalWrite(Pins::kBlue, blue ? HIGH : LOW);
}

void allOff() {
  digitalWrite(Pins::kSpotlight, LOW);
  setTraffic(false, false, false, false);
}

void setup() {
  pinMode(Pins::kSpotlight, OUTPUT);
  pinMode(Pins::kRed, OUTPUT);
  pinMode(Pins::kYellow, OUTPUT);
  pinMode(Pins::kGreen, OUTPUT);
  pinMode(Pins::kBlue, OUTPUT);

  allOff();
}

void loop() {
  // Spotlight on, all traffic lights off
  digitalWrite(Pins::kSpotlight, HIGH);
  setTraffic(false, false, false, false);
  delay(2000);

  // Spotlight off, red on
  digitalWrite(Pins::kSpotlight, LOW);
  setTraffic(true, false, false, false);
  delay(2000);

  // Yellow on
  setTraffic(false, true, false, false);
  delay(2000);

  // Green on
  setTraffic(false, false, true, false);
  delay(2000);

  // Blue on
  setTraffic(false, false, false, true);
  delay(2000);

  // All traffic lights on
  setTraffic(true, true, true, true);
  delay(2000);

  // Everything off
  allOff();
  delay(2000);
}
