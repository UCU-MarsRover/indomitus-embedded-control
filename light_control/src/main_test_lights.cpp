#include <Arduino.h>

#ifdef DEBUG_ENABLED
#define DEBUG_PORT Serial0
#endif


namespace Pins {
    constexpr uint8_t kSpotlight = 40;
    constexpr uint8_t kRed = 1;
    constexpr uint8_t kYellow = 2;
    constexpr uint8_t kGreen = 42;
    constexpr uint8_t kBlue = 41;
}

static void logStep(const char* label) {
#if DEBUG_ENABLED
    DEBUG_PORT.println(label);
#endif
}

static void setTraffic(bool red, bool yellow, bool green, bool blue) {
    digitalWrite(Pins::kRed, red ? HIGH : LOW);
    digitalWrite(Pins::kYellow, yellow ? HIGH : LOW);
    digitalWrite(Pins::kGreen, green ? HIGH : LOW);
    digitalWrite(Pins::kBlue, blue ? HIGH : LOW);
}

static void allOff() {
    digitalWrite(Pins::kSpotlight, LOW);
    setTraffic(false, false, false, false);
}

void setup() {
#if DEBUG_ENABLED
    DEBUG_PORT.begin(115200);
    delay(300);
#endif

    pinMode(Pins::kSpotlight, OUTPUT);
    pinMode(Pins::kRed, OUTPUT);
    pinMode(Pins::kYellow, OUTPUT);
    pinMode(Pins::kGreen, OUTPUT);
    pinMode(Pins::kBlue, OUTPUT);

    allOff();
    logStep("light_test: setup complete, all outputs OFF");
}

void loop() {
    logStep("light_test: spotlight ON");
    digitalWrite(Pins::kSpotlight, HIGH);
   
    delay(2000);

    logStep("light_test: yellow ON");
    setTraffic(true, true, true, true);
    delay(4000);
}
