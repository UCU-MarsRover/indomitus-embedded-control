# `test/` — Unit & Integration Tests

This directory is reserved for PlatformIO test runner tests.

---

## Current Status

No tests have been written yet. This is a future improvement area.

## Recommended Test Strategy

### Unit Tests (Native Platform)

PlatformIO supports running tests on the host machine (without hardware) using the `native` platform. These tests can validate logic that doesn't depend on ESP32 hardware:

- **pH variance calculation:** Feed known arrays into `computeVariance()` and verify output
- **pH ADC-to-pH conversion:** Verify `adcToPh()` against known calibration points
- **State transition logic:** Mock the CAN and sensor interfaces, verify FSM transitions
- **CAN ID encoding:** Verify `enum class` values match expected hex IDs

### Integration Tests (On Hardware)

These require the actual ESP32-C3 with sensors and CAN bus connected:

- **CAN loopback test:** Set TWAI to loopback mode, send a frame, verify receipt
- **Sensor read test:** Read all three sensors and print values to Serial
- **Pump test:** Run pump for 2 seconds at various duty cycles
- **Full sequence test:** Execute the complete POSITIONING → EXTRACTION → ACQUISITION sequence

### Getting Started

```bash
# Run all tests on native platform (no hardware needed)
pio test -e native

# Run tests on the ESP32-C3 (hardware required)
pio test -e adafruit_qtpy_esp32c3
```

### Example Test File

```cpp
// test/test_ph_variance/test_main.cpp
#include <unity.h>

void test_variance_of_constant_array_is_zero() {
    float buffer[5] = {7.0, 7.0, 7.0, 7.0, 7.0};
    float sum = 0;
    for (int i = 0; i < 5; i++) sum += buffer[i];
    float mean = sum / 5.0f;
    float var = 0;
    for (int i = 0; i < 5; i++) {
        float d = buffer[i] - mean;
        var += d * d;
    }
    var /= 5.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.0, var);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_variance_of_constant_array_is_zero);
    UNITY_END();
}

void loop() {}
```

### More Information

- [PlatformIO Unit Testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)
- [Unity Test Framework](https://github.com/ThrowTheSwitch/Unity)
