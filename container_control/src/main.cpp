#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <TM1637TinyDisplay6.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "weight_sensor.hpp"
#include "shared_state.hpp"
#include "pins.hpp"

MPU6050 mpu(Wire);
bool mpu_initialized = false;
SemaphoreHandle_t i2c_mutex = nullptr;
SemaphoreHandle_t weight_mutex = nullptr;

// TM1637 6-digit display moved to free UART pins: GPIO21 (TX0) and GPIO20 (RX0)
TM1637TinyDisplay6 display(PIN_DISPLAY_CLK, PIN_DISPLAY_DIO);

// Примусове очищення шини I2C від залипань SDA
void i2c_bus_recover() {
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(PIN_I2C_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(5);
    }
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
}

// I2C Scanner for debugging
void i2c_scan() {
#ifdef DEBUG_ENABLED
    Serial.println("\n===== I2C SCANNER =====");
    Serial.println("Scanning I2C bus (0x01-0x7F)...");
    int device_count = 0;
    
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  [FOUND] I2C Device at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            device_count++;
        }
    }
    
    if (device_count == 0) {
        Serial.println("  No I2C devices found!");
    } else {
        Serial.print("  Total devices found: ");
        Serial.println(device_count);
    }
    Serial.println("========================\n");
#endif
}

// MPU6050 initialization with detailed debug
void mpu_init_with_debug() {
#ifdef DEBUG_ENABLED
    Serial.println("\n===== MPU6050 INITIALIZATION =====");
    Serial.println("Attempting MPU6050 initialization...");
#endif

    byte status = mpu.begin();

    // Перевірка МАЄ виконуватися завжди, навіть у Release
    if (status != 0) {
#ifdef DEBUG_ENABLED
        Serial.printf("  [ERROR] MPU6050 init failed with status: %d\n", status);
        Serial.println("  Possible causes:");
        Serial.println("    - Chip not found at I2C address");
        Serial.println("    - Wrong I2C pins configured");
        Serial.println("    - SDA/SCL not connected properly");
#endif
        mpu_initialized = false;
        return;
    }

#ifdef DEBUG_ENABLED
    Serial.println("  [INFO] Calculating IMU offsets (keep device still)...");
#endif

    mpu.calcOffsets(false, true);
    mpu_initialized = true;

#ifdef DEBUG_ENABLED
    Serial.println("  [SUCCESS] MPU6050 initialized successfully!");
    Serial.println("  Sensor info:");
    Serial.print("    - Temperature: ");
    Serial.print(mpu.getTemp());
    Serial.println(" °C");
    Serial.print("    - Gyro X offset: ");
    Serial.println(mpu.getGyroXoffset());
    Serial.print("    - Gyro Y offset: ");
    Serial.println(mpu.getGyroYoffset());
    Serial.print("    - Gyro Z offset: ");
    Serial.println(mpu.getGyroZoffset());
    Serial.println("==================================\n");
#endif
}

static void print_weight_status() {
    float left = g_state.weight1_g.load();
    float right = g_state.weight2_g.load();
    bool left_ok = !g_state.weight1_error.load();
    bool right_ok = !g_state.weight2_error.load();

    Serial.printf("[STATUS] LEFT = %.1f g  %s | RIGHT = %.1f g  %s\n",
                  left_ok ? left : -9999.0f,
                  left_ok ? "OK" : "ERR",
                  right_ok ? right : -9999.0f,
                  right_ok ? "OK" : "ERR");
}

static void print_serial_help() {
    Serial.println("\n=== Weight UI ===");
    Serial.println("  TR   = tare right sensor");
    Serial.println("  ZR   = zero right sensor (soft tare)");
    Serial.println("  CR [g] = calibrate right sensor with known weight in grams");
    Serial.println("           Example: CR 29.5  or  CR 29,5  => 29.5 g");
    Serial.println("  TL   = tare left sensor");
    Serial.println("  ZL   = zero left sensor (soft tare)");
    Serial.println("  CL [g] = calibrate left sensor with known weight in grams");
    Serial.println("           Example: CL 29.5  or  CL 29,5  => 29.5 g");
    Serial.println("  STATUS / READ = print current weight values");
    Serial.println("  HELP / ? = this menu");
    Serial.println("=================\n");
}

static void handle_serial_command() {
    while (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
            continue;
        }

        String cmd = line;
        cmd.replace(" ", "");
        cmd.toUpperCase();

        if (cmd == "HELP" || cmd == "?") {
            print_serial_help();
            continue;
        }

        if (cmd == "STATUS" || cmd == "READ") {
            print_weight_status();
            continue;
        }

        WeightSensor* sensor = nullptr;
        bool left_side = false;
        bool right_side = false;
        bool do_calibrate = false;

        if (cmd.startsWith("TR") || cmd.startsWith("RT")) {
            sensor = &sensor2;
            right_side = true;
        } else if (cmd.startsWith("TL") || cmd.startsWith("LT")) {
            sensor = &sensor1;
            left_side = true;
        } else if (cmd.startsWith("ZR") || cmd.startsWith("RZ")) {
            sensor = &sensor2;
            right_side = true;
        } else if (cmd.startsWith("ZL") || cmd.startsWith("LZ")) {
            sensor = &sensor1;
            left_side = true;
        } else if (cmd.startsWith("CR") || cmd.startsWith("RC")) {
            sensor = &sensor2;
            right_side = true;
            do_calibrate = true;
        } else if (cmd.startsWith("CL") || cmd.startsWith("LC")) {
            sensor = &sensor1;
            left_side = true;
            do_calibrate = true;
        } else {
            Serial.println("Unknown command. Type HELP or ?");
            continue;
        }

        if (do_calibrate) {
            String rest = line.substring(2);
            rest.trim();
            rest.replace(',', '.');
            float known_weight = rest.length() > 0 ? rest.toFloat() : 500.0f;

            if (known_weight <= 0.0f) {
                Serial.println("Calibration requires known weight > 0 g");
                continue;
            }

            if (weight_mutex == nullptr || xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
                Serial.println("[CALIB] cannot lock weight mutex; try again");
                continue;
            }

            // ВАЖЛИВО: нуль має бути знятий раніше через ZL/ZR на порожньому датчику.
            // Не викликаємо calibrate_zero() тут: це знищує різницю під навантаженням.
            float scale_factor = sensor->calibrate_weight(known_weight, 30);
            xSemaphoreGive(weight_mutex);

            if (scale_factor <= 0.0f) {
                Serial.printf("[%s] CALIB ERROR: factor=%.4f invalid — previous calibration kept\n",
                              right_side ? "RIGHT" : "LEFT",
                              scale_factor);
                continue;
            }

            Serial.printf("[%s] calibrated with %.1f g => scale=%.4f\n",
                          right_side ? "RIGHT" : "LEFT",
                          known_weight,
                          scale_factor);
            continue;
        }

        if (cmd.startsWith("TR") || cmd.startsWith("TL") || cmd.startsWith("LT") || cmd.startsWith("RT")) {
            if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
                sensor->tare(10);
                xSemaphoreGive(weight_mutex);
            } else {
                Serial.println("[TARE] cannot lock weight mutex; try again");
                continue;
            }
            Serial.printf("[%s] tare OK\n", right_side ? "RIGHT" : "LEFT");
        } else if (cmd.startsWith("ZR") || cmd.startsWith("ZL") || cmd.startsWith("LZ") || cmd.startsWith("RZ")) {
            if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
                sensor->calibrate_zero(40);
                xSemaphoreGive(weight_mutex);
            } else {
                Serial.println("[ZERO] cannot lock weight mutex; try again");
                continue;
            }
            Serial.printf("[%s] zeroed OK\n", right_side ? "RIGHT" : "LEFT");
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    uint32_t start = millis();
    while (!Serial && (millis() - start < 1500)) {
        delay(10);
    }

#ifdef DEBUG_ENABLED
    Serial.println("\n[BOOT] ESP32-C3 Super Mini Started");
#else
    Serial.println("\n[BOOT] ESP32-C3 Super Mini Started in calibration mode");
#endif
    print_serial_help();

    // Initialize weight sensors for both debug and release calibration mode.
    weight_sensors_init();

    // Default calibration values for both sensors; manual recalibration remains possible.
    sensor1.set_calibration(0, 298.3729f);
    sensor2.set_calibration(0, 298.3729f);

#ifndef DEBUG_ENABLED
    Serial.println("[RELEASE] Calibration-only mode active. Use TR/TL/ZR/ZL/CR/CL or STATUS");
    Serial.println("[RELEASE] Live weight logs are enabled every 1 second.");
#endif

    // Створюємо мьютекси для синхронізації I2C та вагових датчиків
    i2c_mutex = xSemaphoreCreateMutex();
    weight_mutex = xSemaphoreCreateMutex();

    // Очищаємо шину перед ініціалізацією
    i2c_bus_recover();

    // 100 кГц для стабільності I2C на ESP32-C3
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
    Wire.setTimeOut(50);
    delay(50);

    // i2c_scan(); // Вимкнено для запобігання Error 263

    // Initialize MPU6050
    mpu_init_with_debug();

    Serial.println("[DEBUG] Weight sensors initialized");

    // Initialize TM1637 display
    display.begin();
    display.setBrightness(7);
    display.clear();
    Serial.println("[DEBUG] TM1637 display initialized");
    Serial.println("[TEST] Displaying 888888 for 1 second...");
    display.showNumberDec(888888);
    delay(1000);
    display.clear();
    Serial.println("[DEBUG] Display test complete - ready for Yaw/Pitch output");

    // Create tasks with distinct priorities: imu (high), display (medium), weight (low)
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, &g_state, 3, nullptr, 0);
    xTaskCreatePinnedToCore(display_task, "display_task", 2048, &g_state, 2, nullptr, 0);
    xTaskCreatePinnedToCore(weight_sensors_task, "sensor_task", 3072, &g_state, 1, nullptr, 0);

    xTaskCreatePinnedToCore([](void*) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (weight_mutex != nullptr && xSemaphoreTake(weight_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
            sensor1.tare(20);
            sensor2.tare(20);
            xSemaphoreGive(weight_mutex);
            Serial.println("[BOOT] Initial auto-tare applied successfully");
        }
        vTaskDelete(nullptr);
    }, "boot_tare", 2048, nullptr, 1, nullptr, 0);

    Serial.println("[DEBUG] ===== READY (Tasks started) =====\n");
}

void loop() {
    handle_serial_command();

#ifdef DEBUG_ENABLED
    vTaskDelay(portMAX_DELAY);
#else
    static uint32_t last_status_ms = 0;
    if (millis() - last_status_ms >= 1000UL) {
        print_weight_status();
        last_status_ms = millis();
    }
    delay(50);
#endif
}
