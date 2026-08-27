#include <Arduino.h>
#include "can_driver.hpp"
#include "can_manager.hpp"
#include "light_manager.hpp"
#include "power_sensor.hpp"
#include "freertos/queue.h"
#include "pins.hpp"
#include <cstring>


static void print_uart_help() {
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  spot on|off");
    Serial.println("  spot left on|off");
    Serial.println("  spot right on|off");
    Serial.println("  beautiful on|off");
    Serial.println("  beautiful 1|2|3|4 on|off");
    Serial.println("  buzzer on|off");
    Serial.println("  tower on|off");
    Serial.println("  red on|off");
    Serial.println("  green on|off");
    Serial.println("  blue on|off");
    Serial.println("  traffic <0-15>");
    Serial.println("  all off");
}

static bool parse_on_off(const String& token, bool& enabled) {
    if (token == "on") {
        enabled = true;
        return true;
    }

    if (token == "off") {
        enabled = false;
        return true;
    }

    return false;
}

static bool handle_single_light_command(const String& line) {
    bool enabled = false;

    if (line.startsWith("spot left ")) {
        const String state = line.substring(10);
        if (!parse_on_off(state, enabled)) {
            return false;
        }

        light_set_spotlight_left(enabled);
        Serial.printf("OK spot left %s\n", enabled ? "on" : "off");
        return true;
    }

    if (line.startsWith("spot right ")) {
        const String state = line.substring(11);
        if (!parse_on_off(state, enabled)) {
            return false;
        }

        light_set_spotlight_right(enabled);
        Serial.printf("OK spot right %s\n", enabled ? "on" : "off");
        return true;
    }

    if (line.startsWith("beautiful ")) {
        const String rest = line.substring(10);
        const int space = rest.indexOf(' ');
        if (space < 0) {
            return false;
        }

        const int index = rest.substring(0, space).toInt();
        const String state = rest.substring(space + 1);
        if (!parse_on_off(state, enabled)) {
            return false;
        }

        switch (index) {
            case 1: light_set_beautiful_1(enabled); break;
            case 2: light_set_beautiful_2(enabled); break;
            case 3: light_set_beautiful_3(enabled); break;
            case 4: light_set_beautiful_4(enabled); break;
            default:
                return false;
        }

        Serial.printf("OK beautiful %d %s\n", index, enabled ? "on" : "off");
        return true;
    }

    return false;
}

static void handle_uart_command(String line) {
    line.trim();
    line.toLowerCase();

    if (line.length() == 0) {
        return;
    }

    if (line == "help") {
        print_uart_help();
        return;
    }

    if (line == "spot on") {
        light_set_spotlight(true);
        Serial.println("OK spot on");
        return;
    }

    if (line == "spot off") {
        light_set_spotlight(false);
        Serial.println("OK spot off");
        return;
    }

    if (line == "beautiful on") {
        light_set_beautiful(true);
        Serial.println("OK beautiful on");
        return;
    }

    if (line == "beautiful off") {
        light_set_beautiful(false);
        Serial.println("OK beautiful off");
        return;
    }

    if (handle_single_light_command(line)) {
        return;
    }

    if (line == "buzzer on") {
        light_set_buzzer(true);
        Serial.println("OK buzzer on");
        return;
    }

    if (line == "buzzer off") {
        light_set_buzzer(false);
        Serial.println("OK buzzer off");
        return;
    }

    if (line == "tower on") {
        light_set_traffic_mask(0x07);
        Serial.println("OK tower on");
        return;
    }

    if (line == "tower off") {
        light_set_traffic_mask(0x00);
        Serial.println("OK tower off");
        return;
    }

    if (line == "red on") {
        light_set_red(true);
        Serial.println("OK red on");
        return;
    }

    if (line == "red off") {
        light_set_red(false);
        Serial.println("OK red off");
        return;
    }

    if (line == "green on") {
        light_set_green(true);
        Serial.println("OK green on");
        return;
    }

    if (line == "green off") {
        light_set_green(false);
        Serial.println("OK green off");
        return;
    }

    if (line == "blue on") {
        light_set_blue(true);
        Serial.println("OK blue on");
        return;
    }

    if (line == "blue off") {
        light_set_blue(false);
        Serial.println("OK blue off");
        return;
    }

    if (line == "all off") {
        light_set_spotlight(false);
        light_set_beautiful(false);
        light_set_buzzer(false);
        light_set_traffic_mask(0x00);
        Serial.println("OK all off");
        return;
    }

    if (line.startsWith("traffic ")) {
        const int mask = line.substring(8).toInt();
        if (mask < 0 || mask > 15) {
            Serial.println("ERR traffic mask must be 0-15");
            return;
        }

        light_set_traffic_mask(static_cast<uint8_t>(mask));
        Serial.printf("OK traffic %d\n", mask);
        return;
    }

    Serial.println("ERR unknown command");
    print_uart_help();
}

static void uart_cli_task(void*) {
    String line;

    Serial.print("> ");

    for (;;) {
        while (Serial.available() > 0) {
            const char ch = static_cast<char>(Serial.read());
                if (ch == '\r' || ch == '\n') {
                    if (line.length() > 0) {
                        Serial.print("RX: ");
                        Serial.println(line);
                        handle_uart_command(line);
                        line = "";
                        Serial.print("> ");
                    }
                } else {
                    line += ch;
                }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


QueueHandle_t can_tx_queue = nullptr;
QueueHandle_t light_queue = nullptr;


void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        delay(10);
    }
    Serial.flush();

    light_init();
    can_init();
    power_sensor_init();

    can_tx_queue = xQueueCreate(10, sizeof(CanTxMsg));
    light_queue = xQueueCreate(16, sizeof(LightCommand));

    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(light_task, "light_task", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(power_telemetry_task, "power_telemetry_task", 4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(uart_cli_task, "uart_cli", 4096, nullptr, 2, nullptr, 1);

#ifdef DEBUG_ENABLED
    // xTaskCreatePinnedToCore(can_monitor_task, "can_monitor_task", 4096, nullptr, 4, nullptr, 1);
#endif
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
