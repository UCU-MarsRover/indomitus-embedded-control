#include "TwaiCanBus.h"
#include <ESP32-TWAI-CAN.hpp>  // only file in the project that touches the library

TwaiCanBus::TwaiCanBus(int8_t tx_pin, int8_t rx_pin)
    : _tx(tx_pin), _rx(rx_pin) {}

bool TwaiCanBus::begin() {
    ESP32Can.setPins(_tx, _rx);
    ESP32Can.setRxQueueSize(32);
    ESP32Can.setTxQueueSize(16);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));  // 1 Mbps
    return ESP32Can.begin();
}

bool TwaiCanBus::send(const CanMsg& msg) {
    CanFrame f = {0};
    f.identifier       = msg.id;
    f.data_length_code = msg.len;
    for (uint8_t i = 0; i < msg.len; i++) f.data[i] = msg.data[i];
    return ESP32Can.writeFrame(f, 1);  // 1 ms TX timeout
}

bool TwaiCanBus::recv(CanMsg& msg, uint32_t timeout_ms) {
    CanFrame f;
    if (!ESP32Can.readFrame(f, timeout_ms)) return false;
    msg.id  = f.identifier;
    msg.len = f.data_length_code;
    for (uint8_t i = 0; i < f.data_length_code; i++) msg.data[i] = f.data[i];
    return true;
}
