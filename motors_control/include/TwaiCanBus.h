#pragma once
#include "ICanBus.h"
#include <stdint.h>

// ESP32 TWAI peripheral implementation of ICanBus.
// To swap the CAN transport, replace this class and its .cpp — nothing else changes.
class TwaiCanBus : public ICanBus {
public:
    // Pins default to project hardware: TX=GPIO5, RX=GPIO4.
    TwaiCanBus(int8_t tx_pin = 5, int8_t rx_pin = 4);

    // Call once in setup(). Returns false if the TWAI driver fails to start.
    bool begin();

    bool send(const CanMsg& msg) override;
    bool recv(CanMsg& msg, uint32_t timeout_ms = 0) override;

private:
    int8_t _tx;
    int8_t _rx;
};
