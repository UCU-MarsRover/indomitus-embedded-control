#pragma once
#include <stdint.h>

// Plain POD frame — motor code never touches twai_message_t or any library type.
// TwaiCanBus.cpp is the only translation point between this and the library.
struct CanMsg {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
};

class ICanBus {
public:
    // Returns true on success.
    virtual bool send(const CanMsg& msg) = 0;

    // Fills msg if a frame is available. timeout_ms=0 is non-blocking.
    // Returns true if a frame was read.
    virtual bool recv(CanMsg& msg, uint32_t timeout_ms = 0) = 0;

    virtual ~ICanBus() {}
};
