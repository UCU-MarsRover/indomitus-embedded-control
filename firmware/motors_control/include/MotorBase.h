#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Shared types used by all motor drivers
// ---------------------------------------------------------------------------

enum class MotorMode : uint8_t {
    MIT              = 1,
    PositionVelocity = 2,
    Velocity         = 3,
};

enum class MotorError : uint8_t {
    None         = 0x0,
    Enabled      = 0x1,  // normal operating state, not an error
    Overvoltage  = 0x8,
    Undervoltage = 0x9,
    Overcurrent  = 0xA,
    MosOvertemp  = 0xB,
    CoilOvertemp = 0xC,
    CommLost     = 0xD,
    Overload     = 0xE,
    Unknown      = 0xF,
};

struct MotorFeedback {
    float      pos     = 0.0f;             // rad  (motor shaft)
    float      vel     = 0.0f;             // rad/s (motor shaft)
    float      tor     = 0.0f;             // Nm
    uint8_t    t_mos   = 0;               // °C — driver MOSFET temperature
    uint8_t    t_rotor = 0;               // °C — motor coil temperature
    MotorError err     = MotorError::None;
    bool       valid   = false;            // true once first feedback received
};

// ---------------------------------------------------------------------------
// Abstract motor interface
// ---------------------------------------------------------------------------

class MotorBase {
public:
    // --- Identity ---
    virtual uint8_t escId() const = 0;

    // --- Lifecycle ---
    virtual bool enable()  = 0;
    virtual bool disable() = 0;
    virtual bool setZero() = 0;  // set current position as software zero

    // --- Mode & control ---
    virtual bool setMode(MotorMode mode) = 0;
    virtual bool sendVelocity(float v_rad_s) = 0;
    virtual bool sendPositionVelocity(float p_rad, float v_max_rad_s) = 0;
    virtual bool sendMIT(float p, float v, float kp, float kd, float tff) = 0;

    // --- ID management ---
    // Changes the motor CAN receive ID (register 8). Updates internal state.
    // persist=true calls storeToFlash() so the change survives reboot.
    virtual bool setEscId(uint8_t newId, bool persist = true) = 0;

    // Changes the feedback frame CAN ID (register 7).
    // After this the feedback dispatch loop in main must use the new ID.
    virtual bool setMstId(uint16_t newId, bool persist = true) = 0;

    // --- Raw register access ---
    virtual bool readRegister(uint8_t rid) = 0;
    virtual bool writeRegister(uint8_t rid, uint32_t val) = 0;

    // --- Persistence ---
    // Saves ALL current register values to on-board flash so they survive reboot.
    // Called automatically by setEscId/setMstId when persist=true.
    virtual bool storeToFlash() = 0;

    // --- Feedback dispatch ---
    // Called by the main loop for every received frame at the feedback CAN ID.
    // Returns true if this motor's ESC ID matched D[0] & 0x0F and the frame
    // was decoded.
    virtual bool acceptFeedback(const uint8_t data[8], uint8_t dlc) = 0;

    // Raw feedback struct — use convenience getters below for normal access.
    virtual const MotorFeedback& feedback() const = 0;

    // --- Convenience getters (non-virtual, implemented once here) ---
    float      getPosition()  const { return feedback().pos; }
    float      getVelocity()  const { return feedback().vel; }
    float      getTorque()    const { return feedback().tor; }
    uint8_t    getMosTemp()   const { return feedback().t_mos; }
    uint8_t    getRotorTemp() const { return feedback().t_rotor; }
    MotorError getError()     const { return feedback().err; }
    bool       isEnabled()    const { return feedback().err == MotorError::Enabled; }
    bool       hasFeedback()  const { return feedback().valid; }

    virtual ~MotorBase() {}
};
