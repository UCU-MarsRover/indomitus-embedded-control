#pragma once
#include "MotorBase.h"
#include "ICanBus.h"

// ---------------------------------------------------------------------------
// Per-model configuration
// pMax/vMax/tMax must match what is stored in the motor's registers 21/22/23.
// Verify J4340 values by reading those registers on first boot.
// ---------------------------------------------------------------------------

struct DamiaoConfig {
    float pMax      = 12.5f;   // rad   — position encoding range [-pMax, +pMax]
    float vMax      = 50.0f;   // rad/s — velocity encoding range
    float tMax      = 20.0f;   // Nm    — torque encoding range
    float gearRatio = 10.0f;   // motor-to-output shaft (informational)

    // Protocol constants — identical for all Damiao motors
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MAX = 5.0f;
};

namespace DamiaoConfigs {
    // DM-J4340-2EC  (40:1 gearbox, 24 V)
    // tMax is a placeholder — read register 23 from the physical motor to confirm.
    inline DamiaoConfig J4340() {
        DamiaoConfig c;
        c.pMax      = 12.5f;
        c.vMax      = 50.0f;
        c.tMax      = 10.0f;
        c.gearRatio = 40.0f;
        return c;
    }

    // DM-J10010L-2EC  (10:1 gearbox, 24–48 V)
    inline DamiaoConfig J10010L() {
        DamiaoConfig c;
        c.pMax      = 12.5f;
        c.vMax      = 50.0f;
        c.tMax      = 20.0f;
        c.gearRatio = 10.0f;
        return c;
    }
}

// ---------------------------------------------------------------------------
// Damiao motor driver — works for both J4340-2EC and J10010L-2EC
// ---------------------------------------------------------------------------

class DamiaoMotor : public MotorBase {
public:
    // escId  — motor ESC_ID (register 8, default 0x01 on factory motors)
    // mstId  — feedback frame CAN ID (register 7, default 0x000)
    // bus    — shared CAN transport; must outlive this object
    // config — per-model parameters
    DamiaoMotor(uint8_t escId, uint16_t mstId, ICanBus& bus,
                const DamiaoConfig& config = DamiaoConfig());

    // --- Identity ---
    uint8_t  escId() const override { return _escId; }
    uint16_t mstId() const          { return _mstId; }

    // --- Lifecycle ---
    bool enable()  override;
    bool disable() override;
    bool setZero() override;

    // --- Mode & control ---
    bool setMode(MotorMode mode) override;
    bool sendVelocity(float v_rad_s) override;
    bool sendPositionVelocity(float p_rad, float v_max_rad_s) override;
    bool sendMIT(float p, float v, float kp, float kd, float tff) override;

    // --- ID management ---
    bool setEscId(uint8_t  newId, bool persist = true) override;
    bool setMstId(uint16_t newId, bool persist = true) override;

    // --- Raw register access ---
    bool readRegister(uint8_t rid) override;
    bool writeRegister(uint8_t rid, uint32_t val) override;

    // --- Persistence ---
    bool storeToFlash() override;

    // --- Feedback ---
    bool acceptFeedback(const uint8_t data[8], uint8_t dlc) override;
    const MotorFeedback& feedback() const override { return _fb; }

private:
    bool sendSpecial(uint8_t code);  // 0xFC enable / 0xFD disable / 0xFE zero

    static uint16_t floatToUint(float x, float xmin, float xmax, int bits);
    static float    uintToFloat(uint16_t x, float xmin, float xmax, int bits);

    uint8_t       _escId;
    uint16_t      _mstId;
    ICanBus&      _bus;
    DamiaoConfig  _cfg;
    MotorFeedback _fb;
};
