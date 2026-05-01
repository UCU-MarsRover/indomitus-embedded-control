#include "DamiaoMotor.h"
#include <string.h>  // memcpy

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

DamiaoMotor::DamiaoMotor(uint8_t escId, uint16_t mstId,
                         ICanBus& bus, const DamiaoConfig& config)
    : _escId(escId), _mstId(mstId), _bus(bus), _cfg(config) {}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool DamiaoMotor::enable()  { return sendSpecial(0xFC); }
bool DamiaoMotor::disable() { return sendSpecial(0xFD); }
bool DamiaoMotor::setZero() { return sendSpecial(0xFE); }

bool DamiaoMotor::sendSpecial(uint8_t code) {
    CanMsg m;
    m.id  = _escId;
    m.len = 8;
    for (uint8_t i = 0; i < 7; i++) m.data[i] = 0xFF;
    m.data[7] = code;
    return _bus.send(m);
}

// ---------------------------------------------------------------------------
// Mode & control
// ---------------------------------------------------------------------------

bool DamiaoMotor::setMode(MotorMode mode) {
    return writeRegister(10, static_cast<uint32_t>(mode));
}

bool DamiaoMotor::sendVelocity(float v_rad_s) {
    CanMsg m;
    m.id  = 0x200 + _escId;
    m.len = 4;
    memcpy(m.data, &v_rad_s, 4);
    return _bus.send(m);
}

bool DamiaoMotor::sendPositionVelocity(float p_rad, float v_max_rad_s) {
    CanMsg m;
    m.id  = 0x100 + _escId;
    m.len = 8;
    memcpy(&m.data[0], &p_rad,       4);
    memcpy(&m.data[4], &v_max_rad_s, 4);
    return _bus.send(m);
}

bool DamiaoMotor::sendMIT(float p, float v, float kp, float kd, float tff) {
    uint16_t p_int  = floatToUint(p,   -_cfg.pMax,         _cfg.pMax,         16);
    uint16_t v_int  = floatToUint(v,   -_cfg.vMax,         _cfg.vMax,         12);
    uint16_t kp_int = floatToUint(kp,   0.0f,              DamiaoConfig::KP_MAX, 12);
    uint16_t kd_int = floatToUint(kd,   0.0f,              DamiaoConfig::KD_MAX, 12);
    uint16_t t_int  = floatToUint(tff, -_cfg.tMax,         _cfg.tMax,         12);

    CanMsg m;
    m.id  = _escId;
    m.len = 8;
    m.data[0] =  p_int >> 8;
    m.data[1] =  p_int & 0xFF;
    m.data[2] =  v_int >> 4;
    m.data[3] = ((v_int  & 0xF) << 4) | (kp_int >> 8);
    m.data[4] =  kp_int & 0xFF;
    m.data[5] =  kd_int >> 4;
    m.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    m.data[7] =  t_int & 0xFF;
    return _bus.send(m);
}

// ---------------------------------------------------------------------------
// ID management
// ---------------------------------------------------------------------------

bool DamiaoMotor::setEscId(uint8_t newId, bool persist) {
    if (!writeRegister(8, newId)) return false;
    _escId = newId;  // update local state — future control frames use new ID
    return persist ? storeToFlash() : true;
}

bool DamiaoMotor::setMstId(uint16_t newId, bool persist) {
    if (!writeRegister(7, newId)) return false;
    _mstId = newId;
    return persist ? storeToFlash() : true;
}

// ---------------------------------------------------------------------------
// Raw register access
// ---------------------------------------------------------------------------

bool DamiaoMotor::readRegister(uint8_t rid) {
    CanMsg m = {0};
    m.id     = 0x7FF;
    m.len    = 8;
    m.data[0] = _escId & 0xFF;
    m.data[1] = (_escId >> 8) & 0xFF;
    m.data[2] = 0x33;
    m.data[3] = rid;
    return _bus.send(m);
}

bool DamiaoMotor::writeRegister(uint8_t rid, uint32_t val) {
    CanMsg m = {0};
    m.id     = 0x7FF;
    m.len    = 8;
    m.data[0] = _escId & 0xFF;
    m.data[1] = (_escId >> 8) & 0xFF;
    m.data[2] = 0x55;
    m.data[3] = rid;
    m.data[4] =  val        & 0xFF;
    m.data[5] = (val >> 8)  & 0xFF;
    m.data[6] = (val >> 16) & 0xFF;
    m.data[7] = (val >> 24) & 0xFF;
    return _bus.send(m);
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

bool DamiaoMotor::storeToFlash() {
    CanMsg m = {0};
    m.id     = 0x7FF;
    m.len    = 8;
    m.data[0] = _escId & 0xFF;
    m.data[1] = (_escId >> 8) & 0xFF;
    m.data[2] = 0xAA;
    m.data[3] = 0x01;
    return _bus.send(m);
}

// ---------------------------------------------------------------------------
// Feedback decoding
// ---------------------------------------------------------------------------

bool DamiaoMotor::acceptFeedback(const uint8_t data[8], uint8_t dlc) {
    if (dlc < 8) return false;
    if ((data[0] & 0x0F) != _escId) return false;

    uint16_t p_int = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    uint16_t v_int = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    uint16_t t_int = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    _fb.err     = static_cast<MotorError>((data[0] >> 4) & 0x0F);
    _fb.pos     = uintToFloat(p_int, -_cfg.pMax, _cfg.pMax, 16);
    _fb.vel     = uintToFloat(v_int, -_cfg.vMax, _cfg.vMax, 12);
    _fb.tor     = uintToFloat(t_int, -_cfg.tMax, _cfg.tMax, 12);
    _fb.t_mos   = data[6];
    _fb.t_rotor = data[7];
    _fb.valid   = true;
    return true;
}

// ---------------------------------------------------------------------------
// Encoding helpers
// ---------------------------------------------------------------------------

uint16_t DamiaoMotor::floatToUint(float x, float xmin, float xmax, int bits) {
    if (x < xmin) x = xmin;
    if (x > xmax) x = xmax;
    float span = xmax - xmin;
    return static_cast<uint16_t>((x - xmin) * static_cast<float>((1 << bits) - 1) / span);
}

float DamiaoMotor::uintToFloat(uint16_t x, float xmin, float xmax, int bits) {
    float span = xmax - xmin;
    return static_cast<float>(x) * span / static_cast<float>((1 << bits) - 1) + xmin;
}
