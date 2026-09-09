#include "StateManager.h"

// =============================================================================
// StateManager.cpp — FSM implementation
// =============================================================================

// Helper: human-readable state names for Serial logging
static const char* stateToStr(SystemState s) {
    switch (s) {
        case SystemState::IDLE:         return "IDLE";
        case SystemState::POSITIONING:  return "POSITIONING";
        case SystemState::EXTRACTION:   return "EXTRACTION";
        case SystemState::ACQUISITION:  return "ACQUISITION";
        case SystemState::ERROR:        return "ERROR";
        default:                        return "UNKNOWN";
    }
}

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------

StateManager::StateManager(CanManager& can, SensorArray& sensors, PumpController& pump)
    : _can(can)
    , _sensors(sensors)
    , _pump(pump)
{}

// -----------------------------------------------------------------------------
// Public interface
// -----------------------------------------------------------------------------

SystemState StateManager::getState() const {
    return _state;
}

void StateManager::run() {
    // ---- Global safety check (runs before any state logic) ---------------
    if (checkSafety()) {
        return;     // State was forced by safety event
    }

    // ---- Dispatch to current state handler -------------------------------
    switch (_state) {
        case SystemState::IDLE:         handleIdle();         break;
        case SystemState::POSITIONING:  handlePositioning();  break;
        case SystemState::EXTRACTION:   handleExtraction();   break;
        case SystemState::ACQUISITION:  handleAcquisition();  break;
        case SystemState::ERROR:        handleError();        break;
    }
}

// -----------------------------------------------------------------------------
// Safety
// -----------------------------------------------------------------------------

bool StateManager::checkSafety() {
    // E-STOP: immediate pump halt + ERROR state
    if (_can.hasEmergencyStop()) {
        _pump.stopPump();
        transitionTo(SystemState::ERROR);
        _can.sendStatus(CanStatus::STATUS_ERROR);
        Serial.println(F("[FSM] !!! E-STOP → ERROR"));
        return true;
    }

    // Bus health: if bus is disconnected and pump is running, halt
    if (!_can.isBusHealthy() && _pump.isRunning()) {
        _pump.stopPump();
        transitionTo(SystemState::ERROR);
        _can.sendStatus(CanStatus::STATUS_ERROR);
        Serial.println(F("[FSM] !!! CAN bus timeout → pump halted → ERROR"));
        return true;
    }

    return false;
}

// -----------------------------------------------------------------------------
// IDLE — pump OFF, waiting for CAN commands
// -----------------------------------------------------------------------------

void StateManager::handleIdle() {
    CanCommand cmd;
    if (!_can.popCommand(cmd)) {
        return;     // Nothing to do
    }

    switch (cmd) {
        case CanCommand::CMD_START_POSITIONING:
            _posPhase = PositioningPhase::WAIT_FOR_BOTTOM;
            transitionTo(SystemState::POSITIONING);
            break;

        case CanCommand::CMD_START_EXTRACTION:
            transitionTo(SystemState::EXTRACTION);
            _pump.startPump(PUMP_DEFAULT_DUTY);
            break;

        case CanCommand::CMD_START_ACQUISITION:
            transitionTo(SystemState::ACQUISITION);
            break;

        default:
            // Ignore unexpected commands while IDLE
            break;
    }
}

// -----------------------------------------------------------------------------
// POSITIONING — two-phase bottom detection + lift-off
// -----------------------------------------------------------------------------

void StateManager::handlePositioning() {
    // Allow abort via CAN
    CanCommand cmd;
    if (_can.popCommand(cmd) && cmd == CanCommand::CMD_RETURN_TO_IDLE) {
        transitionTo(SystemState::IDLE);
        _can.sendStatus(CanStatus::STATUS_IDLE);
        return;
    }

    switch (_posPhase) {
        case PositioningPhase::WAIT_FOR_BOTTOM:
            if (_sensors.isBottomDetected()) {
                // Black strip detected — tell Main Computer to stop lowering
                _can.sendStatus(CanStatus::STATUS_BOTTOM_REACHED);
                _posPhase = PositioningPhase::WAIT_FOR_LIFTOFF;
                Serial.println(F("[FSM] Bottom detected → waiting for lift-off"));
            }
            break;

        case PositioningPhase::WAIT_FOR_LIFTOFF:
            if (!_sensors.isBottomDetected()) {
                // Sensor returned to white — arm was pulled up to optimal height
                _can.sendStatus(CanStatus::STATUS_OPTIMAL_POSITION);
                transitionTo(SystemState::IDLE);
                _can.sendStatus(CanStatus::STATUS_IDLE);
                Serial.println(F("[FSM] Lift-off confirmed → IDLE"));
            }
            break;
    }
}

// -----------------------------------------------------------------------------
// EXTRACTION — pump running, monitoring water level
// -----------------------------------------------------------------------------

void StateManager::handleExtraction() {
    // Allow abort via CAN
    CanCommand cmd;
    if (_can.popCommand(cmd) && cmd == CanCommand::CMD_RETURN_TO_IDLE) {
        _pump.stopPump();
        transitionTo(SystemState::IDLE);
        _can.sendStatus(CanStatus::STATUS_IDLE);
        return;
    }

    uint16_t level = _sensors.getWaterLevel();

    if (level >= WATER_LEVEL_TARGET) {
        _pump.stopPump();
        _can.sendStatus(CanStatus::STATUS_COLLECTION_DONE);
        transitionTo(SystemState::IDLE);
        _can.sendStatus(CanStatus::STATUS_IDLE);
        Serial.printf("[FSM] Water target reached (ADC=%u) → pump OFF → IDLE\n", level);
    }
}

// -----------------------------------------------------------------------------
// ACQUISITION — polling pH until variance stabilises
// -----------------------------------------------------------------------------

void StateManager::handleAcquisition() {
    // Allow abort via CAN
    CanCommand cmd;
    if (_can.popCommand(cmd) && cmd == CanCommand::CMD_RETURN_TO_IDLE) {
        transitionTo(SystemState::IDLE);
        _can.sendStatus(CanStatus::STATUS_IDLE);
        return;
    }

    float stablePh = 0.0f;
    if (_sensors.isPhStable(stablePh)) {
        // Transmit stable pH as a 4-byte float in the CAN payload
        _can.sendFloat(CanStatus::STATUS_PH_READY, stablePh);
        transitionTo(SystemState::IDLE);
        _can.sendStatus(CanStatus::STATUS_IDLE);
        Serial.printf("[FSM] pH stable = %.3f → transmitted → IDLE\n", stablePh);
    }
}

// -----------------------------------------------------------------------------
// ERROR — pump halted, awaiting manual recovery
// -----------------------------------------------------------------------------

void StateManager::handleError() {
    // Only CMD_RETURN_TO_IDLE can exit ERROR state
    CanCommand cmd;
    if (_can.popCommand(cmd) && cmd == CanCommand::CMD_RETURN_TO_IDLE) {
        transitionTo(SystemState::IDLE);
        _can.sendStatus(CanStatus::STATUS_IDLE);
        Serial.println(F("[FSM] Error cleared → IDLE"));
    }
}

// -----------------------------------------------------------------------------
// State transition helper
// -----------------------------------------------------------------------------

void StateManager::transitionTo(SystemState newState) {
    if (_state != newState) {
        Serial.printf("[FSM] %s → %s\n", stateToStr(_state), stateToStr(newState));
        _state = newState;
    }
}
