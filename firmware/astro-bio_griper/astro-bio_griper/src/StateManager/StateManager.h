#pragma once

#include <Arduino.h>
#include "Config.h"
#include "StateDefinitions.h"
#include "CanDictionary.h"
#include "CanManager.h"
#include "SensorArray.h"
#include "PumpController.h"

// =============================================================================
// StateManager — Finite State Machine controlling the extraction sequence
// =============================================================================

class StateManager {
public:
    /// Construct with references to all hardware abstractions.
    StateManager(CanManager& can, SensorArray& sensors, PumpController& pump);

    /// Main FSM tick — call every loop(). Strictly non-blocking.
    void run();

    /// @return current FSM state.
    SystemState getState() const;

private:
    // State handlers
    void handleIdle();
    void handlePositioning();
    void handleExtraction();
    void handleAcquisition();
    void handleError();

    /// Check global safety conditions (E-STOP, bus health).
    /// @return true if a safety event forced a state change.
    bool checkSafety();

    /// Transition to a new state with logging.
    void transitionTo(SystemState newState);

    // Hardware references (non-owning)
    CanManager&     _can;
    SensorArray&    _sensors;
    PumpController& _pump;

    // FSM state
    SystemState     _state = SystemState::IDLE;
    PositioningPhase _posPhase = PositioningPhase::WAIT_FOR_BOTTOM;
};
