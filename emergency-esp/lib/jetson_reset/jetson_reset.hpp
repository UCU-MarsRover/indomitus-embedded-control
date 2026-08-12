#pragma once

#include <stdint.h>

/**
 * @brief Jetson reset line (Pins::JETSON_RESET, GPIO2).
 *
 * Drives an optocoupler that pulls SYS_RESET* on the reComputer's REC switch
 * header down to GND --- electrically the same thing the case's RESET pinhole
 * button does.
 *
 * ACTIVE LOW, unlike PowerCut and JetsonCanCut. GPIO2 is a strapping pin that
 * must read HIGH at reset, and SYS_RESET* is a 1.8V open-drain input that an
 * ESP pin must never drive. The opto solves both: see pins.hpp for the wiring.
 *
 * Built on SafetyOutput, so it gets the same glitch-free bring-up, redundant
 * state and repairing verify() as the two cut lines --- with the safe state
 * inverted to HIGH.
 *
 * A reset is a timed pulse, so this is a small non-blocking state machine:
 * call reset(), then update() regularly until is_busy() goes false. Nothing
 * blocks, because the node must keep polling its e-stop button throughout.
 */
namespace JetsonReset {

/// Pulse width, comfortably longer than the Orin's minimum reset assertion.
constexpr uint32_t PULSE_MS = 100;

/// Hard ceiling on how long the line may stay asserted. If update() ever sees
/// a pulse running past this --- a lost or corrupted timer --- the line is
/// released and the fault counted. A Jetson held permanently in reset is just
/// a dead Jetson.
constexpr uint32_t MAX_ASSERT_MS = 1000;

/**
 * @brief Park the line in its idle (HIGH) state and configure the pad.
 *
 * Call early in setup(), alongside PowerCut::init() and JetsonCanCut::init().
 */
void init();

/**
 * @brief Begin a reset pulse.
 *
 * @return false if a pulse is already in progress.
 */
bool reset();

/**
 * @brief Release the line once the pulse has run its course.
 *
 * Call every loop iteration, or at least every 20 ms. Non-blocking.
 */
void update();

/// Release the line immediately.
void abort();

/// @return true while a reset pulse is in progress.
bool is_busy();

/// Completed reset pulses since boot.
uint32_t reset_count();

/**
 * @brief Re-assert the intended level and repair pad corruption.
 *
 * Call every 20-100 ms. On detected corruption the line is forced HIGH, which
 * here is the safe state: Jetson left running.
 *
 * @return true if a fault was detected and corrected.
 */
bool verify();

/// Faults detected by verify() and by the MAX_ASSERT_MS guard since boot.
uint32_t fault_count();

}  // namespace JetsonReset
