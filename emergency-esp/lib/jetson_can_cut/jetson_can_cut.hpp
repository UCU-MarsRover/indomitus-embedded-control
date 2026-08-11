#pragma once

#include <stdint.h>

/**
 * @brief Jetson CAN isolation line (Pins::JETSON_CAN_CUT, GPIO4).
 *
 * Drives the gate of the transistor that disconnects the Jetson from the CAN
 * bus:
 *   LOW  = Jetson connected to CAN   (default, normal operation)
 *   HIGH = Jetson isolated
 *
 * Built on SafetyOutput for the same reasons as PowerCut: glitch-free bring-up,
 * redundant state, RTC pad hold, and a repairing verify().
 *
 * @warning Needs a 10k external pulldown to GND at the gate. The pad floats
 *          from power-on until init() and during every reset or brownout.
 *
 * @note Isolating the Jetson does not touch rover power, and cutting power does
 *       not touch this line. Whether the two move together is policy and
 *       belongs in the application, not here.
 */
namespace JetsonCanCut {

/**
 * @brief Park the line LOW and configure the pad.
 *
 * Call early in setup(), alongside PowerCut::init().
 */
void init();

/// Disconnect the Jetson from CAN. Drives the line HIGH.
void isolate();

/// Reconnect the Jetson to CAN. Drives the line LOW.
void connect();

/// @return true if the Jetson is currently isolated from the bus.
bool is_isolated();

/**
 * @brief Re-assert the intended level and repair pad corruption.
 *
 * Call every 20-100 ms. On detected corruption the line is forced LOW, leaving
 * the Jetson connected.
 *
 * @return true if a fault was detected and corrected.
 */
bool verify();

/// Faults detected by verify() since boot.
uint32_t fault_count();

}  // namespace JetsonCanCut
