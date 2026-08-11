#pragma once

#include <stdint.h>

/**
 * @brief Rover main power cut line (Pins::POWER_CUT, GPIO5).
 *
 * Drives the gate of the transistor that kills the entire rover supply:
 *   LOW  = transistor open, rover powered   (default, normal operation)
 *   HIGH = transistor closed, rover dead
 *
 * This is the most destructive output on the board, so it is built on
 * SafetyOutput: glitch-free bring-up, redundant state, RTC pad hold, and a
 * verify() that repairs the pad if anything corrupts it. See safety_output.hpp
 * for what that does and does not guarantee.
 *
 * @warning The line still needs a 10k external pulldown to GND at the gate.
 *          Between power-on and init(), and during every reset or brownout,
 *          the pad floats and no firmware can control it.
 *
 * @note Engaging is unconditional; releasing is a deliberate act by the caller.
 *       This module holds no policy about when either should happen.
 */
namespace PowerCut {

/**
 * @brief Park the line LOW and configure the pad.
 *
 * Call first in setup(), before CAN, radio, or anything else, so the window
 * where the pad is undriven is as short as possible.
 */
void init();

/// Cut rover power. Drives the line HIGH.
void engage();

/// Restore rover power. Drives the line LOW.
void release();

/// @return true if power is currently being cut.
bool is_engaged();

/**
 * @brief Re-assert the intended level and repair pad corruption.
 *
 * Call every 20-100 ms. On detected corruption the line is forced LOW (rover
 * powered), which is the non-destructive state.
 *
 * @return true if a fault was detected and corrected.
 */
bool verify();

/// Faults detected by verify() since boot.
uint32_t fault_count();

}  // namespace PowerCut
