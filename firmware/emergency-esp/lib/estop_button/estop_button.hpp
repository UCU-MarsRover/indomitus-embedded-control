#pragma once

#include <stdint.h>

/**
 * @brief Debounced E-stop button input (Pins::ESTOP_BUTTON, GPIO3).
 *
 * Electrically: the button drives +3V3 when pressed. The pad uses the internal
 * pulldown, so an open, unplugged, or broken button reads 0 = not pressed.
 *
 * @note This is a fail-*silent* wiring: a cut button wire looks identical to a
 *       button nobody is pressing, and the ESP cannot tell the difference. That
 *       is inherent to an active-high button with a pulldown, which is what the
 *       hardware does. If a broken wire must be detectable, the button needs a
 *       second, inverted contact so the two lines can be cross-checked.
 *
 * The module reports state and edges; it decides nothing. Latching, rearm
 * rules, and what a press actually triggers are application policy.
 */
namespace EstopButton {

/// Edge reported by poll(), consumed once.
enum class Event : uint8_t {
    NONE = 0,
    PRESSED,   ///< Debounced transition to pressed.
    RELEASED,  ///< Debounced transition to released.
};

/// Debounce window. A level must hold this long before it is accepted.
constexpr uint32_t DEBOUNCE_MS = 25;

/// Suggested interval between poll() calls.
constexpr uint32_t POLL_INTERVAL_MS = 5;

/**
 * @brief Configure the pad as an input with the internal pulldown enabled.
 *
 * The debounced state is seeded from the pad's actual level, so a button
 * already held down at boot is reported as pressed rather than as a press edge.
 */
void init();

/**
 * @brief Sample the pin and advance the debounce filter.
 *
 * Call every POLL_INTERVAL_MS. Non-blocking.
 *
 * @return The edge that just completed debouncing, or Event::NONE. Each edge is
 *         returned exactly once.
 */
Event poll();

/// @return Current debounced state. true = pressed.
bool is_pressed();

/// Milliseconds the button has been in its current debounced state.
uint32_t time_in_state_ms();

/// Total debounced presses since boot.
uint32_t press_count();

}  // namespace EstopButton
