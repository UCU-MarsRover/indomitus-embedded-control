#pragma once

#include "can_driver.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Optional FreeRTOS layer over can_driver.
 *
 * Same structure as light_control/lib/can/can_manager: an RX task, a queued TX
 * task that survives bus-off, and an optional monitor task. The difference is
 * that nothing here knows the protocol --- where light_control switch()es on
 * its own command bytes, this layer hands each frame to a callback you supply.
 *
 * Entirely optional. If you would rather poll can_recv() from your own loop,
 * ignore this file; can_driver works standalone.
 */
namespace CanService {

/**
 * @brief Called from the RX task for every received frame.
 *
 * Runs in task context, so it may block --- but keep it short, since frames
 * queue up behind it.
 */
using RxHandler = void (*)(const CanMsg& msg);

/// Depth of the outbound queue.
constexpr uint32_t TX_QUEUE_LEN = 16;

/**
 * @brief Start the RX and TX tasks.
 *
 * Call after can_init().
 *
 * @param handler      Invoked for each received frame; may be nullptr.
 * @param with_monitor Also start a task that logs controller counters at
 *                     DEBUG level every 500 ms.
 * @return true if the tasks and queue were created.
 */
bool start(RxHandler handler, bool with_monitor = false);

/**
 * @brief Queue a frame for the TX task.
 *
 * Non-blocking beyond a short queue wait, so it is safe to call from timing
 * sensitive code that must not stall on a busy bus.
 *
 * @return true if the frame was queued; false if the queue was full.
 */
bool enqueue(uint32_t id, const uint8_t* data, uint8_t len);

/// Frames dropped because the TX queue was full.
uint32_t tx_dropped_count();

}  // namespace CanService
