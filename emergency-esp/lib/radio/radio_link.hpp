#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Framed UART link to the radio adapter (UART1, GPIO0 RX / GPIO1 TX).
 *
 * Transport only. Payload contents and any policy --- what a remote command may
 * do, whether loss of link should trigger anything --- belong to the
 * application.
 *
 * A radio link delivers noise, truncated bursts, and bit errors, so a raw byte
 * stream is not safe to act on: garbage can look exactly like a valid command.
 * Every payload is therefore wrapped:
 *
 *   AA 55 | len | payload[len] | crc16_lo crc16_hi
 *
 * CRC-16/CCITT-FALSE over the length byte and the payload. The receiver
 * resynchronises on the sync word, so it recovers from any amount of junk
 * between frames, and a payload only surfaces from receive() once its CRC
 * checks out.
 *
 * @note Link supervision is left to the caller: use ms_since_last_frame() to
 *       implement whatever timeout policy the rover needs. This module
 *       intentionally does not act on silence by itself.
 */
namespace RadioLink {

/// Largest payload a single frame can carry.
constexpr uint8_t MAX_PAYLOAD = 64;

/// Total framing overhead in bytes (sync 2 + len 1 + crc 2).
constexpr uint8_t FRAME_OVERHEAD = 5;

/// A validated, received payload.
struct Frame {
    uint8_t len;
    uint8_t data[MAX_PAYLOAD];
};

/**
 * @brief Bring up UART1 on the radio pins.
 *
 * @param baud Radio baud rate; defaults to Pins::RADIO_BAUD.
 */
void init(uint32_t baud = 0);

/**
 * @brief Frame and transmit a payload.
 *
 * @param payload Bytes to send.
 * @param len     Length, must be 1..MAX_PAYLOAD.
 * @return true if the whole frame was handed to the UART driver.
 */
bool send(const uint8_t* payload, uint8_t len);

/**
 * @brief Drain the UART and return one complete, CRC-valid frame.
 *
 * Non-blocking. Call frequently (every 5-20 ms); the parser is incremental, so
 * partial frames are retained between calls. Call in a loop while it returns
 * true to avoid a backlog when several frames arrive together.
 *
 * @param out Populated only when true is returned.
 * @return true if a valid frame was produced.
 */
bool receive(Frame& out);

/// Milliseconds since the last CRC-valid frame. UINT32_MAX if none yet.
uint32_t ms_since_last_frame();

/// @return true if at least one valid frame has ever been received.
bool has_link();

/// Frames rejected for a bad CRC since boot.
uint32_t crc_error_count();

/// Frames dropped for an out-of-range length byte since boot.
uint32_t framing_error_count();

/// Valid frames received since boot.
uint32_t rx_frame_count();

}  // namespace RadioLink
