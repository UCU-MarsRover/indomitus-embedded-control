#include "radio_link.hpp"

#include "pins.hpp"

extern "C" {
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "RADIO";

namespace {

constexpr uint8_t SYNC0 = 0xAA;
constexpr uint8_t SYNC1 = 0x55;

constexpr int RX_BUF_SIZE = 512;
constexpr int TX_BUF_SIZE = 512;

// --- E32 mode/config -----------------------------------------------------
/// The E32's config mode runs its UART at a FIXED 9600 8N1, whatever baud the
/// module is configured to use in transparent mode. Config traffic must
/// therefore be sent at 9600 and the baud restored afterwards.
constexpr uint32_t E32_CONFIG_BAUD = 9600;
/// Write parameters and save them across power cycles (0xC2 = don't save).
constexpr uint8_t E32_CMD_WRITE = 0xC0;
/// Read back the current parameters (sent three times).
constexpr uint8_t E32_CMD_READ = 0xC1;
/// Settling time after an M0/M1 change. The datasheet says to wait for AUX to
/// go high; AUX is not wired on this board, so use a generous fixed delay.
constexpr uint32_t MODE_SETTLE_MS = 100;
/// Time the module needs to commit parameters to its internal flash.
constexpr uint32_t CONFIG_WRITE_MS = 500;

/// Drive the mode-select pins. M0=M1=0 is transparent, M0=M1=1 is sleep/config.
void set_mode(bool m0, bool m1) {
    gpio_set_level(Pins::RADIO_M0, m0 ? 1 : 0);
    gpio_set_level(Pins::RADIO_M1, m1 ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(MODE_SETTLE_MS));
}

/**
 * @brief Write channel/baud/power parameters into the module, then verify.
 *
 * Leaves the module back in transparent mode at @p normal_baud. Costs roughly
 * a second of boot time, which is spent once and buys certainty that both
 * radios are on the same channel --- a mismatch there is silent and looks
 * exactly like a wiring fault.
 *
 * @return true if the read-back matched what was written.
 */
bool configure_module(uint32_t normal_baud) {
    const uart_port_t port = (uart_port_t)Pins::RADIO_UART_NUM;

    set_mode(true, true);  // sleep/config
    uart_set_baudrate(port, E32_CONFIG_BAUD);
    uart_flush_input(port);

    const uint8_t cmd[6] = {E32_CMD_WRITE, 0x00, 0x00,
                            Pins::RADIO_SPED, Pins::RADIO_CHANNEL,
                            Pins::RADIO_OPTION};
    uart_write_bytes(port, (const char*)cmd, sizeof(cmd));
    uart_wait_tx_done(port, pdMS_TO_TICKS(200));
    vTaskDelay(pdMS_TO_TICKS(CONFIG_WRITE_MS));

    // Read the parameters back rather than trusting the write: a module stuck
    // in the wrong mode accepts the bytes and silently does nothing.
    uart_flush_input(port);
    const uint8_t rd[3] = {E32_CMD_READ, E32_CMD_READ, E32_CMD_READ};
    uart_write_bytes(port, (const char*)rd, sizeof(rd));
    uart_wait_tx_done(port, pdMS_TO_TICKS(200));

    uint8_t resp[6] = {0};
    const int n = uart_read_bytes(port, resp, sizeof(resp), pdMS_TO_TICKS(500));

    bool ok = false;
    if (n == (int)sizeof(resp)) {
        ESP_LOGI(TAG, "E32 params: %02X %02X %02X SPED=%02X CHAN=%02X OPT=%02X",
                 resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
        ok = (resp[3] == Pins::RADIO_SPED) &&
             (resp[4] == Pins::RADIO_CHANNEL) &&
             (resp[5] == Pins::RADIO_OPTION);
        if (!ok) {
            ESP_LOGE(TAG, "E32 params NOT applied (wanted SPED=%02X CHAN=%02X "
                          "OPT=%02X)",
                     Pins::RADIO_SPED, Pins::RADIO_CHANNEL, Pins::RADIO_OPTION);
        }
    } else {
        ESP_LOGE(TAG, "E32 did not answer in config mode (%d bytes) --- check "
                      "M0/M1 wiring, module power, and TX/RX orientation", n);
    }

    set_mode(false, false);  // back to transparent
    uart_set_baudrate(port, normal_baud);
    uart_flush_input(port);
    return ok;
}

/// Incremental parser state. Kept across receive() calls so a frame split
/// across several reads is reassembled rather than lost.
enum class RxState : uint8_t {
    WAIT_SYNC0,
    WAIT_SYNC1,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC_LO,
    WAIT_CRC_HI,
};

RxState  g_state = RxState::WAIT_SYNC0;
uint8_t  g_len   = 0;
uint8_t  g_idx   = 0;
uint8_t  g_buf[RadioLink::MAX_PAYLOAD];
uint16_t g_crc_rx = 0;

uint32_t g_last_frame_ms  = 0;
bool     g_ever_received  = false;
uint32_t g_crc_errors     = 0;
uint32_t g_framing_errors = 0;
uint32_t g_rx_frames      = 0;

bool g_initialised = false;

inline uint32_t now_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
uint16_t crc16(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                             : (uint16_t)(crc << 1);
    }
    return crc;
}

/// CRC over the length byte followed by the payload.
uint16_t frame_crc(uint8_t len, const uint8_t* payload) {
    uint16_t crc = crc16(0xFFFF, len);
    for (uint8_t i = 0; i < len; ++i) {
        crc = crc16(crc, payload[i]);
    }
    return crc;
}

/**
 * @brief Feed one byte to the parser.
 * @return true when @p out has been filled with a validated frame.
 */
bool parse_byte(uint8_t b, RadioLink::Frame& out) {
    switch (g_state) {
        case RxState::WAIT_SYNC0:
            if (b == SYNC0) {
                g_state = RxState::WAIT_SYNC1;
            }
            break;

        case RxState::WAIT_SYNC1:
            // A second 0xAA keeps us waiting on the same sync word rather than
            // discarding what may be the real start of a frame.
            if (b == SYNC1) {
                g_state = RxState::WAIT_LEN;
            } else if (b != SYNC0) {
                g_state = RxState::WAIT_SYNC0;
            }
            break;

        case RxState::WAIT_LEN:
            if (b == 0 || b > RadioLink::MAX_PAYLOAD) {
                ++g_framing_errors;
                g_state = RxState::WAIT_SYNC0;
            } else {
                g_len   = b;
                g_idx   = 0;
                g_state = RxState::WAIT_PAYLOAD;
            }
            break;

        case RxState::WAIT_PAYLOAD:
            g_buf[g_idx++] = b;
            if (g_idx >= g_len) {
                g_state = RxState::WAIT_CRC_LO;
            }
            break;

        case RxState::WAIT_CRC_LO:
            g_crc_rx = b;
            g_state  = RxState::WAIT_CRC_HI;
            break;

        case RxState::WAIT_CRC_HI: {
            g_crc_rx |= (uint16_t)b << 8;
            g_state = RxState::WAIT_SYNC0;

            if (g_crc_rx != frame_crc(g_len, g_buf)) {
                ++g_crc_errors;
                ESP_LOGW(TAG, "CRC mismatch, frame dropped (total=%lu)",
                         (unsigned long)g_crc_errors);
                break;
            }

            out.len = g_len;
            for (uint8_t i = 0; i < g_len; ++i) {
                out.data[i] = g_buf[i];
            }

            ++g_rx_frames;
            g_last_frame_ms = now_ms();
            g_ever_received = true;
            return true;
        }
    }

    return false;
}

}  // namespace

namespace RadioLink {

void init(uint32_t baud) {
    if (baud == 0) {
        baud = Pins::RADIO_BAUD;
    }

    uart_config_t cfg = {};
    cfg.baud_rate = (int)baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    // Clock the UART from the crystal rather than APB: the crystal does not
    // move if APB frequency scaling ever kicks in, so the baud rate cannot
    // drift out from under the radio link.
    cfg.source_clk = UART_SCLK_XTAL;

    const uart_port_t port = (uart_port_t)Pins::RADIO_UART_NUM;

    if (uart_is_driver_installed(port)) {
        uart_driver_delete(port);
    }

    ESP_ERROR_CHECK(uart_driver_install(port, RX_BUF_SIZE, TX_BUF_SIZE,
                                        0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, Pins::RADIO_TX, Pins::RADIO_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Mode pins as driven outputs. The internal pulldown stays on so the pads
    // are biased low during the high-impedance window before this runs; the
    // external 10k pulldowns are still what actually cover that window.
    gpio_config_t mode_cfg = {};
    mode_cfg.pin_bit_mask = (1ULL << Pins::RADIO_M0) | (1ULL << Pins::RADIO_M1);
    mode_cfg.mode         = GPIO_MODE_OUTPUT;
    mode_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    mode_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    mode_cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&mode_cfg));

    configure_module(baud);

    g_state        = RxState::WAIT_SYNC0;
    g_initialised  = true;

    ESP_LOGI(TAG, "UART%d up @%lu tx=GPIO%d rx=GPIO%d ch=0x%02X",
             Pins::RADIO_UART_NUM, (unsigned long)baud,
             (int)Pins::RADIO_TX, (int)Pins::RADIO_RX, Pins::RADIO_CHANNEL);
}

bool send(const uint8_t* payload, uint8_t len) {
    if (!g_initialised || payload == nullptr || len == 0 || len > MAX_PAYLOAD) {
        return false;
    }

    uint8_t frame[MAX_PAYLOAD + FRAME_OVERHEAD];
    frame[0] = SYNC0;
    frame[1] = SYNC1;
    frame[2] = len;
    for (uint8_t i = 0; i < len; ++i) {
        frame[3 + i] = payload[i];
    }

    const uint16_t crc = frame_crc(len, payload);
    frame[3 + len]     = (uint8_t)(crc & 0xFF);
    frame[4 + len]     = (uint8_t)(crc >> 8);

    const int total   = len + FRAME_OVERHEAD;
    const int written = uart_write_bytes((uart_port_t)Pins::RADIO_UART_NUM,
                                         (const char*)frame, total);
    return written == total;
}

bool receive(Frame& out) {
    if (!g_initialised) {
        return false;
    }

    const uart_port_t port = (uart_port_t)Pins::RADIO_UART_NUM;

    uint8_t b;
    // One byte at a time keeps the parser simple and lets us return the instant
    // a frame completes, leaving any remaining bytes buffered in the driver for
    // the next call.
    while (uart_read_bytes(port, &b, 1, 0) == 1) {
        if (parse_byte(b, out)) {
            return true;
        }
    }

    return false;
}

uint32_t ms_since_last_frame() {
    if (!g_ever_received) {
        return UINT32_MAX;
    }
    return now_ms() - g_last_frame_ms;
}

bool has_link() {
    return g_ever_received;
}

uint32_t crc_error_count() {
    return g_crc_errors;
}

uint32_t framing_error_count() {
    return g_framing_errors;
}

uint32_t rx_frame_count() {
    return g_rx_frames;
}

}  // namespace RadioLink
