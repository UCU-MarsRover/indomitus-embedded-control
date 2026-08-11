#pragma once

#include <stdint.h>
#include "driver/gpio.h"

/**
 * @brief Pin map for the emergency (E-stop) ESP32-C3-MINI-1.
 *
 * ---------------------------------------------------------------------------
 * Why these pins
 * ---------------------------------------------------------------------------
 * The C3-MINI-1 module breaks out GPIO 0..10 and 18..21. Of those, the
 * following are unusable or unsafe for this board and are deliberately avoided:
 *
 *   GPIO 11..17   SPI flash, module-internal, not broken out.
 *   GPIO 2, 8, 9  Strapping pins. Sampled at every reset and driven by
 *                 boot-time pulls, so they are never safe for an output that
 *                 must not glitch, and they distort an input at boot.
 *                 GPIO9 is also the BOOT button on most dev boards.
 *   GPIO 18, 19   USB D-/D+ (USB-Serial-JTAG), used for flashing and console.
 *
 * That leaves GPIO 0, 1, 3, 4, 5, 6, 7, 10, 20, 21 --- enough for all seven
 * signals with room to spare.
 *
 * The two transistor-gate lines (POWER_CUT, JETSON_CAN_CUT) are placed on
 * GPIO4 and GPIO5 because those are RTC-capable pads: they can be latched with
 * gpio_hold_en() so the level survives sleep and any peripheral reconfiguration
 * elsewhere in the firmware. Losing MTMS/MTDI (external JTAG) costs nothing
 * here since the C3 debugs over its built-in USB-Serial-JTAG.
 *
 * The radio is put on GPIO0/GPIO1 driven by UART1 rather than the default
 * UART0 pads (GPIO20/21). GPIO21 is U0TXD and emits the ROM bootloader log at
 * every single reset; sending that garbage into a radio link wastes airtime and
 * can upset modems that parse their input. GPIO20/21 stay free as a clean
 * debug-console header. GPIO0/1 are the 32 kHz crystal pads, which the
 * C3-MINI-1 leaves unpopulated, so they are ordinary GPIOs here.
 *
 * ---------------------------------------------------------------------------
 * Pin map
 * ---------------------------------------------------------------------------
 *   GPIO 0   RADIO_RX        <-- radio TX
 *   GPIO 1   RADIO_TX        --> radio RX
 *   GPIO 3   ESTOP_BUTTON    <-- button (active high)
 *   GPIO 4   JETSON_CAN_CUT  --> transistor gate  [needs 10k pulldown]
 *   GPIO 5   POWER_CUT       --> transistor gate  [needs 10k pulldown]
 *   GPIO 6   CAN_TX          --> transceiver TXD
 *   GPIO 7   CAN_RX          <-- transceiver RXD
 *   free: 2, 8, 9 (strapping), 10, 20, 21 (debug UART)
 */
namespace Pins {

// --- E-stop button -------------------------------------------------------
/// Button drives +3V3 when pressed. Idle is held at 0 by the internal
/// pulldown, so a disconnected or broken button reads as "not pressed".
constexpr gpio_num_t ESTOP_BUTTON = GPIO_NUM_3;

// --- CAN transceiver (TWAI) ----------------------------------------------
constexpr gpio_num_t CAN_TX = GPIO_NUM_20;
constexpr gpio_num_t CAN_RX = GPIO_NUM_10;

// --- Critical transistor-gate outputs ------------------------------------
/// Rover main power cut. HIGH closes the transistor and kills the entire rover
/// supply, so LOW is the normal running state.
///
/// REQUIRES a 10k external pulldown to GND at the gate --- see SafetyOutput for
/// why firmware alone cannot hold this line low across reset and brownout.
constexpr gpio_num_t POWER_CUT = GPIO_NUM_5;

/// Cuts the Jetson off the CAN bus. HIGH = Jetson isolated, LOW = connected.
///
/// REQUIRES a 10k external pulldown to GND at the gate.
constexpr gpio_num_t JETSON_CAN_CUT = GPIO_NUM_4;

// --- Radio adapter (UART1) -----------------------------------------------
/// ESP RX, wired to the radio module's TX.
constexpr gpio_num_t RADIO_RX = GPIO_NUM_0;
/// ESP TX, wired to the radio module's RX.
constexpr gpio_num_t RADIO_TX = GPIO_NUM_1;

/// UART port number used for the radio. UART0 is left alone for the console.
constexpr int RADIO_UART_NUM = 1;
/// Default radio baud rate. Override in RadioLink::init() if the modem differs.
constexpr uint32_t RADIO_BAUD = 115200;

}  // namespace Pins
