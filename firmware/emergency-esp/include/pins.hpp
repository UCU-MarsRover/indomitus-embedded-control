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
 * The three command lines (GPIO2/3/4) are MOMENTARY PULSES, not held levels:
 * the power system latches its own state, so this node emulates button presses
 * rather than holding a cut line.
 *
 * The radio is put on GPIO0/GPIO1 driven by UART1. GPIO0/1 are the 32 kHz
 * crystal pads, which the C3-MINI-1 leaves unpopulated, so they are ordinary
 * GPIOs here.
 *
 * CAN uses GPIO20 for TX and GPIO10 for RX. GPIO20 is U0RXD, an input at reset
 * with an internal pullup, which conveniently holds the transceiver's TXD
 * recessive through the boot window. GPIO21 is deliberately NOT used for CAN:
 * it is U0TXD and emits the ROM plus second-stage bootloader log at every
 * reset, which through a transceiver would inject that burst straight onto the
 * bus as error frames. It stays free as an output-only debug TX.
 *
 * ---------------------------------------------------------------------------
 * Pin map
 * ---------------------------------------------------------------------------
 *   GPIO 0   RADIO_RX        <-- radio TX
 *   GPIO 1   RADIO_TX        --> radio RX
 *   GPIO 2   JETSON_RESET    --> pulse HIGH 1s   [NO external pulldown!]
 *   GPIO 3   POWER_ON        --> pulse HIGH 1s
 *   GPIO 4   POWER_OFF       --> pulse LOW 1s    (idles HIGH)
 *   GPIO 5   ESTOP_BUTTON    <-- button (active high)
 *   GPIO 6   RADIO_M0        --> radio mode select [needs 10k pulldown]
 *   GPIO 7   RADIO_M1        --> radio mode select [needs 10k pulldown]
 *   GPIO 10  CAN_RX          <-- transceiver RXD
 *   GPIO 20  CAN_TX          --> transceiver TXD  [needs 10k pullup to 3V3]
 *   spare: 8, 9 (strapping), 21 (debug TX)
 */
namespace Pins {

// --- E-stop button -------------------------------------------------------
/// Button drives +3V3 when pressed. Idle is held at 0 by the internal
/// pulldown, so a disconnected or broken button reads as "not pressed".
///
/// NOT GPIO3 --- that pad is the POWER_ON output. GPIO5 is the pad freed by
/// the old held-level POWER_CUT line: no strapping function, RTC-capable, so
/// it is a clean input.
///
/// @note If the button is wired to a different pad on your board, change this
///       one constant. Nothing else depends on the number.
constexpr gpio_num_t ESTOP_BUTTON = GPIO_NUM_5;

// --- CAN transceiver (TWAI) ----------------------------------------------
constexpr gpio_num_t CAN_TX = GPIO_NUM_20;
constexpr gpio_num_t CAN_RX = GPIO_NUM_10;

// --- Rover power command lines -------------------------------------------
// Two MOMENTARY lines, not one held cut line. The power system latches its own
// state, so the ESP presses buttons: one pulse to switch on, one to switch off.
// Between pulses the ESP has no influence on rover power at all.
//
// Polarities below are the ones verified on the physical board by
// main_test_hw_commands.cpp. Note that the two lines are opposite senses.
//
// Consequence worth stating: a dead or unpowered emergency ESP leaves the
// rover in whatever state it was last commanded into. Losing this board will
// not stop the rover --- and equally cannot spuriously stop it.

/// Switches the rover ON. Idles LOW, pulses HIGH.
constexpr gpio_num_t POWER_ON = GPIO_NUM_3;

/// Switches the rover OFF. Idles HIGH, pulses LOW.
constexpr gpio_num_t POWER_OFF = GPIO_NUM_4;

// --- Radio adapter (UART1) -----------------------------------------------
/// ESP RX, wired to the radio module's TX.
constexpr gpio_num_t RADIO_RX = GPIO_NUM_0;
/// ESP TX, wired to the radio module's RX.
constexpr gpio_num_t RADIO_TX = GPIO_NUM_1;

/// UART port number used for the radio. UART0 is left alone for the console.
constexpr int RADIO_UART_NUM = 1;
/// Default radio baud rate. Override in RadioLink::init() if the modem differs.
constexpr uint32_t RADIO_BAUD = 115200;

/// Radio mode-select pins. M0=M1=0 is normal transparent mode.
///
/// These must never float: the module's internal pull-ups would read 1,1 =
/// sleep/config mode and the radio would sit silent. RadioLink::init() drives
/// both as outputs, so the firmware no longer depends on external pulldowns --
/// but a 10k pulldown on each is still worth fitting, because the pads are
/// high-impedance inputs from power-on until init() runs.
constexpr gpio_num_t RADIO_M0 = GPIO_NUM_6;
constexpr gpio_num_t RADIO_M1 = GPIO_NUM_7;

// --- E32 module configuration --------------------------------------------
// Written into the module by RadioLink::init() at every boot. ALL THREE bytes
// must be identical to the ground station's (e32-e-stop-gs) or the two radios
// will not hear each other.

/// RF channel. Carrier is 410 + CHAN MHz, so 0x14 = channel 20 = 430 MHz.
///
/// NOT the module's factory default (0x17 = 433 MHz). 0x17 is reserved for the
/// rover's teleoperation LoRa link, which the mast Pi already transmits on
/// (mast/lora_bridge.py, CFG_CHAN = 0x17) at 1 W. Leaving the emergency link
/// on the factory channel puts two 1 W transmitters on one carrier with their
/// receivers side by side, so the safety link is moved one band away instead.
constexpr uint8_t RADIO_CHANNEL = 0x14;

/// SPED: 8N1 parity (00) | UART 115200 (111) | air data rate 2.4k (010).
/// The air rate is the one that sets range; 2.4k is the factory default and a
/// good compromise. It is unrelated to RADIO_BAUD, which is only the wired
/// ESP<->module link.
constexpr uint8_t RADIO_SPED = 0x3A;

/// OPTION: transparent transmission (bit7=0) | push-pull IO (bit6=1) |
/// 250 ms wake-up (000) | FEC on (bit2=1) | max TX power (00).
constexpr uint8_t RADIO_OPTION = 0x44;

// --- Jetson reset --------------------------------------------------------
/// Resets the Jetson. Idles LOW, pulses HIGH --- the polarity verified on the
/// board by main_test_hw_commands.cpp.
///
/// @warning GPIO2 is an ESP32-C3 strapping pin: it must read HIGH during the
///          ROM boot window. The firmware only drives this pad after setup()
///          runs, so it is safe --- but do NOT fit an external pulldown here,
///          which would hold the strap low and stop the ESP booting. If the
///          driver circuit needs a pulldown, move this signal to another GPIO.
constexpr gpio_num_t JETSON_RESET = GPIO_NUM_2;

}  // namespace Pins
