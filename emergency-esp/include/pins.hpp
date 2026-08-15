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
 *   GPIO 2   JETSON_RESET    --> opto LED, ACTIVE LOW  [needs 10k pullup]
 *   GPIO 3   ESTOP_BUTTON    <-- button (active high)
 *   GPIO 4   JETSON_CAN_CUT  --> transistor gate  [needs 10k pulldown]
 *   GPIO 5   POWER_CUT       --> transistor gate  [needs 10k pulldown]
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

/// RF channel. Carrier is 410 + CHAN MHz, so 0x17 = 433 MHz (factory default).
constexpr uint8_t RADIO_CHANNEL = 0x17;

/// SPED: 8N1 parity (00) | UART 115200 (111) | air data rate 2.4k (010).
/// The air rate is the one that sets range; 2.4k is the factory default and a
/// good compromise. It is unrelated to RADIO_BAUD, which is only the wired
/// ESP<->module link.
constexpr uint8_t RADIO_SPED = 0x3A;

/// OPTION: transparent transmission (bit7=0) | push-pull IO (bit6=1) |
/// 250 ms wake-up (000) | FEC on (bit2=1) | max TX power (00).
constexpr uint8_t RADIO_OPTION = 0x44;

// --- Jetson reset --------------------------------------------------------
/// Resets the Jetson. Wired to SYS_RESET* on the reComputer's REC switch
/// header (pin 8, with pin 7 as its GND).
///
/// ACTIVE LOW, unlike the two cut lines. Two facts force that:
///
///  1. SYS_RESET* is a 1.8V open-drain input, asserted by pulling it to GND.
///     3V3 from an ESP pin can damage it, so the ESP must never drive it
///     directly --- there has to be a stage in between.
///  2. GPIO2 is an ESP32-C3 strapping pin and must read HIGH at reset. A 10k
///     pulldown, which a directly-driven MOSFET gate would need, holds it low
///     and stops the ESP booting.
///
/// Both are satisfied by an optocoupler the ESP *sinks*, which inverts the
/// sense and isolates the two voltage domains:
///
///     3V3 --[330R]--|>|-- GPIO2      (opto LED, cathode to the ESP)
///     GPIO2 --[10k]-- 3V3            (holds the strap high at reset)
///     opto collector -> header pin 8 (SYS_RESET*)
///     opto emitter   -> header pin 11/7 (header GND)
///
/// GPIO2 HIGH  = LED dark   = idle, Jetson running.
/// GPIO2 LOW   = ESP sinks the LED = SYS_RESET* pulled down = Jetson resets.
///
/// @warning A fault that holds GPIO2 low across an ESP reset puts the ESP into
///          serial-download mode instead of running the firmware. That is the
///          cost of using the last strapping pin; it is recoverable by
///          clearing the fault and power-cycling.
constexpr gpio_num_t JETSON_RESET = GPIO_NUM_2;

}  // namespace Pins
