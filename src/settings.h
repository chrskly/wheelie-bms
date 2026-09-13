/*
 * This file is part of the ev mustang bms project.
 *
 * Copyright (C) 2024 Christian Kelly <chrskly@chrskly.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BMS_SRC_SETTINGS_H_
#define BMS_SRC_SETTINGS_H_

#include <stdint.h>   // QUARTZ_FREQUENCY is a uint32_t

#define VERSION 1.0

/*==============================================================================
 * PIN MAP  --  ESP32-S3
 *
 * VERIFY THESE AGAINST YOUR BOARD BEFORE FLASHING. The numbers below are a
 * valid, conflict-free assignment, but they are NOT derived from a schematic;
 * they were chosen to satisfy the constraints listed below after the original
 * map was found to be unusable. The original was inherited verbatim from the
 * RP2040 build and was broken in several ways at once:
 *
 *   - LED_PIN 25 does not exist on the ESP32-S3 (GPIO22-25 are absent)
 *   - SPI_MOSI 19 and CS_PINS[0] 20 are the native USB D-/D+ pins
 *   - the main CAN RX/TX pins collided with SPI_MISO and MAIN_CAN_CS
 *   - INHIBIT_CONTACTOR_PINS[1] was GPIO3, a strapping pin
 *   - SPI_PORT/UART_ID were Pico SDK identifiers that mean nothing here
 *
 * Constraints applied, all enforced by the static_asserts at the end of this
 * block so that editing these numbers cannot silently reintroduce a clash:
 *
 *   - GPIO22-25 do not exist on this part
 *   - GPIO26-37 are SPI flash and (on octal-PSRAM modules) PSRAM
 *   - GPIO19/20 are native USB
 *   - GPIO43/44 are the UART0 console
 *   - GPIO0/3/45/46 are strapping pins
 *   - GPIO38 and GPIO48 are the devkit's RGB LED (revision dependent), avoided
 *   - no pin is used twice
 *============================================================================*/

// Status LED. A plain GPIO driving an external LED, not the devkit's RGB LED.
#define LED_PIN 47

/* Clock fed to the MCP2515 CAN controllers.
 *
 * This value is used for BOTH (a) generating the clock on CAN_CLK_PIN and
 * (b) computing MCP2515 bit timing, so the two can no longer disagree.
 *
 * HARDWARE CHECK REQUIRED: the previous (pico) firmware generated 8 MHz
 * (clk_sys 80 MHz / 10) while this constant claimed 16 MHz -- the two
 * contradicted each other. 8 MHz is used here because it matches the clock the
 * hardware was actually given. If your MCP2515 boards have their own crystal,
 * set CAN_CLK_PIN to -1 to disable generation and set this to the crystal's
 * frequency instead. */
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 8 MHz
#define CAN_CLK_PIN 21                              // LEDC-generated MCP2515 oscillator
#define CAN_CLK_LEDC_CHANNEL 0                      // LEDC channel used to synthesise the clock

/* Console. printf() goes to UART0, which the ROM bootloader brings up on
 * GPIO43/44; setup() calls Serial.begin() so the rate is set explicitly rather
 * than inherited. The old UART_ID/UART_TX_PIN/UART_RX_PIN defines were Pico
 * SDK config, were referenced nowhere, and named pins used for other things. */
#define CONSOLE_BAUD_RATE 115200

// Number of paralleled packs. The pin arrays below are sized by it.
#define NUM_PACKS_CFG 2

/* Main CAN bus. This is the ESP32's own TWAI controller, not an MCP2515, so
 * there is no chip select -- the old MAIN_CAN_CS was dead and collided with
 * the TX pin that bms.cpp hardcoded. */
#define MAIN_CAN_TX_PIN 39
#define MAIN_CAN_RX_PIN 40

// SPI bus shared by the per-pack MCP2515 CAN controllers
#define SPI_CLK  18
#define SPI_MISO 16
#define SPI_MOSI 17
constexpr int CS_PINS[NUM_PACKS_CFG] = { 15, 8 };   // Chip select, one per pack

/* The pack MCP2515 controllers are driven in POLLED mode (ACAN2515 treats INT
 * pin 255 as "no interrupt"). Previously both packs were constructed with INT
 * pin 0, which (a) gave both controllers the same interrupt pin and (b) is the
 * ESP32-S3 strapping/boot pin. The RX path is polled by the worker task, so no
 * interrupt pin is needed. */
#define PACK_CAN_NO_INTERRUPT_PIN 255

// Inputs
#define IGNITION_ENABLE_PIN        10               // Ignition on input signal
#define CHARGE_ENABLE_PIN           9               // Charge enabled input signal
#define POS_CONTACTOR_FEEDBACK_PIN 11               // Feedback from the HVJB positive contactor for welding detection
#define NEG_CONTACTOR_FEEDBACK_PIN 12               // Feedback from the HVJB negative contactor for welding detection
constexpr int CONTACTOR_FEEDBACK_PINS[NUM_PACKS_CFG] = { 13, 14 };  // Battery box contactor feedback

// Outputs
#define CHARGE_INHIBIT_PIN 4                        // Low-side switch to create CHARGE_INHIBIT signal. a.k.a OUT1
#define HEATER_ENABLE_PIN 5                         // Low-side switch to turn on battery heaters. a.k.a. OUT2
constexpr int INHIBIT_CONTACTOR_PINS[NUM_PACKS_CFG] = { 2, 42 };    // Disallow closing of battery box contactors
#define DRIVE_INHIBIT_PIN 6                         // Low-side switch to disallow driving. a.k.a OUT3

//------------------------------------------------------------------------------
// Compile-time pin map validation
//------------------------------------------------------------------------------

constexpr bool pin_exists_on_esp32s3(int pin) {
    // GPIO22-25 are absent from the package
    return ( pin >= 0 && pin <= 21 ) || ( pin >= 26 && pin <= 48 );
}

constexpr bool pin_is_safe_to_use(int pin) {
    return pin_exists_on_esp32s3(pin)
        && pin != 0 && pin != 3                       // strapping
        && pin != 19 && pin != 20                     // native USB D-/D+
        && !( pin >= 26 && pin <= 37 )                // SPI flash / octal PSRAM
        && !( pin >= 43 && pin <= 46 );               // UART0 console + strapping
}

constexpr int CONFIGURED_PINS[] = {
    LED_PIN, CAN_CLK_PIN,
    MAIN_CAN_TX_PIN, MAIN_CAN_RX_PIN,
    SPI_CLK, SPI_MISO, SPI_MOSI, CS_PINS[0], CS_PINS[1],
    IGNITION_ENABLE_PIN, CHARGE_ENABLE_PIN,
    POS_CONTACTOR_FEEDBACK_PIN, NEG_CONTACTOR_FEEDBACK_PIN,
    CONTACTOR_FEEDBACK_PINS[0], CONTACTOR_FEEDBACK_PINS[1],
    CHARGE_INHIBIT_PIN, HEATER_ENABLE_PIN,
    INHIBIT_CONTACTOR_PINS[0], INHIBIT_CONTACTOR_PINS[1],
    DRIVE_INHIBIT_PIN,
};
constexpr int CONFIGURED_PIN_COUNT = (int)( sizeof(CONFIGURED_PINS) / sizeof(CONFIGURED_PINS[0]) );

constexpr bool every_pin_is_safe(int i = 0) {
    return ( i >= CONFIGURED_PIN_COUNT )
        || ( pin_is_safe_to_use(CONFIGURED_PINS[i]) && every_pin_is_safe(i + 1) );
}

constexpr bool pin_is_unique(int index, int other = 0) {
    return ( other >= CONFIGURED_PIN_COUNT )
        || ( ( other == index || CONFIGURED_PINS[other] != CONFIGURED_PINS[index] )
             && pin_is_unique(index, other + 1) );
}

constexpr bool every_pin_is_unique(int i = 0) {
    return ( i >= CONFIGURED_PIN_COUNT )
        || ( pin_is_unique(i) && every_pin_is_unique(i + 1) );
}

static_assert(NUM_PACKS_CFG == 2,
    "CONFIGURED_PINS lists the per-pack pins explicitly; extend it if NUM_PACKS_CFG changes");
static_assert(every_pin_is_safe(),
    "A configured pin does not exist on the ESP32-S3, or is reserved for flash, PSRAM, "
    "native USB, the UART0 console, or is a strapping pin. See the pin map notes above.");
static_assert(every_pin_is_unique(),
    "Two entries in the pin map are assigned the same GPIO.");

// Pack/module configuration
#define NUM_PACKS  NUM_PACKS_CFG                    // The total number of paralleled packs in this battery
#define CELLS_PER_MODULE 16                         // The number of cells in each module
#define TEMPS_PER_MODULE  4                         // The number of temperature sensors in each module
#define MODULES_PER_PACK  6                         // The number of modules in each pack

// Timeouts
//
// ALL timeouts are in MILLISECONDS and are compared against get_clock_ms().
// They used to be a mix of "seconds" (per the comments) measured against a
// 10ms tick that was then divided by CLOCKS_PER_SEC, which made every one of
// them wrong by a factor of 10,000.
#define MODULE_TTL_MS 5000                          // If we have not seen an update from a module in MODULE_TTL_MS
                                                    // milliseconds, then mark the module as dead.

#define SHUNT_TTL_MS 3000                           // If we have not seen an update from the ISA shunt in
                                                    // SHUNT_TTL_MS milliseconds, then mark it as dead.

#define PACKS_IMBALANCED_TTL_MS 3000                // If the packs are imbalanced for more than
                                                    // PACKS_IMBALANCED_TTL_MS milliseconds, then actually inhibit
                                                    // the contactors. (The old value of 3000 was documented as
                                                    // seconds -- 50 minutes -- which was clearly not intended.)

/* When closing contactors, the voltage difference between the packs shall not
 * be greater than this, in MILLIVOLTS.
 *
 * HARDWARE CHECK REQUIRED. This was 10 mV, which is a delta two independently
 * measured ~384,000 mV packs will essentially never achieve -- packs_are_imbalanced()
 * was therefore permanently true and one pack was permanently inhibited. 1000 mV
 * (1 V, about 10 mV per cell across 96 cells) is an engineering estimate: with a
 * plausible 100-200 mOhm paralleled loop resistance that is roughly 5-10 A of
 * inrush on contactor close. Confirm against your packs' measured internal
 * resistance and lower it if you can hold a tighter match. */
#define SAFE_VOLTAGE_DELTA_BETWEEN_PACKS 1000

#define CELL_DELTA_WARN_THRESHOLD 20                // If the cell delta is greater than this value, then raise a warning.
#define CELL_DELTA_ALARM_THRESHOLD 200              // If the cell delta is greater than this value, then raise an alarm.

/* Stored in place of a temperature when no sensor is fitted in that slot. The
 * module reports a raw count of 0 for an unpopulated slot, which decodes to
 * -40 C and would otherwise look like a real, very cold reading. */
#define NO_TEMPERATURE_READING (-127)

/* Sentinel returned by the cell-voltage getters when no module has reported
 * yet. Deliberately above any real reading so a "lowest cell" search works. */
#define NO_CELL_VOLTAGE_READING 10000

#define DEAD_CELL_VOLTAGE 2500                       // Min cell voltage is 2800mV, so lets consider 2500mV as dead.

// Temperature
#define PACK_TEMP_SAMPLE_INTERVAL_MS 60000          // How often to sample the pack temperature, in milliseconds.
                                                    // temperatureDelta is therefore a per-minute rate, which is
                                                    // what the charge derating logic expects.
/* Hysteresis band, in degrees C, applied to the too-hot and too-cold-to-charge
 * thresholds. Without it the health check flips state every cycle when sitting
 * on a threshold, which oscillated the BMS between charging and batteryHeating. */
#define TEMPERATURE_HYSTERESIS 2

#define WARNING_TEMPERATURE 30                      // 
#define MAXIMUM_TEMPERATURE 50                      // Stop everything if the battery is above this temperature
#define CHARGE_TEMPERATURE_MINIMUM (-10)            // minimum temperature required to allow charging
#define CHARGE_TEMPERATURE_DERATING_MINIMUM 15      // where temperature based derating kicks in
#define CHARGE_TEMPERATURE_DERATING_THRESHOLD 1     // Allow temperature to increase this much per minute. Above that, derate.

// Battery capacity/voltages/etc.
/* Both of these are in the SAME units the ISA shunt reports, so recalculate_soc()
 * can compare them directly: watt-hours against shunt wattHours, and amp-seconds
 * against shunt ampSeconds (the shunt's 0x527 counter is amp-seconds -- raw/3600
 * is amp-hours). Verified against an independent driver for the same device. */
#define BATTERY_CAPACITY_WH 14800                   // Wh. 7.4kWh usable per pack, x2 packs == 14.8kWh
#define BATTERY_CAPACITY_AS 187200                  // As. 26Ah per pack (93,600 As), x2 packs == 187,200 As
#define CALCULATE_SOC_FROM_AMP_SECONDS 1            // Should we calculate SoC from amp seconds (value = 1) or
                                                    // kWh (value = 0)? 
#define CELL_EMPTY_VOLTAGE 2900                     // Official min pack voltage = 269V. 269 / 6 / 16 = 2.8020833333V
#define CELL_FULL_VOLTAGE 4000                      // Official max pack voltage = 398V. 398 / 6 / 16 = 4.1458333333V

/* Charge / discharge current policy.
 *
 * POLICY CHOICE -- REVIEW THESE. get_max_charge_current_by_soc() was a stub that
 * returned 0, and because it was combined with std::min() the charger was
 * always told 0 A. update_max_discharge_current() was hardcoded to 100 A with a
 * FIXME. The values below are deliberately conservative starting points, not
 * manufacturer figures. */
#define CHARGE_CURRENT_MAX_PER_PACK_A 125           // A. Matches the top of chargeCurrentMax[]
#define DISCHARGE_CURRENT_MAX_PER_PACK_A 100        // A. Per pack, when not derated
#define CHARGE_TAPER_START_SOC 90                   // %. Above this, taper charge current linearly to 0 at 100%

/*==============================================================================
 * Cell balancing
 *
 * Mechanism, per the BMWPhevBMS reference implementation:
 *
 *   The module boards bleed cells down towards a target voltage sent in bytes
 *   0-1 of the poll message (little endian), with byte 4 set to 0x48 instead of
 *   0x40 to arm balancing. The target is the LOWEST cell in the pack plus a
 *   small offset, so every cell above it bleeds down to meet it. When not
 *   balancing the target is 0x10C7 (4295 mV), which is above any real cell and
 *   therefore inert.
 *
 *   While a cell is bleeding its measured voltage is depressed, so readings
 *   must be discarded during balancing and for a settling period afterwards.
 *   The module also reports its own balance status, and readings are discarded
 *   whenever that is non-zero.
 *
 * Balancing runs on a duty cycle: balance for CELL_BALANCE_DUTY_MS, then rest
 * for CELL_BALANCE_REST_MS so the cells recover and can be measured honestly
 * before deciding whether another burst is needed.
 *============================================================================*/
#define CELL_BALANCING_ENABLED 1                    // Set to 0 to disable balancing entirely

#define CELL_BALANCE_VOLTAGE 3900                   // mV. Only balance when the highest cell is above this
#define CELL_BALANCE_HYSTERESIS_MV 40               // mV. Only balance when (highest - lowest) exceeds this
#define CELL_BALANCE_TARGET_OFFSET_MV 5             // mV added to the lowest cell to form the bleed target

#define CELL_BALANCE_DUTY_MS 60000                  // How long one balancing burst lasts
#define CELL_BALANCE_REST_MS 60000                  // Rest between bursts, so cells recover before re-measuring
#define CELL_BALANCE_SETTLE_MS 5000                 // After a burst ends, ignore readings for this long

/* Byte 4 of the poll message: 0x48 arms balancing, 0x40 does not. */
#define MODULE_CMD_BALANCE_ON  0x48
#define MODULE_CMD_BALANCE_OFF 0x40

/* Transmit a burst of dummy frames at init to prove each CAN port can send.
 *
 * OFF by default, and it should stay off on a vehicle. The main-bus burst uses
 * IDs 0x100-0x104, which this project does not own and which may belong to
 * another ECU; the pack-bus burst uses ID 0x000, the highest priority
 * identifier on the bus. Useful on the bench, not on a car. */
#define CAN_SELF_TEST_AT_INIT 0

/* Give up heating after this long. A heater that cannot bring the pack up to
 * the minimum charge temperature within this window is not working, and sitting
 * in batteryHeating indefinitely means running it with no end condition. */
#define BATTERY_HEATING_TIMEOUT_MS 1800000          // 30 minutes

/* Hardware watchdog. The worker task must check in at least this often or the
 * chip resets. Set to 0 to disable. */
#define WATCHDOG_TIMEOUT_S 5

/* Interval between status prints on the console, in milliseconds. Must be a
 * multiple of BMS_WORKER_TICK_MS. Set to 0 to disable. Useful during hardware
 * bring-up; turn it off once the bus is trusted. */
#define STATUS_PRINT_INTERVAL_MS 10000

/* Include every cell voltage in the status print. This is ~200 numbers, close
 * to a kilobyte, which at 115200 baud blocks the worker task for most of a
 * tenth of a second -- long enough to stop draining CAN and overflow the
 * driver's receive buffer. Off by default; turn it on deliberately and accept
 * that module data will be dropped while it prints. */
#define STATUS_PRINT_CELL_DETAIL 0

/* The single task that runs all periodic BMS work. Priority sits above the
 * idle task and below the ACAN2515 driver task (16), which must stay
 * responsive to service the MCP2515s. */
#define BMS_WORKER_TICK_MS 5
#define BMS_WORKER_STACK_BYTES 8192
#define BMS_WORKER_PRIORITY 3

/* Consecutive agreeing samples required before an input change is accepted.
 * Inputs are polled every IO_POLL_INTERVAL_MS, so this is the debounce time. */
#define IO_DEBOUNCE_SAMPLES 3
#define IO_POLL_INTERVAL_MS 10

/* How long a contactor must have been commanded open before its feedback is
 * believed for weld detection. Contactors take time to physically open. */
#define WELD_CHECK_SETTLE_MS 500

// Communication
#define CAN_MUTEX_TIMEOUT_MS 200                    // Timeout for the CAN mutex
#define SEND_FRAME_RETRIES 6                        // Number of times to retry sending a frame before giving up
#define READ_FRAME_RETRIES 3                        // Number of times to retry reading a frame before giving up
#define READ_FRAMES_PER_CYCLE 16                    // Max frames drained from a pack per service tick. A poll of a
                                                    // 6-module pack bursts more frames than the driver's 32-frame
                                                    // receive buffer holds, so one-per-tick loses module replies.

#endif  // BMS_SRC_SETTINGS_H_
