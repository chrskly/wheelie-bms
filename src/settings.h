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

/* Firmware version. This was `#define VERSION 1.0` -- a floating-point literal,
 * referenced nowhere, so the firmware could not tell you what it was running
 * over CAN, over the web page, or on the console. Split into integers so it can
 * be encoded, and into a string so it can be printed. */
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_STRING "1.0"
/* Byte 2-3 of 0x35F, per the CAN-bus BMS convention: major in the high byte. */
#define VERSION_U16 ( ( VERSION_MAJOR << 8 ) | VERSION_MINOR )
/* Byte 0-1 of 0x35F. Arbitrary but stable, so a receiver can tell one BMS
 * design from another on a shared bus. */
#define BATTERY_MODEL_ID 0x0001

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

/* Number of paralleled packs. The pin arrays below are sized by it.
 *
 * Overridable from the build. The firmware has explicit, safety-relevant
 * single-pack handling -- most importantly the dead-cell response, which has
 * nothing to isolate and so must inhibit outright instead -- and that code
 * could not previously be compiled at all, let alone exercised. The native test
 * suite builds a second binary with -DNUM_PACKS_CFG=1 to cover it. */
#ifndef NUM_PACKS_CFG
#define NUM_PACKS_CFG 2
#endif

/* Main CAN bus. This is the ESP32's own TWAI controller, not an MCP2515, so
 * there is no chip select -- the old MAIN_CAN_CS was dead and collided with
 * the TX pin that bms.cpp hardcoded. */
#define MAIN_CAN_TX_PIN 39
#define MAIN_CAN_RX_PIN 40

// SPI bus shared by the per-pack MCP2515 CAN controllers
#define SPI_CLK  18
#define SPI_MISO 16
#define SPI_MOSI 17
#if NUM_PACKS_CFG == 2
constexpr int CS_PINS[NUM_PACKS_CFG] = { 15, 8 };   // Chip select, one per pack
#else
constexpr int CS_PINS[NUM_PACKS_CFG] = { 15 };
#endif

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
#if NUM_PACKS_CFG == 2
constexpr int CONTACTOR_FEEDBACK_PINS[NUM_PACKS_CFG] = { 13, 14 };  // Battery box contactor feedback
#else
constexpr int CONTACTOR_FEEDBACK_PINS[NUM_PACKS_CFG] = { 13 };
#endif

// Outputs
#define CHARGE_INHIBIT_PIN 4                        // Low-side switch to create CHARGE_INHIBIT signal. a.k.a OUT1
#define HEATER_ENABLE_PIN 5                         // Low-side switch to turn on battery heaters. a.k.a. OUT2
#if NUM_PACKS_CFG == 2
constexpr int INHIBIT_CONTACTOR_PINS[NUM_PACKS_CFG] = { 2, 42 };    // Disallow closing of battery box contactors
#else
constexpr int INHIBIT_CONTACTOR_PINS[NUM_PACKS_CFG] = { 2 };
#endif
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
    SPI_CLK, SPI_MISO, SPI_MOSI, CS_PINS[0],
    IGNITION_ENABLE_PIN, CHARGE_ENABLE_PIN,
    POS_CONTACTOR_FEEDBACK_PIN, NEG_CONTACTOR_FEEDBACK_PIN,
    CONTACTOR_FEEDBACK_PINS[0],
    CHARGE_INHIBIT_PIN, HEATER_ENABLE_PIN,
    INHIBIT_CONTACTOR_PINS[0],
    DRIVE_INHIBIT_PIN,
#if NUM_PACKS_CFG == 2
    /* Second pack. Listed explicitly so the uniqueness and safety checks below
     * cover every per-pack pin, not just pack 0's. */
    CS_PINS[1], CONTACTOR_FEEDBACK_PINS[1], INHIBIT_CONTACTOR_PINS[1],
#endif
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

static_assert(NUM_PACKS_CFG == 1 || NUM_PACKS_CFG == 2,
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

/* Below this the shunt reading is too small to say anything about direction,
 * so the plausibility check below stays quiet. */
#define SHUNT_SIGNIFICANT_CURRENT_MA 2000
/* How long a contradiction must persist before it is reported. Long enough to
 * ride out the moment a charger starts or stops. */
#define SHUNT_IMPLAUSIBLE_MS 10000

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
/* Coldest value a module sensor can actually encode. Raw counts are offset by
 * 40, and a raw 0 is the "no sensor fitted" marker rather than a reading, so the
 * lowest real count is 1 and the floor is -39, not -40. */
#define MODULE_SENSOR_MINIMUM_C (-39)
#define MODULE_SENSOR_MAXIMUM_C (127)               // decode clamps here

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
/* Scaled by NUM_PACKS rather than written out for two. These were hardcoded to
 * the two-pack totals, so a single-pack build divided the real capacity into a
 * figure twice its size and reported half the true state of charge. */
/* BOTH capacities must describe the SAME cell voltage window as CELL_EMPTY_VOLTAGE
 * and CELL_FULL_VOLTAGE below, because recalculate_soc() counts a full 0-100%
 * against whichever one CALCULATE_SOC_FROM_AMP_SECONDS selects. These are the
 * gross figures, matching the full 2.8-4.2 V window: BMW's own numbers for HV
 * battery generation 3.0 are 9.1 kWh storable of which 7.3 kWh usable, and the
 * 7.3 kWh that used to be here was the usable one -- the narrower window BMW
 * keeps the cells in, not the one this BMS enforces. Paired with a 26 Ah
 * amp-second figure it made every SoC reading disagree with itself.
 *
 * 96 cells * 3.66 V * 26 Ah = 9135 Wh, which is where BMW's 9.1 kWh comes from. */
#define BATTERY_CAPACITY_WH_PER_PACK 9100           // Wh gross per pack, over the full cell window
#define BATTERY_CAPACITY_AS_PER_PACK 93600          // As per pack (26 Ah)
#define BATTERY_CAPACITY_WH ( BATTERY_CAPACITY_WH_PER_PACK * NUM_PACKS )
#define BATTERY_CAPACITY_AS ( BATTERY_CAPACITY_AS_PER_PACK * NUM_PACKS )
#define CALCULATE_SOC_FROM_AMP_SECONDS 1            // Should we calculate SoC from amp seconds (value = 1) or
                                                    // kWh (value = 0)?


/* Cell voltage window, in millivolts. These are hard limits, not targets:
 * has_empty_cell() / has_full_cell() inhibit drive and charge at them, and the
 * cell-range plausibility check in Battery treats anything outside as a bad
 * reading.
 *
 * The OEM window for this pack, per the BMW technical training document for the
 * G12 LCI PHEV high-voltage battery (generation 3.0, the 26 Ah cell): min 269 V,
 * max 403 V across 96 cells in series, so 2.802 V and 4.198 V per cell.
 *
 * The previous 2.9 / 4.0 V left roughly 5 Ah of the cell's 26 Ah outside the
 * window while BATTERY_CAPACITY_AS_PER_PACK still claimed all 26, so SoC read
 * high everywhere and bottomed out near 20% instead of 0.
 *
 * Nothing else stands between these and the cells, so the margin has to come
 * from the layers above: CHARGE_TAPER_START_SOC winds the charger down before
 * the top, and CHARGE_ACCEPTANCE_PERCENT_OF_C in pack.cpp limits what goes in
 * cold. Samsung SDI do not publish a datasheet for this cell; 4.2 V is the
 * OEM's own ceiling, not a headroom figure. */
#define CELL_EMPTY_VOLTAGE 2800                     // Official min pack voltage = 269V. 269 / 96 = 2.802V
#define CELL_FULL_VOLTAGE 4200                      // Official max pack voltage = 403V. 403 / 96 = 4.198V

/* Nothing enforced the "SAME window" requirement stated with the capacities above, and it is exactly the
 * mistake that was already made here once: a 7.3 kWh usable figure paired with
 * a 26 Ah gross one, which made the two SoC paths disagree with each other and
 * bottom out around 20% instead of 0. Flipping CALCULATE_SOC_FROM_AMP_SECONDS
 * would have silently changed every reading.
 *
 * Dividing one capacity by the other gives the pack voltage they jointly imply,
 * and that is checkable: for a lithium cell the average voltage over a full
 * discharge sits near the middle of its window, never out at either end. A
 * figure down against CELL_EMPTY_VOLTAGE means the two capacities were measured
 * over different windows. The old pairing implied 2965 mV/cell against a
 * 2900-4000 window -- essentially at the floor. The current one implies about
 * 3646 mV against 2800-4200, comfortably mid-window.
 *
 * Wide on purpose: this catches the two capacities describing different
 * windows, not a cell whose discharge curve is unusually shaped. */
#define IMPLIED_NOMINAL_CELL_MV                                                  \
    ( (long long)BATTERY_CAPACITY_WH_PER_PACK * 3600LL * 1000LL                  \
      / ( (long long)BATTERY_CAPACITY_AS_PER_PACK                                \
          * CELLS_PER_MODULE * MODULES_PER_PACK ) )
#define CELL_WINDOW_QUARTER_MV ( ( CELL_FULL_VOLTAGE - CELL_EMPTY_VOLTAGE ) / 4 )

static_assert(IMPLIED_NOMINAL_CELL_MV >= CELL_EMPTY_VOLTAGE + CELL_WINDOW_QUARTER_MV
           && IMPLIED_NOMINAL_CELL_MV <= CELL_FULL_VOLTAGE - CELL_WINDOW_QUARTER_MV,
    "BATTERY_CAPACITY_WH_PER_PACK and BATTERY_CAPACITY_AS_PER_PACK imply an average "
    "cell voltage outside the middle half of the CELL_EMPTY_VOLTAGE..CELL_FULL_VOLTAGE "
    "window, which means they were measured over different windows. Both must be the "
    "gross figures for the window this BMS enforces.");

/* Charge / discharge current policy.
 *
 * POLICY CHOICE -- REVIEW THESE. get_max_charge_current_by_soc() was a stub that
 * returned 0, and because it was combined with std::min() the charger was
 * always told 0 A. update_max_discharge_current() was hardcoded to 100 A with a
 * FIXME. The values below are engineering estimates, not manufacturer figures.
 *
 * WHY THE CHARGE SIDE IS A C-RATE AND NOT AN AMPERE FIGURE.
 *
 * A bare ampere number has no stated relationship to the cells, and this one had
 * drifted a long way from them. The limit and the lookup table both plateaued at
 * 125 A, which on a 26 Ah pack is 4.8C: about twelve times what the donor
 * vehicle's own 3.7 kW charger ever pushes into this pack (~0.4C), far above the
 * 0.5-1C that NMC/graphite normally accepts, and -- the giveaway -- HIGHER than
 * the discharge limit below, which is backwards for essentially every lithium
 * cell. Deriving it from BATTERY_CAPACITY_AS_PER_PACK means the two cannot
 * silently drift apart again, and expressing the policy in C makes the number
 * reviewable against a datasheet instead of against nothing.
 *
 * 2.20C is the setting, and it is the REGEN figure, deliberately chosen -- read
 * the next paragraph before changing anything on the back of it.
 *
 * REGEN. maxChargeCurrent is published in the 0x351 frame, which is the only
 * charge-side number the BMS sends: update_max_charge_current() computes one
 * number and both the charger and (if your inverter honours it) regen are held
 * to it. There is no field that can tell the two apart.
 *
 * So this is the deliberate choice that paragraph used to warn about: a
 * short-pulse rating now governs a sustained limit. 2.2C is where the donor
 * vehicle's 20 kW recuperation figure lands on a 26 Ah pack at 355 V (~57 A),
 * so it is a rate BMW themselves push into these cells -- but in bursts, off
 * the brake pedal, not for the length of a charge session. What BMW sustain is
 * 3.7 kW, about 0.4C.
 *
 * The cells cannot tell you which is which either: Samsung SDI publish no
 * datasheet for the 26 Ah PHEV cell, and the closest published sibling (the
 * 94 Ah NCM) gives 0.77C standard charge and no pulse rating at all.
 *
 * What keeps this honest is that nothing downstream sustains 2.2C anyway. A
 * wall charger that large is not what this pack will be plugged into, the SoC
 * taper above CHARGE_TAPER_START_SOC pulls the ceiling down through the top of
 * the charge, and the acceptance curve in pack.cpp only reaches 2.2C above
 * 25C -- cold cells are still held to the plating limits, which did not move.
 * If you ever do put a >1C charger on it, drop this back to 100 and accept the
 * lower regen, because at that point the two uses really do conflict. */
#define PACK_CAPACITY_AH ( BATTERY_CAPACITY_AS_PER_PACK / 3600 )   // 26 Ah

#define CHARGE_C_RATE_PERCENT 220                   // Sustained charge rate at the plateau, in % of 1C
#define CHARGE_CURRENT_MAX_PER_PACK_A ( ( PACK_CAPACITY_AH * CHARGE_C_RATE_PERCENT + 50 ) / 100 )
#define DISCHARGE_CURRENT_MAX_PER_PACK_A 100        // A. Per pack, when not derated

static_assert(PACK_CAPACITY_AH > 0,
    "BATTERY_CAPACITY_AS_PER_PACK must be at least one amp-hour");
static_assert(CHARGE_CURRENT_MAX_PER_PACK_A > 0,
    "CHARGE_C_RATE_PERCENT rounds the charge limit down to zero amps");
/* Charge ratings are below discharge ratings on essentially every lithium cell,
 * so the inverse is far more likely to be a mistake than a decision. If a
 * datasheet really does say otherwise, relax this deliberately. */
static_assert(CHARGE_CURRENT_MAX_PER_PACK_A <= DISCHARGE_CURRENT_MAX_PER_PACK_A,
    "charge limit exceeds the discharge limit; check CHARGE_C_RATE_PERCENT against the cell datasheet");
#define CHARGE_TAPER_START_SOC 90                   // %. Above this, taper charge current linearly to 0 at 100%
/* Above this state of charge the BMS tells the inverter not to regenerate.
 * This was written out as a bare `soc > 90` in bms.h, the only policy threshold
 * in the project that did not live here -- so it could not be changed from the
 * settings file and would not follow CHARGE_TAPER_START_SOC if that moved. */
#define REGEN_BLOCK_SOC 90

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

/*==============================================================================
 * Web interface
 *
 * A small read-only HTTP server, aimed at a phone held next to the car. It
 * serves three static files from LittleFS (upload them with
 * `pio run -t uploadfs`) plus /api/status, which the page polls for JSON.
 *
 * READ ONLY, deliberately. There are no endpoints that change anything: no
 * contactor control, no threshold editing, no firmware upload. A BMS is a
 * safety device and its only write interface should be the one on the bench.
 *
 * The server runs BELOW the BMS worker in priority and never touches a Battery,
 * Bms, Shunt or Io -- it reads a snapshot the worker publishes. See webstatus.h.
 *============================================================================*/
#define WEB_INTERFACE_ENABLED 1

#define WEB_SERVER_PORT 80

/* Advertised over mDNS as http://<name>.local, so the phone does not have to be
 * told an IP address. Set to "" to skip advertising. */
#define WEB_MDNS_HOSTNAME "wheelie-bms"

/* 0 = run our own access point and let the phone join it (works anywhere,
 * including a car park with no WiFi). 1 = join an existing network instead. */
#define WEB_WIFI_STATION_MODE 0

/* Access point credentials, used when WEB_WIFI_STATION_MODE is 0.
 * The password must be at least 8 characters -- softAP() rejects anything
 * shorter and the AP then never appears, with nothing on the console to say
 * why. Set it to "" for a deliberately open network. */
#define WEB_WIFI_AP_SSID     "wheelie-bms"
#define WEB_WIFI_AP_PASSWORD "wheeliebms"

// Existing network to join, used when WEB_WIFI_STATION_MODE is 1.
#define WEB_WIFI_STA_SSID     ""
#define WEB_WIFI_STA_PASSWORD ""
#define WEB_WIFI_STA_TIMEOUT_MS 15000               // Give up joining after this long and carry on without WiFi

/* How often the worker republishes the snapshot the browser reads. Must be a
 * multiple of BMS_WORKER_TICK_MS. The page polls at its own rate; anything
 * faster than this just re-reads the same numbers. */
#define WEB_SNAPSHOT_INTERVAL_MS 500
#define WEB_SNAPSHOT_TICKS ( WEB_SNAPSHOT_INTERVAL_MS / BMS_WORKER_TICK_MS )
static_assert(WEB_SNAPSHOT_TICKS * BMS_WORKER_TICK_MS == WEB_SNAPSHOT_INTERVAL_MS,
    "WEB_SNAPSHOT_INTERVAL_MS must be a whole number of BMS_WORKER_TICK_MS ticks");
static_assert(WEB_SNAPSHOT_TICKS >= 1,
    "WEB_SNAPSHOT_INTERVAL_MS must be at least one worker tick");

/* Priority 1 sits below BMS_WORKER_PRIORITY, so serving a page can never delay
 * a CAN poll or a contactor decision, and below the ACAN2515 driver task. It is
 * above the idle task, which is why the accept loop must yield -- see the
 * comment on the delay in webserver.cpp. */
#define WEB_TASK_STACK_BYTES 8192
#define WEB_TASK_PRIORITY 1
#define WEB_ACCEPT_POLL_MS 10                       // How long to sleep between checks for a new client
#define WEB_CLIENT_TIMEOUT_MS 3000                  // Drop a client that stops sending mid-request
#define WEB_JSON_BUFFER_BYTES 6144                  // /api/status is ~3 KB with two 6-module packs

/* Consecutive agreeing samples required before an input change is accepted.
 * Inputs are polled every IO_POLL_INTERVAL_MS, so this is the debounce time. */
#define IO_DEBOUNCE_SAMPLES 3
#define IO_POLL_INTERVAL_MS 10
#define IO_POLL_TICKS ( IO_POLL_INTERVAL_MS / BMS_WORKER_TICK_MS )
static_assert(IO_POLL_TICKS * BMS_WORKER_TICK_MS == IO_POLL_INTERVAL_MS,
    "IO_POLL_INTERVAL_MS must be a whole number of BMS_WORKER_TICK_MS ticks");
static_assert(IO_POLL_TICKS >= 1,
    "IO_POLL_INTERVAL_MS cannot be shorter than one worker tick");

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
