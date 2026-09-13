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

#define VERSION 1.0

#define LED_PIN 25

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
#define CAN_CLK_LEDC_CHANNEL 0                      // LEDC channel used to synthesise the CAN clock

// Serial port
#define UART_ID      uart0
#define BAUD_RATE   115200
#define UART_TX_PIN      0                          // pin 1
#define UART_RX_PIN      1                          // pin 2

// Number of paralleled packs. Declared here because the pin arrays below are
// sized by it.
#define NUM_PACKS_CFG 2

// CAN bus
#define SPI_PORT      spi0
#define SPI_MISO        16                          // pin 21
#define SPI_CLK         18                          // pin 24
#define SPI_MOSI        19                          // pin 25
#define CAN_CLK_PIN     21                          // pin 27
#define MAIN_CAN_CS     17                          // pin 22
const int CS_PINS[NUM_PACKS_CFG] = { 20, 15 };      // Chip select pins for the CAN controllers for each battery pack.

/* The pack MCP2515 controllers are driven in POLLED mode (ACAN2515 treats INT
 * pin 255 as "no interrupt"). Previously both packs were constructed with INT
 * pin 0, which (a) gave both controllers the same interrupt pin and (b) is the
 * ESP32-S3 strapping/boot pin, also claimed by UART_TX_PIN. The RX path is
 * already timer-polled, so no interrupt pin is needed. */
#define PACK_CAN_NO_INTERRUPT_PIN 255

// Inputs
#define IGNITION_ENABLE_PIN        10               // Ignition on input signal
#define CHARGE_ENABLE_PIN           9               // Charge enabled input signal
#define POS_CONTACTOR_FEEDBACK_PIN 11               // Feedback from the HVJB positive contactor for welding detection
#define NEG_CONTACTOR_FEEDBACK_PIN 12               // Feedback from the HVJB negative contactor for welding detection
const int CONTACTOR_FEEDBACK_PINS[NUM_PACKS_CFG] = { 13, 14 };  // Feedback from the battery box contactors for welding detection

// Outputs
#define CHARGE_INHIBIT_PIN 4                        // Low-side switch to create CHARGE_INHIBIT signal. a.k.a OUT1
#define HEATER_ENABLE_PIN 5                         // Low-side switch to turn on battery heaters. a.k.a. OUT2
const int INHIBIT_CONTACTOR_PINS[NUM_PACKS_CFG] = { 2, 3 };     // Low-side switch to disallow closing of battery box contactors
#define DRIVE_INHIBIT_PIN 6                         // Low-side switch to disallow driving. a.k.a OUT3
#define OUT_4_PIN 7                                 // unused

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

#define DEAD_CELL_VOLTAGE 2500                       // Min cell voltage is 2800mV, so lets consider 2500mV as dead.

// Temperature
#define PACK_TEMP_SAMPLE_INTERVAL_MS 60000          // How often to sample the pack temperature, in milliseconds.
                                                    // temperatureDelta is therefore a per-minute rate, which is
                                                    // what the charge derating logic expects.
#define WARNING_TEMPERATURE 30                      // 
#define MAXIMUM_TEMPERATURE 50                      // Stop everything if the battery is above this temperature
#define CHARGE_TEMPERATURE_MINIMUM -10              // minimum temperature required to allow charging
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

// Cell balancing
#define CELL_BALANCE_VOLTAGE 3900                   // Cell balancing should only happen above this voltage
#define CELL_BALANCE_INTERVAL 60000                 // Interval between cell balancing sessions in milliseconds

// Communication
#define CAN_MUTEX_TIMEOUT_MS 200                    // Timeout for the CAN mutex
#define SEND_FRAME_RETRIES 6                        // Number of times to retry sending a frame before giving up
#define READ_FRAME_RETRIES 3                        // Number of times to retry reading a frame before giving up
#define READ_FRAMES_PER_CYCLE 16                    // Max frames drained from a pack per service tick. A poll of a
                                                    // 6-module pack bursts more frames than the driver's 32-frame
                                                    // receive buffer holds, so one-per-tick loses module replies.

#endif  // BMS_SRC_SETTINGS_H_
