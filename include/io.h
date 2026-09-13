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

#ifndef BMS_SRC_INCLUDE_IO_H_
#define BMS_SRC_INCLUDE_IO_H_

#include <stdint.h>
#include <string>     // std::string parameters below

class Bms;

class Io {
    private:
        Bms* bms = nullptr;
        // Inputs
        // Debounced input states. These are what ignition_is_on() etc. report.
        bool ignitionOn = false;
        bool chargeEnable = false;     // Charger is asking to charge
        uint8_t ignitionSettleCount = 0;
        uint8_t chargeEnableSettleCount = 0;
        int lastInterrupt = 0;
        /* Outputs. Tracked here rather than read back with digitalRead(): a pin
         * configured with plain OUTPUT has its input buffer disabled on ESP32,
         * so digitalRead() returns 0 no matter what was written. That made
         * drive_is_inhibited() permanently false, which in turn meant
         * disable_drive_inhibit() never actually cleared DRIVE_INHIBIT.
         * Defaults are the safe state: everything inhibited until told otherwise. */
        bool driveInhibited = true;
        bool chargeInhibited = true;
        bool heaterEnabled = false;
    public:
        Io() {};
        /* Configure pins. Safe to call before the rest of the system exists. */
        void init();
        /* Sample the ignition and charge-enable inputs, debounce them, and
         * dispatch state machine events on a confirmed change.
         *
         * These used to be pin-change interrupts whose handlers built a
         * std::string, called printf and ran the entire state machine (GPIO
         * writes and CAN sends) in interrupt context, with no debouncing.
         * Called from the BMS worker task instead. */
        void poll_inputs();
        void enable_drive_inhibit(std::string context);
        void disable_drive_inhibit(std::string context);
        bool drive_is_inhibited();
        void enable_charge_inhibit(std::string context);
        void disable_charge_inhibit(std::string context);
        bool charge_is_inhibited();
        void enable_heater();
        void disable_heater();
        bool heater_is_enabled();

        bool ignition_is_on();
        bool charge_enable_is_on();
        /* Raw feedback level. "Closed" is not the same as "welded" -- see
         * Bms::do_welding_checks(), which only believes these once the
         * contactors are supposed to be open and have had time to open. */
        bool pos_contactor_feedback_closed();
        bool neg_contactor_feedback_closed();
};

#endif  // BMS_SRC_INCLUDE_IO_H_
