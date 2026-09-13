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

#include "bms.h"

class Bms;

class Io {
    private:
        Bms* bms = nullptr;
        // Inputs
        bool ignitionOn = false;
        bool chargeEnable = false;     // Charger is asking to charge
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
        /* Attach the ignition / charge-enable interrupts. MUST be called only
         * once `bms` is fully constructed: both handlers drive the state
         * machine, and attaching them in Io's constructor meant an edge during
         * startup ran the state machine through a null state pointer. */
        void attach_interrupts();
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
        bool pos_contactor_is_welded();
        bool neg_contactor_is_welded();
};

#endif  // BMS_SRC_INCLUDE_IO_H_
