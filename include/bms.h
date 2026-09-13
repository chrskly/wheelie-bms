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

#ifndef BMS_SRC_INCLUDE_BMS_H_
#define BMS_SRC_INCLUDE_BMS_H_

#include <stdio.h>
#include <string>
#include <time.h>
#include "statemachine.h"
#include "io.h"
#include "led.h"
#include "battery.h"
#include "shunt.h"
#include "util.h"


bool send_limits_message(struct repeating_timer *t);
bool send_bms_state_message(struct repeating_timer *t);
bool send_module_liveness_message(struct repeating_timer *t);
bool send_soc_message(struct repeating_timer *t);
bool send_status_message(struct repeating_timer *t);
bool send_alarm_message(struct repeating_timer *t);
bool handle_main_CAN_messages(struct repeating_timer *t);
bool check_liveness(struct repeating_timer *t);

enum InternalErrorSource {
    IE_LOW_CELL_RANGE  = 1 << 0,   // lowest cell voltage outside the plausible range
    IE_HIGH_CELL_RANGE = 1 << 1,   // highest cell voltage outside the plausible range
    IE_LOW_TEMP_RANGE  = 1 << 2,   // lowest sensor temperature outside the plausible range
    IE_HIGH_TEMP_RANGE = 1 << 3,   // highest sensor temperature outside the plausible range
};

enum InhibitReason {
    R_NONE,
    R_TOO_HOT,
    R_TOO_COLD,
    R_BATTERY_FULL,
    R_BATTERY_EMPTY,
    R_CHARGING,
    R_ILLEGAL_STATE_TRANSITION,
    R_MODULE_UNRESPONSIVE,
    R_SHUNT_UNRESPONSIVE,
    R_CRITICAL_FAULT,
    R_DEAD_CELL,
};

/* Every member below carries a default initialiser. These run for ANY
 * constructor, including the defaulted one used for the global `bms` object,
 * so a member cannot be left holding garbage if init() is not reached or if a
 * new field is added later and someone forgets to assign it. */
class Bms {
    private:
        Battery* battery = nullptr;            //
        State state = nullptr;                 // set by init(); send_event() refuses to run until then
        Io* io = nullptr;                      //
        Shunt* shunt = nullptr;                //
        StatusLight statusLight;               // (has its own default initialisers)
        uint16_t maxChargeCurrent = 0;         // Tell the charger how much current it's allowed to push into the battery
        uint16_t maxDischargeCurrent = 0;      //
        uint8_t soc = 0;                       // State of charge of the battery
        uint8_t internalErrorFlags = 0;        // bitmask of InternalErrorSource
        bool watchdogReboot = false;           //
        uint64_t lastTimePackVoltagesMatched = 0;  // get_clock_ms() when pack voltages last matched
        struct CANMessage canFrame;            //
        uint16_t invalidEventCounter = 0;      // Count how many times the state machine has seen an invalid event
        bool illegalStateTransition = false;   //
        int8_t chargeInhibitReason = R_NONE;   //
        int8_t driveInhibitReason = R_NONE;    //
        bool posContactorWelded = false;       //
        bool negContactorWelded = false;       //
        bool packContactorsWelded[NUM_PACKS] = { false };  //

        uint32_t canTxErrorCount = 0;          // Track number of times we've failed to send a CAN message on the main bus
        uint32_t canRxErrorCount = 0;          // Track number of times we've failed to read a CAN message on the main bus

    public:
        Bms() {};
        /* Two-phase startup, deliberately:
         *   init()  wires up collaborators and brings up the main CAN port;
         *   start() begins the periodic timers.
         * Never construct a temporary Bms and copy-assign it: init() hands
         * `this` to StatusLight, so a temporary leaves a dangling back-pointer. */
        void init(Battery* battery, Io* io, Shunt* shunt);
        void start();

        // State and events
        void set_state(State _state, std::string reason);
        State get_state();
        void send_event(Event event);
        void print();

        // Watchdog
        void set_watchdog_reboot(bool value);

        // DRIVE_INHIBIT
        void enable_drive_inhibit(std::string context, InhibitReason reason);
        void disable_drive_inhibit(std::string context);
        bool drive_is_inhibited();
        void set_drive_inhibit_reason(InhibitReason reason);
        void clear_drive_inhibit_reason();
        int8_t get_drive_inhibit_reason();

        // CHARGE_INHIBIT
        void enable_charge_inhibit(std::string context, InhibitReason reason);
        void disable_charge_inhibit(std::string context);
        bool charge_is_inhibited();
        void set_charge_inhibit_reason(InhibitReason reason);
        void clear_charge_inhibit_reason();
        int8_t get_charge_inhibit_reason();

        // HEATER
        void enable_heater();
        void disable_heater();
        bool heater_is_enabled();

        // IGNITION
        bool ignition_is_on();

        // CHARGE_ENABLE
        bool charge_is_enabled();

        // SOC
        uint8_t get_soc();
        void recalculate_soc();

        /* Error.
         *
         * Tracked as a bitmask of independent sources. It used to be a single
         * bool that every check set and nothing ever cleared -- clear_internal_error()
         * had no callers at all -- so the first out-of-range reading at boot
         * latched it for the lifetime of the program. Each source now sets and
         * clears its own bit. */
        void set_internal_error(InternalErrorSource source);
        void clear_internal_error(InternalErrorSource source);
        bool get_internal_error() { return internalErrorFlags != 0; };

        uint8_t get_error_byte();
        uint8_t get_status_byte();

        bool regen_not_allowed() { return soc > 90; };

        void increment_invalid_event_count();
        uint16_t get_invalid_event_count() { return invalidEventCounter; };
        uint8_t get_welding_byte();
        void do_welding_checks();

        void set_illegal_state_transition() { illegalStateTransition = true; }
        void clear_illegal_state_transition() { illegalStateTransition = false; }
        bool get_illegal_state_transition() { return illegalStateTransition; }

        // Charger
        uint16_t get_max_charge_current_by_soc();
        void update_max_charge_current();
        uint16_t get_max_charge_current();
        void update_max_discharge_current();
        uint16_t get_max_discharge_current();

        // Status light
        void led_blink();

        void pack_voltages_match_heartbeat();
        bool packs_are_imbalanced();

        // CAN
        bool send_frame(CANMessage* frame, bool doChecksum);
        bool read_frame(CANMessage* frame);
        void send_shunt_reset_message();

        void increment_can_tx_error_count() { canTxErrorCount++; }
        void increment_can_rx_error_count() { canRxErrorCount++; }
        uint32_t get_can_tx_error_count() { return canTxErrorCount; }
        uint32_t get_can_rx_error_count() { return canRxErrorCount; }
};

#endif  // BMS_SRC_INCLUDE_BMS_H_
