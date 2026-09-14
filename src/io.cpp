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

#include <stdio.h>

#include "statemachine.h"
#include "settings.h"
#include "io.h"
#include "bms.h"

extern Bms bms;

void Io::init() {
    // IGNITION input
    pinMode(IGNITION_ENABLE_PIN, INPUT);

    // CHARGE_ENABLE input
    pinMode(CHARGE_ENABLE_PIN, INPUT);

    /* Seed the debounced states from the pins so the first poll does not report
     * a spurious change. */
    ignitionOn = ( digitalRead(IGNITION_ENABLE_PIN) == HIGH );
    chargeEnable = ( digitalRead(CHARGE_ENABLE_PIN) == HIGH );
    ignitionSettleCount = 0;
    chargeEnableSettleCount = 0;
    /* Re-arm the one-shot boot dispatch. init() runs once in production, but
     * leaving this latched means any re-initialisation silently skips
     * reporting the initial input levels. */
    initialStateDispatched = false;

    // POS_CONTACTOR_FEEDBACK input
    pinMode(POS_CONTACTOR_FEEDBACK_PIN, INPUT);

    // NEG_CONTACTOR_FEEDBACK input
    pinMode(NEG_CONTACTOR_FEEDBACK_PIN, INPUT);

    /* Outputs: set direction BEFORE driving them. Writing to a pin that is
     * still an input only toggles its pull-up on ESP32, so the old ordering
     * (write, then pinMode) silently lost the initial state.
     *
     * All three start in the SAFE state: drive and charge inhibited, heater
     * off. Previously drive inhibit was released and charge inhibit was never
     * driven at all, so charging was permitted from power-on before a single
     * cell voltage had been read. */
    pinMode(DRIVE_INHIBIT_PIN, OUTPUT);
    driveInhibited = false;                        // force the write below
    enable_drive_inhibit("initialization, safe default");

    pinMode(CHARGE_INHIBIT_PIN, OUTPUT);
    chargeInhibited = false;                       // force the write below
    enable_charge_inhibit("initialization, safe default");

    pinMode(HEATER_ENABLE_PIN, OUTPUT);
    heaterEnabled = true;                          // force the write below
    disable_heater();
}

/*
 * Debounce one input. Returns true if the confirmed state changed.
 */
static bool debounce_input(int pin, bool& state, uint8_t& settleCount) {
    const bool level = ( digitalRead(pin) == HIGH );
    if ( level == state ) {
        settleCount = 0;
        return false;
    }
    settleCount++;
    if ( settleCount < IO_DEBOUNCE_SAMPLES ) {
        return false;
    }
    state = level;
    settleCount = 0;
    return true;
}

void Io::poll_inputs() {
    /* Booting with the ignition already on, or the charger already connected,
     * used to leave the state machine sitting in standby for as long as the
     * signal stayed put: nothing changed, so nothing was dispatched. The
     * vehicle drove with the BMS reporting standby, and a charger connected at
     * power-on never armed drive-away protection.
     *
     * Charge is dispatched first deliberately. Taking ignition first would move
     * us into the drive state, where E_CHARGING_INITIATED with the (still
     * inhibited) startup contactors trips the illegal-transition fault. */
    if ( !initialStateDispatched ) {
        initialStateDispatched = true;
        printf("[io] initial input state : ignition %s, charge %s\n",
               ignitionOn ? "on" : "off", chargeEnable ? "on" : "off");
        if ( chargeEnable ) {
            bms.send_event(E_CHARGING_INITIATED);
        }
        if ( ignitionOn ) {
            bms.send_event(E_IGNITION_ON);
        }
        return;
    }

    if ( debounce_input(IGNITION_ENABLE_PIN, ignitionOn, ignitionSettleCount) ) {
        printf("[io] Ignition signal changed to : %s\n", ignitionOn ? "on" : "off");
        bms.send_event( ignitionOn ? E_IGNITION_ON : E_IGNITION_OFF );
    }

    if ( debounce_input(CHARGE_ENABLE_PIN, chargeEnable, chargeEnableSettleCount) ) {
        printf("[io] Charge signal changed to : %s\n", chargeEnable ? "on" : "off");
        bms.send_event( chargeEnable ? E_CHARGING_INITIATED : E_CHARGING_TERMINATED );
    }
}

// REMINDER : THESE OUTPUTS ARE A LOW SIDE SWITCHES.
//     gpio high == on  == output low
//     gpio low  == off == output high/floating?

// DRIVE_INHIBIT output

void Io::enable_drive_inhibit(const char* context) {
    if ( !driveInhibited ) {
        printf("[io] Enabling drive inhibit : %s\n", context);
    }
    driveInhibited = true;
    digitalWrite(DRIVE_INHIBIT_PIN, HIGH);
}

void Io::disable_drive_inhibit(const char* context) {
    if ( driveInhibited ) {
        printf("[io] Disabling drive inhibit : %s\n", context);
    }
    driveInhibited = false;
    digitalWrite(DRIVE_INHIBIT_PIN, LOW);
}

bool Io::drive_is_inhibited() {
    return driveInhibited;
}

// CHARGE_INHIBIT output

void Io::enable_charge_inhibit(const char* context) {
    if ( !chargeInhibited ) {
        printf("[io] Enabling charge inhibit : %s\n", context);
    }
    chargeInhibited = true;
    digitalWrite(CHARGE_INHIBIT_PIN, HIGH);
}

void Io::disable_charge_inhibit(const char* context) {
    if ( chargeInhibited ) {
        printf("[io] Disabling charge inhibit : %s\n", context);
    }
    chargeInhibited = false;
    digitalWrite(CHARGE_INHIBIT_PIN, LOW);
}

bool Io::charge_is_inhibited() {
    return chargeInhibited;
}

// HEATER output

void Io::enable_heater() {
    if ( !heaterEnabled ) {
        printf("[io] Enabling heater\n");
    }
    heaterEnabled = true;
    digitalWrite(HEATER_ENABLE_PIN, HIGH);
}

void Io::disable_heater() {
    if ( heaterEnabled ) {
        printf("[io] Disabling heater\n");
    }
    heaterEnabled = false;
    digitalWrite(HEATER_ENABLE_PIN, LOW);
}

bool Io::heater_is_enabled() {
    return heaterEnabled;
}

// Inputs

/* Report the debounced state rather than re-reading the pin, so every caller
 * within a cycle sees the same value as the event that was dispatched. */
bool Io::ignition_is_on() {
    return ignitionOn;
}

bool Io::charge_enable_is_on() {
    return chargeEnable;
}

bool Io::pos_contactor_feedback_closed() {
    return digitalRead(POS_CONTACTOR_FEEDBACK_PIN) == HIGH;
}

bool Io::neg_contactor_feedback_closed() {
    return digitalRead(NEG_CONTACTOR_FEEDBACK_PIN) == HIGH;
}
