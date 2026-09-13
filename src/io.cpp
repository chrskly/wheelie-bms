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

/*
 * Interrupt handler for ignition signal changes. This is called when the
 * ignition signal changes state (on/off). It reads the new state and sends
 * the appropriate event to the state machine.
 */
void ignition_signal_changed() {
    extern State state;
    int newState = digitalRead(IGNITION_ENABLE_PIN);
    std::string newStateStr = newState == 1 ? "on" : "off";
    printf("[io] Ignition signal changed to : %s\n", newStateStr.c_str());
    if ( newState ) {
        bms.send_event(E_IGNITION_ON);
    } else {
        bms.send_event(E_IGNITION_OFF);
    }
}

/*
 * Interrupt handler for charge signal changes. This is called when the
 * charge signal changes state (on/off). It reads the new state and sends
 * the appropriate event to the state machine.
 */
void charge_signal_changed() {
    extern State state;
    int newState = digitalRead(CHARGE_ENABLE_PIN);
    std::string newStateStr = newState == 1 ? "on" : "off";
    printf("[io] Charge signal changed to : %s\n", newStateStr.c_str());
    if ( newState ) {
        bms.send_event(E_CHARGING_INITIATED);
    } else {
        bms.send_event(E_CHARGING_TERMINATED);
    }
}

void Io::init() {
    ignitionOn = false;
    chargeEnable = false;

    // IGNITION input
    pinMode(IGNITION_ENABLE_PIN, INPUT);

    // CHARGE_ENABLE input
    pinMode(CHARGE_ENABLE_PIN, INPUT);

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

void Io::attach_interrupts() {
    printf("[io] attaching ignition and charge-enable interrupts\n");
    attachInterrupt(digitalPinToInterrupt(IGNITION_ENABLE_PIN), ignition_signal_changed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CHARGE_ENABLE_PIN), charge_signal_changed, CHANGE);
}

// REMINDER : THESE OUTPUTS ARE A LOW SIDE SWITCHES.
//     gpio high == on  == output low
//     gpio low  == off == output high/floating?

// DRIVE_INHIBIT output

void Io::enable_drive_inhibit(std::string context) {
    if ( !driveInhibited ) {
        printf("[io] Enabling drive inhibit : %s\n", context.c_str());
    }
    driveInhibited = true;
    digitalWrite(DRIVE_INHIBIT_PIN, HIGH);
}

void Io::disable_drive_inhibit(std::string context) {
    if ( driveInhibited ) {
        printf("[io] Disabling drive inhibit : %s\n", context.c_str());
    }
    driveInhibited = false;
    digitalWrite(DRIVE_INHIBIT_PIN, LOW);
}

bool Io::drive_is_inhibited() {
    return driveInhibited;
}

// CHARGE_INHIBIT output

void Io::enable_charge_inhibit(std::string context) {
    if ( !chargeInhibited ) {
        printf("[io] Enabling charge inhibit : %s\n", context.c_str());
    }
    chargeInhibited = true;
    digitalWrite(CHARGE_INHIBIT_PIN, HIGH);
}

void Io::disable_charge_inhibit(std::string context) {
    if ( chargeInhibited ) {
        printf("[io] Disabling charge inhibit : %s\n", context.c_str());
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

bool Io::ignition_is_on() {
    return digitalRead(IGNITION_ENABLE_PIN) == HIGH;
}

bool Io::charge_enable_is_on() {
    return digitalRead(CHARGE_ENABLE_PIN) == HIGH;
}

bool Io::pos_contactor_is_welded() {
    return digitalRead(POS_CONTACTOR_FEEDBACK_PIN) == HIGH;
}

bool Io::neg_contactor_is_welded() {
    return digitalRead(NEG_CONTACTOR_FEEDBACK_PIN) == HIGH;
}
