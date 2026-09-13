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
#include "battery.h"
#include "bms.h"
#include "led.h"


extern Battery battery;
extern Bms bms;
extern Shunt shunt;


/* 
 * ~~ Note 1 ~~
 *
 * If we're driving around with some of the packs inhibited, and we want to go
 * directly into charge mode, dealing with the contactors is too awkward. While
 * driving, the high pack(s) will be enabled and the low pack(s) will be disabled.
 * However, when charging we want that to be the other way around. There's no
 * clean way to do this. So lets not try. Just go into fault mode. The same is
 * the case if we're charging and want to go directly into drive mode.
 *
 * ~~ Note 2 ~~
 *
 * The inverter-controlled contactors are potentially open in the standby,
 * batteryEmtpy, and overTempFault states only. So we can only change the
 * inhibition of the battery contactors from either of these states.
 */


/*
 * State               : standby
 * Ignition            : off
 * Battery contactors  : open && ( inhibited || not inhibited )
 * Inverter contactors : open
 * CHARGE_INHIBIT      : on || off
 * CHARGE_ENABLE       : off
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : off
 *
 * Notes:
 * - CHARGE_INHIBIT can be on in this state either because of full battery or 
 *   battery too hot.
 * - DRIVE_INHIBIT is normally off here, but is held on from power-on until the
 *   battery reports (R_STARTUP), and while a charge is being requested
 *   (R_CHARGING). It is withdrawn by the reconciliation pass in the health
 *   check once those conditions clear, not by this state.
 */
void state_standby(Event event) {
    // Safeties
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            bms.enable_charge_inhibit("[S01] too cold to charge", R_TOO_COLD);
            break;
        case E_TEMPERATURE_OK:
            /* No has_full_cell() guard: that was left over from when disable
             * cleared every reason at once, and it meant a full battery blocked
             * withdrawal of an unrelated temperature hold. R_BATTERY_FULL keeps
             * the inhibit asserted on its own. */
            bms.disable_charge_inhibit("[S02] no longer too cold to charge", R_TOO_COLD);
            break;
        case E_TOO_HOT:
            bms.enable_drive_inhibit("[S03] battery too hot", R_TOO_HOT);
            bms.enable_charge_inhibit("[S04] battery too hot", R_TOO_HOT);
            bms.set_state(&state_overTempFault, "battery too hot");
            break;
        case E_BATTERY_EMPTY:
            bms.enable_drive_inhibit("[S05] empty battery", R_BATTERY_EMPTY);
            bms.set_state(&state_batteryEmpty, "empty battery");
            break;
        case E_BATTERY_NOT_EMPTY:
            /* R_BATTERY_EMPTY is only ever a DRIVE inhibit reason, so
             * withdrawing it from the charge mask here did nothing. */
            bms.disable_charge_inhibit("[S06] battery neither empty nor full", R_BATTERY_FULL);
            bms.disable_drive_inhibit("[S06] battery neither empty nor full", R_BATTERY_EMPTY);
            break;
        case E_BATTERY_FULL:
            bms.enable_charge_inhibit("[S07] full battery", R_BATTERY_FULL);
            break;
        case E_PACKS_IMBALANCED:
            /* This event can only fire when we have multiple packs. The
             * contactors are currently open (or inhibited). We don't want to
             * allow the contactors to close when the packs have different
             * voltages. So we inhibit the contactors on all packs here. When we
             * switch into another state we'll decide which contactors to allow
             * to close then. This will depend on which state we switch into. */
            battery.enable_inhibit_contactor_close();
            break;
        case E_PACKS_NOT_IMBALANCED:
            /* This event can only fire when we have multiple packs. The packs
             * have equalised so we can allow the contactors to close again. */
            battery.disable_inhibit_contactor_close();
            break;
        case E_IGNITION_ON:
            if ( battery.has_multiple_packs() ) {
                /* If packs are imbalanced, decide which contactors to allow to
                 * close. Since we're going into drive mode, we want to pick the
                 * high pack(s). */
                if ( battery.one_or_more_contactors_inhibited() ) {
                    battery.reevaluate_contactor_inhibition_for_drive();
                }
            }
            bms.set_state(&state_drive, "ignition turned on");
            break;
        case E_IGNITION_OFF:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_IGNITION_OFF while in standby state\n");
            break;
        case E_CHARGING_INITIATED:
            if ( battery.has_multiple_packs() ) {
                // Deal with battery contactor inhibition
                if ( battery.one_or_more_contactors_inhibited() ) {
                    battery.reevaluate_contactor_inhibition_for_charge();
                }
            }
            // Drive away protection
            bms.enable_drive_inhibit("[S08] charge requested", R_CHARGING);
            /* If the batteries are not warm enough to be charged, turn on the
             * battery heater, and disallow charging until they're warm enough. */
            if ( battery.too_cold_to_charge() ) {
                bms.enable_heater();
                bms.enable_charge_inhibit("[S09] too cold to charge", R_TOO_COLD);
                bms.set_state(&state_batteryHeating, "charge requested, but too cold to charge");
                break;
            }
            bms.set_state(&state_charging, "charge requested");
            break;
        case E_CHARGING_TERMINATED:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_CHARGING_TERMINATED while in standby state\n");
            break;
        case E_MODULE_UNRESPONSIVE:
            bms.enable_charge_inhibit("[S10] dead module", R_MODULE_UNRESPONSIVE);
            bms.enable_drive_inhibit("[S11] dead module", R_MODULE_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead module");
            break;
        case E_MODULES_ALL_RESPONSIVE:
            /* Withdraw the holds this resolves. Nothing in standby used to clear
             * DRIVE_INHIBIT at all, so a hold taken here outlived its cause. */
            bms.disable_charge_inhibit("[S14] modules responsive", R_MODULE_UNRESPONSIVE);
            bms.disable_drive_inhibit("[S15] modules responsive", R_MODULE_UNRESPONSIVE);
            break;
        case E_SHUNT_UNRESPONSIVE:
            bms.enable_charge_inhibit("[S12] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.enable_drive_inhibit("[S13] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead shunt");
            break;
        case E_SHUNT_RESPONSIVE:
            bms.disable_charge_inhibit("[S16] shunt responsive", R_SHUNT_UNRESPONSIVE);
            bms.disable_drive_inhibit("[S17] shunt responsive", R_SHUNT_UNRESPONSIVE);
            break;
        case E_DEAD_CELL:
            if ( battery.has_multiple_packs() ) {
                battery.reevaluate_dead_cell_inhibition();
            }
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in standby state\n");
    }
}

/*
 * State               : drive
 * Ignition            : on
 * Battery contactors  : closed && ( inhibited || not inhibited )
 * Inverter contactors : closed
 * CHARGE_INHIBIT      : on || off
 * CHARGE_ENABLE       : off
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : off
 * 
 * Notes:
 * - CHARGE_INHIBIT can be on in this state either because of full battery or 
 *   battery too hot.
 */
void state_drive(Event event) {
    /* Safeties. Withdraw only the reasons that being in the drive state actually
     * resolves -- a blanket clear would also release a fault hold. Faults
     * transition out of this state, and each fault state withdraws its own
     * reason on exit. */
    bms.disable_drive_inhibit("[D00] driving", R_CHARGING);
    bms.disable_drive_inhibit("[D00] driving", R_BATTERY_EMPTY);
    bms.disable_drive_inhibit("[D00] driving", R_ILLEGAL_STATE_TRANSITION);
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            bms.enable_charge_inhibit("[D01] too cold to charge", R_TOO_COLD);
            break;
        case E_TEMPERATURE_OK:
            // See [S02]: R_BATTERY_FULL holds the inhibit on its own if needed.
            bms.disable_charge_inhibit("[D02] not too cold to charge", R_TOO_COLD);
            break;
        case E_TOO_HOT:
            bms.enable_drive_inhibit("[D03] battery too hot", R_TOO_HOT);
            bms.enable_charge_inhibit("[D04] battery too hot", R_TOO_HOT);
            bms.set_state(&state_overTempFault, "battery too hot");
            break;
        case E_BATTERY_EMPTY:
            bms.enable_drive_inhibit("[D05] empty battery", R_BATTERY_EMPTY);
            bms.set_state(&state_batteryEmpty, "empty battery");
            break;
        case E_BATTERY_NOT_EMPTY:
            /* Was withdrawing R_BATTERY_EMPTY from the CHARGE mask, which
             * nothing ever sets -- a no-op. The hold this event resolves is the
             * full-battery one. */
            bms.disable_charge_inhibit("[D06] battery neither empty nor full", R_BATTERY_FULL);
            bms.disable_drive_inhibit("[D06] battery not empty", R_BATTERY_EMPTY);
            break;
        case E_BATTERY_FULL:
            bms.enable_charge_inhibit("[D07] full battery", R_BATTERY_FULL);
            break;
        case E_PACKS_IMBALANCED:
        case E_PACKS_NOT_IMBALANCED:
            /* Deliberately does nothing. Changing contactor inhibition here
             * would open a pack contactor while current is flowing, which arcs
             * and welds it -- see Note 2 at the top of this file. Both of these
             * used to call reevaluate_contactor_inhibition_for_drive(). The
             * imbalance is re-evaluated on the way back into standby. */
            break;
        case E_IGNITION_ON:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_IGNITION_ON while in drive state\n");
            break;
        case E_IGNITION_OFF:
            /* Note 2: contactor inhibition is not changed here -- asserting the
             * inhibit line can drop a contactor coil, and the inverter
             * contactors may still be closed as we leave. standby's own
             * E_PACKS_IMBALANCED handler applies the hold on the next health
             * check, by which point the inverter contactors are open. */
            bms.set_state(&state_standby, "ignition turned off");
            break;
        case E_CHARGING_INITIATED:
            // Drive away protection
            bms.enable_drive_inhibit("[D08] imbalanced packs", R_CHARGING);
            /* Cannot go straight from drive mode to charge mode when packs are
            * imbalanced. See note 1 above. */
            if ( battery.one_or_more_contactors_inhibited() ) {
                bms.enable_charge_inhibit("[D09] imbalanced packs", R_ILLEGAL_STATE_TRANSITION);
                bms.set_illegal_state_transition();
                bms.set_state(&state_illegalStateTransitionFault, "cannot switch directly from drive to charge with imbalanced packs");
                break;
            }
            bms.set_state(&state_charging, "charge requested");
            break;
        case E_CHARGING_TERMINATED:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_CHARGING_TERMINATED while in drive state\n");
            break;
        case E_MODULE_UNRESPONSIVE:
            /* Disallow charging. Consider disallowing driving. */
            bms.enable_charge_inhibit("[D10] dead module", R_MODULE_UNRESPONSIVE);
            break;
        case E_MODULES_ALL_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_SHUNT_UNRESPONSIVE:
            /* Disallow charging. Consider disallowing driving. */
            bms.enable_charge_inhibit("[D11] dead shunt", R_SHUNT_UNRESPONSIVE);
            break;
        case E_SHUNT_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_DEAD_CELL:
            /* Deliberately does nothing. Note 2 at the top of this file: the
             * inverter contactors are closed in this state, so opening a pack
             * contactor here would break current under load. Dead-cell
             * inhibition is re-evaluated on the way into standby, batteryEmpty
             * or overTempFault, where the contactors may actually be open. */
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in drive state\n");
    }
}

/*
 * State               : batteryHeating
 * Ignition            : on || off
 * Battery contactors  : closed && ( inhibited || not inhibited )
 * Inverter contactors : closed
 * CHARGE_INHIBIT      : on
 * CHARGE_ENABLE       : on
 * HEATER_ENABLE       : on
 * DRIVE_INHIBIT       : on
 * 
 * Notes:
 * - Inverter contactors are always closed in this state.
 *   
 */
void state_batteryHeating(Event event) {
    // Safeties
    bms.enable_charge_inhibit("[H00] battery heating", R_TOO_COLD);
    bms.enable_drive_inhibit("[H00] battery heating", R_CHARGING);

    /* The heater only runs while we can see what it is doing and while there is
     * still a prospect of it working. Without these guards, stale temperature
     * data keeps too_cold_to_charge() true forever, E_TEMPERATURE_OK never
     * arrives, and the heater runs indefinitely with no feedback. */
    const bool heatingBlind = battery.temperature_data_is_stale();
    const bool heatingTimedOut = ( bms.time_in_state_ms() > BATTERY_HEATING_TIMEOUT_MS );
    if ( heatingBlind || heatingTimedOut ) {
        bms.disable_heater();
        bms.set_internal_error(IE_HEATER_INEFFECTIVE);
        printf("[statemachine] heating abandoned : %s\n",
               heatingBlind ? "temperature data stale" : "timed out");
    } else {
        bms.clear_internal_error(IE_HEATER_INEFFECTIVE);
        bms.enable_heater();
    }

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_TEMPERATURE_OK:
            bms.disable_heater();
            bms.disable_charge_inhibit("[H01] battery warmed to minimum charging temperature", R_TOO_COLD);
            bms.set_state(&state_charging, "battery warmed to minimum charging temperature");
            break;
        case E_TOO_HOT:
            bms.disable_heater();
            bms.enable_charge_inhibit("[H02] battery too hot", R_TOO_HOT);
            bms.set_state(&state_overTempFault, "battery too hot");
            break;
        case E_BATTERY_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_NOT_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_FULL:
            bms.disable_heater();
            // Was transitioning into the charging state without inhibiting charge.
            bms.enable_charge_inhibit("[H12] full battery", R_BATTERY_FULL);
            bms.set_state(&state_charging, "battery full");
            break;
        case E_PACKS_IMBALANCED:
        case E_PACKS_NOT_IMBALANCED:
            /* Deliberately does nothing. This used to call
             * reevaluate_contactor_inhibition_for_charge() with a "FIXME is this
             * right?" beside it -- it was not. Note 2: the inverter contactors
             * are closed in this state, so a pack contactor opened here breaks
             * current under load. */
            break;
        case E_IGNITION_ON:
            break;  // Valid event, but we don't need to do anything with it.
        case E_IGNITION_OFF:
            break;  // Valid event, but we don't need to do anything with it.
        case E_CHARGING_INITIATED:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_CHARGING_INITIATED while in batteryHeating state\n");
            break;
        case E_CHARGING_TERMINATED:
            // We're no longer seeking to charge. No need to continue heating.
            bms.disable_heater();
            /* Cannot go straight from charge mode to drive mode when packs are
             * imbalanced. See note 1 above. */
            if ( battery.one_or_more_contactors_inhibited() && bms.ignition_is_on() ) {
                bms.set_illegal_state_transition();
                bms.set_state(&state_illegalStateTransitionFault, "cannot switch directly from charge to drive with imbalanced packs");
                break;
            }
            // Battery empty
            if ( battery.has_empty_cell() ) {
                bms.disable_charge_inhibit("[H03] charge terminated but battery still empty", R_TOO_COLD);
                bms.set_state(&state_batteryEmpty, "charge terminated but battery still empty");
                break;
            }
            // Drive mode
            if ( bms.ignition_is_on() ) {
                bms.disable_charge_inhibit("[H04] charging terminated + ignition on", R_TOO_COLD);
                bms.disable_drive_inhibit("[H05] charging terminated + ignition on", R_CHARGING);
                bms.set_state(&state_drive, "charging terminated + ignition on");
                break;
            }
            // Standby mode. See the note in state_drive's E_IGNITION_OFF.
            bms.disable_drive_inhibit("[H06] ignition off", R_CHARGING);
            bms.disable_charge_inhibit("[H07] ignition off", R_TOO_COLD);
            bms.set_state(&state_standby, "charging terminated");
            break;
        case E_MODULE_UNRESPONSIVE:
            bms.enable_charge_inhibit("[H08] dead module", R_MODULE_UNRESPONSIVE);
            bms.enable_drive_inhibit("[H09] dead module", R_MODULE_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead module");
            break;
        case E_MODULES_ALL_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_SHUNT_UNRESPONSIVE:
            bms.enable_charge_inhibit("[H10] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.enable_drive_inhibit("[H11] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead shunt");
            break;
        case E_SHUNT_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_DEAD_CELL:
            /* Deliberately does nothing. Note 2 at the top of this file: the
             * inverter contactors are closed in this state, so opening a pack
             * contactor here would break current under load. Dead-cell
             * inhibition is re-evaluated on the way into standby, batteryEmpty
             * or overTempFault, where the contactors may actually be open. */
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in batteryHeating state\n");

    }
}

/*
 * State               : charging
 * Ignition            : on || off
 * Battery contactors  : closed && ( inhibited || not inhibited )
 * Inverter contactors : closed
 * CHARGE_INHIBIT      : on || off
 * CHARGE_ENABLE       : on
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : on
 */
void state_charging(Event event) {
    // Safeties
    bms.enable_drive_inhibit("[C00] charging", R_CHARGING);
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            bms.enable_heater();
            bms.enable_charge_inhibit("[C01] too cold to charge", R_TOO_COLD);
            bms.set_state(&state_batteryHeating, "too cold to charge");
            break;
        case E_TEMPERATURE_OK:
            break;  // Valid event, but we don't need to do anything with it.
        case E_TOO_HOT:
            bms.enable_charge_inhibit("[C02] battery too hot", R_TOO_HOT);
            bms.set_state(&state_overTempFault, "battery too hot");
            break;
        case E_BATTERY_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_NOT_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_FULL:
            /* Tell the charger to stop, but don't switch out of this state
             * until we get a CHARGING_TERMINATED event. */
            bms.enable_charge_inhibit("[C03] full battery", R_BATTERY_FULL);
            break;
        case E_PACKS_IMBALANCED:
        case E_PACKS_NOT_IMBALANCED:
            /* Deliberately does nothing while charge current is flowing, for the
             * same reason as the drive state -- see Note 2 at the top of this
             * file. Re-evaluated when charging terminates. */
            break;
        case E_IGNITION_ON:
            break;  // Valid event, but we don't need to do anything with it.
        case E_IGNITION_OFF:
            break;  // Valid event, but we don't need to do anything with it.
        case E_CHARGING_INITIATED:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_CHARGING_INITIATED while in charging state\n");
            break;
        case E_CHARGING_TERMINATED:
            /* If battery is full (i.e., we've charged to 100%), reset kWh/Ah
             * counters on the ISA shunt. */
            if ( battery.has_full_cell() ) {
                bms.send_shunt_reset_message();
            }
            /* Cannot go straight from charge mode to drive mode when packs are
             * imbalanced. See note 1 above. */
            if ( battery.one_or_more_contactors_inhibited() && bms.ignition_is_on() ) {
                bms.enable_charge_inhibit("[C04] imbalanced packs", R_ILLEGAL_STATE_TRANSITION);
                bms.set_illegal_state_transition();
                bms.set_state(&state_illegalStateTransitionFault, "cannot switch directly from charge to drive with imbalanced packs");
                break;
            }
            /* Did we start charging with an empty battery, but cancel the
             * charge before actually putting any energy into the battery? */
            if ( battery.has_empty_cell() ) {
                bms.set_state(&state_batteryEmpty, "charge terminated but battery still empty");
                break;
            }
            // If ignition is already on, switch directly to drive mode
            if ( bms.ignition_is_on() ) {
                bms.disable_drive_inhibit("[C05] charging terminated + ignition on", R_CHARGING);
                bms.set_state(&state_drive, "charging terminated + ignition on");
                break;
            }
            bms.disable_drive_inhibit("[C06] charging terminated", R_CHARGING);
            bms.set_state(&state_standby, "charging terminated");
            break;
        case E_MODULE_UNRESPONSIVE:
            bms.enable_charge_inhibit("[C07] dead module", R_MODULE_UNRESPONSIVE);
            bms.enable_drive_inhibit("[C07] dead module", R_MODULE_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead module");
            break;
        case E_MODULES_ALL_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_SHUNT_UNRESPONSIVE:
            bms.enable_charge_inhibit("[C08] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.enable_drive_inhibit("[C08] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead shunt");
            break;
        case E_SHUNT_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_DEAD_CELL:
            /* Deliberately does nothing. Note 2 at the top of this file: the
             * inverter contactors are closed in this state, so opening a pack
             * contactor here would break current under load. Dead-cell
             * inhibition is re-evaluated on the way into standby, batteryEmpty
             * or overTempFault, where the contactors may actually be open. */
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in charging state\n");
    }
}


/*
 * State               : batteryEmpty
 * Ignition            : on || off
 * Battery contactors  : open && ( inhibited || not inhibited )
 * Inverter contactors : open
 * CHARGE_INHIBIT      : on || off
 * CHARGE_ENABLE       : off
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : on
 * 
 * Notes:
 * - Chaging the battery contactor inhibition is allowed in this state. It is
 *   dependent only on the ignition state. If the ignition is off, we can change
 *   the battery contactor inhibition. If the ignition is on, we cannot.
 * 
 */
void state_batteryEmpty(Event event) {
    // Safeties
    bms.enable_drive_inhibit("[E00] battery empty", R_BATTERY_EMPTY);
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            bms.enable_charge_inhibit("[E01] too cold to charge", R_TOO_COLD);
            break;
        case E_TEMPERATURE_OK:
            bms.disable_charge_inhibit("[E02] no longer too cold to charge", R_TOO_COLD);
            break;
        case E_TOO_HOT:
            bms.enable_charge_inhibit("[E03] battery too hot", R_TOO_HOT);
            bms.set_state(&state_overTempFault, "battery too hot");
            break;
        case E_BATTERY_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_NOT_EMPTY:
            bms.disable_drive_inhibit("[E04] battery not empty", R_BATTERY_EMPTY);
            if ( bms.ignition_is_on() ) {
                bms.set_state(&state_drive, "battery level rose");
                break;
            }
            if ( bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            bms.set_state(&state_standby, "battery level rose");
            break;
        case E_BATTERY_FULL:
            bms.disable_drive_inhibit("[E05] battery no longer empty", R_BATTERY_EMPTY);
            bms.enable_charge_inhibit("[E06] full battery", R_BATTERY_FULL);
            if ( bms.ignition_is_on() ) {
                bms.set_state(&state_drive, "battery full");
                break;
            }
            if ( bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            bms.set_state(&state_standby, "battery full");
            break;
        case E_PACKS_IMBALANCED:
            if ( ! bms.ignition_is_on() ) {
                battery.enable_inhibit_contactor_close();
            }
            break;
        case E_PACKS_NOT_IMBALANCED:
            if ( ! bms.ignition_is_on() ) {
                battery.disable_inhibit_contactor_close();
            }
            break;
        case E_IGNITION_ON:
            if ( bms.packs_are_imbalanced() ) {
                battery.disable_inhibit_contactors_for_drive();
            }
            break;
        case E_IGNITION_OFF:
            if ( bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            break;
        case E_CHARGING_INITIATED:
            if ( ! bms.ignition_is_on() && bms.packs_are_imbalanced() ) {
                battery.disable_inhibit_contactors_for_charge();
            }
            if ( battery.too_cold_to_charge() ) {
                bms.enable_charge_inhibit("[E07] too cold to charge", R_TOO_COLD);
                bms.enable_heater();
                bms.set_state(&state_batteryHeating, "charge requested, but too cold to charge");
                break;
            }
            bms.set_state(&state_charging, "charge requested");
            break;
        case E_CHARGING_TERMINATED:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : E_CHARGING_TERMINATED while in batteryEmpty state\n");
            break;
        case E_MODULE_UNRESPONSIVE:
            bms.enable_charge_inhibit("[E08] dead module", R_MODULE_UNRESPONSIVE);
            bms.enable_drive_inhibit("[E09] dead module", R_MODULE_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead module");
            break;
        case E_MODULES_ALL_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_SHUNT_UNRESPONSIVE:
            bms.enable_charge_inhibit("[E10] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.enable_drive_inhibit("[E11] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead shunt");
            break;
        case E_SHUNT_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_DEAD_CELL:
            if ( battery.has_multiple_packs() ) {
                battery.reevaluate_dead_cell_inhibition();
            }
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in batteryEmpty state\n");
    }
}

/*
 * State               : overTempFault
 * Ignition            : on || off
 * Battery contactors  : ( open || closed ) && ( inhibited || not inhibited )
 * Inverter contactors : open || closed
 * CHARGE_INHIBIT      : on || off
 * CHARGE_ENABLE       : on || off
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : on
 *
 * Notes:
 * 1. The only way to get out of this state is for the battery to cool down.
 * 2. If a charge is requested while in this state, we stay here and just
 *    disallow. We don't switch over to the charging state for safety reasons. 
 */
void state_overTempFault(Event event) {
    // Safeties
    bms.enable_drive_inhibit("[T00] battery too hot", R_TOO_HOT);
    bms.enable_charge_inhibit("[T00] battery too hot", R_TOO_HOT);
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            bms.enable_charge_inhibit("[T01] too cold to charge", R_TOO_COLD);
            /* "Too cold to charge" implies "no longer too hot" -- but only if
             * the readings are real. too_cold_to_charge() also returns true when
             * the temperature data has gone stale, and leaving an
             * over-temperature fault because the sensors stopped reporting is
             * exactly the wrong response. Stay in the fault until a reading we
             * trust says the pack has cooled. */
            if ( battery.temperature_data_is_stale() ) {
                break;
            }
            if ( bms.charge_is_enabled() ) {
                bms.set_state(&state_batteryHeating, "no longer too hot");
                break;
            }
            if ( bms.ignition_is_on() ) {
                bms.set_state(&state_drive, "no longer too hot");
                break;
            }
            if ( bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            bms.set_state(&state_standby, "no longer too hot");
            break;
        case E_TEMPERATURE_OK:
            // Charge mode overrides drive mode
            if ( bms.charge_is_enabled() ) {
                bms.disable_charge_inhibit("[T02] battery has cooled", R_TOO_HOT);
                bms.set_state(&state_charging, "battery has cooled");
                break;
            }
            // Drive mode
            if ( bms.ignition_is_on() ) {
                bms.disable_charge_inhibit("[T03] battery has cooled", R_TOO_HOT);
                bms.disable_drive_inhibit("[T04] battery has cooled", R_TOO_HOT);
                bms.set_state(&state_drive, "battery has cooled");
                break;
            }
            // Standby mode
            bms.disable_drive_inhibit("[T05] battery has cooled", R_TOO_HOT);
            bms.disable_charge_inhibit("[T06] battery has cooled", R_TOO_HOT);
            if ( bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            bms.set_state(&state_standby, "battery has cooled");
            break;
        case E_TOO_HOT:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_NOT_EMPTY:
            break;  // Valid event, but we don't need to do anything with it.
        case E_BATTERY_FULL:
            break; // Valid event, but we don't need to do anything with it.
        case E_PACKS_IMBALANCED:
            if ( ! bms.ignition_is_on() && ! bms.charge_is_enabled() ) {
                battery.enable_inhibit_contactor_close();
            }
            break;
        case E_PACKS_NOT_IMBALANCED:
            if ( ! bms.ignition_is_on() && ! bms.charge_is_enabled() ) {
                battery.disable_inhibit_contactor_close();
            }
            break;
        case E_IGNITION_ON:
            if ( ! bms.charge_is_enabled() && bms.packs_are_imbalanced() ) {
                battery.disable_inhibit_contactors_for_drive();
            }
            break;
        case E_IGNITION_OFF:
            if ( ! bms.charge_is_enabled() && bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            break;
        case E_CHARGING_INITIATED:
            if ( ! bms.ignition_is_on() && bms.packs_are_imbalanced() ) {
                battery.disable_inhibit_contactors_for_charge();
            }
            break;
        case E_CHARGING_TERMINATED:
            if ( ! bms.ignition_is_on() && bms.packs_are_imbalanced() ) {
                battery.enable_inhibit_contactor_close();
            }
            break;
        case E_MODULE_UNRESPONSIVE:
            /* Was disable_inhibit_contactor_close(), i.e. it PERMITTED the
             * contactors to close in response to a critical fault. */
            if (! bms.ignition_is_on() && ! bms.charge_is_enabled() ) {
                battery.enable_inhibit_contactor_close();
            }
            bms.set_state(&state_criticalFault, "dead module");
            break;
        case E_MODULES_ALL_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_SHUNT_UNRESPONSIVE:
            // Was disable_inhibit_contactor_close(); see the dead-module case above.
            if (! bms.ignition_is_on() && ! bms.charge_is_enabled() ) {
                battery.enable_inhibit_contactor_close();
            }
            bms.set_state(&state_criticalFault, "dead shunt");
            break;
        case E_SHUNT_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_DEAD_CELL:
            if ( battery.has_multiple_packs() ) {
                battery.reevaluate_dead_cell_inhibition();
            }
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in overTempFault state\n");
    }
}


/*
 * State               : illegalStateTransitionFault
 * Ignition            : on || off
 * Battery contactors  : ( open || closed ) && ( inhibited || not inhibited )
 * Inverter contactors : closed
 * CHARGE_INHIBIT      : on
 * CHARGE_ENABLE       : on || off
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : on
 *
 * Reasons we can be in this state:
 *   - We tried to go straight from drive to charge with imbalanced packs
 */
void state_illegalStateTransitionFault(Event event) {
    // Safeties
    bms.enable_drive_inhibit("[I00] illegal state transition", R_ILLEGAL_STATE_TRANSITION);
    bms.enable_charge_inhibit("[I00] illegal state transition", R_ILLEGAL_STATE_TRANSITION);
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            break;
        case E_TEMPERATURE_OK:
            break;
        case E_TOO_HOT:
            break;
        case E_BATTERY_EMPTY:
            break;
        case E_BATTERY_NOT_EMPTY:
            break;
        case E_BATTERY_FULL:
            break;
        case E_PACKS_IMBALANCED:
            break;
        case E_PACKS_NOT_IMBALANCED:
            break;
        case E_IGNITION_ON:
            break;
        case E_IGNITION_OFF:
            if ( ! bms.charge_is_enabled() ) {
                bms.clear_illegal_state_transition();
                bms.disable_drive_inhibit("[I03] fault cleared", R_ILLEGAL_STATE_TRANSITION);
                bms.disable_charge_inhibit("[I03] fault cleared", R_ILLEGAL_STATE_TRANSITION);
                bms.set_state(&state_standby, "ignition and charging off");
            }
            break;
        case E_CHARGING_INITIATED:
            break;
        case E_CHARGING_TERMINATED:
            if ( ! bms.ignition_is_on() ) {
                bms.clear_illegal_state_transition();
                bms.disable_drive_inhibit("[I04] fault cleared", R_ILLEGAL_STATE_TRANSITION);
                bms.disable_charge_inhibit("[I04] fault cleared", R_ILLEGAL_STATE_TRANSITION);
                bms.set_state(&state_standby, "ignition and charging off");
            }
            break;
        case E_MODULE_UNRESPONSIVE:
            bms.enable_charge_inhibit("[I01] dead module", R_MODULE_UNRESPONSIVE);
            bms.enable_drive_inhibit("[I01] dead module", R_MODULE_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead module");
            break;
        case E_MODULES_ALL_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_SHUNT_UNRESPONSIVE:
            bms.enable_charge_inhibit("[I02] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.enable_drive_inhibit("[I02] dead shunt", R_SHUNT_UNRESPONSIVE);
            bms.set_state(&state_criticalFault, "dead shunt");
            break;
        case E_SHUNT_RESPONSIVE:
            break;  // Valid event, but we don't need to do anything with it.
        case E_DEAD_CELL:
            /* Deliberately does nothing. Note 2 at the top of this file: the
             * inverter contactors are closed in this state, so opening a pack
             * contactor here would break current under load. Dead-cell
             * inhibition is re-evaluated on the way into standby, batteryEmpty
             * or overTempFault, where the contactors may actually be open. */
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in illegalStateTransitionFault state\n");
    }
}

/*
 * State               : criticalFault
 * Ignition            : on || off
 * Battery contactors  : ( open || closed ) && ( inhibited || not inhibited )
 * Inverter contactors : open || closed
 * CHARGE_INHIBIT      : on
 * CHARGE_ENABLE       : on || off
 * HEATER_ENABLE       : off
 * DRIVE_INHIBIT       : on
 * 
 * Notes:
 * - The only way to exit this state is for the shunt AND all modules to be
 *   responsive again. Both exit paths below check both conditions.
 * - An earlier version of this comment also required "all cells", which was
 *   never implemented and should not be: a dead cell is handled by inhibiting
 *   that pack's contactors, not by holding the whole system in a critical
 *   fault, so requiring it here would deadlock on a single bad cell. 
 */
//------------------------------------------------------------------------------
void state_criticalFault(Event event) {
    // Safeties
    bms.enable_drive_inhibit("[CF00] critical fault", R_CRITICAL_FAULT);
    bms.enable_charge_inhibit("[CF00] critical fault", R_CRITICAL_FAULT);
    bms.disable_heater();

    switch (event) {
        case E_TOO_COLD_TO_CHARGE:
            break;
        case E_TEMPERATURE_OK:
            break;
        case E_TOO_HOT:
            break;
        case E_BATTERY_EMPTY:
            break;
        case E_BATTERY_NOT_EMPTY:
            break;
        case E_BATTERY_FULL:
            break;
        case E_PACKS_IMBALANCED:
            break;
        case E_PACKS_NOT_IMBALANCED:
            break;
        case E_IGNITION_ON:
            break;
        case E_IGNITION_OFF:
            break;
        case E_CHARGING_INITIATED:
            break;
        case E_CHARGING_TERMINATED:
            break;
        case E_MODULE_UNRESPONSIVE:
            break;
        case E_MODULES_ALL_RESPONSIVE:
            if ( ! shunt.is_dead() ) {
                /* Re-check the conditions that own their own fault state before
                 * handing control back to drive or charge. */
                if ( battery.too_hot() ) {
                    bms.set_state(&state_overTempFault, "critical fault cleared but battery too hot");
                    break;
                }
                bms.disable_drive_inhibit("[CF01] critical fault cleared", R_CRITICAL_FAULT);
                bms.disable_charge_inhibit("[CF01] critical fault cleared", R_CRITICAL_FAULT);
                bms.disable_charge_inhibit("[CF01] critical fault cleared", R_MODULE_UNRESPONSIVE);
                bms.disable_drive_inhibit("[CF01] critical fault cleared", R_MODULE_UNRESPONSIVE);
                bms.disable_charge_inhibit("[CF01] critical fault cleared", R_SHUNT_UNRESPONSIVE);
                bms.disable_drive_inhibit("[CF01] critical fault cleared", R_SHUNT_UNRESPONSIVE);
                if ( bms.charge_is_enabled() ) {
                    bms.set_state(&state_charging, "critical fault cleared");
                    break;
                }
                if ( bms.ignition_is_on() ) {
                    bms.set_state(&state_drive, "critical fault cleared");
                    break;
                }
                // See the note in state_drive's E_IGNITION_OFF.
                bms.set_state(&state_standby, "critical fault cleared");
            }
            break;
        case E_SHUNT_UNRESPONSIVE:
            break;
        case E_SHUNT_RESPONSIVE:
            if ( battery.is_alive() ) {
                // See [CF01]: do not hand control back into an over-temperature pack.
                if ( battery.too_hot() ) {
                    bms.set_state(&state_overTempFault, "critical fault cleared but battery too hot");
                    break;
                }
                bms.disable_drive_inhibit("[CF02] critical fault cleared", R_CRITICAL_FAULT);
                bms.disable_charge_inhibit("[CF02] critical fault cleared", R_CRITICAL_FAULT);
                bms.disable_charge_inhibit("[CF02] critical fault cleared", R_MODULE_UNRESPONSIVE);
                bms.disable_drive_inhibit("[CF02] critical fault cleared", R_MODULE_UNRESPONSIVE);
                bms.disable_charge_inhibit("[CF02] critical fault cleared", R_SHUNT_UNRESPONSIVE);
                bms.disable_drive_inhibit("[CF02] critical fault cleared", R_SHUNT_UNRESPONSIVE);
                if ( bms.charge_is_enabled() ) {
                    bms.set_state(&state_charging, "critical fault cleared");
                    break;
                }
                if ( bms.ignition_is_on() ) {
                    bms.set_state(&state_drive, "critical fault cleared");
                    break;
                }
                // See the note in state_drive's E_IGNITION_OFF.
                bms.set_state(&state_standby, "critical fault cleared");
            }
            break;
        case E_DEAD_CELL:
            /* Deliberately does nothing. Note 2 at the top of this file: the
             * inverter contactors are closed in this state, so opening a pack
             * contactor here would break current under load. Dead-cell
             * inhibition is re-evaluated on the way into standby, batteryEmpty
             * or overTempFault, where the contactors may actually be open. */
            break;
        /* Unreachable while every Event value has an explicit case above; kept
         * so that adding an Event without handling it here is reported rather
         * than silently ignored. */
        default:
            bms.increment_invalid_event_count();
            printf("WARNING : invalid event : UNKNOWN while in criticalFault state\n");
    }
}

// Mapping between state functions and their names

typedef struct stateName {
    State state;
    const char * stateName;
} stateName;

static const stateName stateNames[] = {
    {state_standby, "standby"},
    {state_drive, "drive"},
    {state_batteryHeating, "batteryHeating"},
    {state_charging, "charging"},
    {state_batteryEmpty, "batteryEmpty"},
    {state_overTempFault, "overTempFault"},
    {state_illegalStateTransitionFault, "illegalStateTransitionFault"},
    {state_criticalFault, "criticalFault"}
};

// Return the name of the current state
const char* get_state_name(State state) {
    // Derived from the table rather than a hardcoded 8 in two places
    const int stateCount = (int)( sizeof(stateNames) / sizeof(stateNames[0]) );
    for ( int i = 0; i < stateCount; i++ ) {
        if ( state == stateNames[i].state ) {
            return stateNames[i].stateName;
        }
    }
    return "unknownState";
}
