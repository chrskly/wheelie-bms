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


#include "Arduino.h"
#include <ACAN_ESP32.h>
#include <esp_task_wdt.h>

#include "bms.h"
#include "shunt.h"
#include "util.h"
#include "webstatus.h"

#include "settings.h"


/*
 * Perform all health checks.
 */
/*
 * Withdraw inhibit reasons whose underlying condition no longer holds.
 *
 * Reasons are applied by the state handlers, but each was only ever withdrawn by
 * the particular states that knew about it. An exit path that did not know to
 * withdraw one left it latched with no route back: R_TOO_HOT, for instance, was
 * withdrawn only inside state_overTempFault's E_TEMPERATURE_OK branch, so every
 * other way of leaving that state held DRIVE_INHIBIT and CHARGE_INHIBIT on for
 * the rest of the session.
 *
 * Reconciling against live conditions here makes the masks self-healing however
 * a state was left. It runs BEFORE events are dispatched, so a state that still
 * requires an inhibit simply re-asserts it in its own safety block -- being in
 * overTempFault still holds R_TOO_HOT, this only clears it once we have left.
 */
static void reconcile_inhibit_reasons() {
    extern Bms bms;
    extern Battery battery;
    extern Shunt shunt;

    if ( !battery.too_hot() ) {
        bms.disable_drive_inhibit("[RC] not too hot", R_TOO_HOT);
        bms.disable_charge_inhibit("[RC] not too hot", R_TOO_HOT);
    }
    if ( !battery.too_cold_to_charge() ) {
        bms.disable_charge_inhibit("[RC] not too cold", R_TOO_COLD);
    }
    if ( !battery.has_full_cell() ) {
        bms.disable_charge_inhibit("[RC] not full", R_BATTERY_FULL);
    }
    if ( !battery.has_empty_cell() ) {
        bms.disable_drive_inhibit("[RC] not empty", R_BATTERY_EMPTY);
    }
    if ( !bms.charge_is_enabled() ) {
        // Drive-away protection only applies while a charge is actually requested
        bms.disable_drive_inhibit("[RC] no charge requested", R_CHARGING);
    }
    if ( battery.is_alive() ) {
        bms.disable_charge_inhibit("[RC] modules responsive", R_MODULE_UNRESPONSIVE);
        bms.disable_drive_inhibit("[RC] modules responsive", R_MODULE_UNRESPONSIVE);
    }
    if ( !shunt.is_dead() ) {
        bms.disable_charge_inhibit("[RC] shunt responsive", R_SHUNT_UNRESPONSIVE);
        bms.disable_drive_inhibit("[RC] shunt responsive", R_SHUNT_UNRESPONSIVE);
    }
    /* The flag is the condition behind R_ILLEGAL_STATE_TRANSITION, so it has to
     * be cleared here rather than reasoned about: leaving
     * illegalStateTransitionFault via criticalFault (dead module or shunt) left
     * it set, and criticalFault's exits do not clear it, so the reason below
     * could never be withdrawn. */
    if ( bms.get_state() != &state_illegalStateTransitionFault ) {
        bms.clear_illegal_state_transition();
    }
    if ( !bms.get_illegal_state_transition() ) {
        bms.disable_drive_inhibit("[RC] no illegal transition", R_ILLEGAL_STATE_TRANSITION);
        bms.disable_charge_inhibit("[RC] no illegal transition", R_ILLEGAL_STATE_TRANSITION);
    }
    if ( bms.get_state() != &state_criticalFault ) {
        /* Belt and braces: criticalFault withdraws this on its own exits, but an
         * exit path that forgets to would otherwise strand it permanently --
         * exactly the failure this whole pass is about. */
        bms.disable_drive_inhibit("[RC] not in critical fault", R_CRITICAL_FAULT);
        bms.disable_charge_inhibit("[RC] not in critical fault", R_CRITICAL_FAULT);
    }
    if ( !battery.has_dead_cell() ) {
        bms.disable_drive_inhibit("[RC] no dead cell", R_DEAD_CELL);
        bms.disable_charge_inhibit("[RC] no dead cell", R_DEAD_CELL);
    }
    // Per-pack contactor holds need the same treatment, in every state
    battery.release_recovered_dead_cell_packs();
    /* R_STARTUP is deliberately not reconciled here; it is withdrawn below once
     * the battery reports. */
}

static void health_check_callback() {
    extern Bms bms;
    extern Battery battery;
    extern Shunt shunt;

    reconcile_inhibit_reasons();

    /* The pack CAN controllers report their own bus errors, and nothing used
     * to read them. See BatteryPack::check_can_health(). */
    battery.check_pack_can_health();

    // Diagnostic cross-check of the shunt; see Bms::check_shunt_plausibility().
    bms.check_shunt_plausibility();

    /* Weld detection runs every cycle regardless of state. It used to be called
     * only from state_standby, so a contactor that welded during drive or
     * charge was never noticed. The check itself decides when the feedback is
     * meaningful. */
    bms.do_welding_checks();

    // Report stale temperature data, which silently gates several decisions
    if ( battery.temperature_data_is_stale() ) {
        bms.set_internal_error(IE_TEMPERATURE_STALE);
    } else {
        bms.clear_internal_error(IE_TEMPERATURE_STALE);
    }

    /* Dead cell detection. With multiple packs the affected pack's contactors
     * are held open and the rest carries on. With a single pack there is
     * nothing to isolate, so the only safe response is to inhibit outright --
     * previously a dead cell in a single-pack battery was not acted on at all. */
    if ( battery.has_dead_cell() ) {
        if ( battery.has_multiple_packs() ) {
            bms.send_event(E_DEAD_CELL);
        } else {
            bms.enable_drive_inhibit("[HC] dead cell, single pack", R_DEAD_CELL);
            bms.enable_charge_inhibit("[HC] dead cell, single pack", R_DEAD_CELL);
        }
    }

    // Temperature
    if ( battery.too_hot() ) {
        bms.send_event(E_TOO_HOT);
    } else if ( battery.too_cold_to_charge() ) {
        bms.send_event(E_TOO_COLD_TO_CHARGE);
    } else {
        bms.send_event(E_TEMPERATURE_OK);
    }

    // Voltage
    if ( battery.has_empty_cell() ) {
        bms.send_event(E_BATTERY_EMPTY);
    } else if ( battery.has_full_cell() ) {
        bms.send_event(E_BATTERY_FULL);
    } else {
        bms.send_event(E_BATTERY_NOT_EMPTY);
    }

    // Do pack imbalance test (if we have more than one pack)
    if ( battery.has_multiple_packs() ) {
        if ( bms.packs_are_imbalanced() ) {
            bms.send_event(E_PACKS_IMBALANCED);
        } else {
            bms.send_event(E_PACKS_NOT_IMBALANCED);
        }
    }

    /* Withdraw the power-on hold once every module has reported. Until then
     * both inhibits stay asserted no matter what else the state machine does. */
    if ( battery.is_alive() ) {
        bms.disable_charge_inhibit("[HC] battery reporting", R_STARTUP);
        bms.disable_drive_inhibit("[HC] battery reporting", R_STARTUP);
    }

    // Module liveness
    if ( ! battery.is_alive() ) {
        bms.send_event(E_MODULE_UNRESPONSIVE);
    } else {
        bms.send_event(E_MODULES_ALL_RESPONSIVE);
    }

    // Shunt liveness
    if ( shunt.is_dead() ) {
        bms.send_event(E_SHUNT_UNRESPONSIVE);
    } else {
        bms.send_event(E_SHUNT_RESPONSIVE);
    }
}


/*
 * Run recurring calculations
 */
static void calculations_callback() {
    extern Bms bms;
    /* SoC first: get_max_charge_current_by_soc() tapers on it, so computing the
     * charge current before refreshing it always used a one-second-old SoC. */
    bms.recalculate_soc();
    bms.update_max_charge_current();
    bms.update_max_discharge_current();
    // TODO : range estimate
}



//// ----
//
// Outbound message handlers
//
//// ----


/*
 * Send CAN messages to ISA shunt to tell it to reset. Resets the kw/ah
 * counters.
 */
void Bms::send_shunt_reset_message() {
    CANMessage shuntResetFrame;
    zero_frame(&shuntResetFrame);
    shuntResetFrame.id = 0x411;
    shuntResetFrame.data[0] = 0x3F;
    shuntResetFrame.data[1] = 0x00;
    shuntResetFrame.data[2] = 0x00;
    shuntResetFrame.data[3] = 0x00;
    shuntResetFrame.data[4] = 0x00;
    shuntResetFrame.data[5] = 0x00;
    shuntResetFrame.data[6] = 0x00;
    shuntResetFrame.data[7] = 0x00;
    this->send_frame(&shuntResetFrame, false);
}


/*
 * Limits message 0x351
 *
 * Follows the SimpBMS format.
 *
 * byte 0 = Charge voltage LSB, scale 0.1, unit V
 * byte 1 = Charge voltage MSB, scale 0.1, unit V
 * byte 2 = Charge current LSB, scale 0.1, unit A
 * byte 3 = Charge current MSB, scale 0.1, unit A
 * byte 4 = Discharge current LSB, scale 0.1, unit A
 * byte 5 = Discharge current MSB, scale 0.1, unit A
 * byte 6 = Discharge voltage LSB, scale 0.1, unit V
 * byte 7 = Discharge voltage MSB, scale 0.1, unit V
 */

static void send_limits_message_callback() {
    extern Bms bms;
    extern Battery battery;
    /* Recompute immediately before transmitting rather than relying on the
     * cached values. calculations_callback() runs on a different tick offset,
     * so an inhibit asserted just after it left this frame advertising the old
     * non-zero limit for up to a second -- the inhibit pin was already asserted,
     * but an inverter that obeys only the CAN limit would have kept drawing. */
    bms.recalculate_soc();
    bms.update_max_charge_current();
    bms.update_max_discharge_current();
    CANMessage limitsFrame;
    zero_frame(&limitsFrame);
    limitsFrame.id = 0x351;
    /* Battery voltages are held in millivolts; these fields are scaled 0.1 V,
     * so volts*10 == mV/100. Currents are already in whole amps and the fields
     * are scaled 0.1 A, so those are *10. */
    put_u16_le(&limitsFrame, 0, (uint16_t)( battery.get_max_voltage() / 100 ));
    put_u16_le(&limitsFrame, 2, (uint16_t)( bms.get_max_charge_current() * 10 ));
    put_u16_le(&limitsFrame, 4, (uint16_t)( bms.get_max_discharge_current() * 10 ));
    put_u16_le(&limitsFrame, 6, (uint16_t)( battery.get_min_voltage() / 100 ));
    bms.send_frame(&limitsFrame, false);
}



/*
 * BMS state message 0x352
 *
 * Custom message format (not in SimpBMS)
 *
 * byte 0 = bms state
 *   00 = standby
 *   01 = drive
 *   02 = batteryHeating
 *   03 = charging
 *   04 = batteryEmpty
 *   05 = overTempFault
 *   06 = illegalStateTransitionFault
 *   07 = criticalFault
 *   FF = Undefined error
 * byte 1 = error bits
 *   bit 0 = internalError          - something has gone wrong in the BMS
 *   bit 1 = packsImbalanced        - the voltage between two or more packs varies by an unsafe amount
 *   bit 2 = shuntIsDead            - the shunt has not sent a message in SHUNT_TTL_MS milliseconds
 *   bit 3 = illegalStateTransition - We tried to transistion between states in an illegal way
 *   bit 4 = module(s) dead         - one or more modules have not sent a message in MODULE_TTL_MS milliseconds
 *   bit 5 = 
 *   bit 6 = 
 *   bit 7 = 
 * byte 2 = status bits
 *   bit 0 = inhibitCharge
 *   bit 1 = inhibitDrive
 *   bit 2 = heaterEnabled
 *   bit 3 = ignitionOn
 *   bit 4 = chargeEnable
 *   bit 5 = disableRegen
 *   bit 6 =
 *   bit 7 =
 * byte 3 = charge inhibit reason
 *   00 = R_NONE
 *   01 = R_TOO_HOT
 *   02 = R_TOO_COLD
 *   03 = R_BATTERY_FULL
 *   04 = R_BATTERY_EMPTY
 *   05 = R_CHARGING
 *   06 = R_ILLEGAL_STATE_TRANSITION
 * byte 4 = drive inhibit reason
 *   Same mapping as charge inhibit reason
 * byte 5 = welding bits
 *   bit 0 = posContactorWelded   - the positive contactor is welded shut
 *   bit 1 = negContactorWelded   - the negative contactor is welded shut
 *   bit 2 = batt1ContactorWelded - the battery 1 contactor is welded shut
 *   bit 3 = batt2ContactorWelded - the battery 2 contactor is welded shut
 * byte 6 = reboot cause
 *   bit 0 = last reset was caused by a watchdog timeout
 * byte 7 = checksum
 */

static void send_bms_state_message_callback() {
    extern Bms bms;
    CANMessage bmsStateFrame;
    zero_frame(&bmsStateFrame);

    bmsStateFrame.id = 0x352;

    if ( bms.get_state() == &state_standby ) {
        bmsStateFrame.data[0] = 0x00;
    } else if ( bms.get_state() == &state_drive ) {
        bmsStateFrame.data[0] = 0x01;
    } else if ( bms.get_state() == &state_batteryHeating ) {
        bmsStateFrame.data[0] = 0x02;
    } else if ( bms.get_state() == &state_charging ) {
        bmsStateFrame.data[0] = 0x03;
    } else if ( bms.get_state() == &state_batteryEmpty ) {
        bmsStateFrame.data[0] = 0x04;
    } else if ( bms.get_state() == &state_overTempFault ) {
        bmsStateFrame.data[0] = 0x05;
    } else if ( bms.get_state() == &state_illegalStateTransitionFault ) {
        bmsStateFrame.data[0] = 0x06;
    } else if ( bms.get_state() == &state_criticalFault ) {
        bmsStateFrame.data[0] = 0x07;
    } else {
        bmsStateFrame.data[0] = 0xFF;
    }

    bmsStateFrame.data[1] = bms.get_error_byte();
    bmsStateFrame.data[2] = bms.get_status_byte();
    bmsStateFrame.data[3] = bms.get_charge_inhibit_reason();
    bmsStateFrame.data[4] = bms.get_drive_inhibit_reason();
    bmsStateFrame.data[5] = bms.get_welding_byte();
    bmsStateFrame.data[6] = bms.get_watchdog_reboot() ? 0x01 : 0x00;
    // data[7] is the checksum; send_frame() computes it because doChecksum is true
    bms.send_frame(&bmsStateFrame, true);
}



/*
 * Module liveness message 0x353
 *
 * Custom message format (not in SimpBMS)
 *
 * byte 0 = modules 0-7 heartbeat status (0 alive, 1 dead)
 * byte 1 = modules 8-15 hearbeat status (0 alive, 1 dead)
 * byte 2 = modules 16-23 heartbeat status (0 alive, 1 dead)
 * byte 3 = modules 24-31 heartbeat status (0 alive, 1 dead)
 * byte 4 = modules 32-39 heartbeat status (0 alive, 1 dead)
 * byte 5 = invalidEventCounter LSB
 * byte 6 = invalidEventCounter MSB
 * byte 7 = checksum
 */

static void send_module_liveness_message_callback() {
    extern Bms bms;
    extern Battery battery;
    CANMessage moduleLivenessFrame;
    zero_frame(&moduleLivenessFrame);
    moduleLivenessFrame.id = 0x353;
    moduleLivenessFrame.data[0] = battery.get_module_liveness_byte(0);
    moduleLivenessFrame.data[1] = battery.get_module_liveness_byte(8);
    moduleLivenessFrame.data[2] = battery.get_module_liveness_byte(16);
    moduleLivenessFrame.data[3] = battery.get_module_liveness_byte(24);
    moduleLivenessFrame.data[4] = battery.get_module_liveness_byte(32);
    put_u16_le(&moduleLivenessFrame, 5, bms.get_invalid_event_count());
    // data[7] is the checksum; send_frame() computes it because doChecksum is true
    bms.send_frame(&moduleLivenessFrame, true);
}


/*
 * Main CAN bus tx/rx error counters message 0x354
 *
 * Custom message format (not in SimpBMS)
 *
 * byte 0 - 3 = can tx error counters (32bit counter)
 * byte 4 - 7 = can rx error counters (32bit counter)
 */

static void send_main_can_error_counters_message_callback() {
    extern Bms bms;
    CANMessage mainCanErrorCountersFrame;
    zero_frame(&mainCanErrorCountersFrame);
    mainCanErrorCountersFrame.id = 0x354;
    put_u32_le(&mainCanErrorCountersFrame, 0, bms.get_can_tx_error_count());
    put_u32_le(&mainCanErrorCountersFrame, 4, bms.get_can_rx_error_count());
    bms.send_frame(&mainCanErrorCountersFrame, false);
}



/*
 * SoC message 0x355
 *
 * Follows the SimpBMS format.
 *
 * byte 0 = SoC LSB, scale 1, unit %
 * byte 1 = SoC MSB, scale 1, unit %
 * byte 2 = SoH LSB, scale 1, unit %
 * byte 3 = SoH MSB, scale 1, unit %
 * byte 4 = SoC LSB, scale 0.01, unit %
 * byte 5 = SoC MSB, scale 0.01, unit %
 * byte 6 = unused
 * byte 7 = unused
 */

static void send_soc_message_callback() {
    extern Bms bms;
    CANMessage socFrame;
    zero_frame(&socFrame);
    socFrame.id = 0x355;
    put_u16_le(&socFrame, 0, bms.get_soc());                          // SoC, scale 1 %
    put_u16_le(&socFrame, 2, 0);                                      // SoH, not implemented
    put_u16_le(&socFrame, 4, (uint16_t)( bms.get_soc() * 100 ));      // SoC, scale 0.01 %
    socFrame.data[6] = 0x00;                                          // unused
    socFrame.data[7] = 0x00;                                          // unused
    bms.send_frame(&socFrame, false);
}


/*
 * Status message 0x356
 *
 * More or less follows the SimpBMS format.
 *
 * byte 0 = Voltage LSB, scale 0.01, unit V
 * byte 1 = Voltage MSB, scale 0.01, unit V
 * byte 2 = Current LSB, scale 0.1, unit A
 * byte 3 = Current MSB, scale 0.1, unit A
 * byte 4 = Temperature LSB, scale 0.1, unit C
 * byte 5 = Temperature MSB, scale 0.1, unit C
 * byte 6 = Voltage LSB (measured by shunt), scale 0.01, unit V
 * byte 7 = Voltage MSB (measured by shunt), scale 0.01, unit V
 */

static void send_status_message_callback() {
    extern Bms bms;
    extern Battery battery;
    extern Shunt shunt;
    CANMessage statusFrame;
    zero_frame(&statusFrame);
    statusFrame.id = 0x356;
    /* Scalings, with current and temperature signed:
     *   voltage       field 0.01 V, battery voltage is mV       -> mV / 10
     *   current       field 0.1 A,  shunt amps are mA           -> mA / 100
     *   temperature   field 0.1 C,  sensors report whole C      -> C * 10
     *   shunt voltage field 0.01 V, shunt voltage1 is mV        -> mV / 10 */
    put_u16_le(&statusFrame, 0, (uint16_t)( battery.get_voltage() / 10 ));
    put_i16_le(&statusFrame, 2, (int16_t)( shunt.get_amps() / 100 ));
    /* With no module reporting, get_highest_sensor_temperature() is the -126
     * sentinel, and this used to broadcast it as a -126.0 C battery temperature.
     * The field has no "invalid" encoding, so send the coldest value a real
     * sensor can produce instead: that keeps the conservative direction (a
     * charger reading it will not charge) without putting an impossible number
     * on the bus. Invalidity itself is already signalled -- too_cold_to_charge()
     * is true while the data is stale, so 0x351 is already advertising 0 A, and
     * IE_TEMPERATURE_STALE raises the general alarm in 0x35A. */
    const int16_t reportedTemperature = battery.have_temperature_reading()
        ? (int16_t)( battery.get_highest_sensor_temperature() * 10 )
        : (int16_t)( MODULE_SENSOR_MINIMUM_C * 10 );
    put_i16_le(&statusFrame, 4, reportedTemperature);
    put_u16_le(&statusFrame, 6, (uint16_t)( shunt.get_voltage1() / 10 ));
    bms.send_frame(&statusFrame, false);
}


/*
 * Pack CAN bus tx/rx error counters message 0x357
 *
 * Custom message format (not in SimpBMS)
 *
 * byte 0 - 1 = pack 0 can tx error counters (16bit counter)
 * byte 2 - 3 = pack 0 can rx error counters (16bit counter)
 * byte 4 - 5 = pack 1 can tx error counters (16bit counter)
 * byte 6 - 7 = pack 1 can rx error counters (16bit counter)
 */

static void send_pack_can_error_counters_message_callback() {
    extern Bms bms;
    extern Battery battery;
    CANMessage packCanErrorCountersFrame;
    zero_frame(&packCanErrorCountersFrame);
    packCanErrorCountersFrame.id = 0x357;
    /* Two pairs of counters, but only NUM_PACKS packs exist. Pack 1 used to be
     * read unconditionally, which indexes one past the end of Battery::packs[]
     * in a single-pack build -- the same defect as the welding byte, but this
     * one slipped past the sanitizers because the index arrives as a runtime
     * parameter into a member array rather than as a constant. Unused slots
     * stay zero, which zero_frame() has already written. */
    /* The frame has room for two pairs; a battery may have fewer packs. Written
     * as a preprocessor choice rather than a ternary so neither configuration
     * compiles a comparison whose operands are identical. */
#if NUM_PACKS < 2
    const int packsInFrame = NUM_PACKS;
#else
    const int packsInFrame = 2;
#endif
    for ( int p = 0; p < packsInFrame; p++ ) {
        put_u16_le(&packCanErrorCountersFrame, p * 4,
                   battery.get_can_tx_error_count_for_pack(p));
        put_u16_le(&packCanErrorCountersFrame, p * 4 + 2,
                   battery.get_can_rx_error_count_for_pack(p));
    }
    bms.send_frame(&packCanErrorCountersFrame, false);
}


/*
 * Alarms message 0x35A
 *
 * Mostly follows the SimpBMS format, with a couple of variations. Also used the
 * victron format for some of the bits.
 *
 * First 4 bytes are alarms, second 4 bytes are warnings.
 *
 * byte 0
 *   bit 0 = general alarm
 *   bit 2 = high cell alarm
 *   bit 4 = low cell alarm
 *   bit 6 = high temp alarm
 * byte 1
 *   bit 0 = low temp alarm
 *   bit 2 = high temp charge alarm
 *   bit 4 = low temp charge alarm
 *   bit 6 = high current alarm
 * byte 2
 *   bit 0 = high charge current alarm
 *   bit 2 = contactor on
 *   bit 4 = short circuit alarm
 *   bit 6 = internal error
 * byte 3
 *   bit 0 = cell delta alarm
 * byte 4
 *   bit 0 = general warn
 *   bit 2 = high cell warn
 *   bit 4 = low cell warn
 *   bit 6 = high temp warn
 * byte 5
 *   bit 0 = low temp warn
 *   bit 2 = high temp charge warn
 *   bit 4 = low temp charge warn
 *   bit 6 = high current warn
 * byte 6
 *   bit 0 = high charge current warn
 *   bit 2 = contactor on
 *   bit 4 = short circuit warn
 *   bit 6 = internal error
 * byte 7
 *   bit 0 = cell delta warn
 *
 * NB: byte 7 carries the cell-delta warning, NOT a checksum -- this frame is
 * sent with doChecksum = false. Each alarm occupies a 2-bit field (Victron
 * convention), and 0b01 means "active", so the masks below are 0x01, 0x04,
 * 0x10 and 0x40 for bits 0, 2, 4 and 6 respectively.
 */

/*
 * One alarm or warning field: two bits, per the CAN-bus BMS convention these
 * ids follow.
 *
 *   01  active
 *   10  explicitly not active
 *   00  not evaluated by this firmware
 *
 * Every field used to be written as a single bit set when active and left at 00
 * otherwise, which conflates "no alarm" with "this BMS does not check". Victron
 * tolerates that -- it only looks for the active bit -- but a stricter receiver
 * reads 00 as unknown, so a pack with nothing wrong reported nothing known.
 * The fields this firmware genuinely does not compute are still left at 00,
 * which is now a true statement rather than an accident.
 */
static void set_alarm_field(CANMessage* frame, int byteIndex, int fieldIndex, bool active) {
    frame->data[byteIndex] |= (uint8_t)( ( active ? 0x01u : 0x02u ) << ( fieldIndex * 2 ) );
}

/*
 * 0x35F -- battery model, firmware version and nameplate capacity.
 *
 * The version was previously reported nowhere at all, so a BMS on the bench and
 * one in the car were indistinguishable. Layout follows the same CAN-bus BMS
 * convention as the other 0x35x ids: model id, firmware version, then online
 * capacity in whole amp-hours.
 */
static void send_battery_info_message_callback() {
    extern Bms bms;
    CANMessage infoFrame;
    zero_frame(&infoFrame);
    infoFrame.id = 0x35F;
    put_u16_le(&infoFrame, 0, (uint16_t)BATTERY_MODEL_ID);
    put_u16_le(&infoFrame, 2, (uint16_t)VERSION_U16);
    put_u16_le(&infoFrame, 4, (uint16_t)( BATTERY_CAPACITY_AS / 3600 ));
    bms.send_frame(&infoFrame, false);
}

static void send_alarm_message_callback() {
    extern Bms bms;
    extern Battery battery;
    CANMessage alarmFrame;
    zero_frame(&alarmFrame);
    alarmFrame.id = 0x35A;

    const bool contactorsClosed = bms.charge_is_enabled() || bms.ignition_is_on();

    // ---- bytes 0-3: alarms ----
    set_alarm_field(&alarmFrame, 0, 0, bms.get_internal_error());     // general
    set_alarm_field(&alarmFrame, 0, 1, battery.has_full_cell());      // overvolt
    set_alarm_field(&alarmFrame, 0, 2, battery.has_empty_cell());     // undervolt
    set_alarm_field(&alarmFrame, 0, 3, battery.too_hot());            // high temp

    set_alarm_field(&alarmFrame, 1, 0, battery.too_cold_to_charge()); // low temp
    set_alarm_field(&alarmFrame, 1, 1, battery.too_hot());            // high temp, charge
    set_alarm_field(&alarmFrame, 1, 2, battery.too_cold_to_charge()); // low temp, charge
    // byte 1 field 3: high current -- not evaluated, left at 00

    // byte 2 field 0: high charge current -- not evaluated, left at 00
    set_alarm_field(&alarmFrame, 2, 1, contactorsClosed);             // contactor
    // byte 2 field 2: short circuit -- not evaluated, left at 00
    set_alarm_field(&alarmFrame, 2, 3, bms.get_internal_error());     // internal error

    set_alarm_field(&alarmFrame, 3, 0, battery.cell_delta_above_alarm());

    // ---- bytes 4-7: warnings, same layout ----
    // byte 4 field 0: general warning -- not evaluated, left at 00
    set_alarm_field(&alarmFrame, 4, 1, battery.has_full_cell());
    set_alarm_field(&alarmFrame, 4, 2, battery.has_empty_cell());
    set_alarm_field(&alarmFrame, 4, 3, battery.too_hot());

    set_alarm_field(&alarmFrame, 5, 0, battery.too_cold_to_charge());
    set_alarm_field(&alarmFrame, 5, 1, battery.too_hot());
    set_alarm_field(&alarmFrame, 5, 2, battery.too_cold_to_charge());
    // byte 5 field 3: high current warning -- not evaluated, left at 00

    // byte 6 field 0: high charge current warning -- not evaluated, left at 00
    set_alarm_field(&alarmFrame, 6, 1, contactorsClosed);
    // byte 6 field 2: short circuit warning -- not evaluated, left at 00
    set_alarm_field(&alarmFrame, 6, 3, bms.get_internal_error());

    set_alarm_field(&alarmFrame, 7, 0, battery.cell_delta_above_warn());

    bms.send_frame(&alarmFrame, false);
}



//// ----
//
// Inbound message handlers
//
//// ----


// Handle messages coming in on the main CAN bus

/*
 * Assemble the 32-bit value an ISA shunt frame carries in bytes 2..5.
 *
 * Each byte is widened to uint32_t before shifting. Previously these were
 * written as `m.data[5] << 24`: data[5] is a uint8_t that promotes to int, so
 * for values >= 0x80 the result exceeds INT_MAX. That is UB under strict
 * C++11, but this project builds as gnu++14, where DR1457 makes it well
 * defined (the value is computed in the corresponding unsigned type, then
 * converted to int -- implementation-defined, two's complement on gcc). So
 * this was not a live miscompilation risk; widening simply removes the
 * reliance on that conversion and states the intent.
 *
 * Byte order: data[5] is the most significant byte, data[2] the least. This was
 * checked against two independent implementations of the same device (the
 * Stm32-vcu ISA driver and its round-trip tests, which pack `value & 0xFF` into
 * byte 2 and `value >> 24` into byte 5). An earlier review claimed this frame
 * was big-endian and that the order here was reversed; that was wrong, and the
 * order below is correct as-is.
 */
/* The reading occupies bytes 2-5, so a frame shorter than 6 bytes does not
 * contain one. Without this check a truncated or spurious frame on one of the
 * shunt ids was decoded from whatever those bytes happened to hold and stored
 * as a real measurement -- and, worse, the caller's heartbeat() then kept the
 * shunt marked alive on the strength of it. */
static const int SHUNT_PAYLOAD_BYTES = 6;

static bool shunt_frame_is_complete(const CANMessage& m) {
    return m.len >= SHUNT_PAYLOAD_BYTES;
}

static int32_t shunt_payload(const CANMessage& m) {
    return (int32_t)( ((uint32_t)m.data[5] << 24)
                    | ((uint32_t)m.data[4] << 16)
                    | ((uint32_t)m.data[3] <<  8)
                    | ((uint32_t)m.data[2]) );
}


static void handle_main_CAN_messages_callback() {
    CANMessage m;
    extern Shunt shunt;
    extern Bms bms;

    /* B157: canRxErrorCount had no increment anywhere and always transmitted 0.
     * statusFlags() bit 0 is a hardware receive FIFO overflow and bit 1 a
     * driver receive FIFO overflow, which are genuine dropped-frame events.
     * Counted on the rising edge only, since the flags are level-based.
     * Bus-off (bit 2) is also recovered from here -- otherwise the controller
     * stays off the bus permanently and the BMS goes silently deaf. */
    static uint32_t previousStatusFlags = 0;
    const uint32_t statusFlags = ACAN_ESP32::can.statusFlags();
    const uint32_t newFlags = statusFlags & ~previousStatusFlags;
    previousStatusFlags = statusFlags;
    if ( newFlags & 0x03 ) {
        bms.increment_can_rx_error_count();
    }
    if ( newFlags & 0x04 ) {
        printf("[bms] main CAN bus-off, attempting recovery\n");
        ACAN_ESP32::can.recoverFromBusOff();
    }
    if ( bms.read_frame(&m) ) {
        /* Every id handled below is an ISA shunt reading in bytes 2-5. */
        if ( m.id >= 0x521 && m.id <= 0x528 && !shunt_frame_is_complete(m) ) {
            bms.increment_can_rx_error_count();
            return;
        }
        switch ( m.id ) {
            // ISA shunt amps
            case 0x521:
                shunt.set_amps( shunt_payload(m) );  // milliamps
                shunt.heartbeat();
                break;
            // ISA shunt voltage 1
            case 0x522:
                shunt.set_voltage1( shunt_payload(m) );  // millivolts
                shunt.heartbeat();
                break;
            // ISA shunt voltage 2
            case 0x523:
                shunt.set_voltage2( shunt_payload(m) );  // millivolts
                shunt.heartbeat();
                break;
            // ISA shunt voltage 3
            case 0x524:
                shunt.set_voltage3( shunt_payload(m) );  // millivolts
                shunt.heartbeat();
                break;
            // ISA shunt temperature
            case 0x525:
                shunt.set_temperature( shunt_payload(m) / 10 );
                shunt.heartbeat();
                break;
            // ISA shunt power (raw watts; /1000 would be kW)
            case 0x526:
                shunt.set_watts( shunt_payload(m) );  // watts
                shunt.heartbeat();
                break;
            // ISA shunt charge counter, in amp-seconds (raw/3600 would be Ah)
            case 0x527:
                shunt.set_ampSeconds( shunt_payload(m) );
                shunt.heartbeat();
                break;
            // ISA shunt energy counter, in watt-hours (raw/1000 would be kWh)
            case 0x528:
                shunt.set_wattHours( shunt_payload(m) );
                shunt.heartbeat();
                break;
            default:
                break;
        }
    }
}




void Bms::init(Battery* _battery, Io* _io, Shunt* _shunt) {
    battery = _battery;
    state = &state_standby;
    stateEnteredAt = get_clock_ms();
    io = _io;
    shunt = _shunt;
    internalErrorFlags = 0;
    /* init() is a genuine reset, not just a constructor helper: everything the
     * object accumulates at runtime goes back to its power-on value here. The
     * counters below used to survive it, which made init() a partial reset and
     * left stale diagnostics reachable through the web snapshot after a
     * re-init. */
    invalidEventCounter = 0;
    shuntImplausibleSince = 0;
    illegalStateTransition = false;
    canTxErrorCount = 0;
    canRxErrorCount = 0;
    statusLight = StatusLight();
    /* init() assigns `state` directly rather than going through set_state(), so
     * select the matching blink pattern explicitly -- otherwise both durations
     * stay at their defaults of 0 and the light sits solid on. */
    statusLight.set_mode(STANDBY);
    /* Io::init() physically asserts both inhibits as the fail-safe power-on
     * state. Seed the reason masks to match: with empty masks the first
     * disable_*_inhibit() call of the first health check would find nothing
     * holding the inhibit and release it, dropping the fail-safe state before
     * a single cell voltage had been read. Withdrawn once the battery reports. */
    chargeInhibitReasons = inhibit_reason_bit(R_STARTUP);
    driveInhibitReasons = inhibit_reason_bit(R_STARTUP);

    printf("[bms][init] setting up main CAN port\n");
    ACAN_ESP32_Settings settings(500 * 1000);
    settings.mRxPin = (gpio_num_t)MAIN_CAN_RX_PIN;
    settings.mTxPin = (gpio_num_t)MAIN_CAN_TX_PIN;
    const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
    if ( errorCode == 0 ) {
        printf("[bms][init] main CAN port initialized successfully\n");
    } else {
        printf("[bms][init] WARNING problem initializing main CAN port : %u\n", (unsigned int)errorCode);
    }

#if CAN_SELF_TEST_AT_INIT
    printf("[bms][init] sending 5 test messages\n");
    for ( int i = 0; i < 5; i++ ) {
        CANMessage m;
        m.id = 0x100 + i;
        m.len = 8;
        for ( int j = 0; j < 8; j++ ) {
            m.data[j] = j;
        }
        this->send_frame(&m, true);
    }
#endif

}

/*
 * The single task that runs all periodic BMS work.
 *
 * Everything used to be a FreeRTOS software timer, which meant every callback
 * ran on the shared timer service task -- a small stack shared with anything
 * else in the system that uses timers, where one blocking SPI transaction
 * delays every other callback. Worse, the ignition and charge-enable
 * interrupts dispatched state machine events from interrupt context, so the
 * state machine could be re-entered concurrently with a timer callback already
 * inside it.
 *
 * Now there is exactly one task, with a known stack, and it is the only place
 * that enters the state machine or touches battery state. Work is staggered
 * across the 5 ms tick so no single tick does everything at once.
 */
/* The tick phase lives outside the task function rather than as a local.
 * On the target this is equivalent -- the task is created once and its loop
 * never exits -- but the native test harness stops the worker by unwinding out
 * of it and re-enters at the top on the next call, which reset a local `tick`
 * to 0 every time. Everything scheduled off a `tick % N` offset (the health
 * check at 10, the periodic CAN sends, the web snapshot) then never fired
 * unless a single call happened to run long enough to reach its offset, so
 * short simulated intervals silently ran no health checks at all.
 * Bms::start() resets it, so each simulated boot still begins at phase 0. */
static uint32_t workerTick = 0;

static void bms_worker_task(void* /*pvParameters*/) {
    extern Bms bms;
    extern Battery battery;
    extern Io io;

    TickType_t lastWake = xTaskGetTickCount();

#if WATCHDOG_TIMEOUT_S > 0
    if ( esp_task_wdt_add(NULL) != ESP_OK ) {
        printf("[bms] WARNING could not subscribe the worker task to the watchdog\n");
    }
#endif

    for ( ;; ) {
        // Every 5 ms: drain both CAN buses
        battery.read_message();
        handle_main_CAN_messages_callback();

        /* Sample the debounced inputs every IO_POLL_INTERVAL_MS. The divisor was
         * written out as 2, so IO_POLL_INTERVAL_MS was documentation that
         * nothing read: changing it moved the debounce time quoted in
         * settings.h without moving the rate inputs were actually sampled at. */
        if ( ( workerTick % IO_POLL_TICKS ) == 0 ) {
            io.poll_inputs();
        }

        // Every 100 ms
        /* request_data() sends ONE module poll per call, so it is called often
         * enough to complete a sweep in about 90ms: 6 modules x 3 ticks x 5ms. */
        if ( ( workerTick % 3 ) == 0 ) { battery.request_data(); }
        if ( ( workerTick % 20 ) == 10 ) { health_check_callback(); }
        if ( ( workerTick % 20 ) ==  5 ) { bms.led_blink(); }

        // Every 1 s, staggered so the tick that builds one frame builds only that one
        if ( ( workerTick % 200 ) ==   0 ) { send_limits_message_callback(); }
        if ( ( workerTick % 200 ) ==  20 ) { send_bms_state_message_callback(); }
        if ( ( workerTick % 200 ) ==  40 ) { send_main_can_error_counters_message_callback(); }
        if ( ( workerTick % 200 ) ==  60 ) { send_pack_can_error_counters_message_callback(); }
        if ( ( workerTick % 200 ) ==  80 ) { send_soc_message_callback(); }
        if ( ( workerTick % 200 ) == 100 ) { send_status_message_callback(); }
        if ( ( workerTick % 200 ) == 120 ) { send_alarm_message_callback(); }
        if ( ( workerTick % 200 ) == 140 ) { calculations_callback(); }

        // Every 5 s
        if ( ( workerTick % 1000 ) == 160 ) { send_module_liveness_message_callback(); }
        /* Model, firmware version and nameplate capacity never change, so 5 s
         * is ample. Offset 360 so it shares a tick with nothing else. */
        if ( ( workerTick % 1000 ) == 360 ) { send_battery_info_message_callback(); }

#if WEB_INTERFACE_ENABLED
        /* Refresh what the web interface serves. Fires on the LAST tick of each
         * window rather than a fixed offset, so it stays staggered away from
         * the CAN sends above whatever WEB_SNAPSHOT_INTERVAL_MS is set to. */
        if ( ( workerTick % WEB_SNAPSHOT_TICKS ) == ( WEB_SNAPSHOT_TICKS - 1 ) ) {
            bms.publish_web_snapshot();
        }
#endif

#if STATUS_PRINT_INTERVAL_MS > 0
        if ( ( workerTick % (STATUS_PRINT_INTERVAL_MS / BMS_WORKER_TICK_MS) ) == 180 ) {
            bms.print();
        }
#endif

        // Tell the watchdog we are still making progress
#if WATCHDOG_TIMEOUT_S > 0
        esp_task_wdt_reset();
#endif
        workerTick++;
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BMS_WORKER_TICK_MS));
    }
}

void Bms::start() {
#if WATCHDOG_TIMEOUT_S > 0
    /* The pico build had a watchdog; the port left it entirely commented out,
     * so a hung worker task would simply stop the BMS with the contactors in
     * whatever state they were last left. panic = true resets the chip. */
    if ( esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true) != ESP_OK ) {
        printf("[bms][start] WARNING could not configure the task watchdog\n");
    }
#endif
    workerTick = 0;
    printf("[bms][start] starting BMS worker task\n");
    const BaseType_t created = xTaskCreate(
        bms_worker_task,
        "bmsWorker",
        BMS_WORKER_STACK_BYTES,
        NULL,
        BMS_WORKER_PRIORITY,
        NULL);
    if ( created != pdPASS ) {
        printf("[bms][start] ERROR could not create the BMS worker task\n");
    }
}

void Bms::set_state(State newState, const char* reason) {
    stateEnteredAt = get_clock_ms();
    printf("[bms][set_state] switching from state %s to state %s, reason : %s\n",
           get_state_name(state), get_state_name(newState), reason);
    state = newState;
    // Change light blinking pattern based on state
    /* Written as a single assignment rather than a chain of branches that
     * mostly set FAULT -- the chain tripped -Wduplicated-branches, which is a
     * warning worth keeping sensitive. */
    LED_MODE mode = FAULT;
    if ( state == state_standby ) {
        mode = STANDBY;
    } else if ( state == state_drive ) {
        mode = DRIVE;
    } else if ( state == state_batteryHeating || state == state_charging ) {
        mode = CHARGING;
    }
    statusLight.set_mode(mode);
}

State Bms::get_state() {
    return state;
}

uint64_t Bms::time_in_state_ms() {
    return get_clock_ms() - stateEnteredAt;
}

void Bms::send_event(Event event) {
    /* `state` is null until init() runs. Nothing should reach here before then
     * (timers and interrupts are both started afterwards), but dispatching
     * through a null function pointer is not a failure mode worth risking. */
    if ( state == nullptr ) {
        printf("[bms][send_event] WARNING event %d dropped, state machine not started\n", (int)event);
        return;
    }
    state(event);
}

void Bms::publish_web_snapshot() {
    /* Static, not a local: WebSnapshot is the best part of a kilobyte and the
     * worker task's stack is sized for CAN work, not for a second copy of the
     * whole battery. Only ever touched by the worker task. */
    static WebSnapshot snapshot;

    snapshot.uptimeMs = get_clock_ms();
    snapshot.stateName = get_state_name(state);
    snapshot.timeInStateMs = time_in_state_ms();
    snapshot.illegalStateTransition = illegalStateTransition;
    snapshot.invalidEventCount = invalidEventCounter;
    snapshot.watchdogReboot = watchdogReboot;

    snapshot.driveInhibited = drive_is_inhibited();
    snapshot.driveInhibitReasons = driveInhibitReasons;
    snapshot.chargeInhibited = charge_is_inhibited();
    snapshot.chargeInhibitReasons = chargeInhibitReasons;
    snapshot.heaterOn = heater_is_enabled();
    snapshot.ignitionOn = ignition_is_on();
    snapshot.chargeEnabled = charge_is_enabled();

    snapshot.soc = soc;
    snapshot.maxChargeCurrent = maxChargeCurrent;
    snapshot.maxDischargeCurrent = maxDischargeCurrent;

    snapshot.internalErrorFlags = internalErrorFlags;
    snapshot.errorByte = get_error_byte();
    snapshot.statusByte = get_status_byte();
    snapshot.weldingByte = get_welding_byte();

    snapshot.shuntAlive = !shunt->is_dead();
    snapshot.shuntAmps = shunt->get_amps();
    snapshot.shuntVoltage1 = shunt->get_voltage1();
    snapshot.shuntVoltage2 = shunt->get_voltage2();
    snapshot.shuntVoltage3 = shunt->get_voltage3();
    snapshot.shuntTemperature = shunt->get_temperature();
    snapshot.shuntWatts = shunt->get_watts();
    snapshot.shuntAmpSeconds = shunt->get_ampSeconds();
    snapshot.shuntWattHours = shunt->get_wattHours();

    snapshot.mainCanTxErrors = canTxErrorCount;
    snapshot.mainCanRxErrors = canRxErrorCount;

    battery->fill_snapshot(snapshot);

    webstatus_publish(snapshot);
}

void Bms::print() {
    const char* chg_inh = io->charge_is_inhibited() ? "true" : "false";
    const char* drv_inh = io->drive_is_inhibited() ? "true" : "false";
    const char* ign = io->ignition_is_on() ? "true" : "false";
    const char* chg_en = io->charge_enable_is_on() ? "true" : "false";
    int8_t Tmax = battery->get_highest_sensor_temperature();
    int8_t Tmin = battery->get_lowest_sensor_temperature();
    int16_t Vmax = battery->get_highest_cell_voltage();
    int16_t Vmin = battery->get_lowest_cell_voltage();
    printf("State:%s, SoC:%d, DRV_INH:%s, CHG_INH:%s, IGN:%s, CHG_EN:%s\n",
        get_state_name(get_state()), soc, drv_inh, chg_inh, ign, chg_en);
    printf(" V:%u, VMax:%d, VMin:%d\n", (unsigned int)(battery->get_voltage()/1000), Vmax, Vmin );
    printf(" TMax:%d, TMin:%d\n", Tmax, Tmin );
#if STATUS_PRINT_CELL_DETAIL
    battery->print();
#endif
}

// Watchdog

void Bms::set_watchdog_reboot(bool value) {
    watchdogReboot = value;
}

// DRIVE_INHIBIT

/* Reasons are reported most-severe-first, so the single byte in the state
 * message names the most important thing currently holding the inhibit on. */
static const InhibitReason kReasonsBySeverity[] = {
    R_CRITICAL_FAULT, R_MODULE_UNRESPONSIVE, R_SHUNT_UNRESPONSIVE, R_DEAD_CELL,
    R_ILLEGAL_STATE_TRANSITION, R_STARTUP, R_TOO_HOT, R_TOO_COLD,
    R_BATTERY_EMPTY, R_BATTERY_FULL, R_CHARGING,
};

/* Every InhibitReason except R_NONE must appear above. A reason missing from
 * the table is not a compile error and not visibly wrong in operation: the
 * inhibit is still applied, but bytes 3 and 4 of 0x352 report R_NONE for it,
 * which reads on the bus as "not inhibited". */
static_assert(sizeof(kReasonsBySeverity) / sizeof(kReasonsBySeverity[0])
              == (size_t)R_STARTUP,
    "kReasonsBySeverity must list every InhibitReason except R_NONE; "
    "R_STARTUP is the highest-numbered reason and doubles as the count");

static int8_t most_severe_reason(uint16_t mask) {
    for ( unsigned i = 0; i < sizeof(kReasonsBySeverity)/sizeof(kReasonsBySeverity[0]); i++ ) {
        if ( mask & inhibit_reason_bit(kReasonsBySeverity[i]) ) {
            return (int8_t)kReasonsBySeverity[i];
        }
    }
    return (int8_t)R_NONE;
}

/* Asserting an inhibit zeroes the advertised limit in the same breath.
 *
 * The limits are a cache refreshed once a second. Zeroing here means nothing can
 * ever observe "inhibited, but you may draw 300 A" in the window between the
 * decision and the next refresh -- not the 0x351 frame, not the web snapshot,
 * not anything added later. Withdrawing an inhibit deliberately does NOT restore
 * a limit here: that waits for the next recalculation, so the permissive
 * direction is always the considered one. */
void Bms::enable_drive_inhibit(const char* context, InhibitReason reason) {
    driveInhibitReasons |= inhibit_reason_bit(reason);
    maxDischargeCurrent = 0;
    io->enable_drive_inhibit(context);
}

void Bms::disable_drive_inhibit(const char* context, InhibitReason reason) {
    driveInhibitReasons &= (uint16_t)~inhibit_reason_bit(reason);
    if ( driveInhibitReasons == 0 ) {
        io->disable_drive_inhibit(context);
    }
}


bool Bms::drive_is_inhibited() {
    return io->drive_is_inhibited();
}

int8_t Bms::get_drive_inhibit_reason() {
    return most_severe_reason(driveInhibitReasons);
}

// CHARGE_INHIBIT

// See enable_drive_inhibit() for why the limit is zeroed here.
void Bms::enable_charge_inhibit(const char* context, InhibitReason reason) {
    chargeInhibitReasons |= inhibit_reason_bit(reason);
    maxChargeCurrent = 0;
    io->enable_charge_inhibit(context);
}

void Bms::disable_charge_inhibit(const char* context, InhibitReason reason) {
    chargeInhibitReasons &= (uint16_t)~inhibit_reason_bit(reason);
    if ( chargeInhibitReasons == 0 ) {
        io->disable_charge_inhibit(context);
    }
}


bool Bms::charge_is_inhibited() {
    return io->charge_is_inhibited();
}

int8_t Bms::get_charge_inhibit_reason() {
    return most_severe_reason(chargeInhibitReasons);
}

// HEATER

void Bms::enable_heater() {
    io->enable_heater();
}

void Bms::disable_heater() {
    io->disable_heater();
}

bool Bms::heater_is_enabled() {
    return io->heater_is_enabled();
}

// IGNITION

bool Bms::ignition_is_on() {
    return io->ignition_is_on();
}

// CHARGE_ENABLE

bool Bms::charge_is_enabled() {
    return io->charge_enable_is_on();
}

// SoC

uint8_t Bms::get_soc() {
    return soc;
}

/*
 * Recalculate the SoC based on the latest data from the ISA shunt.
 *
 * 0 khw/ah == 100% charged. Value goes negative as we draw energy from the pack.
 */
/*
 * Cross-check the shunt against itself and against what the BMS believes it is
 * doing. Diagnostic only: this raises an internal error and nothing else, and
 * never gates a contactor, a limit or a state transition.
 *
 * WHY THIS EXISTS. The sign convention on 0x356 is not something this firmware
 * can verify from the bench. It is pinned by recalculate_soc(), which treats
 * the shunt's amp-second counter as reading 0 at full and going negative as
 * energy is drawn; that counter is the integral of current, so current must be
 * negative on discharge and positive on charge -- which is exactly what the
 * 0x356 current field wants, so the reading is passed through unscaled in sign.
 * That chain is sound but it rests on the device behaving as its documentation
 * says, and a shunt wired backwards would satisfy every internal consistency
 * check while reporting the opposite of the truth. Charging is the one case
 * where the BMS independently knows the answer, so that is what is checked.
 *
 * Two contradictions are looked for:
 *   1. Charging with a sustained current flowing the wrong way. A shunt fitted
 *      or decoded backwards shows up here and nowhere else.
 *   2. Reported power disagreeing with current x voltage. This catches one of
 *      the three readings being decoded wrongly while the others are fine.
 */
void Bms::check_shunt_plausibility() {
    if ( shunt->is_dead() ) {
        shuntImplausibleSince = 0;
        clear_internal_error(IE_SHUNT_IMPLAUSIBLE);
        return;
    }

    const int32_t amps = shunt->get_amps();
    const int32_t millivolts = shunt->get_voltage1();
    bool contradiction = false;

    /* 1. Direction, while the BMS knows a charger is pushing current in. */
    if ( state == state_charging && charge_is_enabled()
         && amps < -SHUNT_SIGNIFICANT_CURRENT_MA ) {
        contradiction = true;
    }

    /* 2. Power against current x voltage. Both sides in watts; the shunt
     *    reports milliamps and millivolts, so the product needs 1e6. Compared
     *    with a wide tolerance because these are three independent samples
     *    taken at slightly different moments, not a simultaneous triple. */
    if ( millivolts > 0 && ( amps > SHUNT_SIGNIFICANT_CURRENT_MA
                          || amps < -SHUNT_SIGNIFICANT_CURRENT_MA ) ) {
        const int64_t expectedWatts =
            ( (int64_t)amps * (int64_t)millivolts ) / 1000000;
        const int64_t reportedWatts = (int64_t)shunt->get_watts();
        const int64_t difference = ( reportedWatts > expectedWatts )
                                 ? ( reportedWatts - expectedWatts )
                                 : ( expectedWatts - reportedWatts );
        const int64_t magnitude = ( expectedWatts < 0 ) ? -expectedWatts : expectedWatts;
        if ( difference > magnitude / 2 + 50 ) {
            contradiction = true;
        }
    }

    if ( !contradiction ) {
        shuntImplausibleSince = 0;
        clear_internal_error(IE_SHUNT_IMPLAUSIBLE);
        return;
    }
    if ( shuntImplausibleSince == 0 ) {
        shuntImplausibleSince = get_clock_ms();
        return;
    }
    if ( ( get_clock_ms() - shuntImplausibleSince ) >= SHUNT_IMPLAUSIBLE_MS ) {
        if ( !has_internal_error(IE_SHUNT_IMPLAUSIBLE) ) {
            printf("[bms] WARNING shunt readings contradict the BMS state: "
                   "%d mA, %d mV, %d W while charging -- check the shunt wiring "
                   "and the 0x356 current sign\n",
                   (int)amps, (int)millivolts, (int)shunt->get_watts());
        }
        set_internal_error(IE_SHUNT_IMPLAUSIBLE);
    }
}

void Bms::recalculate_soc() {
    /* The shunt counter reads 0 at full and goes negative as energy is drawn,
     * so remaining = capacity + counter. Computed in 64-bit and clamped: the
     * old expression assigned an unclamped signed result straight into a
     * uint8_t, so a counter below -capacity wrapped round to a large SoC. */
    int64_t capacity;
    int64_t remaining;
    if ( CALCULATE_SOC_FROM_AMP_SECONDS == 1 ) {
        capacity  = (int64_t)BATTERY_CAPACITY_AS;
        remaining = capacity + (int64_t)shunt->get_ampSeconds();
    } else {
        capacity  = (int64_t)BATTERY_CAPACITY_WH;
        remaining = capacity + (int64_t)shunt->get_wattHours();
    }
    int64_t percent = ( 100 * remaining ) / capacity;
    if ( percent < 0 )   { percent = 0; }
    if ( percent > 100 ) { percent = 100; }
    soc = (uint8_t)percent;
}

// Error

void Bms::set_internal_error(InternalErrorSource source) {
    internalErrorFlags |= (uint8_t)source;
}

void Bms::clear_internal_error(InternalErrorSource source) {
    internalErrorFlags &= (uint8_t)~source;
}

// Combine error bits into error byte to send out in status CAN message
/* NB: uses Bms::packs_are_imbalanced() (the debounced test the state machine
 * acts on), not Battery's instantaneous one -- the reported bit used to
 * disagree with the behaviour it was supposed to describe. */
uint8_t Bms::get_error_byte() {
    return (uint8_t)(
        0x00 | \
        (internalErrorFlags != 0) | \
        packs_are_imbalanced() << 1 | \
        shunt->is_dead() << 2 | \
        illegalStateTransition << 3 | \
        ( ! battery->is_alive() ) << 4 | \
        /* Bit 5: a dead cell. It was reported nowhere on the bus. The undervolt
         * alarm does fire alongside it, because DEAD_CELL_VOLTAGE is below
         * CELL_EMPTY_VOLTAGE, but that is also what a merely flat battery looks
         * like -- and a flat battery is a normal condition you drive to a
         * charger, while a dead cell means a pack is being isolated. Nothing on
         * the bus could tell the two apart. */
        battery->has_dead_cell() << 5
    );
}

/* Combine status bits into status byte to send out in status CAN message
 * bit 0 = charge inhibited
 * bit 1 = drive inhibited
 * bit 2 = heater enabled
 * bit 3 = ignition on
 * bit 4 = charge enabled
 * bit 5 = regen not allowed
 * bit 6 =
 * bit 7 =
 */
uint8_t Bms::get_status_byte() {
    return (
        0x00 | \
        charge_is_inhibited() | \
        drive_is_inhibited() << 1 | \
        heater_is_enabled() << 2 | \
        ignition_is_on() << 3 | \
        charge_is_enabled() << 4 | \
        regen_not_allowed() << 5
    );
}

void Bms::increment_invalid_event_count() {
    invalidEventCounter++;
}

uint8_t Bms::get_welding_byte() {
    uint8_t weldingByte = (uint8_t)( ( posContactorWelded ? 0x01 : 0x00 )
                                   | ( negContactorWelded ? 0x02 : 0x00 ) );
    /* One bit per pack, from bit 2 up. The pack indices used to be written out
     * as [0] and [1], which reads one past the end of a NUM_PACKS-sized array
     * in a single-pack build and put an indeterminate value on the bus. */
    for ( int p = 0; p < NUM_PACKS && p < 6; p++ ) {
        if ( packContactorsWelded[p] ) {
            weldingByte |= (uint8_t)( 1u << ( p + 2 ) );
        }
    }
    return weldingByte;
}

/*
 * A contactor is only "welded" if its feedback says closed at a moment when it
 * should be open. This used to read the feedback unconditionally, so every
 * legitimately closed contactor was reported as welded.
 *
 * The HVJB contactors are commanded by the inverter, not by us, so the best
 * proxy for "should be open" is ignition off and no charge request. Allow
 * WELD_CHECK_SETTLE_MS for them to physically open before believing feedback.
 * The pack contactors apply the same rule against their own inhibit state --
 * see BatteryPack::contactors_are_welded().
 */
void Bms::do_welding_checks() {
    const bool hvShouldBeOpen = !ignition_is_on() && !charge_is_enabled();
    if ( !hvShouldBeOpen ) {
        hvContactorsShouldBeOpenSince = 0;
    } else {
        if ( hvContactorsShouldBeOpenSince == 0 ) {
            hvContactorsShouldBeOpenSince = get_clock_ms();
        }
        if ( ( get_clock_ms() - hvContactorsShouldBeOpenSince ) >= WELD_CHECK_SETTLE_MS ) {
            posContactorWelded = io->pos_contactor_feedback_closed();
            negContactorWelded = io->neg_contactor_feedback_closed();
        }
    }
    for ( int p = 0; p < NUM_PACKS; p++ ) {
        packContactorsWelded[p] = battery->contactor_is_welded((uint8_t)p);
    }
}

// Charging

/* Charge current ceiling as a function of state of charge: unrestricted below
 * CHARGE_TAPER_START_SOC, then tapering linearly to zero at 100%.
 *
 * This used to be `return 0`, and since update_max_charge_current() takes the
 * min() of this and the temperature limit, the charger was told 0 A always.
 * The taper is a conservative default policy, not a manufacturer curve --
 * see the note in settings.h. Cell-level overvoltage protection is separate
 * and still handled by has_full_cell(). */
uint16_t Bms::get_max_charge_current_by_soc() {
    const uint16_t unrestricted = (uint16_t)( CHARGE_CURRENT_MAX_PER_PACK_A * NUM_PACKS );
    const uint8_t currentSoc = get_soc();

    if ( currentSoc >= 100 ) {
        return 0;
    }
    if ( currentSoc < CHARGE_TAPER_START_SOC ) {
        return unrestricted;
    }
    const uint32_t taperSpan = 100u - CHARGE_TAPER_START_SOC;
    const uint32_t remaining = 100u - currentSoc;
    return (uint16_t)( ( (uint32_t)unrestricted * remaining ) / taperSpan );
}

void Bms::update_max_charge_current() {
    // Safeties
    if ( battery->too_hot() || charge_is_inhibited() ) {
        maxChargeCurrent = 0;
        return;
    }
    maxChargeCurrent = std::min(battery->get_max_charge_current_by_temperature(), get_max_charge_current_by_soc());
}

uint16_t Bms::get_max_charge_current() {
    return maxChargeCurrent;
}

/* Discharge current ceiling. Previously hardcoded to 100 A regardless of
 * temperature, cell state or how many packs were actually connected.
 * Conservative policy -- see the note in settings.h. */
void Bms::update_max_discharge_current() {
    if ( battery->too_hot() || drive_is_inhibited() || battery->has_empty_cell() ) {
        maxDischargeCurrent = 0;
        return;
    }
    const uint8_t activePacks = battery->number_of_active_packs();
    maxDischargeCurrent = (uint16_t)( DISCHARGE_CURRENT_MAX_PER_PACK_A * activePacks );
}

uint16_t Bms::get_max_discharge_current() {
    return maxDischargeCurrent;
}

// statusLight

void Bms::led_blink() {
    statusLight.led_blink();
}

// Track when the pack voltages match each other
void Bms::pack_voltages_match_heartbeat() {
    lastTimePackVoltagesMatched = get_clock_ms();
}

bool Bms::packs_are_imbalanced() {
    return ( get_clock_ms() - lastTimePackVoltagesMatched ) > PACKS_IMBALANCED_TTL_MS;
}


// Comms

bool Bms::send_frame(CANMessage* frame, bool doChecksum) {
    for ( int t = 0; t < SEND_FRAME_RETRIES; t++ ) {
        // printf("[bms][send_frame] 0x%03X  [ ", frame->can_id);
        // for ( int i = 0; i < frame->can_dlc; i++ ) {
        //     printf("%02X ", frame->data[i]);
        // }
        // printf("]\n");

        if ( doChecksum ) {
            // Calculate XOR checksum
            frame->data[7] = 0;
            for ( int i = 0; i < 7; i++ ) {
                frame->data[7] ^= frame->data[i];
            }
        }

        // if ( !mutex_enter_timeout_ms(&canMutex, CAN_MUTEX_TIMEOUT_MS) ) {
        //     increment_can_tx_error_count();
        //     continue;
        // }

        const bool status = ACAN_ESP32::can.tryToSend(*frame);
        // mutex_exit(&canMutex);

        if ( !status ) {
            increment_can_tx_error_count();
            continue;
        }

        // Sending failed, try again
        // if ( result != MCP2515::ERROR_OK ) {
        //     if ( result == MCP2515::ERROR_FAIL ) {
        //         printf(" [send_frame %d] ERROR_FAIL, try again\n", t);
        //         increment_can_tx_error_count();
        //     } else if ( result == MCP2515::ERROR_ALLTXBUSY ) {
        //         printf(" [send_frame %d] ERROR_ALLTXBUSY, try again\n", t);
        //         increment_can_tx_error_count();
        //     } else if ( result == MCP2515::ERROR_FAILINIT ) {
        //         printf(" [send_frame %d] ERROR_FAILINIT, try again\n", t);
        //         increment_can_tx_error_count();
        //     } else if ( result == MCP2515::ERROR_FAILTX ) {
        //         printf(" [send_frame %d] ERROR_FAILTX, try again\n", t);
        //         increment_can_tx_error_count();
        //     } else if ( result == MCP2515::ERROR_NOMSG ) {
        //         printf(" [send_frame %d] ERROR_NOMSG, try again\n", t);
        //         increment_can_tx_error_count();
        //     }
        //     continue;
        // }
        // Frame was sent
        return true;
    }
    // Failed to send after all retries
    return false;
}

bool Bms::read_frame(CANMessage* frame) {
    /* Single attempt, deliberately. This used to sit in a READ_FRAME_RETRIES
     * loop that returned on its first iteration either way, so the retry count
     * never meant anything. "No frame waiting" is the normal case rather than a
     * failure worth retrying: the caller is polled again 5ms later. */
    return ACAN_ESP32::can.receive(*frame);
}
