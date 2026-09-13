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

#include "bms.h"
#include "shunt.h"
#include "util.h"

#include "settings.h"


/*
 * Perform all health checks.
 */
void health_check_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    extern Battery battery;
    extern Shunt shunt;

    // Do dead cell detection (if we have more than one pack)
    if ( battery.has_multiple_packs() && battery.has_dead_cell() ) {
        bms.send_event(E_DEAD_CELL);
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

static TimerHandle_t healthCheckTimer = NULL;

/*
 * Run recurring calculations
 */
void calculations_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    bms.update_max_charge_current();
    bms.update_max_discharge_current();
    bms.recalculate_soc();
    // TODO : range estimate
}

static TimerHandle_t calculationsTimer = NULL;


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

void send_limits_message_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    extern Battery battery;
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

static TimerHandle_t limitsMessageTimer = NULL;


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
 * byte 6 = unused
 * byte 7 = checksum
 */

void send_bms_state_message_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    extern Battery battery;
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
    bmsStateFrame.data[6] = 0x00;
    // data[7] is the checksum; send_frame() computes it because doChecksum is true
    bms.send_frame(&bmsStateFrame, true);
}

static TimerHandle_t bmsStateMessageTimer = NULL;


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

void send_module_liveness_message_callback(TimerHandle_t xTimer) {
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

static TimerHandle_t moduleLivenessMessageTimer = NULL;

/*
 * Main CAN bus tx/rx error counters message 0x354
 *
 * Custom message format (not in SimpBMS)
 *
 * byte 0 - 3 = can tx error counters (32bit counter)
 * byte 4 - 7 = can rx error counters (32bit counter)
 */

void send_main_can_error_counters_message_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    CANMessage mainCanErrorCountersFrame;
    zero_frame(&mainCanErrorCountersFrame);
    mainCanErrorCountersFrame.id = 0x354;
    put_u32_le(&mainCanErrorCountersFrame, 0, bms.get_can_tx_error_count());
    put_u32_le(&mainCanErrorCountersFrame, 4, bms.get_can_rx_error_count());
    bms.send_frame(&mainCanErrorCountersFrame, false);
}

static TimerHandle_t mainCanErrorCountersMessageTimer = NULL;


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

void send_soc_message_callback(TimerHandle_t xTimer) {
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

static TimerHandle_t sendSocMessageTimer = NULL;

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

void send_status_message_callback(TimerHandle_t xTimer) {
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
    put_i16_le(&statusFrame, 4, (int16_t)( battery.get_highest_sensor_temperature() * 10 ));
    put_u16_le(&statusFrame, 6, (uint16_t)( shunt.get_voltage1() / 10 ));
    bms.send_frame(&statusFrame, false);
}

static TimerHandle_t sendStatusMessageTimer = NULL;

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

void send_pack_can_error_counters_message_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    extern Battery battery;
    CANMessage packCanErrorCountersFrame;
    zero_frame(&packCanErrorCountersFrame);
    packCanErrorCountersFrame.id = 0x357;
    put_u16_le(&packCanErrorCountersFrame, 0, battery.get_can_tx_error_count_for_pack(0));
    put_u16_le(&packCanErrorCountersFrame, 2, battery.get_can_rx_error_count_for_pack(0));
    put_u16_le(&packCanErrorCountersFrame, 4, battery.get_can_tx_error_count_for_pack(1));
    put_u16_le(&packCanErrorCountersFrame, 6, battery.get_can_rx_error_count_for_pack(1));
    bms.send_frame(&packCanErrorCountersFrame, false);
}

static TimerHandle_t sendPackCanErrorCountersMessageTimer = NULL;

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

void send_alarm_message_callback(TimerHandle_t xTimer) {
    extern Bms bms;
    extern Battery battery;
    CANMessage alarmFrame;
    zero_frame(&alarmFrame);
    alarmFrame.id = 0x35A;

    // byte 0, bit 0, general alarm
    if ( bms.get_internal_error() ) { alarmFrame.data[0] |= 0x01; }
    // byte 0, bit 2 : overvolt alarm
    if ( battery.has_full_cell() ) { alarmFrame.data[0] |= 0x04; }
    // byte 0, bit 4 : undervolt alarm
    if ( battery.has_empty_cell() ) { alarmFrame.data[0] |= 0x10; }
    // byte 0, bit 6 : high temp alarm
    if ( battery.too_hot() ) { alarmFrame.data[0] |= 0x40; }

    // byte 1, bit 0 : low temp alarm
    if ( battery.too_cold_to_charge() ) { alarmFrame.data[1] |= 0x01; }
    // byte 1, bit 2 : high temp charge alarm
    if ( battery.too_hot() ) { alarmFrame.data[1] |= 0x04; }
    // byte 1, bit 4 : low temp charge alarm
    if ( battery.too_cold_to_charge() ) { alarmFrame.data[1] |= 0x10; }
    // FIXME byte 1, bit 6 : high current alarm

    // FIXME byte 2, bit 0 : high charge current alarm
    // byte 2, bit 2 : contactor on alarm
    if ( bms.charge_is_enabled() || bms.ignition_is_on() ) { alarmFrame.data[2] |= 0x04; }
    // FIXME byte 2, bit 4 : short circuit alarm
    // byte 2, bit 6 : internal error alarm
    if ( bms.get_internal_error() ) { alarmFrame.data[2] |= 0x40; }

    // byte 3, bit 0 : cell delta alarm
    if ( battery.cell_delta_above_alarm() ) { alarmFrame.data[3] |= 0x01; }

    // FIXME byte 4, bit 0 : general warn
    // byte 4, bit 2 : overvolt warn
    if ( battery.has_full_cell() ) { alarmFrame.data[4] |= 0x04; }
    // byte 4, bit 4 : undervolt warn
    if ( battery.has_empty_cell() ) { alarmFrame.data[4] |= 0x10; }
    // byte 4, bit 6 : high temp warn
    if ( battery.too_hot() ) { alarmFrame.data[4] |= 0x40; }

    // byte 5, bit 0 : low temp warn
    if ( battery.too_cold_to_charge() ) { alarmFrame.data[5] |= 0x01; }
    // byte 5, bit 2 : high temp charge warn
    if ( battery.too_hot() ) { alarmFrame.data[5] |= 0x04; }
    // byte 5, bit 4 : low temp charge warn
    if ( battery.too_cold_to_charge() ) { alarmFrame.data[5] |= 0x10; }
    // FIXME byte 5, bit 6 : high current warn

    // FIXME byte 6, bit 0 : high charge current warn
    // byte 6, bit 2 : contactor on warn
    if ( bms.charge_is_enabled() || bms.ignition_is_on() ) { alarmFrame.data[6] |= 0x04; }
    // FIXME byte 6, bit 4 : short circuit warn
    // byte 6, bit 6 : internal error warn
    if ( bms.get_internal_error() ) { alarmFrame.data[6] |= 0x40; }

    // FIXME byte 7, bit 0 : cell delta warn
    if ( battery.cell_delta_above_warn() ) { alarmFrame.data[7] |= 0x01; }

    bms.send_frame(&alarmFrame, false);
}

static TimerHandle_t sendAlarmMessageTimer = NULL;


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
static int32_t shunt_payload(const CANMessage& m) {
    return (int32_t)( ((uint32_t)m.data[5] << 24)
                    | ((uint32_t)m.data[4] << 16)
                    | ((uint32_t)m.data[3] <<  8)
                    | ((uint32_t)m.data[2]) );
}


void handle_main_CAN_messages_callback(TimerHandle_t xTimer) {
    CANMessage m;
    extern Shunt shunt;
    extern Bms bms;
    if ( bms.read_frame(&m) ) {
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

static TimerHandle_t handleMainCanMessageTimer = NULL;



void Bms::init(Battery* _battery, Io* _io, Shunt* _shunt) {
    battery = _battery;
    state = &state_standby;
    io = _io;
    shunt = _shunt;
    internalError = false;
    statusLight = StatusLight(this);
    chargeInhibitReason = R_NONE;
    driveInhibitReason = R_NONE;

    printf("[bms][init] setting up main CAN port\n");
    ACAN_ESP32_Settings settings(500 * 1000);
    settings.mRxPin = GPIO_NUM_16;
    settings.mTxPin = GPIO_NUM_17;
    const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
    if ( errorCode == 0 ) {
        printf("[bms][init] main CAN port initialized successfully\n");
    } else {
        printf("[bms][init] WARNING problem initializing main CAN port : %lu\n", errorCode);
    }

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

}

/*
 * Begin the periodic timers. Kept separate from init() so that no callback can
 * run against a half-built Battery: every one of these touches `battery`, which
 * used to be a pack array that had not been created yet.
 */
void Bms::start() {
    printf("[bms][start] enabling CAN message handlers\n");
    // outbound periodic messages
    limitsMessageTimer                   = create_and_start_timer("limitsMessage",         1000, send_limits_message_callback);
    bmsStateMessageTimer                 = create_and_start_timer("bmsStateMessage",        1000, send_bms_state_message_callback);
    moduleLivenessMessageTimer           = create_and_start_timer("moduleLiveness",         5000, send_module_liveness_message_callback);
    mainCanErrorCountersMessageTimer     = create_and_start_timer("mainCanErrCounters",     1000, send_main_can_error_counters_message_callback);
    sendPackCanErrorCountersMessageTimer = create_and_start_timer("packCanErrCounters",     1000, send_pack_can_error_counters_message_callback);
    sendSocMessageTimer                  = create_and_start_timer("socMessage",             1000, send_soc_message_callback);
    sendStatusMessageTimer               = create_and_start_timer("statusMessage",          1000, send_status_message_callback);
    sendAlarmMessageTimer                = create_and_start_timer("alarmMessage",           1000, send_alarm_message_callback);
    // inbound main CAN
    handleMainCanMessageTimer            = create_and_start_timer("mainCanRx",                 5, handle_main_CAN_messages_callback);
    // periodic work
    healthCheckTimer                     = create_and_start_timer("healthCheck",             100, health_check_callback);
    calculationsTimer                    = create_and_start_timer("calculations",           1000, calculations_callback);
    // status light
    statuslight_start_blink_timer();
}

void Bms::set_state(State newState, std::string reason) {
    std::string oldStateName = get_state_name(state);
    std::string newStateName = get_state_name(newState);
    printf("[bms][set_state] switching from state %s to state %s, reason : %s\n", oldStateName.c_str(), newStateName.c_str(), reason.c_str());
    state = newState;
    // Change light blinking pattern based on state
    if ( state == state_standby ) {
        statusLight.set_mode(STANDBY);
    } else if ( state == state_drive ) {
        statusLight.set_mode(DRIVE);
    } else if ( state == state_batteryHeating ) {
        statusLight.set_mode(CHARGING);
    } else if ( state == state_charging ) {
        statusLight.set_mode(CHARGING);
    } else if ( state == state_batteryEmpty ) {
        statusLight.set_mode(FAULT);
    } else if ( state == state_overTempFault ) {
        statusLight.set_mode(FAULT);
    } else if ( state == state_illegalStateTransitionFault ) {
        statusLight.set_mode(FAULT);
    } else if ( state == state_criticalFault ) {
        statusLight.set_mode(FAULT);
    } else {
        statusLight.set_mode(FAULT);
    }
}

State Bms::get_state() {
    return state;
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

void Bms::print() {
    std::string chg_inh = io->charge_is_inhibited() ? "true" : "false";
    std::string drv_inh = io->drive_is_inhibited() ? "true" : "false";
    std::string ign = io->ignition_is_on() ? "true" : "false";
    std::string chg_en = io->charge_enable_is_on() ? "true" : "false";
    int8_t Tmax = battery->get_highest_sensor_temperature();
    int8_t Tmin = battery->get_lowest_sensor_temperature();
    int16_t Vmax = battery->get_highest_cell_voltage();
    int16_t Vmin = battery->get_lowest_cell_voltage();
    printf("State:%s, SoC:%d, DRV_INH:%s, CHG_INH:%s, IGN:%s, CHG_EN:%s\n",
        get_state_name(get_state()), soc, drv_inh.c_str(), chg_inh.c_str(), ign.c_str(), chg_en.c_str());
    printf(" V:%d, VMax:%d, VMin:%d\n", battery->get_voltage()/1000, Vmax, Vmin );
    printf(" TMax:%d, TMin:%d\n", Tmax, Tmin );
    battery->print();
}

// Watchdog

void Bms::set_watchdog_reboot(bool value) {
    watchdogReboot = value;
}

// DRIVE_INHIBIT

void Bms::enable_drive_inhibit(std::string context, InhibitReason reason) {
    if ( !drive_is_inhibited() ) {
        set_drive_inhibit_reason(reason);
        io->enable_drive_inhibit(context);
    }
}

void Bms::disable_drive_inhibit(std::string context) {
    clear_drive_inhibit_reason();
    if ( drive_is_inhibited() ) {
        io->disable_drive_inhibit(context);
    }
}

bool Bms::drive_is_inhibited() {
    return io->drive_is_inhibited();
}

void Bms::set_drive_inhibit_reason(InhibitReason reason) {
    driveInhibitReason = reason;
}

void Bms::clear_drive_inhibit_reason() {
    driveInhibitReason = R_NONE;
}

int8_t Bms::get_drive_inhibit_reason() {
    return driveInhibitReason;
}

// CHARGE_INHIBIT

void Bms::enable_charge_inhibit(std::string context, InhibitReason reason) {
    if ( !charge_is_inhibited() ) {
        set_charge_inhibit_reason(reason);
        io->enable_charge_inhibit(context);
    }
}

void Bms::disable_charge_inhibit(std::string context) {
    clear_charge_inhibit_reason();
    if ( charge_is_inhibited() ) {
        io->disable_charge_inhibit(context);
    }
}

bool Bms::charge_is_inhibited() {
    return io->charge_is_inhibited();
}

void Bms::set_charge_inhibit_reason(InhibitReason reason) {
    chargeInhibitReason = reason;
}

void Bms::clear_charge_inhibit_reason() {
    chargeInhibitReason = R_NONE;
}

int8_t Bms::get_charge_inhibit_reason() {
    return chargeInhibitReason;
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
void Bms::recalculate_soc() {
    if ( CALCULATE_SOC_FROM_AMP_SECONDS == 1 ) {
        soc = 100 * (BATTERY_CAPACITY_AS + shunt->get_ampSeconds()) / BATTERY_CAPACITY_AS;
    } else {
        soc = 100 * (BATTERY_CAPACITY_WH + shunt->get_wattHours()) / BATTERY_CAPACITY_WH;
    }
}

// Error

void Bms::set_internal_error() {
    internalError = true;
}

void Bms::clear_internal_error() {
    internalError = false;
}

// Combine error bits into error byte to send out in status CAN message
uint8_t Bms::get_error_byte() {
    return (
        0x00 | \
        internalError | \
        battery->packs_are_imbalanced() << 1 | \
        shunt->is_dead() << 2 | \
        illegalStateTransition << 3 | \
        ! battery->is_alive() << 4
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
    return (
        0x00 | \
        posContactorWelded | \
        negContactorWelded << 1 | \
        packContactorsWelded[0] << 2 | \
        packContactorsWelded[1] << 3
    );

}

void Bms::do_welding_checks() {
    posContactorWelded = io->pos_contactor_is_welded();
    negContactorWelded = io->neg_contactor_is_welded();
    packContactorsWelded[0] = battery->contactor_is_welded(0);
    packContactorsWelded[1] = battery->contactor_is_welded(1);
}

// Charging

// FIXME account for inhibited packs
uint16_t Bms::get_max_charge_current_by_soc() {
    return 0;
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

void Bms::update_max_discharge_current() {
    // FIXME actual implementation
    maxDischargeCurrent = 100;
}

uint16_t Bms::Bms::get_max_discharge_current() {
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
    for ( int t = 0; t < READ_FRAME_RETRIES; t++ ) {    
        // if ( !mutex_enter_timeout_ms(&canMutex, CAN_MUTEX_TIMEOUT_MS) ) {
        //     increment_can_rx_error_count();
        //     return false;
        // }
        const bool received = ACAN_ESP32::can.receive(*frame);
        // mutex_exit(&canMutex);
        // if ( result != MCP2515::ERROR_OK ) {
        //     if ( result == MCP2515::ERROR_FAIL ) {
        //         printf("[bms][read_frame] %d/%d ERROR_FAIL, try again\n", t, READ_FRAME_RETRIES);
        //         increment_can_rx_error_count();
        //     } else if ( result == MCP2515::ERROR_ALLTXBUSY ) {
        //         printf("[bms][read_frame] %d/%d ERROR_ALLTXBUSY, try again\n", t, READ_FRAME_RETRIES);
        //         increment_can_rx_error_count();
        //     } else if ( result == MCP2515::ERROR_FAILINIT ) {
        //         printf("[bms][read_frame] %d/%d ERROR_FAILINIT, try again\n", t, READ_FRAME_RETRIES);
        //         increment_can_rx_error_count();
        //     } else if ( result == MCP2515::ERROR_FAILTX ) {
        //         printf("[bms][read_frame] %d/%d ERROR_FAILTX, try again\n", t, READ_FRAME_RETRIES);
        //         increment_can_rx_error_count();
        //     } else if ( result == MCP2515::ERROR_NOMSG ) {
        //         return true;
        //     }
        //     continue;
        // }
        /* No frame waiting is the normal case, not an error. Returning true
         * here unconditionally (as this used to) made every caller parse an
         * uninitialised CANMessage off the stack. */
        if ( received ) {
            return true;
        }
        return false;
    }
    // Failed to read after all retries
    return false;
}