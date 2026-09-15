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

#include <Arduino.h>

#include <stdio.h>
#include "pack.h"
#include "module.h"
#include "statemachine.h"
#include "bms.h"
#include "settings.h"
#include "util.h"
#include "webstatus.h"

BatteryPack::BatteryPack() {}

/* The CAN driver is heap-allocated in init(); own it properly rather than
 * relying on the object outliving the program. */
BatteryPack::~BatteryPack() { delete CAN; CAN = nullptr; }

void BatteryPack::init(const BatteryPackConfig& config) {

    const int _id = config.id;
    const int CANCSPin = config.canChipSelectPin;
    const int _contactorInhibitPin = config.contactorInhibitPin;
    const int _contactorFeedbackPin = config.contactorFeedbackPin;
    int _numModules = config.numModules;

    id = _id;
    if ( _numModules > MODULES_PER_PACK ) {
        printf("[pack%d] ERROR numModules %d exceeds MODULES_PER_PACK %d, clamping\n",
               _id, _numModules, MODULES_PER_PACK);
        _numModules = MODULES_PER_PACK;
    }
    if ( _numModules < 0 ) {
        _numModules = 0;
    }
    numModules = _numModules;
    numCellsPerModule = config.numCellsPerModule;
    numTemperatureSensorsPerModule = config.numTemperatureSensorsPerModule;

    // Build the CRC table before anything can call getcheck()
    crc8.begin();

    // Initialise modules in place so each module's back-pointer is `this`
    for ( int m = 0; m < numModules; m++ ) {
        modules[m].init(m, this, numCellsPerModule, numTemperatureSensorsPerModule);
    }

    /* Set up dedicated CAN port for communicating with this pack.
     * Release any previous instance: init() is called once per pack in normal
     * operation, but leaking on a second call is still a defect. */
    printf("[pack%d] creating CAN port\n", id);
    delete CAN;
    CAN = new ACAN2515(CANCSPin, SPI, PACK_CAN_NO_INTERRUPT_PIN);
    ACAN2515Settings settings (QUARTZ_FREQUENCY, 500 * 1000);
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    /* Polled mode: ACAN2515 treats INT 255 as "no interrupt pin" and requires a
     * NULL ISR. Its ESP32 driver task still runs; poll_can() wakes it. */
    const uint16_t errorCode = CAN->begin(settings, NULL);
    if ( errorCode != 0 ) {
        printf("[pack%d] ERROR setting up CAN port: %d\n", id, errorCode);
    } else {
        printf("[pack%d] CAN port setup complete\n", id);
    }

#if CAN_SELF_TEST_AT_INIT
    CANMessage testFrame;
    zero_frame(&testFrame);
    printf("[pack%d] sending 10 test messages\n", id);
    for ( int i = 0; i < 10; i++ ) {
        if ( !send_frame(&testFrame) ) {
            printf("[pack%d] ERROR sending test message %d\n", id, i);
        }
    }
#endif

    voltage = 0;
    cellDelta = 0;

    /* Set up contactor control. Starts INHIBITED: the previous code drove this
     * low (contactors permitted) at power-on, before a single voltage reading
     * had arrived from any module. */
    contactorInhibitPin = _contactorInhibitPin;
    printf("[pack%d] setting up contactor control (inhibited)\n", id);
    pinMode(contactorInhibitPin, OUTPUT);
    /* Go through the method that owns this pin rather than driving it here, so
     * the pin state, the reason mask and the weld-check timestamp cannot drift
     * apart. The mask is cleared first so the call logs and timestamps. */
    contactorInhibitReasons = 0;
    previousErrorFlags = 0;
    previousReceivePeak = 0;
    enable_inhibit_contactor_close(CI_STARTUP);

    // Set up contactor feedback
    contactorFeedbackPin = _contactorFeedbackPin;
    pinMode(contactorFeedbackPin, INPUT);

    inStartup = true;
    modulePollingCycle = 0;

    canTxErrorCount = 0;
    canRxErrorCount = 0;

    // See BatteryModule::init: reset every piece of runtime state, not some.
    nextModuleToPoll = 0;
    balancePhase = BALANCE_REST;
    balancePhaseStartedAt = 0;
    balanceBurstEndedAt = 0;
    balanceTargetMv = 0;
    balancingThisSweep = false;
    haveTemperatureBaseline = false;
    lastTemperatureSampleTime = 0;
    lastTemperatureSample = 0;
    temperatureDelta = 0;
    voltage = 0;
    cellDelta = 0;

    printf("[pack%d] setup complete\n", id);
}

void BatteryPack::fill_snapshot(WebPackSnapshot& out) {
    out.voltage = get_voltage();
    out.lowestCellVoltage = get_lowest_cell_voltage();
    out.highestCellVoltage = get_highest_cell_voltage();
    out.cellDelta = cellDelta;
    out.lowestTemperature = get_lowest_temperature();
    out.highestTemperature = get_highest_temperature();
    out.alive = is_alive();
    out.contactorsInhibited = contactors_are_inhibited();
    out.contactorsWelded = contactors_are_welded();
    out.balancing = balancing_is_active();
    out.hasDeadCell = has_dead_cell();
    /* Only meaningful while a burst is running; report 0 the rest of the time
     * rather than the inert 0x10C7 idle target, which reads on screen as a
     * bleed target 400 mV above any real cell. */
    out.balanceTargetMv = balancing_is_active() ? balanceTargetMv : 0;
    out.canTxErrors = canTxErrorCount;
    out.canRxErrors = canRxErrorCount;
    for ( int m = 0; m < MODULES_PER_PACK; m++ ) {
        modules[m].fill_snapshot(out.modules[m]);
    }
}

void BatteryPack::print() {
    printf("[pack%d] %3.2fV : Hi %d : Lo %d : %dmV\n", id, ((float)voltage / 1000.0f), get_highest_cell_voltage(), get_lowest_cell_voltage(), cellDelta);
    for ( int m = 0; m < numModules; m++ ) {
        modules[m].print();
    }
}

/* NB: the second parameter is the MODULE index within this pack, not the pack
 * id. It used to be named `id`, which shadowed the BatteryPack::id member and
 * made this read as though it were indexed by pack. */
uint8_t BatteryPack::getcheck(CANMessage &msg, int moduleId) {
    unsigned char canmes[11];
    const int finalxorCount = (int)(sizeof(finalxor) / sizeof(finalxor[0]));
    if ( moduleId < 0 || moduleId >= finalxorCount ) {
        printf("[pack%d][getcheck] module id %d out of range for finalxor[%d]\n",
               this->id, moduleId, finalxorCount);
        return 0;
    }
    int meslen = msg.len + 1;  // remove one for crc and add two for id bytes
    canmes[1] = msg.id;
    canmes[0] = msg.id >> 8;

    for (int i = 0; i < (msg.len - 1); i++) {
        canmes[i + 2] = msg.data[i];
    }
    return (crc8.get_crc8(canmes, meslen, finalxor[moduleId]));
}

int8_t BatteryPack::get_module_liveness(int8_t moduleId) {
    // Modules beyond numModules were never init()ed, so do not read them
    if ( moduleId < 0 || moduleId >= numModules ) {
        return 0;
    }
    return modules[moduleId].is_alive();
}

bool BatteryPack::all_modules_populated() {
    for ( int m = 0; m < numModules; m++ ) {
        if ( !modules[m].all_module_data_populated() ) {
            return false;
        }
    }
    return numModules > 0;
}

bool BatteryPack::is_alive() {
    for ( int m = 0; m < numModules; m++ ) {
        if ( !modules[m].is_alive() ) {
            return false;
        }
    }
    return true;
}

/*
 * Send CAN frame to each module to request voltage and temperature data.
 *
 * Contents of message
 *   byte 0 : balance data
 *   byte 1 : balance data
 *   byte 2 : 0x00
 *   byte 3 : 0x00
 *   byte 4 : 
 *   byte 5 : 0x01
 *   byte 6 :
 *     bit 0    :
 *     bit 1    :
 *     bit 2    :
 *     bit 3    :
 *     bits 4-7 : module number
 *   byte 7 : checksum
 */
void BatteryPack::request_data() {
    if ( numModules <= 0 ) {
        return;
    }

    /* Start of a sweep: advance the polling cycle and re-evaluate balancing
     * once, so every module in the sweep gets a consistent command. */
    if ( nextModuleToPoll == 0 ) {
        if ( modulePollingCycle == 0xF ) {
            modulePollingCycle = 0;
        }
        update_balance_state();
        balancingThisSweep = balancing_is_active();
    }
    const bool balancing = balancingThisSweep;

    /* ONE module per call. The reference implementation puts a delay(2) between
     * module polls; sending all of them back to back both ignores that spacing
     * and blocks this task while the frames are queued. Spreading the sweep
     * across ticks gives the same spacing without blocking, and lets the CAN
     * receive path keep draining in between. */
    {
        const int m = nextModuleToPoll;
        pollModuleFrame.id = 0x080 | (m);
        pollModuleFrame.len = 8;
        if ( balancing ) {
            /* Bleed target, little endian. Cells above this bleed down to meet
             * it; the offset stops the lowest cell chasing itself. */
            put_u16_le(&pollModuleFrame, 0, get_balance_target_mv());
        } else {
            // 0x10C7 == 4295 mV, above any real cell, so nothing bleeds
            pollModuleFrame.data[0] = 0xC7;
            pollModuleFrame.data[1] = 0x10;
        }
        pollModuleFrame.data[2] = 0x00;
        pollModuleFrame.data[3] = 0x00;
        if ( inStartup ) {
            pollModuleFrame.data[4] = 0x20;
            pollModuleFrame.data[5] = 0x00;
        } else {
            /* 0x48 arms balancing, 0x40 does not. An earlier pass collapsed
             * these two arms to 0x40 on the assumption that the target voltage
             * alone drove balancing -- that was wrong; the reference
             * implementation distinguishes them here. */
            pollModuleFrame.data[4] = balancing ? MODULE_CMD_BALANCE_ON : MODULE_CMD_BALANCE_OFF;
            pollModuleFrame.data[5] = 0x01;
        }
        pollModuleFrame.data[6] = modulePollingCycle << 4;
        if ( inStartup && modulePollingCycle == 2 ) {
            pollModuleFrame.data[6] = pollModuleFrame.data[6] + 0x04;
        }
        pollModuleFrame.data[7] = getcheck(pollModuleFrame, m);
        if ( !send_frame(&pollModuleFrame) ) {
            printf("[pack%d][request_data] ERROR sending poll message to module %d\n", id, m);
        }
    }

    nextModuleToPoll++;
    if ( nextModuleToPoll >= numModules ) {
        nextModuleToPoll = 0;
        if ( inStartup && modulePollingCycle == 2 ) {
            inStartup = false;
        }
        modulePollingCycle++;
    }
}

/*
 * Wake the ACAN2515 driver task so it drains the MCP2515 into the driver's
 * receive buffer. Required because the pack controllers run without an
 * interrupt pin.
 */
void BatteryPack::poll_can() {
    CAN->poll();
}

/*
 * Account for what the pack's CAN controller reports about its own health.
 *
 * The main bus has done this since B157 -- it counts receive-FIFO overflows and
 * recovers from bus-off. The pack controllers had nothing equivalent, so a pack
 * bus losing every frame to a receive overflow, or off the bus entirely, was
 * invisible: the per-pack counters in 0x357 only ever counted module ids that
 * failed to parse. The eventual symptom was the modules ageing out into a
 * critical fault with nothing to say why.
 *
 * TWO SIGNALS, because neither alone is enough.
 *
 * EFLG is the controller's own view. Its bits are LATCHED, and ACAN2515 keeps
 * bitModify2515Register() private and never clears them itself, so once
 * RX0OVR or RX1OVR sets it stays set for the life of the program. That makes
 * this a one-shot indication and not a count, however it is written -- so it is
 * treated as one deliberately: the first overflow is counted and reported, and
 * after that the flag says only "this has happened at least once".
 *
 * The driver's own receive buffer is the second signal, and unlike EFLG it
 * tracks severity: peak count reaching the buffer size means frames were
 * dropped in software, between one 5 ms drain and the next. That is the failure
 * this pack bus is actually prone to, since a poll of six modules bursts far
 * more frames than fit between service ticks, and it is why read_message()
 * drains READ_FRAMES_PER_CYCLE at a time rather than one.
 */
void BatteryPack::check_can_health() {
    const uint8_t flags = CAN->errorFlagRegister();
    const uint8_t newFlags = (uint8_t)( flags & ~previousErrorFlags );
    previousErrorFlags = flags;

    if ( newFlags & 0xC0 ) {            // RX1OVR | RX0OVR
        increment_can_rx_error_count();
        printf("[pack%d][can] controller receive overflow, frames lost (EFLG 0x%02X). "
               "This bit latches and cannot be cleared through the driver, so it "
               "will not be reported again.\n", id, flags);
    }
    if ( newFlags & 0x20 ) {            // TXBO
        printf("[pack%d][can] bus-off\n", id);
    }
    if ( newFlags & 0x18 ) {            // TXEP | RXEP
        printf("[pack%d][can] error-passive (EFLG 0x%02X)\n", id, flags);
    }

    /* Software-side drops. The peak is a high-water mark the driver only ever
     * raises, so compare against the last one seen rather than re-reporting. */
    const uint16_t peak = CAN->receiveBufferPeakCount();
    if ( peak > previousReceivePeak ) {
        previousReceivePeak = peak;
        if ( peak >= CAN->receiveBufferSize() ) {
            increment_can_rx_error_count();
            printf("[pack%d][can] driver receive buffer reached its limit (%u of %u); "
                   "module replies were dropped between drains\n",
                   id, (unsigned int)peak, (unsigned int)CAN->receiveBufferSize());
        }
    }
}

/*
 * Check for messages from battery modules, parse as required.
 *
 * Drains up to READ_FRAMES_PER_CYCLE frames per call rather than one. A single
 * poll of a 6-module pack produces far more frames than the driver's 32-frame
 * receive buffer can hold between 5ms service ticks, so reading one at a time
 * silently dropped most module replies.
 */
void BatteryPack::read_message() {
    CANMessage frame;

    for ( int drained = 0; drained < READ_FRAMES_PER_CYCLE; drained++ ) {

        // Return when there's nothing left to process
        if ( !CAN->receive(frame) ) {
            return;
        }

    // printf("[pack%d][read_message] received message 0x%03X : ", this->id, frame.id);
    // for ( int i = 0; i < frame.can_dlc; i++ ) {
    //     printf("%02X ", frame.data[i]);
    // }
    // printf("\n");

        // Temperature messages
        if ( (frame.id & 0xFF0) == 0x180 ) {
            decode_temperatures(&frame);
            /* Two different jobs, and both need doing. The pack samples its own
             * highest temperature to maintain temperatureDelta, the per-minute
             * rise rate that get_max_charge_current() derates against; the
             * battery refreshes the pack-wide too_hot/too_cold latches. Only
             * the battery half used to be called here, which left
             * temperatureDelta pinned at 0 and the rate derating dead. */
            this->process_temperature_update();
            this->battery->process_temperature_update();
        }
        // Voltage messages
        /* Module replies occupy 0x100-0x17F: decode_voltages() splits the id
         * into (id & 0x0F0) for the message type and (id & 0x00F) for the
         * module. The old lower bound of 0x99 also admitted 0x09A-0x0FF, which
         * are not module replies and would be decoded as though they were. */
        if ( frame.id >= 0x100 && frame.id < 0x180 ) {
            decode_voltages(&frame);
            this->battery->process_voltage_update();
        }
    }
}

/*
 * Send a CAN frame to the battery pack. Return true if successful, false if
 * not. */
bool BatteryPack::send_frame(CANMessage *frame) {
    for ( int i = 0; i < SEND_FRAME_RETRIES; i++ ) {

        // printf("[pack%d][send_frame] 0x%03X  [ ", this->id, frame->id);
        // for ( int i = 0; i < frame->len; i++ ) {
        //     printf("%02X ", frame->data[i]);
        // }
        // printf("]\n");

        if ( CAN->tryToSend(*frame) ) {
            return true;
        } else {
            printf("[pack%d][send_frame] ERROR sending message to battery pack\n", this->id);
            increment_can_tx_error_count();
        }
    }
    return false;
}

/*
 * Should this pack be bleeding cells right now?
 *
 * Only near the top of the range, where the cells are on the steep part of the
 * curve and a bleed resistor can actually close the gap, and only when the
 * spread is worth acting on. Matches the reference implementation's condition:
 * highest cell above the balance voltage AND more than the hysteresis above the
 * lowest cell.
 */
bool BatteryPack::should_start_balancing() {
    if ( !CELL_BALANCING_ENABLED ) {
        return false;
    }
    /* Every module must have reported first. Balancing discards readings, so
     * starting before a late module has populated would keep it unpopulated
     * until the next rest window -- and its pack would hold CI_STARTUP that
     * whole time. */
    if ( !all_modules_populated() ) {
        return false;
    }
    const uint16_t highest = get_highest_cell_voltage();
    const uint16_t lowest = get_lowest_cell_voltage();
    // No usable data yet
    if ( highest == 0 || lowest == NO_CELL_VOLTAGE_READING || highest < lowest ) {
        return false;
    }
    if ( highest <= CELL_BALANCE_VOLTAGE ) {
        return false;
    }
    return ( highest - lowest ) > CELL_BALANCE_HYSTERESIS_MV;
}

/*
 * Advance the balancing duty cycle. Called once per polling cycle.
 *
 * BALANCE_REST  -> BALANCE_BURST once the rest period has elapsed and the cells
 *                  still warrant balancing (decided on settled readings).
 * BALANCE_BURST -> BALANCE_REST once the burst has run for its duty period.
 */
void BatteryPack::update_balance_state() {
    const uint64_t now = get_clock_ms();

    if ( !CELL_BALANCING_ENABLED ) {
        balancePhase = BALANCE_REST;
        return;
    }

    switch ( balancePhase ) {

        case BALANCE_BURST:
            if ( ( now - balancePhaseStartedAt ) >= CELL_BALANCE_DUTY_MS ) {
                balancePhase = BALANCE_REST;
                balancePhaseStartedAt = now;
                balanceBurstEndedAt = now;
                printf("[pack%d][balance] burst finished, resting\n", id);
            }
            break;

        case BALANCE_REST:
        default:
            if ( ( now - balancePhaseStartedAt ) < CELL_BALANCE_REST_MS ) {
                break;
            }
            if ( should_start_balancing() ) {
                /* Latch the target now, from settled readings, so it does not
                 * chase the sagging measurements taken during the burst. */
                balanceTargetMv = (uint16_t)( get_lowest_cell_voltage() + CELL_BALANCE_TARGET_OFFSET_MV );
                balancePhase = BALANCE_BURST;
                balancePhaseStartedAt = now;
                printf("[pack%d][balance] starting burst, target %umV (high %umV, low %umV)\n",
                       id, (unsigned int)balanceTargetMv,
                       (unsigned int)get_highest_cell_voltage(),
                       (unsigned int)get_lowest_cell_voltage());
            } else {
                // Re-arm the rest window so we re-evaluate a period from now
                balancePhaseStartedAt = now;
            }
            break;
    }
}

/*
 * A cell that is bleeding reads low, and needs a moment to recover once the
 * bleed stops, so readings are discarded for the whole burst plus a settling
 * window afterwards. The reference implementation does the same thing via its
 * setBalIgnore() flag.
 */
bool BatteryPack::voltage_readings_are_suspect() {
    if ( balancePhase == BALANCE_BURST ) {
        return true;
    }
    if ( balanceBurstEndedAt == 0 ) {
        return false;
    }
    return ( get_clock_ms() - balanceBurstEndedAt ) < CELL_BALANCE_SETTLE_MS;
}

uint16_t BatteryPack::get_balance_target_mv() {
    return balanceTargetMv;
}


//// ----
//
// Voltage
//
//// ----

// Return the voltage of the whole pack
uint32_t BatteryPack::get_voltage() {
    return voltage;
}

// Update the pack voltage value by summing all of the cell voltages
void BatteryPack::recalculate_total_voltage() {
    /* Summing modules that have not reported yet counts them as 0 V, which
     * understates the pack and then feeds the contactor comparisons. Report
     * 0 (== "no reading", which the callers already guard for) until the whole
     * pack has reported. */
    if ( !all_modules_populated() ) {
        voltage = 0;
        return;
    }
    uint32_t newVoltage = 0;
    for ( int m = 0; m < numModules; m++ ) {
        newVoltage += modules[m].get_voltage();
    }
    voltage = newVoltage;
}

// Return the voltage of the lowest cell in the pack
uint16_t BatteryPack::get_lowest_cell_voltage() {
    uint16_t lowestCellVoltage = NO_CELL_VOLTAGE_READING;
    for ( int m = 0; m < numModules; m++ ) {
        // skip modules with incomplete cell data
        if ( !modules[m].all_module_data_populated() ) {
            continue;
        }
        if ( modules[m].get_lowest_cell_voltage() < lowestCellVoltage ) {
            lowestCellVoltage = modules[m].get_lowest_cell_voltage();
        }
    }
    return lowestCellVoltage;
}

// Return true if any cell in the pack is under min voltage
bool BatteryPack::has_empty_cell() {
    for ( int m = 0; m < numModules; m++ ) {
        if ( modules[m].has_empty_cell() ) {
            return true;
        }
    }
    return false;
}

// Return the voltage of the highest cell in the pack
uint16_t BatteryPack::get_highest_cell_voltage() {
    uint16_t highestCellVoltage = 0;
    for ( int m = 0; m < numModules; m++ ) {
        // skip modules with incomplete cell data
        if ( !modules[m].all_module_data_populated() ) {
            continue;
        }
        if ( modules[m].get_highest_cell_voltage() > highestCellVoltage ) {
            highestCellVoltage = modules[m].get_highest_cell_voltage();
        }
    }
    return highestCellVoltage;
}

// Return true if any cell in the pack is over max voltage
bool BatteryPack::has_full_cell() {
    for ( int m = 0; m < numModules; m++ ) {
        if ( modules[m].has_full_cell() ) {
            return true;
        }
    }
    return false;
}

// Extract voltage readings from CAN message and update stored values
void BatteryPack::decode_voltages(CANMessage *frame) {
    int messageId = (frame->id & 0x0F0);
    int moduleId = (frame->id & 0x00F);

    /* The module id is the low nibble of the frame id, so it can be 0-15, while
     * we only have numModules (6) modules. Anything higher used to be written
     * straight past the end of modules[]. */
    if ( moduleId >= numModules ) {
        increment_can_rx_error_count();
        return;
    }

    /* A bleeding cell measures low. Discard voltage readings for the whole
     * balancing burst and the settling window after it, in addition to the
     * module's own balance-status gate. */
    const bool readingsSuspect = voltage_readings_are_suspect();

    switch (messageId) {
        case 0x000:
            /* Per module: this frame is 0x100 | moduleId, so the status it
             * carries belongs to that module alone. Storing it pack-wide meant
             * the last module to report won, and one module reporting a balance
             * status suppressed voltage capture for the whole pack. */
            modules[moduleId].set_error_status( (uint32_t)frame->data[0]
                                              | ( (uint32_t)frame->data[1] <<  8 )
                                              | ( (uint32_t)frame->data[2] << 16 )
                                              | ( (uint32_t)frame->data[3] << 24 ) );
            modules[moduleId].set_balance_status( (uint32_t)frame->data[4]
                                                | ( (uint32_t)frame->data[5] << 8 ) );
            break;
        case 0x020:
            if ( modules[moduleId].get_balance_status() == 0 && !readingsSuspect ) {
                modules[moduleId].set_cell_voltage(0, static_cast<uint16_t>(frame->data[0] + (frame->data[1] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(1, static_cast<uint16_t>(frame->data[2] + (frame->data[3] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(2, static_cast<uint16_t>(frame->data[4] + (frame->data[5] & 0x3F) * 256));
                modules[moduleId].note_voltage_group(messageId);
            }
            break;
        case 0x030:
            if ( modules[moduleId].get_balance_status() == 0 && !readingsSuspect ) {
                modules[moduleId].set_cell_voltage(3, static_cast<uint16_t>(frame->data[0] + (frame->data[1] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(4, static_cast<uint16_t>(frame->data[2] + (frame->data[3] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(5, static_cast<uint16_t>(frame->data[4] + (frame->data[5] & 0x3F) * 256));
                modules[moduleId].note_voltage_group(messageId);
            }
            break;
        case 0x040:
            if ( modules[moduleId].get_balance_status() == 0 && !readingsSuspect ) {
                modules[moduleId].set_cell_voltage(6, static_cast<uint16_t>(frame->data[0] + (frame->data[1] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(7, static_cast<uint16_t>(frame->data[2] + (frame->data[3] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(8, static_cast<uint16_t>(frame->data[4] + (frame->data[5] & 0x3F) * 256));
                modules[moduleId].note_voltage_group(messageId);
            }
            break;
        case 0x050:
            if ( modules[moduleId].get_balance_status() == 0 && !readingsSuspect ) {
                modules[moduleId].set_cell_voltage(9, static_cast<uint16_t>(frame->data[0] + (frame->data[1] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(10, static_cast<uint16_t>(frame->data[2] + (frame->data[3] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(11, static_cast<uint16_t>(frame->data[4] + (frame->data[5] & 0x3F) * 256));
                modules[moduleId].note_voltage_group(messageId);
            }
            break;
        case 0x060:
            if ( modules[moduleId].get_balance_status() == 0 && !readingsSuspect ) {
                modules[moduleId].set_cell_voltage(12, static_cast<uint16_t>(frame->data[0] + (frame->data[1] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(13, static_cast<uint16_t>(frame->data[2] + (frame->data[3] & 0x3F) * 256));
                modules[moduleId].set_cell_voltage(14, static_cast<uint16_t>(frame->data[4] + (frame->data[5] & 0x3F) * 256));
                modules[moduleId].note_voltage_group(messageId);
            }
            break;
        case 0x070:
            if ( modules[moduleId].get_balance_status() == 0 && !readingsSuspect ) {
                modules[moduleId].set_cell_voltage(15, static_cast<uint16_t>(frame->data[0] + (frame->data[1] & 0x3F) * 256));
                modules[moduleId].note_voltage_group(messageId);
            }
            break;
        default:
            break;
    }

    // Check if this update has left us with a complete set of voltage/temperature readings
    if ( !modules[moduleId].all_module_data_populated() ) {
        modules[moduleId].check_if_module_data_is_populated();
    }

    modules[moduleId].heartbeat();
}

// Update the cellDelta
void BatteryPack::recalculate_cell_delta() {
    const uint16_t highest = get_highest_cell_voltage();
    const uint16_t lowest = get_lowest_cell_voltage();
    /* With no populated modules the getters return their sentinels (0 and
     * 10000), which used to underflow to a huge value and then truncate into a
     * uint8_t. Report no delta until there is real data on both ends. */
    if ( highest == 0 || lowest == NO_CELL_VOLTAGE_READING || highest < lowest ) {
        cellDelta = 0;
        return;
    }
    cellDelta = (uint16_t)( highest - lowest );
}

/*
 * We have new cell voltage data. Recalculate.
 */
void BatteryPack::process_voltage_update() {
    recalculate_total_voltage();
    recalculate_cell_delta();

    /* Withdraw the startup hold once every module has given us a full set of
     * readings. Until then the pack's contactors stay inhibited. */
    if ( contactorInhibitReasons & CI_STARTUP ) {
        bool allPopulated = true;
        for ( int m = 0; m < numModules; m++ ) {
            if ( !modules[m].all_module_data_populated() ) {
                allPopulated = false;
                break;
            }
        }
        if ( allPopulated ) {
            printf("[pack%d] all modules reporting, withdrawing startup contactor hold\n", id);
            disable_inhibit_contactor_close(CI_STARTUP);
        }
    }
}

/*
 * Check for dead cells in the pack
 */
bool BatteryPack::has_dead_cell() {
    for ( int m = 0; m < numModules; m++ ) {
        if ( modules[m].has_dead_cell() ) {
            return true;
        }
    }
    return false;
}


//// ----
//
// Temperature
//
//// ----

// Return true if any cell in the pack is over max temperature
bool BatteryPack::has_temperature_sensor_over_max() {
    for ( int m = 0; m < numModules; m++ ) {
        if ( modules[m].has_temperature_sensor_over_max() ) {
            return true;
        }
    }
    return false;
}

// return the temperature of the lowest sensor in the pack
int8_t BatteryPack::get_lowest_temperature() {
    int8_t lowestModuleTemperature = 126;
    for ( int m = 0; m < numModules; m++ ) {
        if ( ! modules[m].all_module_data_populated() ) {
            continue;
        }
        if ( modules[m].get_lowest_temperature() < lowestModuleTemperature ) {
            lowestModuleTemperature = modules[m].get_lowest_temperature();
        }
    }
    return lowestModuleTemperature;
}

// return the temperature of the highest sensor in the pack
int8_t BatteryPack::get_highest_temperature() {
    int8_t highestModuleTemperature = -126;
    for ( int m = 0; m < numModules; m++ ) {
        if ( ! modules[m].all_module_data_populated() ) {
            continue;
        }
        if ( modules[m].get_highest_temperature() > highestModuleTemperature ) {
            highestModuleTemperature = modules[m].get_highest_temperature();
        }
    }
    return highestModuleTemperature;
}

// Extract temperature sensor readings from CAN frame and update stored values
void BatteryPack::decode_temperatures(CANMessage *temperatureMessageFrame) {
    int moduleId = (temperatureMessageFrame->id & 0x00F);

    // See decode_voltages(): the low nibble can address modules we do not have.
    if ( moduleId >= numModules ) {
        increment_can_rx_error_count();
        return;
    }

    modules[moduleId].heartbeat();
    for ( int t = 0; t < numTemperatureSensorsPerModule; t++ ) {
        const uint8_t raw = temperatureMessageFrame->data[t];

        /* A raw count of zero means no sensor is fitted in that slot. The
         * reference implementation guards on the decoded value being > -40 for
         * exactly this reason. Storing it as a real -40 C reading would drag
         * the pack minimum down, permanently assert too_cold_to_charge(), and
         * latch an out-of-range internal error. Record it as "no reading"
         * instead, which the min/max getters already skip. */
        if ( raw == 0 ) {
            modules[moduleId].update_temperature(t, NO_TEMPERATURE_READING);
            continue;
        }

        /* Raw counts are offset by 40. Computed in int and clamped to the
         * int8_t the module stores: the old code built a float and passed it to
         * a uint8_t parameter, so every reading below 40 counts (i.e. below
         * 0 C) was an out-of-range conversion, which is undefined behaviour. */
        int reading = (int)raw - 40;
        if ( reading > 127 ) {
            reading = 127;
        }
        modules[moduleId].update_temperature(t, (int8_t)reading);
    }

    /* A module is only "populated" once it has given us both voltages and at
     * least one temperature, and the temperature frame is the last of a sweep.
     * Without re-checking here the flag only flipped when the NEXT sweep's
     * voltage frames arrived, delaying everything that waits on populated data
     * -- including the pack's CI_STARTUP contactor hold -- by a full cycle. */
    if ( !modules[moduleId].all_module_data_populated() ) {
        modules[moduleId].check_if_module_data_is_populated();
    }
}

void BatteryPack::process_temperature_update() {
    if ( ( get_clock_ms() - lastTemperatureSampleTime ) <= PACK_TEMP_SAMPLE_INTERVAL_MS ) {
        return;
    }
    const int8_t highest = get_highest_temperature();
    lastTemperatureSampleTime = get_clock_ms();

    /* The first sample only establishes the baseline. Differencing against the
     * initial 0 made a 25 C pack look like it was rising at 25 C/minute, which
     * derated the charge current straight to zero. */
    /* No usable reading this cycle: drop the baseline rather than differencing
     * against a sentinel. get_highest_temperature() returns -126 when no module
     * has data, and -126 against a real +100 is -226, which wraps in an int8_t
     * to +30 -- a fabricated 30 C/minute rise that derates charge current to
     * zero (or, with the signs reversed, hides a real one). */
    if ( highest <= -126 ) {
        haveTemperatureBaseline = false;
        temperatureDelta = 0;
        return;
    }
    if ( !haveTemperatureBaseline ) {
        lastTemperatureSample = highest;
        temperatureDelta = 0;
        haveTemperatureBaseline = true;
        return;
    }
    int delta = (int)highest - (int)lastTemperatureSample;
    if ( delta < -127 ) { delta = -127; }
    if ( delta >  127 ) { delta =  127; }
    temperatureDelta = (int8_t)delta;
    lastTemperatureSample = highest;
}


//// ----
//
// Contactors
//
//// ----

// Add a reason for holding this pack's contactors open
void BatteryPack::enable_inhibit_contactor_close(ContactorInhibitReason reason) {
    const bool wasInhibited = ( contactorInhibitReasons != 0 );
    contactorInhibitReasons |= (uint8_t)reason;
    if ( !wasInhibited ) {
        printf("[pack%d][contactors] inhibiting close (reason mask 0x%02X)\n", id, contactorInhibitReasons);
        contactorInhibitedSince = get_clock_ms();
    }
    digitalWrite(contactorInhibitPin, HIGH);
}

/* Withdraw one reason. The contactors are only released once no reason
 * remains -- clearing the imbalance hold must not release a dead-cell hold. */
void BatteryPack::disable_inhibit_contactor_close(ContactorInhibitReason reason) {
    const bool wasInhibited = ( contactorInhibitReasons != 0 );
    contactorInhibitReasons &= (uint8_t)~reason;
    if ( contactorInhibitReasons != 0 ) {
        digitalWrite(contactorInhibitPin, HIGH);
        return;
    }
    if ( wasInhibited ) {
        printf("[pack%d][contactors] releasing inhibit\n", id);
    }
    digitalWrite(contactorInhibitPin, LOW);
}

// Return true if the contactors for this pack are currently not allowed to close
bool BatteryPack::contactors_are_inhibited() {
    return contactorInhibitReasons != 0;
}

/* A contactor is only "welded" if the feedback says closed while we are holding
 * it open. Returning the raw feedback level (as this used to) reports every
 * legitimately closed contactor as welded. Allow WELD_CHECK_SETTLE_MS for the
 * contactor to physically open before believing the feedback. */
bool BatteryPack::contactors_are_welded() {
    if ( !contactors_are_inhibited() ) {
        return false;
    }
    if ( ( get_clock_ms() - contactorInhibitedSince ) < WELD_CHECK_SETTLE_MS ) {
        return false;
    }
    return digitalRead(contactorFeedbackPin) == HIGH;
}



// Current


/*==============================================================================
 * Charge current by temperature
 *
 * Indexed by (temperature + 10), covering -10C to +39C. Below -10C the pack is
 * heated instead of charged; at +40C and above there is no charging at all.
 *
 * Written as C-rates rather than amperes. The values are the same policy the
 * table always encoded -- plating-limited at the cold end, a ramp, a plateau,
 * and a step down as the pack approaches its temperature ceiling -- but tied to
 * BATTERY_CAPACITY_AS_PER_PACK so the curve and the cells cannot drift apart.
 * They had: the plateau sat at 125 A, which on a 26 Ah pack is 4.8C.
 *
 * The cold end below is unchanged in amperes (3 A at -10C is 0.115C, which is
 * about where the published guidance for charging NMC/graphite at -10C sits).
 * Everything from 0C up is rescaled to the CHARGE_C_RATE_PERCENT envelope, which
 * also removes a cliff the old table had at 0C, where it stepped 6 A -> 13 A.
 *
 * Lives here rather than in the class: it is one shared constant, not per-pack
 * state, and at namespace scope the invariants below can actually be checked.
 *============================================================================*/

namespace {

/*
 * The table is the lower of two separate limits at every temperature.
 *
 *   1. CHARGE_ACCEPTANCE_PERCENT_OF_C -- what the chemistry will take, as an
 *      absolute fraction of 1C. Plating-limited at the cold end, a ramp to 1C
 *      at 15C, a second ramp to the 2.2C ceiling at 25C, then backing off as
 *      the pack nears its temperature ceiling.
 *      This does NOT follow CHARGE_C_RATE_PERCENT: the plating limit at -10C
 *      does not move because a higher sustained rate was configured. Raising
 *      the knob to 2.2C bought nothing below 15C and only part of it between
 *      15C and 25C, which is the intended behaviour -- a cold cell does not
 *      accept a regen burst just because a warm one does.
 *
 *   2. CHARGE_CURRENT_MAX_PER_PACK_A shaped by the same curve -- the policy
 *      limit, which does follow the knob, so turning it down pulls the whole
 *      warm half down with it instead of leaving the ramp stranded above the
 *      plateau.
 *
 * Taking the lower of the two composes correctly at any setting, and is the
 * reason 0.5C does not produce a curve that dips in the middle.
 */
constexpr int CHARGE_ACCEPTANCE_PERCENT_OF_C[] = {
    // -10C to -1C : plating-limited, ~0.12C rising to ~0.23C
    12, 12, 12, 15, 15, 15, 19, 19, 23, 23,
    //   0C to 15C : ramp up as the cell warms
    27, 32, 37, 42, 46, 51, 56, 61, 66, 71, 76, 80, 85, 90, 95, 100,
    //  16C to 24C : second ramp, 1C at 15C to the 2.2C ceiling at 25C
    112, 124, 136, 148, 160, 172, 184, 196, 208,
    //  25C to 35C : full acceptance
    220, 220, 220, 220, 220, 220, 220, 220, 220, 220, 220,
    //  36C to 39C : backing off ahead of MAXIMUM_TEMPERATURE
    40, 40, 40, 40,
};

constexpr int CHARGE_TABLE_ENTRIES =
    (int)( sizeof(CHARGE_ACCEPTANCE_PERCENT_OF_C) / sizeof(CHARGE_ACCEPTANCE_PERCENT_OF_C[0]) );

/* uint16_t, not uint8_t. These are amperes derived from PACK_CAPACITY_AH, and a
 * uint8_t overflows at the 2.2C ceiling above about 116 Ah: a 117 Ah pack
 * computes 257 A and stores 1.
 *
 * That was never a silent wrong answer -- the static_asserts below caught it and
 * the build failed -- but they caught it by the wrong end. A 117 Ah pack was
 * rejected with "CHARGE_C_RATE_PERCENT is above the charge acceptance curve's
 * own ceiling", sending whoever hit it to re-read a datasheet about a C-rate
 * that was perfectly reasonable, when the actual problem was the width of a
 * type three functions away. Widening it lets the configuration simply work. */
constexpr uint16_t c_rate(int percentOfC) {
    return (uint16_t)( ( (long)PACK_CAPACITY_AH * percentOfC + 50 ) / 100 );
}

// Amperes at a fraction of the configured sustained rate, rounded to nearest.
constexpr uint16_t policy_rate(int percentOfPlateau) {
    return (uint16_t)( ( (long)CHARGE_CURRENT_MAX_PER_PACK_A * percentOfPlateau + 50 ) / 100 );
}

constexpr uint16_t charge_amps_at(int index) {
    return ( c_rate(CHARGE_ACCEPTANCE_PERCENT_OF_C[index])
             < policy_rate(CHARGE_ACCEPTANCE_PERCENT_OF_C[index]) )
        ? c_rate(CHARGE_ACCEPTANCE_PERCENT_OF_C[index])
        : policy_rate(CHARGE_ACCEPTANCE_PERCENT_OF_C[index]);
}

constexpr uint16_t table_peak(int i = 0, uint16_t best = 0) {
    return ( i >= CHARGE_TABLE_ENTRIES )
        ? best
        : table_peak(i + 1, charge_amps_at(i) > best ? charge_amps_at(i) : best);
}

/* The highest acceptance the curve itself claims, in percent of 1C, ignoring
 * the policy knob entirely. This is the ceiling CHARGE_C_RATE_PERCENT cannot
 * usefully be raised past, and it moves only when someone edits the curve. */
constexpr int acceptance_ceiling(int i = 0, int best = 0) {
    return ( i >= CHARGE_TABLE_ENTRIES )
        ? best
        : acceptance_ceiling(i + 1,
              CHARGE_ACCEPTANCE_PERCENT_OF_C[i] > best
                  ? CHARGE_ACCEPTANCE_PERCENT_OF_C[i] : best);
}

/* Rises (or holds), then falls (or holds), and never rises again. Equal
 * neighbours are fine in either phase; what is banned is a dip in the middle. */
constexpr bool rises_then_falls(int i = 1, bool falling = false) {
    return ( i >= CHARGE_TABLE_ENTRIES )
        ? true
        : ( charge_amps_at(i) < charge_amps_at(i - 1) )
            ? rises_then_falls(i + 1, true)
            : ( falling && charge_amps_at(i) > charge_amps_at(i - 1) )
                ? false
                : rises_then_falls(i + 1, falling);
}

static_assert(CHARGE_TABLE_ENTRIES == ( 39 - (-10) + 1 ),
    "the acceptance curve must have exactly one entry per whole degree from -10C to +39C");
static_assert(table_peak() > 0,
    "CHARGE_C_RATE_PERCENT rounds every table entry to zero amps");
/* Guards the widening above: if a future pack is large enough that the derived
 * amperes no longer fit the type, this fires rather than wrapping silently. */
static_assert(( (long)PACK_CAPACITY_AH * acceptance_ceiling() + 50 ) / 100 <= 0xFFFF,
    "the charge acceptance curve derives an ampere figure too large for uint16_t; "
    "widen c_rate()/policy_rate() and charge_current_for_temperature()");
static_assert(table_peak() <= CHARGE_CURRENT_MAX_PER_PACK_A,
    "a table entry exceeds the configured sustained charge rate");
/* Going past the curve is not a dial you turn; it is a claim about the cells
 * that has to be made in CHARGE_ACCEPTANCE_PERCENT_OF_C, with a source in hand.
 * The curve now peaks at 2.2C, which is the donor vehicle's 20 kW recuperation
 * figure on a 26 Ah pack -- a burst rating standing in for a sustained one,
 * because 0x351 has only one field. See the REGEN note in settings.h. */
static_assert(CHARGE_CURRENT_MAX_PER_PACK_A <= c_rate(acceptance_ceiling()),
    "CHARGE_C_RATE_PERCENT is above the charge acceptance curve's own ceiling, so "
    "raising it further does nothing. To charge faster than the curve allows, raise "
    "CHARGE_ACCEPTANCE_PERCENT_OF_C against the cell datasheet.");
/* charge_current_for_temperature_range() takes the lower of the two end
 * temperatures and relies on that being the minimum across every module in
 * between. That holds only while the curve has no interior dip. */
static_assert(rises_then_falls(),
    "the charge current table must be unimodal: non-decreasing to the plateau, "
    "then non-increasing. charge_current_for_temperature_range() would step over "
    "a dip in the middle.");

}  // namespace

/* Returns 0 (no charging) outside the table's window, which also covers the
 * -126 / 126 sentinels the temperature getters return before any module data
 * has arrived. The call sites below used to index the array directly with no
 * range check at all. */
uint16_t BatteryPack::charge_current_for_temperature(int8_t temperature) {
    const int index = (int)temperature + 10;
    if ( index < 0 || index >= CHARGE_TABLE_ENTRIES ) {
        return 0;
    }
    return (uint16_t)charge_amps_at(index);
}

/* Returns the maximum charge current as a function of pack temperature. */
/*
 * Charge current the pack can accept, given the spread between its coldest and
 * hottest sensor.
 *
 * Both ends matter, and they are limited by different mechanisms. The cold end
 * is lithium plating: as the graphite anode's intercalation kinetics slow,
 * current pushed into a cold cell deposits as metallic lithium instead of
 * intercalating, which is irreversible capacity loss and a dendrite risk. The
 * hot end is thermal stress and calendar ageing. A pack is never at one
 * temperature -- a few degrees of internal gradient is normal, and much larger
 * ones are documented -- so no single reading can stand in for the whole thing.
 *
 * Taking the lower of the two endpoint lookups is the minimum over EVERY module
 * in the pack, not merely over the two extremes, because the table is unimodal
 * -- a static_assert above enforces that.
 */
uint16_t BatteryPack::charge_current_for_temperature_range(int8_t coldest, int8_t hottest) {
    const uint16_t coldLimit = charge_current_for_temperature(coldest);
    const uint16_t hotLimit = charge_current_for_temperature(hottest);
    return ( coldLimit < hotLimit ) ? coldLimit : hotLimit;
}

uint16_t BatteryPack::get_max_charge_current_by_temperature() {
    // Safety checks first
    if ( has_full_cell() ) {
        return 0;
    }
    if ( has_temperature_sensor_over_max() ) {
        return 0;
    }
    if ( get_lowest_temperature() < CHARGE_TEMPERATURE_MINIMUM ) {
        return 0;
    }

    /* Keyed on BOTH ends of the pack's spread.
     *
     * This used to look up get_highest_temperature() alone, which is correct at
     * the hot end and wrong at the cold end, and did not fail safe: modules at
     * -8C and +14C in the same pack clear the CHARGE_TEMPERATURE_MINIMUM guard
     * above (it tests the coldest) and were then handed the +14C table entry --
     * 111 A into a module entitled to 4 A. */
    const uint16_t currentLimit = charge_current_for_temperature_range(
        get_lowest_temperature(), get_highest_temperature());

    /* The rate-of-rise derate only engages once something in the pack is above
     * CHARGE_TEMPERATURE_DERATING_MINIMUM (15C). Deliberately still keyed on the
     * HOTTEST sensor: it is a thermal guard, so it should engage as soon as any
     * part of the pack is warm enough for self-heating to matter rather than
     * waiting for the coldest module to catch up. */
    if ( get_highest_temperature() < CHARGE_TEMPERATURE_DERATING_MINIMUM ) {
        return currentLimit;
    }

    /* Above that, allow CHARGE_TEMPERATURE_DERATING_THRESHOLD (1C) of rise per
     * minute, and scale back 10% for every further degree per minute. */
    if ( temperatureDelta < CHARGE_TEMPERATURE_DERATING_THRESHOLD ) {
        return currentLimit;
    }
    const int degreesOverThreshold = temperatureDelta - CHARGE_TEMPERATURE_DERATING_THRESHOLD;
    if ( degreesOverThreshold >= 10 ) {
        return 0;
    }
    /* The old expression was
     *   (10 - temperatureDelta - THRESHOLD) / 100
     * which is integer division by 100 and so evaluated to 0 for every reachable
     * input -- the derated charge current was always zero. It also had the wrong
     * shape: the intent is 1 - 0.1*excess, not (10 - excess)/100. */
    const float derateScaleFactor = 1.0f - ( 0.1f * (float)degreesOverThreshold );
    return (uint16_t)( (float)currentLimit * derateScaleFactor );
}