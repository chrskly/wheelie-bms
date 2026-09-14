/*
 * This file is part of the ev mustang bms project.
 *
 * Copyright (C) 2025 Christian Kelly <chrskly@chrskly.com>
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

#ifndef BMS_SRC_INCLUDE_WEBSTATUS_H_
#define BMS_SRC_INCLUDE_WEBSTATUS_H_

#include <stddef.h>
#include <stdint.h>

#include "settings.h"

/*==============================================================================
 * Snapshot of everything the web interface displays.
 *
 * WHY A SNAPSHOT AND NOT DIRECT GETTERS.
 *
 * Battery, BatteryPack, BatteryModule, Shunt and Bms are owned by the BMS
 * worker task, which is the only thing allowed to enter the state machine or
 * touch battery state (see bms_worker_task). The HTTP server runs on its own
 * task, at a lower priority, and on a dual-core part it can genuinely run at
 * the same instant on the other core. Calling get_lowest_cell_voltage() from
 * there would read half-updated arrays while a CAN frame is being decoded, and
 * a serialisation that takes several milliseconds would produce a page whose
 * top half and bottom half describe different moments.
 *
 * So the worker publishes a consistent copy of everything once every
 * WEB_SNAPSHOT_INTERVAL_MS, and the web task only ever reads that copy. No BMS
 * object is reachable from the web task at all.
 *
 * HOW THE HANDOFF IS SYNCHRONISED.
 *
 * A seqlock, not a mutex. The publishing side must never block: it runs inside
 * a worker tick that a 5 second hardware watchdog is watching, and a mutex held
 * by a lower-priority web task that has been preempted would stall it. With a
 * seqlock the writer never waits for anything; a reader that catches a write in
 * progress simply retries. Readers are also rare (one HTTP request) and the
 * copy is under a kilobyte, so a retry essentially never happens in practice.
 *
 * Deliberately free of Arduino, FreeRTOS and WiFi dependencies so it builds and
 * is tested in the native test suite.
 *============================================================================*/

struct WebModuleSnapshot {
    uint16_t cellVoltage[CELLS_PER_MODULE];        // mV
    int8_t   cellTemperature[TEMPS_PER_MODULE];    // degrees C, NO_TEMPERATURE_READING if unfitted
    uint32_t balanceStatus;                        // as reported by the module in its 0x10X frame
    uint32_t errorStatus;                          // ditto
    bool     alive;                                // heard from within MODULE_TTL_MS
    bool     populated;                            // every voltage group has been received
};

struct WebPackSnapshot {
    uint32_t voltage;                              // mV
    uint16_t lowestCellVoltage;                    // mV
    uint16_t highestCellVoltage;                   // mV
    uint16_t cellDelta;                            // mV
    int8_t   lowestTemperature;                    // degrees C
    int8_t   highestTemperature;                   // degrees C
    bool     alive;
    bool     contactorsInhibited;
    bool     contactorsWelded;
    bool     balancing;
    bool     hasDeadCell;
    uint16_t balanceTargetMv;                      // mV, 0 when not balancing
    uint16_t canTxErrors;
    uint16_t canRxErrors;
    WebModuleSnapshot modules[MODULES_PER_PACK];
};

struct WebSnapshot {
    bool        valid;                             // false until the worker has published once
    uint64_t    uptimeMs;                          // get_clock_ms() when this snapshot was taken

    // ---- state machine ----
    const char* stateName;                         // points at a string literal, safe to share
    uint64_t    timeInStateMs;
    bool        illegalStateTransition;
    uint16_t    invalidEventCount;
    bool        watchdogReboot;                    // did the last boot come from a watchdog reset

    // ---- inhibits and outputs ----
    bool        driveInhibited;
    uint16_t    driveInhibitReasons;               // bitmask of InhibitReason
    bool        chargeInhibited;
    uint16_t    chargeInhibitReasons;              // bitmask of InhibitReason
    bool        heaterOn;
    bool        ignitionOn;
    bool        chargeEnabled;

    // ---- limits and charge state ----
    uint8_t     soc;                               // %
    uint16_t    maxChargeCurrent;                  // A
    uint16_t    maxDischargeCurrent;               // A

    // ---- faults ----
    uint8_t     internalErrorFlags;                // bitmask of InternalErrorSource
    uint8_t     errorByte;
    uint8_t     statusByte;
    uint8_t     weldingByte;

    // ---- battery ----
    uint32_t    voltage;                           // mV, whole battery
    uint16_t    lowestCellVoltage;                 // mV
    uint16_t    highestCellVoltage;                // mV
    uint16_t    cellDelta;                         // mV
    int8_t      lowestTemperature;                 // degrees C
    int8_t      highestTemperature;                // degrees C
    bool        batteryAlive;
    bool        tooHot;
    bool        tooColdToCharge;
    bool        hasEmptyCell;
    bool        hasFullCell;
    bool        hasDeadCell;
    bool        packsImbalanced;
    uint8_t     activePacks;

    // ---- ISA shunt. Raw shunt units, exactly as Shunt stores them ----
    bool        shuntAlive;
    int32_t     shuntAmps;                         // mA
    int32_t     shuntVoltage1;                     // mV
    int32_t     shuntVoltage2;                     // mV
    int32_t     shuntVoltage3;                     // mV
    int32_t     shuntTemperature;                  // degrees C
    int32_t     shuntWatts;                        // W
    int32_t     shuntAmpSeconds;                   // As
    int32_t     shuntWattHours;                    // Wh

    // ---- main CAN bus ----
    uint32_t    mainCanTxErrors;
    uint32_t    mainCanRxErrors;

    WebPackSnapshot packs[NUM_PACKS];
};

/*
 * Publish a new snapshot. Called ONLY from the BMS worker task. Never blocks.
 */
void webstatus_publish(const WebSnapshot& snapshot);

/*
 * Take a stable copy of the most recent snapshot. Called from the web task.
 *
 * Returns false if nothing has been published yet, or if the writer kept
 * interrupting for longer than a bounded number of retries -- in which case
 * `out` is left untouched and the caller should tell the client to come back.
 */
bool webstatus_read(WebSnapshot& out);

/*
 * Render a snapshot as JSON into `buf`.
 *
 * Returns the number of bytes written, not counting the terminating NUL, or 0
 * if the buffer was too small. A truncated response is never returned: the
 * caller gets 0 and can answer with an error instead of handing the browser
 * half an object.
 */
size_t webstatus_render_json(const WebSnapshot& snapshot, char* buf, size_t bufLen);

#endif  // BMS_SRC_INCLUDE_WEBSTATUS_H_
