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

/*

    MODULE

*/

#include <stdio.h>
#include <time.h>

#include "module.h"
#include "pack.h"
#include "webstatus.h"
#include "util.h"


BatteryModule::BatteryModule() {
    /* Fill the full extent of both arrays, so a module that is never init()ed
     * (or one whose sensor count gets clamped) still reports "no reading"
     * rather than a stale zero. */
    for ( int c = 0; c < CELLS_PER_MODULE; c++ ) {
        cellVoltage[c] = 0;
    }
    for ( int t = 0; t < TEMPS_PER_MODULE; t++ ) {
        cellTemperature[t] = NO_TEMPERATURE_READING;
    }
}

//
void BatteryModule::init(int _id, BatteryPack* _pack, int _numCells, int _numTemperatureSensors) {
    // printf("Creating module (id:%d, pack:%d, cpm:%d, t:%d)\n", _id, _pack->id, _numCells, _numTemperatureSensors);
    id = _id;
    // Point back to parent pack
    pack = _pack;
    // Initialise all cell voltages to zero
    if ( _numCells > CELLS_PER_MODULE ) {
        printf("[module%d] ERROR numCells %d exceeds CELLS_PER_MODULE %d, clamping\n",
               _id, _numCells, CELLS_PER_MODULE);
        _numCells = CELLS_PER_MODULE;
    }
    if ( _numCells < 0 ) {
        _numCells = 0;
    }
    numCells = _numCells;
    for ( int c = 0; c < CELLS_PER_MODULE; c++ ) {
        cellVoltage[c] = 0;
    }
    // Initialise temperature sensor readings to zero
    if ( _numTemperatureSensors > TEMPS_PER_MODULE ) {
        printf("[module%d] ERROR numTemperatureSensors %d exceeds TEMPS_PER_MODULE %d, clamping\n",
               _id, _numTemperatureSensors, TEMPS_PER_MODULE);
        _numTemperatureSensors = TEMPS_PER_MODULE;
    }
    if ( _numTemperatureSensors < 0 ) {
        _numTemperatureSensors = 0;
    }
    numTemperatureSensors = _numTemperatureSensors;
    for ( int t = 0; t < TEMPS_PER_MODULE; t++ ) {
        cellTemperature[t] = NO_TEMPERATURE_READING;
    }
    /* Reset ALL per-module runtime state, not just the populated flag.
     * Leaving lastHeartbeat and hasReported behind meant a re-initialised
     * module claimed to be alive on the strength of pre-init history, and a
     * stale voltageGroupsSeen could declare it populated over stale cell
     * voltages. init() runs once per module in production, but "runs once" is
     * an assumption, not a guarantee. */
    allModuleDataPopulated = false;
    voltageGroupsSeen = 0;
    hasReported = false;
    lastHeartbeat = 0;
    balanceStatus = 0;
    errorStatus = 0;
}

void BatteryModule::fill_snapshot(WebModuleSnapshot& out) {
    /* Slots beyond this module's configured cell/sensor count are filled with
     * the "no reading" sentinels rather than left as whatever the previous
     * snapshot put there, so a module fitted with fewer sensors than the array
     * allows does not display a stale value from another module. */
    for ( int c = 0; c < CELLS_PER_MODULE; c++ ) {
        out.cellVoltage[c] = ( c < numCells ) ? cellVoltage[c] : 0;
    }
    for ( int t = 0; t < TEMPS_PER_MODULE; t++ ) {
        out.cellTemperature[t] = ( t < numTemperatureSensors )
                                 ? cellTemperature[t]
                                 : (int8_t)NO_TEMPERATURE_READING;
    }
    out.balanceStatus = balanceStatus;
    out.errorStatus = errorStatus;
    out.alive = is_alive();
    out.populated = allModuleDataPopulated;
}

void BatteryModule::print() {
    // printf("    Module id : %d (numCells : %d)\n", id, numCells);
    // printf("        Cell Voltages : ");
    // for ( int c = 0; c < numCells; c++ ) {
    //     printf("%d:%umV ", c, cellVoltage[c]);
    // }
    // printf("\n");
    // printf("        Temperatures : ");
    // for ( int t = 0; t < numTemperatureSensors; t++ ) {
    //     printf("%d:%dC ", t, cellTemperature[t]);
    // }
    // printf("\n");
    printf("  %d : ", id);
    for ( int c = 0; c < numCells; c++ ) {
        printf("%u ", cellVoltage[c]);
    }
    if ( errorStatus != 0 ) {
        printf(" ERR:0x%08X", (unsigned int)errorStatus);
    }
    printf("\n");
}


//// ----
//
// Voltage
//
//// ----

// Return total module voltage by summing the cell voltages
uint32_t BatteryModule::get_voltage() {
    uint32_t voltage = 0;
    for ( int c = 0; c < numCells; c++ ) {
        voltage += cellVoltage[c];
    }
    return voltage;
}

// Return the voltage of the lowest cell voltage in the module
uint16_t BatteryModule::get_lowest_cell_voltage() {
    uint16_t lowestCellVoltage = NO_CELL_VOLTAGE_READING;
    for ( int c = 0; c < numCells; c++ ) {
        if ( cellVoltage[c] < lowestCellVoltage ) {
            lowestCellVoltage = cellVoltage[c];
        }
    }
    return lowestCellVoltage;
}

// Return the voltage of the highest cell in the module
uint16_t BatteryModule::get_highest_cell_voltage() {
    uint16_t highestCellVoltage = 0;
    for ( int c = 0; c < numCells; c++ ) {
        if ( cellVoltage[c] > highestCellVoltage ) {
            highestCellVoltage = cellVoltage[c];
        }
    }
    return highestCellVoltage;
}

// Update the voltage for a single cell
void BatteryModule::set_cell_voltage(int cellIndex, uint16_t newCellVoltage) {
    if ( cellIndex < 0 || cellIndex >= numCells ) {
        return;
    }
    cellVoltage[cellIndex] = newCellVoltage;
}

/* Return true if any of the cells in the module are under min voltage.
 *
 * These three predicates all bail out when the module has not reported yet.
 * Every cell reads 0 mV before the first update, which used to make the whole
 * battery look empty and full of dead cells from power-on. An unresponsive
 * module is caught by is_alive() instead. */
bool BatteryModule::has_empty_cell() {
    if ( !allModuleDataPopulated ) {
        return false;
    }
    for ( int c = 0; c < numCells; c++ ) {
        if ( cellVoltage[c] <= CELL_EMPTY_VOLTAGE ) {
            return true;
        }
    }
    return false;
}

// Return true if any of the cells in the module are over max voltage
bool BatteryModule::has_full_cell() {
    if ( !allModuleDataPopulated ) {
        return false;
    }
    for ( int c = 0; c < numCells; c++ ) {
        if ( cellVoltage[c] >= CELL_FULL_VOLTAGE ) {
            return true;
        }
    }
    return false;
}

/*
 * Check for dead cells in the module
 */
bool BatteryModule::has_dead_cell() {
    if ( !allModuleDataPopulated ) {
        return false;
    }
    for ( int c = 0; c < numCells; c++ ) {
        if ( cellVoltage[c] <= DEAD_CELL_VOLTAGE ) {
            return true;
        }
    }
    return false;
}

//// ----
//
// Status
//
//// ----

// Return true if we have voltage/temp information for all cells
bool BatteryModule::all_module_data_populated() {
    return allModuleDataPopulated;
}

void BatteryModule::check_if_module_data_is_populated() {
    /* Every voltage group received and stored. Testing for "no cell is 0 mV"
     * instead meant a dead cell hid its own module. */
    const uint8_t allGroups = 0x3F;   // six groups, 0x020 through 0x070
    const bool voltageMissing = ( voltageGroupsSeen != allGroups );
    /* At least ONE valid temperature, not all of them. Slots with no sensor
     * fitted report a raw count of 0 and are stored as NO_TEMPERATURE_READING,
     * which will never change -- requiring every slot meant such a module could
     * never become populated, so it was excluded from every min/max, its pack
     * never withdrew CI_STARTUP, and the contactors stayed inhibited forever. */
    bool haveATemperature = false;
    for ( int t = 0; t < numTemperatureSensors; t++ ) {
        if ( cellTemperature[t] > NO_TEMPERATURE_READING ) {
            haveATemperature = true;
            break;
        }
    }
    allModuleDataPopulated = !voltageMissing && haveATemperature;
}

bool BatteryModule::is_alive() {
    /* A module we have never heard from is not alive. Tracked with an explicit
     * flag rather than "lastHeartbeat == 0", which misread a heartbeat that
     * happened to land on millisecond zero. */
    if ( !hasReported ) {
        /* Nothing heard from this module yet. For the first MODULE_TTL_MS after
         * boot that is simply expected -- the polling sweep has not finished --
         * so give it the same window a live module gets before calling it dead.
         * Reporting "not alive" from the first instant dropped the BMS into
         * criticalFault on every single boot. Data-dependent paths are guarded
         * separately by all_module_data_populated() and by CI_STARTUP. */
        return get_clock_ms() < MODULE_TTL_MS;
    }
    return ( get_clock_ms() - lastHeartbeat ) < MODULE_TTL_MS;
}

void BatteryModule::heartbeat() {
    lastHeartbeat = get_clock_ms();
    hasReported = true;
}

/* Record that a voltage message group was received and its values stored. */
void BatteryModule::note_voltage_group(int messageId) {
    const int group = ( messageId >> 4 ) - 2;   // 0x020 -> 0 ... 0x070 -> 5
    if ( group < 0 || group > 5 ) {
        return;
    }
    voltageGroupsSeen |= (uint8_t)( 1u << group );
}

//// ----
//
// Temperature
//
//// ----

// Update the value for one of the temperature sensors
void BatteryModule::update_temperature(int tempSensorId, int8_t newTemperature) {
    if ( tempSensorId < 0 || tempSensorId >= numTemperatureSensors ) {
        return;
    }
    cellTemperature[tempSensorId] = newTemperature;
}

// Return the temperature of the coldest sensor in the module
int8_t BatteryModule::get_lowest_temperature() {
    int8_t lowestTemperature = 126;
    for ( int t = 0; t < numTemperatureSensors; t++ ) {
        // Skip uninitialised readings
        if ( cellTemperature[t] < -126 ) {
            continue;
        }
        if ( cellTemperature[t] < lowestTemperature ) {
            lowestTemperature = cellTemperature[t];
        }
    }
    //printf("lowest temp : %d\n", lowestTemperature);
    return lowestTemperature;
}

// Return the temperature of the hottest sensor in the module
int8_t BatteryModule::get_highest_temperature() {
    int8_t highestTemperature = -126;
    for ( int t = 0; t < numTemperatureSensors; t++ ) {
        // Skip uninitialised readings
        if ( cellTemperature[t] < -126 ) {
            continue;
        }
        if ( cellTemperature[t] > highestTemperature ) {
            highestTemperature = cellTemperature[t];
        }
    }
    return highestTemperature;
}

// Return true if any temperature sensor is over the max temperature
bool BatteryModule::has_temperature_sensor_over_max() {
    return ( get_highest_temperature() > MAXIMUM_TEMPERATURE );
}
