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

#include "battery.h"
#include "bms.h"
#include "pack.h"
#include "io.h"
#include "settings.h"
#include "util.h"
#include "webstatus.h"







// Create all battery packs and modules
void Battery::initialise(Bms* _bms) {

    voltage = 0;
    lowestCellVoltage = 0;
    highestCellVoltage = 0;
    lowestSensorTemperature = 126;      // see the note in battery.h: sentinels,
    highestSensorTemperature = -126;    // not 0, which is a real temperature
    numPacks = NUM_PACKS;
    bms = _bms;

    for ( int p = 0; p < numPacks; p++ ) {
        printf("[battery] Initialising battery pack %d (CS:%d, inh:%d, mod/pack:%d, cell/mod:%d, T/mod:%d)\n", p, CS_PINS[p], INHIBIT_CONTACTOR_PINS[p], MODULES_PER_PACK, CELLS_PER_MODULE, TEMPS_PER_MODULE);
        BatteryPackConfig config;
        config.id                             = p;
        config.canChipSelectPin               = CS_PINS[p];
        config.contactorInhibitPin            = INHIBIT_CONTACTOR_PINS[p];
        config.contactorFeedbackPin           = CONTACTOR_FEEDBACK_PINS[p];
        config.numModules                     = MODULES_PER_PACK;
        config.numCellsPerModule              = CELLS_PER_MODULE;
        config.numTemperatureSensorsPerModule = TEMPS_PER_MODULE;
        packs[p].init(config);
        packs[p].set_battery(this);
        printf("[battery] Initialisation of battery pack %d complete\n", p);
    }

    // Precalculate min/max battery voltages
    maximumBatteryVoltage = CELL_FULL_VOLTAGE * CELLS_PER_MODULE * MODULES_PER_PACK;
    minimumBatteryVoltage = CELL_EMPTY_VOLTAGE * CELLS_PER_MODULE * MODULES_PER_PACK;

}

void Battery::fill_snapshot(WebSnapshot& out) {
    out.voltage = get_voltage();
    out.lowestCellVoltage = get_lowest_cell_voltage();
    out.highestCellVoltage = get_highest_cell_voltage();
    out.cellDelta = get_cell_delta();
    /* The pack getters return -126 when no module has reported, which is a
     * sentinel and not a reading. Pass it on as NO_TEMPERATURE_READING, the
     * sentinel the page already understands from the per-module sensors, rather
     * than as a plausible-looking -126 C. */
    out.lowestTemperature = have_temperature_reading()
                          ? get_lowest_sensor_temperature() : NO_TEMPERATURE_READING;
    out.highestTemperature = have_temperature_reading()
                           ? get_highest_sensor_temperature() : NO_TEMPERATURE_READING;
    out.batteryAlive = is_alive();
    out.tooHot = too_hot();
    out.tooColdToCharge = too_cold_to_charge();
    out.hasEmptyCell = has_empty_cell();
    out.hasFullCell = has_full_cell();
    out.hasDeadCell = has_dead_cell();
    out.packsImbalanced = packs_are_imbalanced();
    out.activePacks = number_of_active_packs();
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].fill_snapshot(out.packs[p]);
    }
}

int Battery::print() {
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].print();
    }
    return 0;
}

// Send messages to all packs to request voltage/temperature data
void Battery::request_data() {
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].request_data();
    }
}

/*
 * Check for and read messages from each pack
 */
void Battery::read_message() {
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].poll_can();
        packs[p].read_message();
    }
}

/* Called from the health check, not from the 5 ms drain: reading EFLG is an SPI
 * transaction per pack and the flags it reports latch, so there is nothing to
 * gain from sampling it at tick rate. */
void Battery::check_pack_can_health() {
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].check_can_health();
    }
}


bool Battery::has_multiple_packs() {
    return numPacks > 1;
}

uint8_t Battery::number_of_active_packs() {
    uint8_t count = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( !packs[p].contactors_are_inhibited() ) {
            count++;
        }
    }
    return count;
}


//// ----
//
// Voltage
//
//// ----

// Return the voltage of the whole battery
uint32_t Battery::get_voltage() {
    return voltage;
}

// Recompute and store the battery voltage based on current cell voltages
void Battery::recalculate_voltage() {
    /* Packs are paralleled, so when their contactors are closed they sit at the
     * same terminal voltage. Average the packs that are actually connected;
     * taking the maximum across all packs (as this used to) both hides a
     * sagging pack and includes packs that are inhibited and therefore not
     * contributing. Fall back to the highest healthy pack if none are active,
     * so we still report something rather than zero. */
    uint64_t total = 0;
    int contributing = 0;
    uint32_t highestHealthy = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].has_dead_cell() ) {
            continue;
        }
        const uint32_t packVoltage = packs[p].get_voltage();
        if ( packVoltage > highestHealthy ) {
            highestHealthy = packVoltage;
        }
        if ( !packs[p].contactors_are_inhibited() ) {
            total += packVoltage;
            contributing++;
        }
    }
    voltage = ( contributing > 0 ) ? (uint32_t)( total / contributing ) : highestHealthy;
}


// Return the maximum allowed voltage of the whole battery
uint32_t Battery::get_max_voltage() {
    return maximumBatteryVoltage;
}

// Return the minimum allowed voltage of the whole battery
uint32_t Battery::get_min_voltage() {
    return minimumBatteryVoltage;
}

// Return the id of the pack that has the highest voltage
int Battery::get_index_of_high_pack() {
    // If we only have one pack, then just return that
    if ( !has_multiple_packs() ) {
        return 0;
    }
    int high_pack_index = 0;
    uint32_t high_pack_voltage = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        // Exclude packs with dead cells, they should be inhibited
        if ( packs[p].has_dead_cell() ) {
            continue;
        }
        const uint32_t packVoltage = packs[p].get_voltage();
        if ( packVoltage > high_pack_voltage ) {
            high_pack_index = p;
            high_pack_voltage = packVoltage;
        }
    }
    return high_pack_index;
}

/*
 * Return the id of the pack that has the lowest voltage. Exclude any packs with
 * dead cells.
 */
int Battery::get_index_of_low_pack() {
    // If we only have one pack, then just return that
    if ( !has_multiple_packs() ) {
        return 0;
    }
    /* The sentinel must start ABOVE every possible pack voltage. It used to be
     * 1000 (mV), which no real ~384,000 mV pack is ever below, so the loop
     * never fired and this always returned pack 0. */
    int low_pack_index = -1;
    uint32_t low_pack_voltage = UINT32_MAX;
    for ( int p = 0; p < numPacks; p++ ) {
        // Exclude packs with dead cells, they should be inhibited
        if ( packs[p].has_dead_cell() ) {
            continue;
        }
        const uint32_t packVoltage = packs[p].get_voltage();
        if ( packVoltage < low_pack_voltage ) {
            low_pack_index = p;
            low_pack_voltage = packVoltage;
        }
    }
    return ( low_pack_index < 0 ) ? 0 : low_pack_index;
}

/*
 * We have new cell voltage data. Process it.
 */
void Battery::process_voltage_update() {
    // Do processing for each pack
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].process_voltage_update();
    }
    // Do processing for overall battery
    recalculate_voltage();
    recalculate_lowest_cell_voltage();
    recalculate_highest_cell_voltage();
    /* The BMS infers "imbalanced" from the ABSENCE of a recent match, so there
     * are three cases here, not two, and the third is easy to get wrong.
     *
     *   comparable and matched     -> refresh: the packs demonstrably agree.
     *   comparable and mismatched  -> leave it: let the timer run and latch.
     *   not comparable             -> refresh, which pauses the timer.
     *
     * The last one looks like "treat unmeasurable as matched", which is exactly
     * the bug that let the imbalance hold withdraw itself -- but that loop only
     * existed because inhibiting the contactors was itself what made the packs
     * unmeasurable. pack_voltage_is_comparable() no longer looks at contactor
     * state, so asserting the hold cannot feed back into the measurement and
     * the loop is gone. Letting the timer run while unmeasurable instead means
     * a 60 s balance burst -- during which readings are deliberately discarded
     * -- decays into a declared imbalance after 3 s and holds every contactor
     * open for the rest of the burst, on packs that are perfectly matched. */
    if ( bms != nullptr && ( !pack_voltages_are_comparable() || !packs_are_imbalanced() ) ) {
        this->bms->pack_voltages_match_heartbeat();
    }
}


// Low cells

// Recompute the lowest cell voltage across the whole battery
void Battery::recalculate_lowest_cell_voltage() {
    uint16_t newLowestCellVoltage = NO_CELL_VOLTAGE_READING;
    for ( int p = 0; p < numPacks; p++ ) {
        const uint16_t packLowestCellVoltage = packs[p].get_lowest_cell_voltage();
        if ( packLowestCellVoltage < newLowestCellVoltage ) {
            newLowestCellVoltage = packLowestCellVoltage;
        }
    }
    /* Safety check. "No reading yet" is not an internal error -- module
     * liveness covers that case -- and the flag is cleared again once the
     * readings come back into range, which the old single latching bool
     * never did. */
    const bool haveLowReading = ( newLowestCellVoltage != NO_CELL_VOLTAGE_READING );
    const bool lowOutOfRange = haveLowReading &&
        ( newLowestCellVoltage < CELL_EMPTY_VOLTAGE ||
          newLowestCellVoltage > CELL_FULL_VOLTAGE );
    if ( bms != nullptr ) {
        if ( lowOutOfRange ) {
            bms->set_internal_error(IE_LOW_CELL_RANGE);
        } else {
            bms->clear_internal_error(IE_LOW_CELL_RANGE);
        }
    }
    lowestCellVoltage = newLowestCellVoltage;
}

uint16_t Battery::get_lowest_cell_voltage() {
    return lowestCellVoltage;
}

// Return true if any cell in the battery is below the minimum voltage level
/* Reports across ALL packs, including inhibited ones. Skipping inhibited packs
 * meant this was unconditionally false at power-on, when every pack is held by
 * CI_STARTUP -- and it hid a genuinely empty cell in a pack that happened to be
 * inhibited for an unrelated reason. */
bool Battery::has_empty_cell() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].has_empty_cell() ) {
            return true;
        }
    }
    return false;
}

// High cells

// Recompute the highest cell voltage
void Battery::recalculate_highest_cell_voltage() {
    uint16_t newHighestCellVoltage = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        const uint16_t packHighestCellVoltage = packs[p].get_highest_cell_voltage();
        if ( packHighestCellVoltage > newHighestCellVoltage ) {
            newHighestCellVoltage = packHighestCellVoltage;
        }
    }
    // See recalculate_lowest_cell_voltage(): zero means "no reading yet".
    const bool haveHighReading = ( newHighestCellVoltage != 0 );
    const bool highOutOfRange = haveHighReading &&
        ( newHighestCellVoltage < CELL_EMPTY_VOLTAGE ||
          newHighestCellVoltage > CELL_FULL_VOLTAGE );
    if ( bms != nullptr ) {
        if ( highOutOfRange ) {
            bms->set_internal_error(IE_HIGH_CELL_RANGE);
        } else {
            bms->clear_internal_error(IE_HIGH_CELL_RANGE);
        }
    }
    highestCellVoltage = newHighestCellVoltage;
}

uint16_t Battery::get_highest_cell_voltage() {
    return highestCellVoltage;
}

// Return true if any cell in the battery is below the minimum voltage level
// See has_empty_cell(): reports across all packs, inhibited or not.
bool Battery::has_full_cell() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].has_full_cell() ) {
            return true;
        }
    }
    return false;
}

/*
 * Return the largest voltage difference between any two packs in this battery.
 */
/*
 * Can this pack's voltage be meaningfully compared with another pack's right
 * now?
 *
 * Contactor state deliberately plays no part. A pack held open because of an
 * imbalance still reports its voltage perfectly well, and that reading is
 * exactly what decides when it is safe to reconnect it. Excluding inhibited
 * packs made the imbalance protection erase its own evidence: asserting the
 * hold dropped the comparable-pack count below two, the delta collapsed to 0,
 * that read as "the packs match", and the hold was withdrawn again about three
 * seconds later. Measured over 20 s with the packs a genuine 7.68 V apart, the
 * contactors ended up permitted to close 97% of the time.
 */
bool Battery::pack_voltage_is_comparable(int p) {
    if ( packs[p].has_dead_cell() ) {
        return false;                    // already inhibited on its own account
    }
    if ( !packs[p].is_alive() || !packs[p].all_modules_populated() ) {
        return false;                    // no complete set of readings yet
    }
    if ( packs[p].voltage_readings_are_suspect() ) {
        return false;                    // mid balance burst, or still settling
    }
    return packs[p].get_voltage() > 0;
}

/*
 * True when at least two packs can be compared, i.e. when the delta below is a
 * real measurement rather than the 0 that means "cannot tell".
 */
bool Battery::pack_voltages_are_comparable() {
    int comparable = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( pack_voltage_is_comparable(p) ) {
            comparable++;
        }
    }
    return comparable >= 2;
}

uint32_t Battery::voltage_delta_between_packs() {
    // Can't be any delta if there's only one pack
    if ( !has_multiple_packs() ) {
        return 0;
    }
    uint32_t highestPackVoltage = 0;
    uint32_t lowestPackVoltage = UINT32_MAX;
    int eligiblePacks = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( !pack_voltage_is_comparable(p) ) {
            continue;
        }
        eligiblePacks++;
        const uint32_t packVoltage = packs[p].get_voltage();
        if ( packVoltage > highestPackVoltage ) {
            highestPackVoltage = packVoltage;
        }
        if ( packVoltage < lowestPackVoltage ) {
            lowestPackVoltage = packVoltage;
        }
    }
    /* With fewer than two eligible packs there is no delta to report. This used
     * to fall through and return 0 - 1000000 as a uint32_t, i.e. ~4.29e9, which
     * read as a catastrophic imbalance. */
    if ( eligiblePacks < 2 ) {
        return 0;
    }
    return highestPackVoltage - lowestPackVoltage;
}



// Return true if the voltage difference between any two packs is too high and
// therefore it's unstafe to close the contactors.
bool Battery::packs_are_imbalanced() {
    return voltage_delta_between_packs() >= SAFE_VOLTAGE_DELTA_BETWEEN_PACKS;
}


/*
 * Return the largest cell delta of any pack in the battery.
 */
uint16_t Battery::get_cell_delta() {
    uint16_t cellDelta = 0;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].get_cell_delta() > cellDelta ) {
            cellDelta = packs[p].get_cell_delta();
        }
    }
    return cellDelta;
}

/*
 * Return true if the battery has any dead cells
 */
bool Battery::has_dead_cell() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].has_dead_cell() ) {
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

void Battery::update_highest_sensor_temperature() {
    int8_t newHighestSensorTemperature = packs[0].get_highest_temperature();
    for ( int p = 1; p < numPacks; p++ ) {
        if ( packs[p].get_highest_temperature() > newHighestSensorTemperature ) {
            newHighestSensorTemperature = packs[p].get_highest_temperature();
        }
    }
    // Saftey check
    /* -126 is the "no sensor data" sentinel from the module getters, not a
     * real reading, so it must not raise an internal error. */
    const bool haveHighTemp = ( newHighestSensorTemperature > -126 );
    const bool highTempOutOfRange = haveHighTemp &&
        ( newHighestSensorTemperature < -20 || newHighestSensorTemperature > 50 );
    if ( bms != nullptr ) {
        if ( highTempOutOfRange ) {
            bms->set_internal_error(IE_HIGH_TEMP_RANGE);
        } else {
            bms->clear_internal_error(IE_HIGH_TEMP_RANGE);
        }
    }
    this->highestSensorTemperature = newHighestSensorTemperature;
}

/*
 * Whether the sensor temperatures are real readings rather than the "no module
 * has reported" sentinels.
 *
 * There are TWO sentinels, not one, and they sit at opposite ends: the highest
 * getter starts its search at -126 and the lowest starts at +126, so that each
 * comparison works before any reading has arrived. Testing both against -126
 * only ever caught the highest -- the lowest reads +126 when there is no data,
 * which is comfortably greater than -126 and so looked like a real reading.
 */
bool Battery::have_temperature_reading() {
    return highestSensorTemperature > -126 && lowestSensorTemperature < 126;
}

int8_t Battery::get_highest_sensor_temperature() {
    return highestSensorTemperature;
}

// Return true if any sensor in the pack is over the max temperature
bool Battery::too_hot() {
    return tooHotLatched;
}

void Battery::update_lowest_sensor_temperature() {
    int8_t newLowestSensorTemperature = packs[0].get_lowest_temperature();
    for ( int p = 1; p < numPacks; p++ ) {
        if ( packs[p].get_lowest_temperature() < newLowestSensorTemperature ) {
            newLowestSensorTemperature = packs[p].get_lowest_temperature();
        }
    }
    // Safety check
    // 126 is the "no sensor data" sentinel from the module getters.
    const bool haveLowTemp = ( newLowestSensorTemperature < 126 );
    const bool lowTempOutOfRange = haveLowTemp &&
        ( newLowestSensorTemperature < -20 || newLowestSensorTemperature > 50 );
    if ( bms != nullptr ) {
        if ( lowTempOutOfRange ) {
            bms->set_internal_error(IE_LOW_TEMP_RANGE);
        } else {
            bms->clear_internal_error(IE_LOW_TEMP_RANGE);
        }
    }
    this->lowestSensorTemperature = newLowestSensorTemperature;
}

int8_t Battery::get_lowest_sensor_temperature() {
    return lowestSensorTemperature;
}

void Battery::process_temperature_update() {
    update_lowest_sensor_temperature();
    update_highest_sensor_temperature();
    update_temperature_latches();
    lastTemperatureUpdate = get_clock_ms();
    haveTemperatureData = true;
}

/*
 * True when no temperature frame has arrived recently.
 *
 * too_hot() and too_cold_to_charge() read latched values that are only
 * refreshed by an incoming temperature frame, so if those frames stop the
 * latches simply keep their last value -- and "not too hot" is the fail-unsafe
 * direction. Module liveness catches a module that goes silent entirely, but
 * not one that keeps sending voltages and stops sending temperatures.
 */
bool Battery::temperature_data_is_stale() {
    if ( !haveTemperatureData ) {
        return true;
    }
    return ( get_clock_ms() - lastTemperatureUpdate ) > MODULE_TTL_MS;
}


//// ----
//
// Charging
//
//// ----

/* Stale temperature data reads as too cold to charge: charging is the
 * temperature-dependent operation, and doing it blind is not acceptable. */
bool Battery::too_cold_to_charge() {
    if ( temperature_data_is_stale() ) {
        return true;
    }
    return tooColdToChargeLatched;
}

/*
 * Apply hysteresis to the temperature thresholds.
 *
 * Entering a fault condition uses the bare threshold; leaving it requires
 * TEMPERATURE_HYSTERESIS degrees of margin. Without this the health check
 * alternated E_TOO_COLD_TO_CHARGE and E_TEMPERATURE_OK on successive cycles
 * whenever the battery sat on a threshold, bouncing the state machine between
 * charging and batteryHeating.
 */
void Battery::update_temperature_latches() {
    if ( tooHotLatched ) {
        if ( highestSensorTemperature <= ( MAXIMUM_TEMPERATURE - TEMPERATURE_HYSTERESIS ) ) {
            tooHotLatched = false;
        }
    } else if ( highestSensorTemperature >= MAXIMUM_TEMPERATURE ) {
        tooHotLatched = true;
    }

    if ( tooColdToChargeLatched ) {
        if ( lowestSensorTemperature >= ( CHARGE_TEMPERATURE_MINIMUM + TEMPERATURE_HYSTERESIS ) ) {
            tooColdToChargeLatched = false;
        }
    } else if ( lowestSensorTemperature < CHARGE_TEMPERATURE_MINIMUM ) {
        tooColdToChargeLatched = true;
    }
}

/* 
 * Return the maximum charge current that the whole battery can handle based on
 * temperature. Since we cannot control how much current each pack gets, this
 * will be determined by what the pack with the lowest max charge current can
 * handle. We also have to account for packs which are inhibited.
 */
uint16_t Battery::get_max_charge_current_by_temperature() {
    // Safeties
    if ( too_hot() || too_cold_to_charge() ) {
        return 0;
    }

    /* Consider ONLY the packs that are actually connected. The old version
     * seeded activePacks at 1 and started the loop at p = 1, so pack 0 was
     * always counted as active whether or not it was inhibited, and it took the
     * minimum across every pack including inhibited ones -- one cold, inhibited
     * pack dragged the whole battery's limit to zero. */
    uint8_t activePacks = 0;
    uint16_t smallestMaxChargeCurrent = 0;

    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].contactors_are_inhibited() ) {
            continue;
        }
        const uint16_t packMax = packs[p].get_max_charge_current_by_temperature();
        if ( activePacks == 0 || packMax < smallestMaxChargeCurrent ) {
            smallestMaxChargeCurrent = packMax;
        }
        activePacks++;
    }

    if ( activePacks == 0 ) {
        return 0;
    }
    return smallestMaxChargeCurrent * activePacks;
}


//// ----
//
// Contactor control
//
//// ----

// Do not allow any contactors to close in any pack
void Battery::enable_inhibit_contactor_close() {
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].enable_inhibit_contactor_close(CI_IMBALANCE);
    }
}

/*
 * Disable inhibit contactor close for all packs unless they have a dead cell
 */
void Battery::disable_inhibit_contactor_close() {
    /* Withdraws only the imbalance hold. A pack held for a dead cell or still
     * in startup keeps its own reason bit and stays inhibited, so the explicit
     * has_dead_cell() check the old version needed here is now redundant. */
    for ( int p = 0; p < numPacks; p++ ) {
        packs[p].disable_inhibit_contactor_close(CI_IMBALANCE);
    }
}

/*
 * During drive, only allow packs that are close in voltage to the highest pack
 * to close their contactors.
 */
void Battery::disable_inhibit_contactors_for_drive() {
    reevaluate_contactor_inhibition_for_drive();
}

/*
 * During charge, only allow packs that are close in voltage to the lowest pack
 * to close their contactors.
 */
void Battery::disable_inhibit_contactors_for_charge() {
    reevaluate_contactor_inhibition_for_charge();
}

// If any of the packs have their contactors inhibited, return true
bool Battery::one_or_more_contactors_inhibited() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].contactors_are_inhibited() ) {
            return true;
        }
    }
    return false;
}

/*
 * Allow contactors to close for the high pack and any other packs which are
 * within SAFE_VOLTAGE_DELTA_BETWEEN_PACKS volts. Don't allow contactors for
 * packs with dead cells to close. Don't allow any other contactors to close.
 */
void Battery::reevaluate_contactor_inhibition_for_drive() {
    if ( !has_multiple_packs() ) {
        return;
    }
    int highPackId = get_index_of_high_pack();
    uint32_t highPackVoltage = packs[highPackId].get_voltage();
    /* No usable voltage reading yet: do not let anything close. */
    if ( highPackVoltage == 0 ) {
        for ( int p = 0; p < numPacks; p++ ) {
            packs[p].enable_inhibit_contactor_close(CI_IMBALANCE);
        }
        return;
    }
    /* Saturate rather than wrap. highPackVoltage - SAFE_VOLTAGE_DELTA underflows
     * to ~4.29e9 for any pack voltage below the delta, which then inhibited
     * every other pack. */
    uint32_t targetVoltage = ( highPackVoltage > SAFE_VOLTAGE_DELTA_BETWEEN_PACKS )
                           ? ( highPackVoltage - SAFE_VOLTAGE_DELTA_BETWEEN_PACKS )
                           : 0;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( p == highPackId ) {
            packs[p].disable_inhibit_contactor_close(CI_IMBALANCE);
            continue;
        }
        if ( packs[p].get_voltage() >= targetVoltage ) {
            packs[p].disable_inhibit_contactor_close(CI_IMBALANCE);
        } else {
            packs[p].enable_inhibit_contactor_close(CI_IMBALANCE);
        }
    }
}

/*
 * Allow contactors to close for the low pack and any other packs which are
 * within SAFE_VOLTAGE_DELTA_BETWEEN_PACKS volts. Don't allow contactors for
 * packs with dead cells to close. Don't allow any other contactors to close.
 */
void Battery::reevaluate_contactor_inhibition_for_charge() {
    if ( !has_multiple_packs() ) {
        return;
    }
    int lowPackId = get_index_of_low_pack();
    uint32_t lowPackVoltage = packs[lowPackId].get_voltage();
    // No usable voltage reading yet: do not let anything close.
    if ( lowPackVoltage == 0 ) {
        for ( int p = 0; p < numPacks; p++ ) {
            packs[p].enable_inhibit_contactor_close(CI_IMBALANCE);
        }
        return;
    }
    uint32_t targetVoltage = lowPackVoltage + SAFE_VOLTAGE_DELTA_BETWEEN_PACKS;
    for ( int p = 0; p < numPacks; p++ ) {
        if ( p == lowPackId ) {
            packs[p].disable_inhibit_contactor_close(CI_IMBALANCE);
            continue;
        }
        if ( packs[p].get_voltage() <= targetVoltage ) {
            packs[p].disable_inhibit_contactor_close(CI_IMBALANCE);
        } else {
            packs[p].enable_inhibit_contactor_close(CI_IMBALANCE);
        }
    }
}

/*
 * Apply or withdraw the dead-cell contactor hold, per pack.
 */
/* E_DEAD_CELL only fires while a cell IS dead, so nothing drove the recovery
 * direction: a pack isolated for a dead cell stayed isolated for the life of
 * the program even after the cell came back. Called from the reconciliation
 * pass, which runs in every state. */
void Battery::release_recovered_dead_cell_packs() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( !packs[p].has_dead_cell() ) {
            packs[p].disable_inhibit_contactor_close(CI_DEAD_CELL);
        }
    }
}

void Battery::reevaluate_dead_cell_inhibition() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( packs[p].has_dead_cell() ) {
            packs[p].enable_inhibit_contactor_close(CI_DEAD_CELL);
        } else {
            packs[p].disable_inhibit_contactor_close(CI_DEAD_CELL);
        }
    }
}

/* Returns a byte representing the liveness of modules starting at moduleId.
 * Zero is alive, one is dead. moduleId is index of the across the whole pack,
 * rather than indexed by pack and then module. */
uint8_t Battery::get_module_liveness_byte(int8_t startModuleId) {
    uint8_t livenessByte = 0;
    const int totalModules = NUM_PACKS * MODULES_PER_PACK;
    // If the module ID is out of range, report everything as alive (zero)
    if ( startModuleId < 0 || startModuleId >= totalModules ) {
        return livenessByte;
    }
    /* startModuleId indexes modules across the whole battery, so the pack is the
     * quotient and the module within that pack is the remainder. This used to be
     * written the other way round, which divided by zero for startModuleId == 0. */
    int packId = startModuleId / MODULES_PER_PACK;
    int moduleId = startModuleId % MODULES_PER_PACK;
    for ( int count = 0; count < 8; count++ ) {
        // Stop at the last real module rather than walking off the end of packs[]
        if ( packId >= numPacks ) {
            break;
        }
        if ( !packs[packId].get_module_liveness(moduleId) ) {
            livenessByte |= (uint8_t)(1u << count);
        }
        moduleId += 1;
        if ( moduleId >= MODULES_PER_PACK ) {
            moduleId = 0;
            packId += 1;
        }
    }
    return livenessByte;
}

bool Battery::is_alive() {
    for ( int p = 0; p < numPacks; p++ ) {
        if ( !packs[p].is_alive() ) {
            return false;
        }
    }
    return true;
}

bool Battery::contactor_is_welded(uint8_t packId) {
    /* Bounds-checked for the same reason as the CAN counter accessors: a
     * runtime pack index into a fixed member array is invisible to ASan and
     * UBSan alike, so an out-of-range id reads adjacent memory silently. */
    if ( packId >= numPacks ) {
        return false;
    }
    return packs[packId].contactors_are_welded();
}