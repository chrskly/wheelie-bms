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

#ifndef BMS_SRC_INCLUDE_BATTERY_H_
#define BMS_SRC_INCLUDE_BATTERY_H_

#include "pack.h"
#include "settings.h"

/* Deliberately does NOT include bms.h. bms.h includes this header, so including
 * it back created a cycle that only compiled because pack.h happens to
 * forward-declare Bms and happens to be included first -- reordering these two
 * lines broke the build. Battery only needs a pointer to Bms, so a forward
 * declaration is enough; battery.cpp includes bms.h for the definition. */
class Bms;
class Io;

class Battery {
   private:
      BatteryPack packs[NUM_PACKS];
      int numPacks = NUM_PACKS;                // Number of battery packs in this battery
      uint32_t voltage = 0;                    // Total voltage of whole battery
      uint16_t lowestCellVoltage = 0;          // Voltage of cell with lowest voltage across whole battery
      uint16_t activePacks_lowestCellVoltage = 0;  // Lowest cell voltage across packs that are not inhibited
      uint16_t highestCellVoltage = 0;         // Voltage of cell with highest voltage across whole battery
      uint16_t activePacks_highestCellVoltage = 0; // Highest cell voltage across packs that are not inhibited
      uint32_t minimumBatteryVoltage = 0;      // Lowest permitted voltage of the whole battery
      uint32_t maximumBatteryVoltage = 0;      // Highest permitted voltage of the whole battery
      float lowestSensorTemperature = 0;       //
      float highestSensorTemperature = 0;      //
      /* Latched threshold results, updated in process_temperature_update().
       * too_hot() / too_cold_to_charge() just read these, so they stay pure
       * getters and every caller in a given cycle sees the same answer. */
      bool tooHotLatched = false;
      bool tooColdToChargeLatched = false;
      Bms* bms = nullptr;

   public:
      Battery() {};
      /* Two-phase startup, deliberately:
       *   initialise() builds the packs (each module stores a back-pointer, so
       *     the packs must be built in place, never copy-assigned);
       *   the BMS worker task begins polling. Nothing may run periodically
       *     until every pack exists and `bms` is set. */
      void initialise(Bms* _bms);
      int print();

      void request_data();
      void read_message();
      bool has_multiple_packs();
      uint8_t number_of_active_packs();
      uint16_t get_can_tx_error_count_for_pack(int packId) { return packs[packId].get_can_tx_error_count(); }
      uint16_t get_can_rx_error_count_for_pack(int packId) { return packs[packId].get_can_rx_error_count(); }

      // Voltage
      uint32_t get_voltage();
      void set_voltage(uint32_t voltage) { this->voltage = voltage; }
      void recalculate_voltage();
      uint32_t get_max_voltage();
      uint32_t get_min_voltage();
      int get_index_of_high_pack();
      int get_index_of_low_pack();
      void process_voltage_update();
      void recalculate_lowest_cell_voltage();
      uint16_t get_lowest_cell_voltage();
      bool has_empty_cell();
      void recalculate_highest_cell_voltage();
      uint16_t get_highest_cell_voltage();
      bool has_full_cell();
      uint32_t voltage_delta_between_packs();
      bool packs_are_imbalanced();
      uint16_t get_cell_delta();
      bool has_dead_cell();
      bool cell_delta_above_warn() { return get_cell_delta() > CELL_DELTA_WARN_THRESHOLD; }
      bool cell_delta_above_alarm() { return get_cell_delta() > CELL_DELTA_ALARM_THRESHOLD; }

      // Temperature
      void update_highest_sensor_temperature();
      int8_t get_highest_sensor_temperature();
      bool too_hot();
      void update_lowest_sensor_temperature();
      int8_t get_lowest_sensor_temperature();
      void process_temperature_update();
      void update_temperature_latches();
      bool too_cold_to_charge();
      uint16_t get_max_charge_current_by_temperature();

      // Contactors
      void disable_inhibit_contactors_for_drive();
      void disable_inhibit_contactors_for_charge();
      void enable_inhibit_contactor_close();
      void disable_inhibit_contactor_close();
      bool one_or_more_contactors_inhibited();
      bool all_contactors_inhibited();
      void reevaluate_contactor_inhibition_for_drive();
      void reevaluate_contactor_inhibition_for_charge();
      /* Applies AND withdraws the dead-cell hold, so a pack whose cell recovers
       * is released again. The old inhibit-only version had no counterpart, so
       * a pack held for a dead cell stayed held for the life of the program. */
      void reevaluate_dead_cell_inhibition();

      uint8_t get_module_liveness_byte(int8_t moduleId);
      bool is_alive();
      bool contactor_is_welded(uint8_t packId);
};

#endif  // BMS_SRC_INCLUDE_BATTERY_H_
