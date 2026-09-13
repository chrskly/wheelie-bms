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

#ifndef BMS_SRC_INCLUDE_MODULE_H_
#define BMS_SRC_INCLUDE_MODULE_H_

#include <stdint.h>
#include "settings.h"

class BatteryPack;

/* Default initialisers on every member: BatteryModule is default-constructed
 * as part of BatteryPack's array long before init() runs on it. */
class BatteryModule {
   private:
      int id = -1;
      int numCells = 0;                          // Number of cells in this module
      int numTemperatureSensors = 0;             // Number of temperature sensors in this module
      /* Both arrays are filled over their FULL extent by the constructor, not by
       * a default member initialiser: `= { -127 }` would set only element 0 and
       * value-initialise the rest to 0, and 0 is a perfectly plausible
       * temperature that the getters would treat as a real reading. */
      uint16_t cellVoltage[CELLS_PER_MODULE];    // Voltages of each cell, stored in mV
      int8_t cellTemperature[TEMPS_PER_MODULE];  // Temperatures of each cell (-127 == no reading yet)
      bool allModuleDataPopulated = false;       // True when we have voltage/temp information for all cells
      /* Reported by this module in its 0x10X status frame. Held per module
       * because the frame is per module: these used to be written into
       * pack-level fields, so the last module to report won and one module
       * reporting a balance status suppressed voltage capture for every
       * module in the pack. */
      uint32_t balanceStatus = 0;
      uint32_t errorStatus = 0;
      uint64_t lastHeartbeat = 0;                // get_clock_ms() when we last got an update from this module
      BatteryPack* pack = nullptr;               // The parent BatteryPack that contains this module

   public:
      BatteryModule();
      /* Initialise in place. Do NOT construct a temporary and copy-assign it:
       * BatteryModule stores a back-pointer to its parent pack, and the old
       * `modules[m] = BatteryModule(m, this, ...)` pattern captured the address
       * of a temporary BatteryPack that was destroyed moments later. */
      void init(int _id, BatteryPack* _pack, int _numCells, int _numTemperatureSensors);
      void print();

      // Voltage
      uint32_t get_voltage();
      uint16_t get_lowest_cell_voltage();
      uint16_t get_highest_cell_voltage();
      void set_cell_voltage(int cellIndex, uint16_t newCellVoltage);
      bool has_empty_cell();
      bool has_full_cell();
      bool has_dead_cell();

      // Module status
      void set_balance_status(uint32_t status) { balanceStatus = status; }
      uint32_t get_balance_status() { return balanceStatus; }
      void set_error_status(uint32_t status) { errorStatus = status; }
      uint32_t get_error_status() { return errorStatus; }
      bool all_module_data_populated();
      void check_if_module_data_is_populated();
      bool is_alive();
      void heartbeat();

      // Temperature
      void update_temperature(int tempSensorId, int8_t newTemperature);
      int8_t get_lowest_temperature();
      int8_t get_highest_temperature();
      bool has_temperature_sensor_over_max();
      bool temperature_at_warning_level();

};

#endif  // BMS_SRC_INCLUDE_MODULE_H_
