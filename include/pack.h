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

#ifndef BMS_SRC_INCLUDE_PACK_H_
#define BMS_SRC_INCLUDE_PACK_H_

#include <ACAN2515.h>

#include "module.h"
#include "CRC8.h"
#include "settings.h"

class Battery;
class Bms;

const uint8_t finalxor[12] = { 0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C };

/* Why a pack's contactors are being held open. Tracked as a bitmask so that
 * independent causes cannot clobber each other: releasing the imbalance hold
 * must not also release a dead-cell hold. The contactors stay inhibited while
 * ANY bit is set. */
enum ContactorInhibitReason {
    CI_STARTUP   = 1 << 0,   // no valid module data yet
    CI_DEAD_CELL = 1 << 1,   // this pack has a cell below DEAD_CELL_VOLTAGE
    CI_IMBALANCE = 1 << 2,   // this pack is too far from the others to parallel safely
};

class BatteryPack {

   public:
      int id = -1;

      BatteryPack();
      /* Initialise in place. Do NOT construct a temporary and copy-assign it:
       * each BatteryModule stores a back-pointer to its parent pack, so building
       * a temporary BatteryPack leaves every module pointing at freed stack. */
      void init(int _id, int CANCSPin, int _contactorPin, int _contactorFeedbackPin, int _numModules,
            int _numCellsPerModule, int _numTemperatureSensorsPerModule, Bms* _bms);

      void set_battery(Battery* battery) { this->battery = battery; }

      void print();
      uint8_t getcheck(CANMessage &msg, int moduleId);  // moduleId, NOT pack id
      int8_t get_module_liveness(int8_t moduleId);
      bool is_alive();
      void request_data();
      void read_message();
      /* Pack CAN controllers run in polled mode; this wakes the ACAN2515
       * driver task so it can service the MCP2515 over SPI. */
      void poll_can();
      bool send_frame(CANMessage *frame);

      void set_pack_error_status(int newErrorStatus);
      int get_pack_error_status();
      void set_pack_balance_status(int newBalanceStatus);
      int get_pack_balance_status();
      bool pack_is_due_to_be_balanced();
      void reset_balance_timer();

      // Voltage
      float get_voltage();
      void recalculate_total_voltage();
      uint16_t get_lowest_cell_voltage();
      bool has_empty_cell();
      uint16_t get_highest_cell_voltage();
      bool has_full_cell();
      void set_cell_voltage(int moduleIndex, int cellIndex, uint32_t newCellVoltage);
      void decode_voltages(CANMessage *frame);
      void recalculate_cell_delta();
      void process_voltage_update();
      bool has_dead_cell();
      uint16_t get_cell_delta() { return cellDelta; }

      // Temperature
      bool has_temperature_sensor_over_max();
      int8_t get_lowest_temperature();
      int8_t get_highest_temperature();
      void decode_temperatures(CANMessage *temperatureMessageFrame);
      void process_temperature_update();

      // Contactors
      void enable_inhibit_contactor_close(ContactorInhibitReason reason);
      void disable_inhibit_contactor_close(ContactorInhibitReason reason);
      bool contactors_are_inhibited();
      bool contactors_are_welded();
      uint8_t get_contactor_inhibit_reasons() { return contactorInhibitReasons; }

      int16_t get_max_discharge_current();
      int16_t get_max_charge_current_by_temperature();
      /* Safe lookup into chargeCurrentMax[], which is indexed by
       * (temperature + 10) and only covers -10C..+39C. */
      int16_t charge_current_for_temperature(int8_t temperature);

      void increment_can_tx_error_count() { canTxErrorCount++; }
      void increment_can_rx_error_count() { canRxErrorCount++; }
      uint16_t get_can_tx_error_count() { return canTxErrorCount; }
      uint16_t get_can_rx_error_count() { return canRxErrorCount; }

   private:
      /* Default initialisers on every member: BatteryPack is default-constructed
       * as part of Battery's array long before init() runs on it, and several of
       * these were read before init() ever assigned them. balanceStatus in
       * particular gates every cell-voltage store in decode_voltages(), so
       * garbage there silently discarded all voltage data. */
      ACAN2515* CAN = nullptr;                         // CAN bus connection to this pack
      Bms* bms = nullptr;
      //absolute_time_t lastUpdate;                      // Time we received last update from BMS
      int numModules = 0;                              //
      int numCellsPerModule = 0;                       //
      int numTemperatureSensorsPerModule = 0;          //
      Battery* battery = nullptr;                      // The parent Battery that contains this BatteryPack
      float voltage = 0.0f;                            // Voltage of the total pack
      uint16_t cellDelta = 0;                          // Difference in voltage between high and low cell, in mV

      // contactors
      int contactorInhibitPin = -1;                    // Pin which controls contactors for this pack
      int contactorFeedbackPin = -1;                   // Pin where feedback from the contactors is read
      /* Tracked rather than read back with digitalRead(): an ESP32 pin set to
       * plain OUTPUT has its input buffer disabled and always reads 0. Starts
       * held by CI_STARTUP -- contactors must not be permitted to close before
       * any cell data has arrived. */
      uint8_t contactorInhibitReasons = CI_STARTUP;
      uint64_t contactorInhibitedSince = 0;   // get_clock_ms(), for weld-check settling

      uint32_t balanceStatus = 0;                      // Status of the balance of the pack
      uint32_t errorStatus = 0;                        //
      bool balancingEnabled = false;                   //
      uint64_t nextBalanceTime = 0;                    // get_clock_ms() at which the next balance may start
      uint8_t pollMessageId = 0;                       //
      bool initialised = false;                        //
      BatteryModule modules[MODULES_PER_PACK];         // The child modules that make up this BatteryPack
      CRC8 crc8;

      bool inStartup = true;
      uint8_t modulePollingCycle = 0;
      CANMessage pollModuleFrame;

      uint8_t dischargeCurve[50] = {
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -10C to -1C
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 0C to 9C
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 10C to 19C
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 20C to 29C
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 30C to 39C
      };

      // C = 26Ah
      // -10° => +39° => whole charging range
      // below -10° => no charging, try and heat the battery
      // -10° to -1° => 3A to 6A
      //   0° to 15° => 4A to 125A
      //  16° to 35° => 125A
      //  36° to 39° => 50A
      // above 40° => no charging
      uint8_t chargeCurrentMax[50] = {
         3, 3, 3, 4, 4, 4, 5, 5, 6, 6,  // -10° to -1°
         13, 20, 27, 34, 41, 48, 55, 62, 69, 76, 83, 90, 97, 104, 111, 118,  // 0° to 15°
         125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,  // 16° to 35°
         50, 50, 50, 50,  // 36° to 39°
      };

      uint64_t lastTemperatureSampleTime = 0;
      int8_t lastTemperatureSample = 0;
      int8_t temperatureDelta = 0;

      int16_t canTxErrorCount = 0;
      int16_t canRxErrorCount = 0;
};

#endif  // BMS_SRC_INCLUDE_PACK_H_

