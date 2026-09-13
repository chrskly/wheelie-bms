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

#ifndef BMS_SRC_INCLUDE_SHUNT_H_
#define BMS_SRC_INCLUDE_SHUNT_H_

#include <stdint.h>

/*
 * Values as reported by an Isabellenhuette IVT-S ("ISA") shunt.
 *
 * UNITS. These are stored in the shunt's own raw units, NOT scaled to SI. They
 * were verified against an independent driver for the same device (Stm32-vcu
 * isa_shunt.cpp plus the divisors its VCU applies):
 *
 *   amps        milliamps   (raw / 1000 = A)
 *   voltage1-3  millivolts  (raw / 1000 = V)
 *   temperature degrees C   (already divided by 10 on receipt)
 *   watts       watts       (raw / 1000 = kW)
 *   ampSeconds  amp-seconds (raw / 3600 = Ah)  <- matches BATTERY_CAPACITY_AS
 *   wattHours   watt-hours  (raw / 1000 = kWh) <- matches BATTERY_CAPACITY_WH
 *
 * Scale at the point of use. Dividing on receipt (as voltage1 and watts used
 * to) truncates to a whole volt / kilowatt and throws away three digits.
 */
class Shunt {
    private:
        /* lastHeartbeat starts at 0, so the shunt reads as alive for the first
         * SHUNT_TTL_MS after boot and then dead until it actually reports. */
        uint64_t lastHeartbeat = 0;  // get_clock_ms() when we last got an update from the ISA Shunt
        int32_t amps = 0;
        int32_t voltage1 = 0;
        int32_t voltage2 = 0;
        int32_t voltage3 = 0;
        int32_t temperature = 0;
        int32_t watts = 0;
        int32_t ampSeconds = 0;
        int32_t wattHours = 0;
    public:
        Shunt();
        void heartbeat();
        bool is_dead();
        int32_t get_amps();
        void set_amps(int32_t _amps);
        int32_t get_voltage1();
        void set_voltage1(int32_t _voltage1);
        int32_t get_voltage2();
        void set_voltage2(int32_t _voltage2);
        int32_t get_voltage3();
        void set_voltage3(int32_t _voltage3);
        int32_t get_temperature();
        void set_temperature(int32_t _temperature);
        int32_t get_watts();
        void set_watts(int32_t _watts);
        int32_t get_ampSeconds();
        void set_ampSeconds(int32_t _ampSeconds);
        int32_t get_wattHours();
        void set_wattHours(int32_t _wattHours);
};

#endif  // BMS_SRC_INCLUDE_SHUNT_H_