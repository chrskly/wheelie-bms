/*
 * This file is part of the ev mustang charge controller project.
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

#ifndef UTIL_H
#define UTIL_H

#include <ACAN2515.h>
#include <stdint.h>

/*
 * Monotonic millisecond clock.
 *
 * Returns 64-bit milliseconds since boot, sourced from esp_timer_get_time()
 * (a 64-bit microsecond counter). Do NOT use micros()/millis() here: both
 * return a 32-bit unsigned long that wraps after ~71 minutes / ~49 days, which
 * makes every "now - lastSeen" comparison in this codebase go haywire.
 *
 * All timeouts in settings.h are expressed in milliseconds to match.
 */
uint64_t get_clock_ms();

void zero_frame(CANMessage* frame);
void print_frame(CANMessage* frame);

/*
 * Create an auto-reload FreeRTOS software timer and start it, reporting failure
 * rather than returning a handle nobody checks. Returns NULL on failure.
 *
 * Timers used to be created by global static initialisers, which runs them
 * before setup() at a point where failure cannot be reported and the ordering
 * between translation units is unspecified. They are now created from the
 * explicit start() calls instead.
 */
TimerHandle_t create_and_start_timer(const char* name, uint32_t periodMs, TimerCallbackFunction_t callback);

#endif
