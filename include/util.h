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
void print_frame(const CANMessage* frame);

/*
 * Little-endian field writers for CAN payloads.
 *
 * Every multi-byte field in this codebase used to be written by hand as
 *     data[n]   = (uint8_t)(value) && 0xFF;
 *     data[n+1] = (uint8_t)(value) >> 8;
 * which is wrong twice over: `&&` is a logical AND yielding 0 or 1, and
 * truncating to uint8_t before shifting right by 8 always yields 0. Use these
 * instead of open-coding the split.
 *
 * Writes are ignored (rather than running off the end of the frame) if the
 * field would not fit within the 8 payload bytes.
 */
void put_u16_le(CANMessage* frame, int offset, uint16_t value);
void put_i16_le(CANMessage* frame, int offset, int16_t value);
void put_u32_le(CANMessage* frame, int offset, uint32_t value);

#endif
