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

#include <Arduino.h>
#include <ACAN2515.h>
#include <esp_timer.h>
#include <stdio.h>

#include "util.h"


uint64_t get_clock_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000);
}

void zero_frame(CANMessage* frame) {
    frame->id = 0;
    frame->len = 8;
    for ( int i = 0; i < 8; i++ ) {
        frame->data[i] = 0;
    }
}

void print_frame(const CANMessage* frame) {
    printf(" [print_frame] ID: 0x%03X, DLC: %d, Data: ", (unsigned int)frame->id, frame->len);
    for ( int i = 0; i < 8; i++ ) {
        printf("%d ", frame->data[i]);
    }
    printf("\n");
}

void put_u16_le(CANMessage* frame, int offset, uint16_t value) {
    if ( frame == nullptr || offset < 0 || offset + 2 > 8 ) {
        return;
    }
    frame->data[offset]     = (uint8_t)( value & 0xFF );
    frame->data[offset + 1] = (uint8_t)( (value >> 8) & 0xFF );
}

void put_i16_le(CANMessage* frame, int offset, int16_t value) {
    put_u16_le(frame, offset, (uint16_t)value);
}

void put_u32_le(CANMessage* frame, int offset, uint32_t value) {
    if ( frame == nullptr || offset < 0 || offset + 4 > 8 ) {
        return;
    }
    frame->data[offset]     = (uint8_t)( value & 0xFF );
    frame->data[offset + 1] = (uint8_t)( (value >>  8) & 0xFF );
    frame->data[offset + 2] = (uint8_t)( (value >> 16) & 0xFF );
    frame->data[offset + 3] = (uint8_t)( (value >> 24) & 0xFF );
}

