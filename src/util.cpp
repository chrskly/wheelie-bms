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

void print_frame(CANMessage* frame) {
    printf(" [print_frame] ID: 0x%03X, DLC: %d, Data: ", (unsigned int)frame->id, frame->len);
    for ( int i = 0; i < 8; i++ ) {
        printf("%d ", frame->data[i]);
    }
    printf("\n");
}

TimerHandle_t create_and_start_timer(const char* name, uint32_t periodMs, TimerCallbackFunction_t callback) {
    if ( periodMs == 0 ) {
        printf("[util] ERROR timer '%s' has a zero period\n", name);
        return NULL;
    }
    TimerHandle_t timer = xTimerCreate(name, pdMS_TO_TICKS(periodMs), pdTRUE, NULL, callback);
    if ( timer == NULL ) {
        printf("[util] ERROR could not create timer '%s' (out of heap?)\n", name);
        return NULL;
    }
    if ( xTimerStart(timer, 0) != pdPASS ) {
        printf("[util] ERROR could not start timer '%s' (timer queue full?)\n", name);
        return NULL;
    }
    printf("[util] timer '%s' running at %ums\n", name, (unsigned int)periodMs);
    return timer;
}
