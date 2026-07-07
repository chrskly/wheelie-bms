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

#include "bms.h"
#include "led.h"


void process_led_blink_step(TimerHandle_t xTimer) {
    //Bms* bms = static_cast<Bms*>(t->user_data);
    extern Bms bms;
    bms.led_blink();
}

TimerHandle_t processLedBlinkTimer = xTimerCreate(
    "processLedBlinkTimer",         // Timer name
    100 / portTICK_PERIOD_MS,       // 100ms period
    pdTRUE,                         // Auto-reload (periodic timer)
    NULL,                           // Timer ID
    process_led_blink_step          // Callback function
);

StatusLight::StatusLight(Bms* _bms) {
    on = false;
    counter = 0;
    onDuration = 0;
    offDuration = 0;
    bms = _bms;

    // Set up the LED pin
    pinMode(LED_PIN, OUTPUT);

    // timer to handle the on-ing and off-ing of the LED
    // add_repeating_timer_ms(100, process_led_blink_step, static_cast<void*>(&bms), &ledBlinkTimer);
}


// Switch status light to a different mode
void StatusLight::set_mode(LED_MODE newMode) {
    switch (newMode) {
        case STANDBY:
            onDuration = 1;
            offDuration = 39;
            break;
        case DRIVE:
            onDuration = 20;
            offDuration = 0;
            break;
        case CHARGING:
            onDuration = 10;
            offDuration = 10;
            break;
        case FAULT:
            onDuration = 1;
            offDuration = 1;
            break;
    }
}

void StatusLight::led_blink() {
    ++counter;

    if ( on ) {
        if ( counter > onDuration ) {
            counter = 0;
            if ( offDuration > 0 ) {
                digitalWrite(LED_PIN, LOW);
                on = false;
            }
        }
    } else {
        if ( counter > offDuration ) {
            digitalWrite(LED_PIN, HIGH);
            counter = 0;
            on = true;
        }
    }
}

