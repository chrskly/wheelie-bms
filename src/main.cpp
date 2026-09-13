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

/*------------------------------------------------------------------------------

EV Mustang BMS

------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>

#include "battery.h"
#include "bms.h"
#include "statemachine.h"
#include "led.h"
#include "io.h"
#include "shunt.h"
#include "settings.h"
#include "util.h"


// mutex_t canMutex;
Io io;
Shunt shunt;
Battery battery;
Bms bms;


// Watchdog

// struct repeating_timer watchdogKeepaliveTimer;

// bool watchdog_keepalive(struct repeating_timer *t) {
//     watchdog_update();
//     return true;
// }

// void enable_watchdog_keepalive() {
//     add_repeating_timer_ms(1000, watchdog_keepalive, NULL, &watchdogKeepaliveTimer);
// }

// Status print

// struct repeating_timer statusPrintTimer;

// bool status_print(struct repeating_timer *t) {
//     extern Bms bms;
//     bms.print();
//     return true;
// }

// void enable_status_print() {
//     printf(" * Enabling status print\n");
//     add_repeating_timer_ms(1000, status_print, NULL, &statusPrintTimer);
// }


/*
 * Generate the oscillator clock for the MCP2515 CAN controllers on CAN_CLK_PIN.
 *
 * The pico build produced this with clock_gpio_init(); on the ESP32 the LEDC
 * peripheral does the same job. Nothing generated this clock at all after the
 * port, so the MCP2515s had no oscillator and could not respond to SPI.
 *
 * QUARTZ_FREQUENCY is the single source of truth: it is both what we generate
 * here and what ACAN2515 uses to compute bit timing, so the two cannot drift
 * apart the way they had (settings claimed 16 MHz, hardware was given 8 MHz).
 */
static void start_can_clock() {
    if ( CAN_CLK_PIN < 0 ) {
        printf(" * MCP2515 clock generation disabled; boards must supply their own crystal\n");
        return;
    }
    // 1-bit resolution: duty 1 of [0,1] == 50%, and keeps the divider integral
    const uint32_t actualFrequency = ledcSetup(CAN_CLK_LEDC_CHANNEL, QUARTZ_FREQUENCY, 1);
    if ( actualFrequency == 0 ) {
        printf(" * ERROR could not generate a %u Hz CAN clock on GPIO%d\n",
               (unsigned int)QUARTZ_FREQUENCY, CAN_CLK_PIN);
        return;
    }
    ledcAttachPin(CAN_CLK_PIN, CAN_CLK_LEDC_CHANNEL);
    ledcWrite(CAN_CLK_LEDC_CHANNEL, 1);
    printf(" * MCP2515 clock on GPIO%d : requested %u Hz, actual %u Hz\n",
           CAN_CLK_PIN, (unsigned int)QUARTZ_FREQUENCY, (unsigned int)actualFrequency);
    if ( actualFrequency != QUARTZ_FREQUENCY ) {
        printf(" * WARNING CAN clock is not the requested frequency; MCP2515 bit timing will be wrong\n");
    }
    // Let the oscillator settle before we start talking to the controllers
    delay(10);
}


void setup() {
    printf("BMS starting up ...\n");

    // Check for unexpected reboot
    // if (watchdog_caused_reboot()) {
    //     printf(" * !!!! Rebooted by Watchdog !!!!\n");
    //     bms.set_watchdog_reboot(true);
    // } else {
    //     printf(" * Clean boot\n");
    //     bms.set_watchdog_reboot(false);
    // }
    // watchdog_enable(5000, 1);
    // enable_watchdog_keepalive();

    // mutex_init(&canMutex);

    // Oscillator for the pack CAN controllers. Must be running before they init.
    start_can_clock();

    // SPI bus shared by the pack CAN controllers. This was never begun, so the
    // pin assignments in settings.h had no effect at all.
    printf(" * Starting SPI (SCK:%d, MISO:%d, MOSI:%d)\n", SPI_CLK, SPI_MISO, SPI_MOSI);
    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI);

    /* Initialisation order matters, and getting it wrong is what most of the
     * startup bugs were:
     *   1. io.init()             - pins only, no interrupts yet
     *   2. battery.initialise()  - builds packs in place (needs SPI + CAN clock)
     *   3. bms.init()            - main CAN port, needs a built battery
     *   4. bms.start()           - starts the single worker task that runs all
     *                               periodic work, polls the inputs and is the
     *                               only thing that enters the state machine
     * The objects themselves are globals and are constructed once; never
     * assign a freshly-built temporary over them, because Bms, BatteryPack and
     * BatteryModule all hand out pointers to `this`. */
    io.init();
    battery.initialise(&io, &bms);
    bms.init(&battery, &io, &shunt);

    bms.start();

    // enable_status_print();

    printf("---- BMS READY ----\n");
}


void loop() {
    /* All work happens in FreeRTOS timer callbacks. Yield rather than spinning:
     * a bare `while (true) {}` here starves the idle task on this core and the
     * task watchdog reboots the board. */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
