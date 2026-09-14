#include "sim/unit.h"
#include "sim/sim.h"
#include "led.h"

static int blink_until_change(StatusLight& l, int startLevel, int limit) {
    for (int i = 0; i < limit; i++) { l.led_blink(); if (sim_gpio[LED_PIN] != startLevel) return i + 1; }
    return -1;
}

void test_led() {
    sim_reset();
    suite("led: standby blinks");
    StatusLight l;
    l.set_mode(STANDBY);
    sim_gpio[LED_PIN] = 0;
    int toOn = blink_until_change(l, 0, 100);
    check("turns on in standby", toOn > 0);
    int toOff = blink_until_change(l, 1, 100);
    check("and back off in standby", toOff > 0);

    suite("led: drive is solid");
    l.set_mode(DRIVE);
    for (int i = 0; i < 50; i++) l.led_blink();
    check_eq("stays on in drive (offDuration 0)", sim_gpio[LED_PIN], 1);

    suite("led: charging and fault patterns");
    l.set_mode(CHARGING);
    check("charging pattern selected", blink_until_change(l, sim_gpio[LED_PIN], 100) > 0);
    l.set_mode(FAULT);
    check("fault pattern selected", blink_until_change(l, sim_gpio[LED_PIN], 100) > 0);
    l.set_mode((LED_MODE)99);             // unknown mode falls through to FAULT
    check("unknown mode handled", blink_until_change(l, sim_gpio[LED_PIN], 100) > 0);
}
