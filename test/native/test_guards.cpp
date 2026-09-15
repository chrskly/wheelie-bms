/*
 * The contactor-hold guards, pinned one input combination at a time.
 *
 * Mutation testing showed that every `&&` in these guards could be flipped to
 * `||` without a single test noticing. They are the conditions that decide
 * whether a pack contactor is opened while something may be drawing through it
 * (Note 2 in statemachine.cpp), and both of the serious contactor bugs found in
 * this audit lived here, so "no test distinguishes them" is not acceptable.
 *
 * Each case below states the input combination and the hold it must produce.
 */
#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery; extern Io io;
void setup();

/* Brings the packs up genuinely imbalanced and settled, then parks the machine
 * in `s` with the given inputs, with no hold applied to start from. */
static void arrange(State s, bool ignition, bool charge) {
    sim_gpio[IGNITION_ENABLE_PIN] = ignition ? 1 : 0;
    sim_gpio[CHARGE_ENABLE_PIN] = charge ? 1 : 0;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 2; i++) io.poll_inputs();
    bms.set_state(s, "test");
    battery.disable_inhibit_contactor_close();
}

static void boot_imbalanced() {
    sim_reset();
    for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -80);
    setup();
    sim_run_ms(PACKS_IMBALANCED_TTL_MS + 8000);
}

struct Case { bool ignition; bool charge; bool expectHold; const char* what; };

static void exercise(State s, Event e, const Case* cases, int n, const char* stateName) {
    for (int i = 0; i < n; i++) {
        boot_imbalanced();
        arrange(s, cases[i].ignition, cases[i].charge);
        bms.send_event(e);
        const bool held = battery.one_or_more_contactors_inhibited();
        char msg[160];
        snprintf(msg, sizeof msg, "%s: %s -> %s", stateName, cases[i].what,
                 cases[i].expectHold ? "held" : "not held");
        check(msg, held == cases[i].expectHold);
    }
}

void test_guards() {
    suite("guards: batteryEmpty, E_PACKS_IMBALANCED");
    {
        /* if ( ! ignition_is_on() ) -> hold.  Charge is not consulted. */
        static const Case cases[] = {
            { false, false, true,  "ignition off, charge off" },
            { false, true,  true,  "ignition off, charge on"  },
            { true,  false, false, "ignition on,  charge off" },
            { true,  true,  false, "ignition on,  charge on"  },
        };
        exercise(&state_batteryEmpty, E_PACKS_IMBALANCED, cases, 4, "batteryEmpty");
    }

    suite("guards: batteryEmpty, E_IGNITION_OFF");
    {
        static const Case cases[] = {
            { false, false, true,  "ignition off, charge off" },
            { false, true,  true,  "ignition off, charge on"  },
        };
        exercise(&state_batteryEmpty, E_IGNITION_OFF, cases, 2, "batteryEmpty");
    }

    suite("guards: batteryEmpty, E_CHARGING_INITIATED releases for charge");
    {
        /* if ( ! ignition_is_on() && packs_are_imbalanced() ) -> release for
         * charge. With ignition on, nothing is released. */
        boot_imbalanced();
        arrange(&state_batteryEmpty, true, true);
        battery.enable_inhibit_contactor_close();
        bms.send_event(E_CHARGING_INITIATED);
        check("batteryEmpty: ignition on -> the hold is not released for charge",
              battery.one_or_more_contactors_inhibited());
    }

    suite("guards: overTempFault, E_PACKS_IMBALANCED");
    {
        /* if ( ! ignition_is_on() && ! charge_is_enabled() ) -> hold.
         * Flipping the && to || would hold with either one off. */
        static const Case cases[] = {
            { false, false, true,  "ignition off, charge off" },
            { false, true,  false, "ignition off, charge on"  },
            { true,  false, false, "ignition on,  charge off" },
            { true,  true,  false, "ignition on,  charge on"  },
        };
        exercise(&state_overTempFault, E_PACKS_IMBALANCED, cases, 4, "overTempFault");
    }

    suite("guards: overTempFault, E_IGNITION_OFF");
    {
        /* if ( ! charge_is_enabled() && packs_are_imbalanced() ) -> hold. */
        static const Case cases[] = {
            { false, false, true,  "ignition off, charge off" },
            { false, true,  false, "ignition off, charge on"  },
        };
        exercise(&state_overTempFault, E_IGNITION_OFF, cases, 2, "overTempFault");
    }

    suite("guards: overTempFault, E_CHARGING_TERMINATED");
    {
        /* if ( ! ignition_is_on() && packs_are_imbalanced() ) -> hold. */
        static const Case cases[] = {
            { false, false, true,  "ignition off, charge off" },
            { true,  false, false, "ignition on,  charge off" },
        };
        exercise(&state_overTempFault, E_CHARGING_TERMINATED, cases, 2, "overTempFault");
    }

    suite("guards: overTempFault, E_MODULE_UNRESPONSIVE holds only with nothing drawing");
    {
        /* if ( ! ignition_is_on() && ! charge_is_enabled() ) -> hold, then
         * escalate to criticalFault either way. */
        static const Case cases[] = {
            { false, false, true,  "ignition off, charge off" },
            { true,  false, false, "ignition on,  charge off" },
            { false, true,  false, "ignition off, charge on"  },
        };
        for (int i = 0; i < 3; i++) {
            boot_imbalanced();
            arrange(&state_overTempFault, cases[i].ignition, cases[i].charge);
            bms.send_event(E_MODULE_UNRESPONSIVE);
            char msg[160];
            snprintf(msg, sizeof msg, "overTempFault: %s -> %s", cases[i].what,
                     cases[i].expectHold ? "held" : "not held");
            check(msg, battery.one_or_more_contactors_inhibited() == cases[i].expectHold);
            check("and it escalates to criticalFault either way",
                  bms.get_state() == &state_criticalFault);
        }
    }

    sim_reset();
}
