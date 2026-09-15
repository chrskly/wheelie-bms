/*
 * The charge acceptance curve, as data rather than as lines of code.
 *
 * pack.cpp's curve is a 50-entry table, and line coverage says nothing useful
 * about a table: every entry is "covered" by one lookup. What matters is what
 * comes out at each temperature, and in particular that
 * charge_current_for_temperature_range() is right -- its correctness argument
 * is that taking the lower of the two endpoint lookups equals the minimum over
 * every module in between, which holds only while the curve has no interior
 * dip. That is asserted at compile time; here it is checked exhaustively.
 */
#include "sim/unit.h"
#include "sim/sim.h"
#include "pack.h"
#include "battery.h"

extern Battery battery;

static BatteryPack& a_pack() {
    static BatteryPack pack;
    static bool built = false;
    if (!built) {
        BatteryPackConfig cfg;
        cfg.id = 0;
        cfg.canChipSelectPin = CS_PINS[0];
        cfg.contactorInhibitPin = INHIBIT_CONTACTOR_PINS[0];
        cfg.contactorFeedbackPin = CONTACTOR_FEEDBACK_PINS[0];
        cfg.numModules = MODULES_PER_PACK;
        cfg.numCellsPerModule = CELLS_PER_MODULE;
        cfg.numTemperatureSensorsPerModule = TEMPS_PER_MODULE;
        pack.init(cfg);
        built = true;
    }
    return pack;
}

void test_curve() {
    BatteryPack& p = a_pack();

    suite("curve: the window it covers");
    check_eq("one below the bottom gives no charge", p.charge_current_for_temperature(-11), 0);
    check("the bottom of the window gives some charge", p.charge_current_for_temperature(-10) > 0);
    check("the top of the window gives some charge", p.charge_current_for_temperature(39) > 0);
    check_eq("one above the top gives no charge", p.charge_current_for_temperature(40), 0);
    /* The sentinels the temperature getters return before any module reports. */
    check_eq("the no-data low sentinel gives no charge", p.charge_current_for_temperature(126), 0);
    check_eq("the no-data high sentinel gives no charge", p.charge_current_for_temperature(-126), 0);
    check_eq("the unfitted-sensor sentinel gives no charge",
             p.charge_current_for_temperature(NO_TEMPERATURE_READING), 0);

    suite("curve: it never exceeds the configured policy");
    {
        int overPolicy = 0, peak = 0;
        for (int t = -128; t <= 127; t++) {
            const int a = p.charge_current_for_temperature((int8_t)t);
            if (a > CHARGE_CURRENT_MAX_PER_PACK_A) overPolicy++;
            if (a > peak) peak = a;
        }
        check_eq("no temperature is offered more than the sustained rate", overPolicy, 0);
        check_eq("and the plateau reaches it exactly", peak, CHARGE_CURRENT_MAX_PER_PACK_A);
    }

    suite("curve: the anchor points its documentation claims");
    {
        /* 1C at 15 C and the 2.2C ceiling from 25 C, per the comments on the
         * table and the REGEN note in settings.h. */
        const int oneC = ( PACK_CAPACITY_AH * 100 + 50 ) / 100;
        const int ceilingC = ( PACK_CAPACITY_AH * CHARGE_C_RATE_PERCENT + 50 ) / 100;
        check_eq("15 C offers 1C", (long)p.charge_current_for_temperature(15), (long)oneC);
        check_eq("25 C offers the ceiling", (long)p.charge_current_for_temperature(25), (long)ceilingC);
        check_eq("35 C still offers the ceiling", (long)p.charge_current_for_temperature(35), (long)ceilingC);
        check("36 C backs off sharply",
              p.charge_current_for_temperature(36) < p.charge_current_for_temperature(35) / 2);
        check("the cold end is plating-limited to well under 1C",
              p.charge_current_for_temperature(-10) * 4 < oneC);
    }

    suite("curve: unimodal, with no interior dip");
    {
        int dips = 0;
        bool falling = false;
        for (int t = -9; t <= 39; t++) {
            const int prev = p.charge_current_for_temperature((int8_t)(t - 1));
            const int here = p.charge_current_for_temperature((int8_t)t);
            if (here < prev) falling = true;
            else if (falling && here > prev) dips++;
        }
        check_eq("the curve rises then falls and never rises again", dips, 0);
    }

    suite("curve: the range lookup really is the minimum over the range");
    {
        /* The claim the whole function rests on, checked for every ordered pair
         * of temperatures in and around the window rather than argued. */
        int mismatches = 0;
        long pairs = 0;
        for (int lo = -20; lo <= 50; lo++) {
            for (int hi = lo; hi <= 50; hi++) {
                pairs++;
                int trueMin = 0x7FFFFFFF;
                for (int t = lo; t <= hi; t++) {
                    const int a = p.charge_current_for_temperature((int8_t)t);
                    if (a < trueMin) trueMin = a;
                }
                const int got = p.charge_current_for_temperature_range((int8_t)lo, (int8_t)hi);
                if (got != trueMin) mismatches++;
            }
        }
        check("the sweep covered a meaningful number of pairs", pairs > 2000);
        check_eq("every endpoint pair equals the true minimum across the span",
                 mismatches, 0);
    }

    suite("curve: a cold module holds back a warm one");
    {
        /* The failure the range lookup was introduced to fix: modules at -8 C
         * and +14 C in the same pack clear the CHARGE_TEMPERATURE_MINIMUM guard
         * and must not be handed the +14 C entry. */
        const int coldAlone = p.charge_current_for_temperature(-8);
        const int warmAlone = p.charge_current_for_temperature(14);
        check("the warm end alone would allow far more", warmAlone > coldAlone * 4);
        check_eq("but the spread is held to the cold end",
                 (long)p.charge_current_for_temperature_range(-8, 14), (long)coldAlone);
    }
    sim_reset();
}
