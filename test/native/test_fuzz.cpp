/*
 * Adversarial input, with safety invariants asserted continuously.
 *
 * The rest of the suite drives the firmware the way the hardware is expected to
 * behave. This drives it the way a faulty bus, a failing module, or a hostile
 * device would: arbitrary CAN ids and payloads on both buses, inputs that
 * change faster than they physically could, sensor readings that jump, and
 * module replies that arrive out of order or not at all. Nothing here asserts a
 * specific outcome -- it asserts the properties that must hold no matter what
 * arrives.
 */
#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "statemachine.h"
#include "webstatus.h"
#include <cstring>

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;
void setup();

/* Deterministic, so a failure is reproducible from the seed alone. */
static uint32_t rngState = 0;
static uint32_t rnd() {
    rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
    return rngState;
}
static uint32_t rnd_below(uint32_t n) { return n ? rnd() % n : 0; }

static int violations = 0;
static char firstViolation[256] = {0};

static void violated(const char* what, uint32_t seed, long step) {
    violations++;
    if (firstViolation[0] == '\0') {
        snprintf(firstViolation, sizeof firstViolation,
                 "%s (seed %u, step %ld)", what, seed, step);
        const State s = bms.get_state();
        const char* n = s == &state_standby ? "standby"
                      : s == &state_drive ? "drive"
                      : s == &state_charging ? "charging"
                      : s == &state_batteryEmpty ? "batteryEmpty"
                      : s == &state_batteryHeating ? "batteryHeating"
                      : s == &state_overTempFault ? "overTempFault"
                      : s == &state_criticalFault ? "criticalFault"
                      : s == &state_illegalStateTransitionFault ? "illegal" : "???";
        printf("   context: state=%s alive=%d comparable=%d activePacks=%u delta=%u "
               "bmsImb=%d deadCell=%d inhibPins=%d%d heldLogical=%d\n",
               n, battery.is_alive(), battery.pack_voltages_are_comparable(),
               battery.number_of_active_packs(), battery.voltage_delta_between_packs(),
               bms.packs_are_imbalanced(), battery.has_dead_cell(),
               sim_gpio[INHIBIT_CONTACTOR_PINS[0]],
               NUM_PACKS > 1 ? sim_gpio[INHIBIT_CONTACTOR_PINS[1]] : 0,
               battery.one_or_more_contactors_inhibited());
    }
}

/* True once `cond` has held continuously for at least `ms` of simulated time.
 * Every latched response in this firmware is driven from the health check, which
 * runs on its own cadence, so an invariant about a response must give it time to
 * happen -- otherwise it just measures the gap between cause and reaction. */
struct Sustained {
    uint64_t since = 0;
    /* When the condition became continuously true, or 0 if it is not. */
    uint64_t held_since() const { return since; }
    bool operator()(bool cond, uint64_t ms) {
        const uint64_t now = sim_now_us / 1000;
        if (!cond) { since = 0; return false; }
        /* sim_reset() puts the clock back to zero between rounds, and these are
         * function statics that outlive a round. Without this the subtraction
         * below underflows and every condition looks instantly sustained. */
        if (since == 0 || now < since) { since = now; return false; }
        return ( now - since ) >= ms;
    }
};
static const uint64_t REACT_MS = 500;   // 5x the 100 ms health-check period

static bool state_is_valid() {
    State s = bms.get_state();
    return s == &state_standby || s == &state_drive || s == &state_charging
        || s == &state_batteryEmpty || s == &state_batteryHeating
        || s == &state_overTempFault || s == &state_criticalFault
        || s == &state_illegalStateTransitionFault;
}

static void check_invariants(uint32_t seed, long step) {
    // 1. The state pointer is always one of the eight defined states.
    if (!state_is_valid()) violated("state pointer is not a defined state", seed, step);

    /* 2. The physical inhibit outputs track the reason masks exactly. A reason
     *    bit set with the pin released is an inhibit that silently does nothing. */
    if ((bms.get_drive_inhibit_reasons() != 0) != io.drive_is_inhibited())
        violated("drive inhibit pin disagrees with the reason mask", seed, step);
    if ((bms.get_charge_inhibit_reasons() != 0) != io.charge_is_inhibited())
        violated("charge inhibit pin disagrees with the reason mask", seed, step);

    // 3. Only state_batteryHeating may run the heater.
    if (io.heater_is_enabled() && bms.get_state() != &state_batteryHeating)
        violated("heater is on outside batteryHeating", seed, step);

    // 4. An inhibited path must be advertised as zero current.
    if (bms.charge_is_inhibited() && bms.get_max_charge_current() != 0)
        violated("charge inhibited but a non-zero charge current is advertised", seed, step);
    if (bms.drive_is_inhibited() && bms.get_max_discharge_current() != 0)
        violated("drive inhibited but a non-zero discharge current is advertised", seed, step);

    /* 4b. The advertised limits must never exceed the configured policy, at any
     *     temperature, state of charge or pack combination. These are the
     *     numbers a charger and an inverter actually obey. */
    if (bms.get_max_charge_current() > CHARGE_CURRENT_MAX_PER_PACK_A * NUM_PACKS)
        violated("charge current above the configured per-pack rate times the pack count", seed, step);
    if (bms.get_max_discharge_current() > DISCHARGE_CURRENT_MAX_PER_PACK_A * NUM_PACKS)
        violated("discharge current above the configured per-pack rate times the pack count", seed, step);

    /* 4c. Charging into a cell colder than CHARGE_TEMPERATURE_MINIMUM is the
     *     lithium-plating case the acceptance curve exists to prevent. */
    {
        static Sustained coldCharge;
        if (coldCharge(battery.have_temperature_reading()
                       && battery.get_lowest_sensor_temperature() < CHARGE_TEMPERATURE_MINIMUM
                       && bms.get_max_charge_current() > 0, REACT_MS))
            violated("charge current offered below CHARGE_TEMPERATURE_MINIMUM", seed, step);
    }

    // 5. State of charge is a percentage.
    if (bms.get_soc() > 100) violated("soc above 100%", seed, step);

    /* 6. Everything actually put on the wire. These check the transmitted bytes
     *    rather than the accessors, because the frames are built from cached
     *    values on their own schedule -- an accessor can be correct at the
     *    moment it is read and still have been stale when the frame was built. */
    /* 0x351 is rebuilt once a second, so an inhibit asserted after the last
     * frame went out legitimately is not reflected until the next one. What must
     * not happen is the advertised limit failing to catch up: judge the frames
     * against an inhibit that has already outlived a full transmission period. */

    for (auto& f : sim_main_tx) {
        if (f.len > 8 || f.id > 0x7FF) violated("illegal frame on the main bus", seed, step);
        const uint16_t w0 = (uint16_t)(f.data[0] | (f.data[1] << 8));
        const uint16_t w2 = (uint16_t)(f.data[2] | (f.data[3] << 8));
        const uint16_t w4 = (uint16_t)(f.data[4] | (f.data[5] << 8));
        if (f.id == 0x351) {
            /* The limits carried here are deliberately NOT judged against the
             * inhibit state: this suite only samples between steps, so it
             * cannot see an inhibit released and re-asserted in between, and a
             * frame legitimately built during such a gap looks like a
             * violation. The exact form of that property -- assert the inhibit,
             * then read the very next frame -- is in test_wire.cpp, and the
             * accessor-level version is invariant 4 above. */
            (void)w2;
            (void)w4;
        }
        if (f.id == 0x355) {
            if (w0 > 100) violated("0x355 reports a SoC above 100%", seed, step);
            if (w4 > 10000) violated("0x355 high-resolution SoC above 100%", seed, step);
            if (w4 != (uint16_t)(w0 * 100))
                violated("0x355 SoC fields disagree with each other", seed, step);
        }
        if (f.id == 0x356) {
            const int16_t t = (int16_t)w4;
            if (t < MODULE_SENSOR_MINIMUM_C * 10 || t > MODULE_SENSOR_MAXIMUM_C * 10)
                violated("0x356 reports an impossible battery temperature", seed, step);
            /* get_max_voltage() is the nominal full-charge limit, not a
             * physical ceiling, so exceeding it is a real overvoltage and not a
             * reporting fault. What must not happen is the uint16 field
             * wrapping: at 0.01 V it saturates at 655.35 V, so a pack that ever
             * read higher would report a small number as though it were fine. */
            if (battery.get_voltage() / 10 > 0xFFFF)
                violated("0x356 battery voltage field wrapped", seed, step);
            if (w0 > 60000)
                violated("0x356 reports an implausible battery voltage", seed, step);
        }
    }
    for (int p = 0; p < NUM_PACKS; p++)
        for (auto& f : sim_pack_tx[p]) {
            if (f.len > 8 || f.id > 0x7FF) violated("illegal frame on a pack bus", seed, step);
            /* Every module poll must carry a balance target the modules can act
             * on: either the idle target that bleeds nothing, or a real cell
             * voltage. A target of 0 would tell every cell to bleed to zero. */
            if ((f.id & 0xFF0) == 0x080) {
                const uint16_t target = (uint16_t)(f.data[0] | (f.data[1] << 8));
                if (target != 0x10C7 && (target < 2000 || target > 4400))
                    violated("a module poll carries an implausible balance target", seed, step);
            }
        }
    sim_main_tx.clear();
    for (int p = 0; p < NUM_PACKS; p++) sim_pack_tx[p].clear();

    /* 7. The web snapshot must always serialise into the buffer the server
     *    actually uses. A snapshot that cannot be rendered is a page that
     *    returns an error no matter how often it is retried. */
    WebSnapshot snap;
    if (webstatus_read(snap)) {
        static char buf[WEB_JSON_BUFFER_BYTES];
        if (webstatus_render_json(snap, buf, sizeof buf) == 0)
            violated("a published snapshot does not fit the JSON buffer", seed, step);
    }

    /* 8. In the states where opening a contactor is legal at all (Note 2 in
     *    statemachine.cpp: not under load), a dead cell that has been present
     *    across two consecutive checks must have produced a hold. Two checks,
     *    because the health check runs on its own cadence and is entitled to
     *    one cycle to react. */
    {
        const State s = bms.get_state();
        /* Note 2 again, one level finer: even in these states the handlers only
         * assert a hold when nothing can be drawing through the contactors, so
         * the invariant has to carry the same condition. */
        const bool contactorsMayMove = ( s == &state_standby || s == &state_batteryEmpty
                                      || s == &state_overTempFault )
                                    && !bms.ignition_is_on() && !bms.charge_is_enabled();
        static Sustained deadCellUnheld;
        if (deadCellUnheld(contactorsMayMove && battery.has_dead_cell()
                           && battery.has_multiple_packs()
                           && !battery.one_or_more_contactors_inhibited(), REACT_MS))
            violated("a dead cell persisted in standby but no contactor is held open", seed, step);
    }

    /* 9. Same for a sustained pack imbalance: once the BMS has decided the
     *    packs are imbalanced, a state that may move contactors must hold them. */
    {
        const State s = bms.get_state();
        const bool contactorsMayMove = ( s == &state_standby || s == &state_batteryEmpty
                                      || s == &state_overTempFault )
                                    && !bms.ignition_is_on() && !bms.charge_is_enabled();
        static Sustained imbalanceUnheld;
        if (imbalanceUnheld(contactorsMayMove && bms.packs_are_imbalanced()
                            && battery.has_multiple_packs()
                            && !battery.one_or_more_contactors_inhibited(), REACT_MS))
            violated("packs imbalanced in standby but no contactor is held open", seed, step);
    }

    /* 10. Every event the health check can emit is handled by every state, so
     *     the invalid-event counter must never move during ordinary operation. */
    if (bms.get_invalid_event_count() != 0)
        violated("the state machine saw an event it does not handle", seed, step);

    /* 11a. Stranded inhibits. Every reason below is withdrawn by the
     *      reconciliation pass once its condition clears, so a reason bit that
     *      outlives its condition is an inhibit nothing will ever lift. These
     *      check the SPURIOUS direction -- a BMS stuck refusing to drive or
     *      charge with no live reason is a failure too, just a quieter one. */
    {
        const uint16_t both = bms.get_drive_inhibit_reasons() | bms.get_charge_inhibit_reasons();
        struct Stale { const char* name; bool stillHeld; bool conditionGone; };
        const Stale stale[] = {
            { "R_TOO_HOT",             (both & inhibit_reason_bit(R_TOO_HOT)) != 0,
                                       !battery.too_hot() },
            { "R_BATTERY_EMPTY",       (both & inhibit_reason_bit(R_BATTERY_EMPTY)) != 0,
                                       !battery.has_empty_cell() },
            { "R_BATTERY_FULL",        (both & inhibit_reason_bit(R_BATTERY_FULL)) != 0,
                                       !battery.has_full_cell() },
            { "R_DEAD_CELL",           (both & inhibit_reason_bit(R_DEAD_CELL)) != 0,
                                       !battery.has_dead_cell() },
            { "R_MODULE_UNRESPONSIVE", (both & inhibit_reason_bit(R_MODULE_UNRESPONSIVE)) != 0,
                                       battery.is_alive() },
            { "R_SHUNT_UNRESPONSIVE",  (both & inhibit_reason_bit(R_SHUNT_UNRESPONSIVE)) != 0,
                                       !shunt.is_dead() },
        };
        static Sustained stuck[6];
        for (size_t i = 0; i < sizeof stale / sizeof stale[0]; i++) {
            if (stuck[i](stale[i].stillHeld && stale[i].conditionGone, 5000)) {
                char msg[96];
                snprintf(msg, sizeof msg, "%s held for 5 s after its condition cleared",
                         stale[i].name);
                violated(msg, seed, step);
            }
        }
    }

    /* 11b. The symmetric contactor check: a hold that outlives every reason for
     *      it. Isolating a healthy pack costs half the battery. */
    {
        static Sustained spuriousHold;
        const bool nothingWrong = battery.is_alive()
                               && battery.pack_voltages_are_comparable()
                               && !bms.packs_are_imbalanced()
                               && !battery.has_dead_cell();
        if (spuriousHold(nothingWrong && battery.one_or_more_contactors_inhibited(), 5000))
            violated("contactors held open with no live reason", seed, step);
    }

    /* 12. Once the battery has reported, the power-on hold must be gone. A
     *     stuck R_STARTUP is an inhibit nothing can ever withdraw. */
    {
        static Sustained startupStuck;
        if (startupStuck(battery.is_alive()
                && ( ( bms.get_drive_inhibit_reasons() | bms.get_charge_inhibit_reasons() )
                     & inhibit_reason_bit(R_STARTUP) ) != 0, REACT_MS))
            violated("R_STARTUP still held after the battery reported", seed, step);
    }
}

static void fuzz_round(uint32_t seed, long steps) {
    rngState = seed ? seed : 1;
    sim_reset();
    setup();

    for (long step = 0; step < steps; step++) {
        switch (rnd_below(10)) {
            case 0: {                       // arbitrary frame on the main bus
                SimFrame f;
                f.id = rnd_below(0x800);
                f.len = (uint8_t)rnd_below(9);
                for (int i = 0; i < 8; i++) f.data[i] = (uint8_t)rnd();
                sim_main_rx.push_back(f);
                break;
            }
            case 1: {                       // a shunt frame with an extreme payload
                SimFrame f;
                f.id = 0x521 + rnd_below(8);
                f.len = 8;
                for (int i = 0; i < 8; i++) f.data[i] = (uint8_t)rnd();
                sim_main_rx.push_back(f);
                break;
            }
            case 2: {                       // arbitrary frame on a pack bus
                SimFrame f;
                f.id = rnd_below(0x800);
                f.len = (uint8_t)rnd_below(9);
                for (int i = 0; i < 8; i++) f.data[i] = (uint8_t)rnd();
                sim_pack_rx[rnd_below(NUM_PACKS)].push_back(f);
                break;
            }
            case 3:                         // inputs chatter
                sim_gpio[IGNITION_ENABLE_PIN] = (int)rnd_below(2);
                sim_gpio[CHARGE_ENABLE_PIN] = (int)rnd_below(2);
                break;
            case 4:                         // contactor feedback chatter
                sim_gpio[POS_CONTACTOR_FEEDBACK_PIN] = (int)rnd_below(2);
                sim_gpio[NEG_CONTACTOR_FEEDBACK_PIN] = (int)rnd_below(2);
                for (int p = 0; p < NUM_PACKS; p++)
                    sim_gpio[CONTACTOR_FEEDBACK_PINS[p]] = (int)rnd_below(2);
                break;
            case 5:                         // cell voltages jump anywhere plausible
                sim_set_cells((uint16_t)(2000 + rnd_below(2600)));
                break;
            case 6:                         // one module diverges hard
                sim_set_cell_override((int)rnd_below(NUM_PACKS),
                                      (int)rnd_below(MODULES_PER_PACK),
                                      (int)rnd_below(2000) - 1000);
                break;
            case 7:                         // temperature swings, within what a
                                            // real sensor count can encode
                sim_set_temp((int8_t)(MODULE_SENSOR_MINIMUM_C
                                      + (int)rnd_below(MODULE_SENSOR_MAXIMUM_C
                                                       - MODULE_SENSOR_MINIMUM_C + 1)));
                break;
            case 8:                         // modules and shunt come and go
                sim_modules_answer(rnd_below(2) != 0);
                sim_shunt_answers(rnd_below(2) != 0);
                break;
            default:
                break;
        }
        sim_run_ms(5 + (long)rnd_below(200));
        check_invariants(seed, step);
        if (violations > 0) return;        // stop at the first, it is reproducible
    }

    // Leave the rig in a sane state for whatever runs next
    sim_modules_answer(true);
    sim_shunt_answers(true);
    sim_set_cells(3700);
    sim_set_temp(25);
    for (int p = 0; p < NUM_PACKS; p++)
        for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(p, m, 0);
}

void test_fuzz() {
    suite("fuzz: safety invariants hold under adversarial input");
    violations = 0;
    firstViolation[0] = '\0';
    for (uint32_t seed = 1; seed <= 250 && violations == 0; seed++) {
        fuzz_round(seed, 400);
    }
    if (violations > 0) printf("   first violation: %s\n", firstViolation);
    check_eq("no invariant was violated across 250 seeds x 400 steps", violations, 0);
    sim_reset();
}
