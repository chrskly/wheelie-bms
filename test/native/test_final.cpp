#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "pack.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery; extern Io io;
void setup();

/* Counts arming commands across both pack buses. */
static int count_balance_commands() {
    int n = 0;
    for (int p = 0; p < NUM_PACKS; p++)
        for (auto& tx : sim_pack_tx[p])
            if ((tx.id & 0xFF0) == 0x080 && tx.data[4] == MODULE_CMD_BALANCE_ON) n++;
    return n;
}

static void enter(State s, bool ignition, bool charge) {
    sim_gpio[IGNITION_ENABLE_PIN] = ignition ? 1 : 0;
    sim_gpio[CHARGE_ENABLE_PIN] = charge ? 1 : 0;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 2; i++) io.poll_inputs();
    bms.set_state(s, "test");
}

/* Brings the packs up genuinely imbalanced: pack 1 sits far enough below pack 0
 * that voltage_delta_between_packs() exceeds SAFE_VOLTAGE_DELTA_BETWEEN_PACKS,
 * which is what bms.packs_are_imbalanced() actually reads. */
static void boot_imbalanced() {
    sim_reset();
    for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -80);
    setup();
    /* Bms::packs_are_imbalanced() is time-qualified: the mismatch has to
     * persist for PACKS_IMBALANCED_TTL_MS before it counts. */
    /* Long enough for the startup contactor hold to be withdrawn first, and
     * then for the imbalance to outlast PACKS_IMBALANCED_TTL_MS. */
    sim_run_ms(PACKS_IMBALANCED_TTL_MS + 8000);
}

static void boot_balanced() {
    sim_reset();
    setup();
    sim_run_ms(3000);
}

void test_final() {
    suite("final: the imbalance flag is genuinely set");
    boot_imbalanced();
    check("delta exceeds the safe threshold",
          battery.voltage_delta_between_packs() > SAFE_VOLTAGE_DELTA_BETWEEN_PACKS);
    check_eq("so the BMS reports imbalance", bms.packs_are_imbalanced(), 1);

    suite("final: the imbalance hold sticks, and releases only on recovery");
    {
        sim_reset();
        for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -80);
        setup();
        sim_run_ms(8000);
        check_eq("a 7.68 V imbalance holds every contactor open",
                 battery.one_or_more_contactors_inhibited(), 1);
        /* The hold must not withdraw itself. Inhibiting the contactors used to
         * make the packs uncomparable, which read as "matched" and released the
         * hold about three seconds later, over and over. */
        int held = 0;
        for (int i = 0; i < 2000; i++) {          // 10 s at 5 ms
            sim_run_ms(5);
            if (battery.one_or_more_contactors_inhibited()) held++;
        }
        check_eq("and stays held for every sample over the next 10 s", held, 2000);

        for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -20);
        sim_run_ms(8000);
        check("1.92 V is still outside the safe window",
              battery.voltage_delta_between_packs() > SAFE_VOLTAGE_DELTA_BETWEEN_PACKS);
        check_eq("so the hold stays on", battery.one_or_more_contactors_inhibited(), 1);

        for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -5);
        sim_run_ms(8000);
        check("480 mV is within the safe window",
              battery.voltage_delta_between_packs() < SAFE_VOLTAGE_DELTA_BETWEEN_PACKS);
        check_eq("and the hold is released", battery.one_or_more_contactors_inhibited(), 0);

        for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, 0);
        sim_run_ms(8000);
        check_eq("fully converged, still released",
                 battery.one_or_more_contactors_inhibited(), 0);
    }

    suite("final: batteryEmpty holds contactors open while imbalanced");
    boot_imbalanced();
    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_BATTERY_NOT_EMPTY);
    check("-> standby with the hold applied", bms.get_state() == &state_standby);

    boot_imbalanced();
    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_BATTERY_FULL);
    check("full -> standby with the hold applied", bms.get_state() == &state_standby);

    suite("final: batteryEmpty contactor holds track ignition and charge");
    boot_imbalanced();
    enter(&state_batteryEmpty, true, false);
    bms.send_event(E_IGNITION_ON);
    check("ignition on while imbalanced releases for drive",
          bms.get_state() == &state_batteryEmpty);
    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_IGNITION_OFF);
    check("ignition off re-asserts the hold", bms.get_state() == &state_batteryEmpty);
    bms.send_event(E_PACKS_IMBALANCED);
    bms.send_event(E_PACKS_NOT_IMBALANCED);
    check("imbalance events are absorbed", bms.get_state() == &state_batteryEmpty);

    boot_imbalanced();
    enter(&state_batteryEmpty, false, true);
    bms.send_event(E_CHARGING_INITIATED);
    check("charge while imbalanced releases for charge and moves on",
          bms.get_state() == &state_charging || bms.get_state() == &state_batteryHeating);

    suite("final: overTempFault contactor holds track ignition and charge");
    boot_imbalanced();
    enter(&state_overTempFault, false, false);
    bms.send_event(E_PACKS_IMBALANCED);
    check("hold applied", bms.get_state() == &state_overTempFault);
    bms.send_event(E_PACKS_NOT_IMBALANCED);
    check("hold released", bms.get_state() == &state_overTempFault);
    enter(&state_overTempFault, true, false);
    bms.send_event(E_IGNITION_ON);
    enter(&state_overTempFault, false, false);
    bms.send_event(E_IGNITION_OFF);
    bms.send_event(E_CHARGING_INITIATED);
    bms.send_event(E_CHARGING_TERMINATED);
    check("every contactor-hold event absorbed without leaving the fault",
          bms.get_state() == &state_overTempFault);

    suite("final: overTempFault cools to standby while imbalanced");
    boot_imbalanced();
    enter(&state_overTempFault, false, false);
    bms.send_event(E_TEMPERATURE_OK);
    check("-> standby", bms.get_state() == &state_standby);

    boot_imbalanced();
    sim_set_temp(-20); sim_run_ms(3000);
    enter(&state_overTempFault, false, false);
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check("too cold -> standby while imbalanced", bms.get_state() == &state_standby);
    sim_set_temp(25);

    suite("final: illegalStateTransitionFault clears on the last input dropping");
    boot_balanced();
    bms.set_illegal_state_transition();
    /* Ignition on, charge off: the top-of-function guard needs BOTH off, so it
     * does not fire, and the E_IGNITION_OFF case is what clears the fault. */
    enter(&state_illegalStateTransitionFault, true, false);
    bms.send_event(E_IGNITION_OFF);
    check("[I03] cleared via the event", bms.get_state() == &state_standby);
    check_eq("flag cleared", bms.get_illegal_state_transition(), 0);
    check_eq("drive inhibit withdrawn", bms.drive_is_inhibited(), 0);

    boot_balanced();
    bms.set_illegal_state_transition();
    enter(&state_illegalStateTransitionFault, false, true);
    bms.send_event(E_CHARGING_TERMINATED);
    check("[I04] cleared via the event", bms.get_state() == &state_standby);
    check_eq("flag cleared", bms.get_illegal_state_transition(), 0);

    suite("final: balanced packs are both permitted to close");
    boot_balanced();
    battery.enable_inhibit_contactor_close();
    battery.reevaluate_contactor_inhibition_for_drive();
    check_eq("neither pack is inhibited for drive", battery.one_or_more_contactors_inhibited(), 0);
    battery.enable_inhibit_contactor_close();
    battery.reevaluate_contactor_inhibition_for_charge();
    check_eq("neither pack is inhibited for charge", battery.one_or_more_contactors_inhibited(), 0);

    /* should_start_balancing() is only consulted once a full CELL_BALANCE_REST_MS
     * rest window has elapsed, so these have to run past that to reach it. */
    suite("final: balancing is refused while the modules have not reported");
    sim_reset();
    sim_modules_answer(false);
    setup();
    sim_run_ms(CELL_BALANCE_REST_MS + 10000);
    check_eq("no balance command was sent", count_balance_commands(), 0);
    sim_modules_answer(true);

    suite("final: balancing is refused when every reading is zero");
    sim_reset();
    sim_set_cells(0);
    setup();
    sim_run_ms(CELL_BALANCE_REST_MS + 10000);
    check_eq("no balance command on all-zero readings", count_balance_commands(), 0);
    sim_set_cells(3700);

    suite("final: a welded pack contactor is reported in the welding byte");
    {
        /* A pack contactor counts as welded when its feedback says closed at a
         * moment the pack is holding it open, after the settle window. Boot
         * imbalanced so the hold is one the firmware itself keeps applied --
         * with balanced packs standby withdraws it again within the settle
         * window, which is correct behaviour and not what is under test here. */
        boot_imbalanced();
        check_eq("nothing welded at rest", bms.get_welding_byte(), 0x00);
        check_eq("but the contactors are held open",
                 battery.one_or_more_contactors_inhibited(), 1);
        sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 1;
        sim_run_ms(WELD_CHECK_SETTLE_MS + 1000);
        check_eq("pack 0 welded shows in bit 2", bms.get_welding_byte() & 0x04, 0x04);
        check_eq("pack 1 is not implicated", (long)(bms.get_welding_byte() & 0x08), 0);
        sim_gpio[CONTACTOR_FEEDBACK_PINS[1]] = 1;
        sim_run_ms(WELD_CHECK_SETTLE_MS + 1000);
        check_eq("pack 1 welded shows in bit 3", bms.get_welding_byte() & 0x08, 0x08);
        sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 0;
        sim_gpio[CONTACTOR_FEEDBACK_PINS[1]] = 0;
        sim_run_ms(1000);
        check_eq("and both clear when the feedback opens", bms.get_welding_byte(), 0x00);
        for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, 0);
    }

    suite("final: a pack configured with no modules is not polled");
    {
        /* init() clamps numModules into range and accepts 0, so a misconfigured
         * pack is a reachable state. request_data() must decline to poll rather
         * than sweep a zero-length module array. */
        sim_reset();
        BatteryPack empty;
        BatteryPackConfig cfg;
        cfg.id = 0;
        cfg.canChipSelectPin = CS_PINS[0];
        cfg.contactorInhibitPin = INHIBIT_CONTACTOR_PINS[0];
        cfg.contactorFeedbackPin = CONTACTOR_FEEDBACK_PINS[0];
        cfg.numModules = 0;
        cfg.numCellsPerModule = CELLS_PER_MODULE;
        cfg.numTemperatureSensorsPerModule = TEMPS_PER_MODULE;
        empty.init(cfg);
        sim_pack_tx[0].clear();
        empty.request_data();
        empty.request_data();
        check_eq("no module poll was sent", (long)sim_pack_tx[0].size(), 0);
        check_eq("and it reports no modules populated", empty.all_modules_populated(), 0);
    }

    suite("final: charge current below the derating threshold");
    boot_balanced();
    sim_set_temp(10);                       // above CHARGE_TEMPERATURE_MINIMUM, below 15
    sim_run_ms(3000);
    check("full current allowed below 15 C",
          battery.get_max_charge_current_by_temperature() > 0);

    suite("final: a runaway temperature rise derates charge current to zero");
    boot_balanced();
    sim_set_temp(20);
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 5000);     // establishes the baseline
    sim_set_temp(45);                                    // +25 C, well over threshold+10
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 5000);     // produces the delta
    check_eq("charge current derated to zero",
             battery.get_max_charge_current_by_temperature(), 0);
    sim_set_temp(25); sim_run_ms(2000);

    suite("final: the temperature baseline drops when the sensors go unfitted");
    boot_balanced();
    sim_set_temp(25);
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 5000);     // baseline established
    sim_set_temp_sensors_fitted(0);                      // frames still arrive, no readings
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 5000);
    check("baseline abandoned rather than treated as a huge drop", true);
    sim_set_temp_sensors_fitted(TEMPS_PER_MODULE);
    sim_set_temp(25);

    sim_reset();
}
