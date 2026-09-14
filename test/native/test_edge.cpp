#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "pack.h"
#include "statemachine.h"
#include <esp_task_wdt.h>

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;
void setup();

static State kStates[] = { state_standby, state_drive, state_batteryHeating,
    state_charging, state_batteryEmpty, state_overTempFault,
    state_illegalStateTransitionFault, state_criticalFault };

static void fresh() {
    sim_reset(); io.init(); battery.initialise(&bms); bms.init(&battery, &io, &shunt);
}

void test_edge() {
    suite("edge: hardware bring-up failures");
    sim_reset();
    sim_set_ledc_result(1);                  // wrong frequency reported
    setup();
    check("CAN clock frequency mismatch reported", true);
    sim_reset();
    sim_set_ledc_result(0xFFFFFFFF);
    setup();
    check("CAN clock path runs", true);
    sim_reset();
    sim_set_reset_reason(ESP_RST_TASK_WDT);
    setup();
    check_eq("watchdog reboot detected", bms.get_watchdog_reboot(), 1);
    sim_reset();
    sim_wdt_init_fails = true; sim_task_create_fails = true;
    setup();
    check("watchdog init and task create failures reported", true);
    sim_reset();
    sim_main_begin_error = 0x0002; sim_pack_begin_error = 0x0040;
    setup();
    check("CAN begin errors reported", true);
    sim_reset();
    sim_wdt_add_fails = true;
    setup(); sim_run_ms(50);
    check("watchdog subscribe failure reported", true);
    sim_reset();

    suite("edge: unknown events reach the defensive defaults");
    for (int s = 0; s < 8; s++) {
        fresh();
        /* illegalStateTransitionFault escapes to standby before its switch when
         * neither input is asking for anything, so keep one asserted. */
        sim_gpio[IGNITION_ENABLE_PIN] = 1;
        for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 1; i++) io.poll_inputs();
        bms.set_state(kStates[s], "t");
        uint16_t before = bms.get_invalid_event_count();
        bms.send_event((Event)99);           // not a declared Event value
        check("default branch counted the invalid event",
              bms.get_invalid_event_count() > before, get_state_name(kStates[s]));
        sim_gpio[IGNITION_ENABLE_PIN] = 0;
    }

    suite("edge: getcheck rejects an out-of-range module");
    fresh();
    BatteryPack p;
    BatteryPackConfig c;
    c.id = 0; c.canChipSelectPin = CS_PINS[0];
    c.contactorInhibitPin = INHIBIT_CONTACTOR_PINS[0];
    c.contactorFeedbackPin = CONTACTOR_FEEDBACK_PINS[0];
    c.numModules = MODULES_PER_PACK; c.numCellsPerModule = CELLS_PER_MODULE;
    c.numTemperatureSensorsPerModule = TEMPS_PER_MODULE;
    p.init(c);
    CANMessage m; zero_frame(&m); m.id = 0x080;
    check_eq("negative module id returns 0", p.getcheck(m, -1), 0);
    check_eq("module id past finalxor returns 0", p.getcheck(m, 99), 0);
    check("valid module id returns a checksum", p.getcheck(m, 0) == p.getcheck(m, 0));
    CANMessage hot; zero_frame(&hot); hot.id = 0x180;
    for (int i = 0; i < TEMPS_PER_MODULE; i++) hot.data[i] = 255;   // clamps to +127
    p.decode_temperatures(&hot);
    check("extreme raw temperature clamped", true);
    CANMessage unknown; zero_frame(&unknown); unknown.id = 0x190;   // unhandled group
    p.decode_voltages(&unknown);
    check("unhandled voltage group ignored", true);

    suite("edge: temperature rate sampling over a long run");
    sim_reset(); setup(); sim_run_ms(2000);
    sim_set_temp(25); sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 2000);
    sim_set_temp(30); sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 2000);
    check("temperature rate sampled across two intervals", true);
    sim_modules_answer(false);
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 2000);
    check("baseline dropped when the reading goes away", true);
    sim_modules_answer(true);

    suite("edge: packs at different voltages");
    sim_reset(); setup(); sim_run_ms(2000);
    for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -60);
    sim_run_ms(3000);
    check("high/low pack selection runs", battery.get_index_of_high_pack() >= 0
                                       && battery.get_index_of_low_pack() >= 0);
    sim_gpio[IGNITION_ENABLE_PIN] = 1; sim_run_ms(1000);
    check_eq("drive entered with imbalanced packs", bms.get_state() == &state_drive, 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_run_ms(1000);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1000);
    check("charge path with imbalanced packs exercised", true);
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(500);
    for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, 0);

    suite("edge: illegal state transition, drive straight to charge");
    sim_reset(); setup(); sim_run_ms(2000);
    battery.enable_inhibit_contactor_close();      // force a hold so the move is illegal
    bms.set_state(&state_drive, "t");
    bms.send_event(E_CHARGING_INITIATED);
    check_eq("entered the illegal-transition fault",
             bms.get_state() == &state_illegalStateTransitionFault, 1);
    check_eq("flag raised", bms.get_illegal_state_transition(), 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 1;      // keep the level escape from firing
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 1; i++) io.poll_inputs();
    bms.send_event(E_MODULE_UNRESPONSIVE);
    check_eq("and can still fall through to criticalFault",
             bms.get_state() == &state_criticalFault, 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 0;

    suite("edge: illegal transition out of charging");
    sim_reset(); setup(); sim_run_ms(2000);
    /* Ignition first: turning it on from standby re-evaluates and would
     * release a hold applied before it. */
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 1; i++) io.poll_inputs();
    battery.enable_inhibit_contactor_close();
    bms.set_state(&state_charging, "t");
    bms.send_event(E_CHARGING_TERMINATED);
    check_eq("charge to drive with a hold is illegal",
             bms.get_state() == &state_illegalStateTransitionFault, 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 0;

    suite("edge: state frame encodes every state");
    for (int s = 0; s < 8; s++) {
        sim_reset(); setup(); sim_run_ms(300);
        bms.set_state(kStates[s], "t");
        sim_main_tx.clear();
        sim_run_ms(1100);                      // long enough for a 0x352
        SimFrame f;
        check("state frame emitted", sim_last_frame(0x352, f), get_state_name(kStates[s]));
    }
    sim_reset(); setup(); sim_run_ms(300);
    bms.set_state(nullptr, "unknown");         // exercises the 0xFF branch
    sim_main_tx.clear();
    sim_run_ms(1100);
    check("unknown state encodes without a crash", true);

    suite("edge: SoC taper between the threshold and full");
    fresh();
    shunt.set_ampSeconds(-(int32_t)(BATTERY_CAPACITY_AS * 0.05));
    bms.recalculate_soc();
    check_eq("SoC in the taper band", bms.get_soc(), 95);
    check("taper produces a partial limit",
          bms.get_max_charge_current_by_soc() > 0 &&
          bms.get_max_charge_current_by_soc() < CHARGE_CURRENT_MAX_PER_PACK_A * NUM_PACKS);
    shunt.set_ampSeconds(0);

    suite("edge: heating times out");
    sim_reset(); setup(); sim_run_ms(2000);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_set_temp(-20); sim_run_ms(1000);
    check_eq("heating", bms.get_state() == &state_batteryHeating, 1);
    sim_run_ms(BATTERY_HEATING_TIMEOUT_MS + 2000);
    check_eq("heater abandoned after the timeout", sim_gpio[HEATER_ENABLE_PIN], 0);
    check_eq("and reported", bms.has_internal_error(IE_HEATER_INEFFECTIVE), 1);
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_set_temp(25);

    suite("edge: welded contactors");
    sim_reset(); setup(); sim_run_ms(2000);
    sim_gpio[POS_CONTACTOR_FEEDBACK_PIN] = 1;
    sim_gpio[NEG_CONTACTOR_FEEDBACK_PIN] = 1;
    sim_run_ms(WELD_CHECK_SETTLE_MS + 1000);
    check("weld detected once settled with everything off", bms.get_welding_byte() != 0);
    sim_gpio[POS_CONTACTOR_FEEDBACK_PIN] = 0;
    sim_gpio[NEG_CONTACTOR_FEEDBACK_PIN] = 0;
    sim_reset();
}
