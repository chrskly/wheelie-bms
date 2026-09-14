#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;
void setup();

/* Drives one state directly with one event, with the inputs the branch needs.
 * The worker is not running: these exercise the transition functions as pure
 * functions of (state, event, inputs), which is the only way to reach branches
 * that need input combinations the physical simulation cannot hold still. */
static void enter(State s, bool ignition, bool charge) {
    sim_gpio[IGNITION_ENABLE_PIN] = ignition ? 1 : 0;
    sim_gpio[CHARGE_ENABLE_PIN] = charge ? 1 : 0;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 2; i++) io.poll_inputs();
    bms.set_state(s, "test");
}

static void boot_idle() {
    sim_reset();
    setup();
    sim_run_ms(2000);
}

void test_matrix() {
    suite("matrix: batteryEmpty recovers by battery level");
    boot_idle();
    enter(&state_batteryEmpty, true, false);
    bms.send_event(E_BATTERY_NOT_EMPTY);
    check("ignition on -> drive", bms.get_state() == &state_drive);

    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_BATTERY_NOT_EMPTY);
    check("ignition off -> standby", bms.get_state() == &state_standby);

    enter(&state_batteryEmpty, true, false);
    bms.send_event(E_BATTERY_FULL);
    check("full with ignition on -> drive", bms.get_state() == &state_drive);
    check_eq("and charging is inhibited", bms.charge_is_inhibited(), 1);

    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_BATTERY_FULL);
    check("full with ignition off -> standby", bms.get_state() == &state_standby);

    suite("matrix: batteryEmpty contactor holds follow imbalance");
    boot_idle();
    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_PACKS_IMBALANCED);
    bms.send_event(E_IGNITION_OFF);
    bms.send_event(E_PACKS_NOT_IMBALANCED);
    bms.send_event(E_IGNITION_ON);
    bms.send_event(E_CHARGING_INITIATED);
    check("charge requested from empty -> charging", bms.get_state() == &state_charging);

    suite("matrix: batteryEmpty, charge requested while too cold");
    boot_idle();
    sim_set_temp(-20);
    sim_run_ms(3000);
    enter(&state_batteryEmpty, false, false);
    bms.send_event(E_CHARGING_INITIATED);
    check("-> batteryHeating", bms.get_state() == &state_batteryHeating);
    sim_set_temp(25); sim_run_ms(2000);

    suite("matrix: overTempFault cools to each destination");
    boot_idle();
    enter(&state_overTempFault, false, true);
    bms.send_event(E_TEMPERATURE_OK);
    check("charge enabled -> charging", bms.get_state() == &state_charging);
    check_eq("R_TOO_HOT withdrawn from charge", bms.charge_is_inhibited(), 0);

    enter(&state_overTempFault, true, false);
    bms.send_event(E_TEMPERATURE_OK);
    check("ignition on -> drive", bms.get_state() == &state_drive);
    check_eq("R_TOO_HOT withdrawn from drive", bms.drive_is_inhibited(), 0);

    enter(&state_overTempFault, false, false);
    bms.send_event(E_TEMPERATURE_OK);
    check("neither -> standby", bms.get_state() == &state_standby);

    suite("matrix: overTempFault via too-cold-to-charge");
    boot_idle();
    sim_set_temp(-20);
    sim_run_ms(3000);
    check_eq("readings are fresh", battery.temperature_data_is_stale(), 0);
    enter(&state_overTempFault, false, true);
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check("charge enabled -> batteryHeating", bms.get_state() == &state_batteryHeating);

    enter(&state_overTempFault, true, false);
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check("ignition on -> drive", bms.get_state() == &state_drive);

    enter(&state_overTempFault, false, false);
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check("neither -> standby", bms.get_state() == &state_standby);
    sim_set_temp(25); sim_run_ms(2000);

    suite("matrix: a stale reading must NOT clear an over-temp fault");
    boot_idle();
    sim_modules_answer(false);
    sim_run_ms(MODULE_TTL_MS + 5000);
    check_eq("data is stale", battery.temperature_data_is_stale(), 1);
    enter(&state_overTempFault, false, false);
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check("stays in the fault", bms.get_state() == &state_overTempFault);
    sim_modules_answer(true);

    suite("matrix: overTempFault inert and contactor-hold events");
    boot_idle();
    enter(&state_overTempFault, false, false);
    const Event inert[] = { E_TOO_HOT, E_BATTERY_EMPTY, E_BATTERY_NOT_EMPTY,
                            E_BATTERY_FULL, E_MODULES_ALL_RESPONSIVE, E_SHUNT_RESPONSIVE,
                            E_DEAD_CELL };
    for (size_t i = 0; i < sizeof inert / sizeof inert[0]; i++) {
        bms.send_event(inert[i]);
        check("stays put", bms.get_state() == &state_overTempFault);
    }
    bms.send_event(E_PACKS_IMBALANCED);
    bms.send_event(E_PACKS_NOT_IMBALANCED);
    bms.send_event(E_IGNITION_ON);
    bms.send_event(E_IGNITION_OFF);
    bms.send_event(E_CHARGING_INITIATED);
    bms.send_event(E_CHARGING_TERMINATED);
    check("contactor-hold events do not move the state",
          bms.get_state() == &state_overTempFault);
    check_eq("no invalid events were counted", bms.get_invalid_event_count(), 0);

    suite("matrix: overTempFault escalates to criticalFault");
    boot_idle();
    enter(&state_overTempFault, false, false);
    bms.send_event(E_MODULE_UNRESPONSIVE);
    check("dead module -> criticalFault", bms.get_state() == &state_criticalFault);

    boot_idle();
    enter(&state_overTempFault, false, false);
    bms.send_event(E_SHUNT_UNRESPONSIVE);
    check("dead shunt -> criticalFault", bms.get_state() == &state_criticalFault);

    suite("matrix: illegalStateTransitionFault holds until both inputs drop");
    boot_idle();
    bms.set_illegal_state_transition();
    enter(&state_illegalStateTransitionFault, true, true);
    bms.send_event(E_IGNITION_OFF);
    check("charge still on, fault holds", bms.get_state() == &state_illegalStateTransitionFault);
    enter(&state_illegalStateTransitionFault, false, false);
    bms.send_event(E_IGNITION_OFF);
    check("both off -> standby", bms.get_state() == &state_standby);
    check_eq("the illegal flag is cleared", bms.get_illegal_state_transition(), 0);

    boot_idle();
    bms.set_illegal_state_transition();
    enter(&state_illegalStateTransitionFault, true, true);
    bms.send_event(E_CHARGING_TERMINATED);
    check("ignition still on, fault holds", bms.get_state() == &state_illegalStateTransitionFault);
    enter(&state_illegalStateTransitionFault, false, false);
    bms.send_event(E_CHARGING_TERMINATED);
    check("both off -> standby", bms.get_state() == &state_standby);

    suite("matrix: illegalStateTransitionFault ignores everything else");
    boot_idle();
    bms.set_illegal_state_transition();
    enter(&state_illegalStateTransitionFault, true, true);
    const Event ignored[] = { E_TOO_COLD_TO_CHARGE, E_TEMPERATURE_OK, E_TOO_HOT,
                              E_BATTERY_EMPTY, E_BATTERY_NOT_EMPTY, E_BATTERY_FULL,
                              E_PACKS_IMBALANCED, E_PACKS_NOT_IMBALANCED,
                              E_IGNITION_ON, E_CHARGING_INITIATED,
                              E_MODULES_ALL_RESPONSIVE, E_SHUNT_RESPONSIVE,
                              E_DEAD_CELL };
    for (size_t i = 0; i < sizeof ignored / sizeof ignored[0]; i++) {
        bms.send_event(ignored[i]);
        check("stays in the fault", bms.get_state() == &state_illegalStateTransitionFault);
    }
    check_eq("drive stays inhibited throughout", bms.drive_is_inhibited(), 1);
    check_eq("charge stays inhibited throughout", bms.charge_is_inhibited(), 1);
    check_eq("and the heater stays off", io.heater_is_enabled(), 0);

    suite("matrix: illegalStateTransitionFault escalates to criticalFault");
    boot_idle();
    bms.set_illegal_state_transition();
    enter(&state_illegalStateTransitionFault, true, true);
    bms.send_event(E_MODULE_UNRESPONSIVE);
    check("dead module -> criticalFault", bms.get_state() == &state_criticalFault);

    boot_idle();
    bms.set_illegal_state_transition();
    enter(&state_illegalStateTransitionFault, true, true);
    bms.send_event(E_SHUNT_UNRESPONSIVE);
    check("dead shunt -> criticalFault", bms.get_state() == &state_criticalFault);

    suite("matrix: criticalFault clears by modules, to each destination");
    boot_idle();
    enter(&state_criticalFault, false, true);
    bms.send_event(E_MODULES_ALL_RESPONSIVE);
    check("charge enabled -> charging", bms.get_state() == &state_charging);

    boot_idle();
    enter(&state_criticalFault, true, false);
    bms.send_event(E_MODULES_ALL_RESPONSIVE);
    check("ignition on -> drive", bms.get_state() == &state_drive);

    boot_idle();
    enter(&state_criticalFault, false, false);
    bms.send_event(E_MODULES_ALL_RESPONSIVE);
    check("neither -> standby", bms.get_state() == &state_standby);
    check_eq("R_CRITICAL_FAULT is withdrawn from drive", bms.drive_is_inhibited(), 0);
    check_eq("R_CRITICAL_FAULT is withdrawn from charge", bms.charge_is_inhibited(), 0);

    boot_idle();
    sim_set_temp(MAXIMUM_TEMPERATURE + 10);
    sim_run_ms(3000);
    enter(&state_criticalFault, false, true);
    bms.send_event(E_MODULES_ALL_RESPONSIVE);
    check("still too hot -> overTempFault", bms.get_state() == &state_overTempFault);
    sim_set_temp(25); sim_run_ms(2000);

    suite("matrix: criticalFault clears by shunt, to each destination");
    boot_idle();
    enter(&state_criticalFault, false, true);
    bms.send_event(E_SHUNT_RESPONSIVE);
    check("charge enabled -> charging", bms.get_state() == &state_charging);

    boot_idle();
    enter(&state_criticalFault, true, false);
    bms.send_event(E_SHUNT_RESPONSIVE);
    check("ignition on -> drive", bms.get_state() == &state_drive);

    boot_idle();
    enter(&state_criticalFault, false, false);
    bms.send_event(E_SHUNT_RESPONSIVE);
    check("neither -> standby", bms.get_state() == &state_standby);

    boot_idle();
    sim_set_temp(MAXIMUM_TEMPERATURE + 10);
    sim_run_ms(3000);
    enter(&state_criticalFault, false, true);
    bms.send_event(E_SHUNT_RESPONSIVE);
    check("still too hot -> overTempFault", bms.get_state() == &state_overTempFault);
    sim_set_temp(25); sim_run_ms(2000);

    suite("matrix: criticalFault will not clear while the other half is dead");
    boot_idle();
    sim_shunt_answers(false);
    sim_run_ms(SHUNT_TTL_MS + 3000);
    check_eq("shunt is dead", shunt.is_dead(), 1);
    enter(&state_criticalFault, false, false);
    bms.send_event(E_MODULES_ALL_RESPONSIVE);
    check("modules back but shunt dead -> stays", bms.get_state() == &state_criticalFault);
    sim_shunt_answers(true);

    boot_idle();
    sim_modules_answer(false);
    sim_run_ms(MODULE_TTL_MS + 3000);
    check_eq("battery is not alive", battery.is_alive(), 0);
    enter(&state_criticalFault, false, false);
    bms.send_event(E_SHUNT_RESPONSIVE);
    check("shunt back but modules dead -> stays", bms.get_state() == &state_criticalFault);
    sim_modules_answer(true);

    suite("matrix: standby to charging when already warm");
    boot_idle();
    enter(&state_standby, false, false);
    bms.send_event(E_CHARGING_INITIATED);
    check("warm battery -> charging", bms.get_state() == &state_charging);

    suite("matrix: charging terminated with ignition on -> drive");
    boot_idle();
    enter(&state_charging, true, false);
    bms.send_event(E_CHARGING_TERMINATED);
    check("-> drive", bms.get_state() == &state_drive);
    check_eq("R_CHARGING withdrawn from drive", bms.drive_is_inhibited(), 0);

    sim_reset();
}
