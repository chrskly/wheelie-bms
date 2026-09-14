#include "sim/unit.h"
#include "sim/sim.h"
#include "statemachine.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;

static State kStates[] = { state_standby, state_drive, state_batteryHeating,
    state_charging, state_batteryEmpty, state_overTempFault,
    state_illegalStateTransitionFault, state_criticalFault };
static const int kStateCount = 8;
static Event kEvents[] = { E_TOO_HOT, E_TOO_COLD_TO_CHARGE, E_TEMPERATURE_OK,
    E_BATTERY_EMPTY, E_BATTERY_NOT_EMPTY, E_BATTERY_FULL, E_PACKS_IMBALANCED,
    E_PACKS_NOT_IMBALANCED, E_IGNITION_ON, E_IGNITION_OFF, E_CHARGING_INITIATED,
    E_CHARGING_TERMINATED, E_MODULE_UNRESPONSIVE, E_MODULES_ALL_RESPONSIVE,
    E_SHUNT_UNRESPONSIVE, E_SHUNT_RESPONSIVE, E_DEAD_CELL };
static const int kEventCount = 17;

static void fresh() {
    sim_reset(); io.init(); battery.initialise(&bms); bms.init(&battery, &io, &shunt);
}

void test_statemachine() {
    suite("statemachine: names");
    check("standby name", strcmp(get_state_name(state_standby), "standby") == 0);
    check("drive name", strcmp(get_state_name(state_drive), "drive") == 0);
    check("heating name", strcmp(get_state_name(state_batteryHeating), "batteryHeating") == 0);
    check("charging name", strcmp(get_state_name(state_charging), "charging") == 0);
    check("empty name", strcmp(get_state_name(state_batteryEmpty), "batteryEmpty") == 0);
    check("overtemp name", strcmp(get_state_name(state_overTempFault), "overTempFault") == 0);
    check("illegal name",
          strcmp(get_state_name(state_illegalStateTransitionFault), "illegalStateTransitionFault") == 0);
    check("critical name", strcmp(get_state_name(state_criticalFault), "criticalFault") == 0);
    check("unknown state", strcmp(get_state_name(nullptr), "unknownState") == 0);

    suite("statemachine: every state handles every event");
    // Drive each of the 8 handlers with all 17 events. Nothing may crash, and
    // whatever state we land in must always be one of the eight known states.
    int combinations = 0;
    for (int s = 0; s < kStateCount; s++) {
        for (int e = 0; e < kEventCount; e++) {
            fresh();
            bms.set_state(kStates[s], "test setup");
            bms.send_event(kEvents[e]);
            State now = bms.get_state();
            bool known = false;
            for (int k = 0; k < kStateCount; k++) if (now == kStates[k]) known = true;
            if (!known) check("landed in a known state", false, get_state_name(now));
            combinations++;
        }
    }
    check_eq("all state/event combinations exercised", combinations, kStateCount * kEventCount);

    suite("statemachine: send_event before init is refused");
    sim_reset();
    Bms fresh_bms;
    fresh_bms.send_event(E_TOO_HOT);          // state is null, must not dispatch
    check("null state dispatch refused", true);

    suite("statemachine: standby transitions");
    fresh(); bms.set_state(&state_standby, "t");
    bms.send_event(E_TOO_HOT);
    check_eq("too hot -> overTempFault", bms.get_state() == &state_overTempFault, 1);
    fresh(); bms.set_state(&state_standby, "t");
    bms.send_event(E_BATTERY_EMPTY);
    check_eq("empty -> batteryEmpty", bms.get_state() == &state_batteryEmpty, 1);
    fresh(); bms.set_state(&state_standby, "t");
    bms.send_event(E_IGNITION_ON);
    check_eq("ignition -> drive", bms.get_state() == &state_drive, 1);
    fresh(); bms.set_state(&state_standby, "t");
    bms.send_event(E_MODULE_UNRESPONSIVE);
    check_eq("dead module -> criticalFault", bms.get_state() == &state_criticalFault, 1);
    fresh(); bms.set_state(&state_standby, "t");
    bms.send_event(E_SHUNT_UNRESPONSIVE);
    check_eq("dead shunt -> criticalFault", bms.get_state() == &state_criticalFault, 1);
    fresh(); bms.set_state(&state_standby, "t");
    bms.send_event(E_IGNITION_OFF);
    check_eq("ignition off in standby is invalid", bms.get_invalid_event_count() > 0, 1);
    bms.send_event(E_CHARGING_TERMINATED);
    check("charging terminated in standby is invalid", bms.get_invalid_event_count() > 1);

    suite("statemachine: drive transitions");
    fresh(); bms.set_state(&state_drive, "t");
    bms.send_event(E_IGNITION_OFF);
    check_eq("ignition off -> standby", bms.get_state() == &state_standby, 1);
    fresh(); bms.set_state(&state_drive, "t");
    bms.send_event(E_TOO_HOT);
    check_eq("too hot -> overTempFault", bms.get_state() == &state_overTempFault, 1);
    fresh(); bms.set_state(&state_drive, "t");
    bms.send_event(E_BATTERY_EMPTY);
    check_eq("empty -> batteryEmpty", bms.get_state() == &state_batteryEmpty, 1);
    fresh(); bms.set_state(&state_drive, "t");
    bms.send_event(E_IGNITION_ON);
    check("ignition on in drive is invalid", bms.get_invalid_event_count() > 0);

    suite("statemachine: charging and heating");
    fresh(); bms.set_state(&state_charging, "t");
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check_eq("too cold -> batteryHeating", bms.get_state() == &state_batteryHeating, 1);
    fresh(); bms.set_state(&state_charging, "t");
    bms.send_event(E_TOO_HOT);
    check_eq("too hot -> overTempFault", bms.get_state() == &state_overTempFault, 1);
    fresh(); bms.set_state(&state_charging, "t");
    bms.send_event(E_MODULE_UNRESPONSIVE);
    check_eq("dead module -> criticalFault", bms.get_state() == &state_criticalFault, 1);
    fresh(); bms.set_state(&state_batteryHeating, "t");
    bms.send_event(E_BATTERY_FULL);
    check_eq("full while heating -> charging", bms.get_state() == &state_charging, 1);
    check_eq("and charge is inhibited", bms.charge_is_inhibited(), 1);
    fresh(); bms.set_state(&state_batteryHeating, "t");
    bms.send_event(E_CHARGING_INITIATED);
    check("charging initiated while heating is invalid", bms.get_invalid_event_count() > 0);

    suite("statemachine: overTempFault will not leave on stale data");
    fresh(); bms.set_state(&state_overTempFault, "t");
    check_eq("temperature data is stale", battery.temperature_data_is_stale(), 1);
    bms.send_event(E_TOO_COLD_TO_CHARGE);
    check_eq("stays in the fault while blind", bms.get_state() == &state_overTempFault, 1);

    suite("statemachine: illegalStateTransition has a level escape");
    fresh(); bms.set_illegal_state_transition();
    bms.set_state(&state_illegalStateTransitionFault, "t");
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_gpio[CHARGE_ENABLE_PIN] = 0;
    bms.send_event(E_TEMPERATURE_OK);
    check_eq("leaves once neither input is asking", bms.get_state() == &state_standby, 1);
    check_eq("and the flag is cleared", bms.get_illegal_state_transition(), 0);

    suite("statemachine: criticalFault exit");
    fresh(); shunt.heartbeat(); bms.set_state(&state_criticalFault, "t");
    bms.send_event(E_MODULES_ALL_RESPONSIVE);
    check_eq("clears when the shunt is alive too", bms.get_state() != &state_criticalFault, 1);
    fresh(); shunt.heartbeat(); bms.set_state(&state_criticalFault, "t");
    bms.send_event(E_SHUNT_RESPONSIVE);
    check_eq("clears via the shunt path too", bms.get_state() != &state_criticalFault, 1);
    sim_reset();
}
