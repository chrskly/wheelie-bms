#include "sim/unit.h"
#include "sim/sim.h"
#include "io.h"
#include "bms.h"
#include "battery.h"
#include "statemachine.h"

extern Io io; extern Bms bms; extern Battery battery;

void test_io() {
    suite("io: init leaves everything in the safe state");
    sim_reset();
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_gpio[CHARGE_ENABLE_PIN] = 0;
    io.init();
    check_eq("drive inhibited at init", sim_gpio[DRIVE_INHIBIT_PIN], 1);
    check_eq("charge inhibited at init", sim_gpio[CHARGE_INHIBIT_PIN], 1);
    check_eq("heater off at init", sim_gpio[HEATER_ENABLE_PIN], 0);
    check_eq("drive_is_inhibited agrees", io.drive_is_inhibited(), 1);
    check_eq("charge_is_inhibited agrees", io.charge_is_inhibited(), 1);
    check_eq("heater_is_enabled agrees", io.heater_is_enabled(), 0);
    check_eq("pins configured as outputs", sim_gpio_mode[DRIVE_INHIBIT_PIN], OUTPUT);

    suite("io: outputs are tracked, not read back");
    io.disable_drive_inhibit("test");
    check_eq("drive released", io.drive_is_inhibited(), 0);
    check_eq("pin follows", sim_gpio[DRIVE_INHIBIT_PIN], 0);
    io.disable_drive_inhibit("test");            // idempotent, no log
    check_eq("still released", io.drive_is_inhibited(), 0);
    io.enable_drive_inhibit("test");
    check_eq("drive asserted", io.drive_is_inhibited(), 1);
    io.enable_drive_inhibit("test");
    check_eq("still asserted", io.drive_is_inhibited(), 1);
    io.disable_charge_inhibit("test");
    check_eq("charge released", io.charge_is_inhibited(), 0);
    io.enable_charge_inhibit("test");
    check_eq("charge asserted", io.charge_is_inhibited(), 1);
    io.enable_heater();
    check_eq("heater on", io.heater_is_enabled(), 1);
    check_eq("heater pin follows", sim_gpio[HEATER_ENABLE_PIN], 1);
    io.enable_heater();
    io.disable_heater();
    check_eq("heater off", io.heater_is_enabled(), 0);
    io.disable_heater();
    check_eq("heater stays off", io.heater_is_enabled(), 0);

    suite("io: contactor feedback");
    sim_gpio[POS_CONTACTOR_FEEDBACK_PIN] = 1;
    check_eq("pos feedback closed", io.pos_contactor_feedback_closed(), 1);
    sim_gpio[POS_CONTACTOR_FEEDBACK_PIN] = 0;
    check_eq("pos feedback open", io.pos_contactor_feedback_closed(), 0);
    sim_gpio[NEG_CONTACTOR_FEEDBACK_PIN] = 1;
    check_eq("neg feedback closed", io.neg_contactor_feedback_closed(), 1);
    sim_gpio[NEG_CONTACTOR_FEEDBACK_PIN] = 0;
    check_eq("neg feedback open", io.neg_contactor_feedback_closed(), 0);

    suite("io: debounced inputs");
    sim_reset();
    battery.initialise(&bms);
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_gpio[CHARGE_ENABLE_PIN] = 0;
    io.init();
    bms.init(&battery, &io, nullptr);
    check_eq("seeded ignition off", io.ignition_is_on(), 0);
    check_eq("seeded charge off", io.charge_enable_is_on(), 0);
    io.poll_inputs();                            // first poll dispatches the initial state
    check_eq("still off after the initial dispatch", io.ignition_is_on(), 0);
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES - 1; i++) io.poll_inputs();
    check_eq("not accepted before the debounce completes", io.ignition_is_on(), 0);
    io.poll_inputs();
    check_eq("accepted once stable", io.ignition_is_on(), 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 0;
    io.poll_inputs();
    sim_gpio[IGNITION_ENABLE_PIN] = 1;           // bounces back before settling
    io.poll_inputs(); io.poll_inputs();
    check_eq("a bounce does not flip the state", io.ignition_is_on(), 1);
    sim_gpio[CHARGE_ENABLE_PIN] = 1;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES; i++) io.poll_inputs();
    check_eq("charge enable accepted", io.charge_enable_is_on(), 1);
    sim_gpio[CHARGE_ENABLE_PIN] = 0;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES; i++) io.poll_inputs();
    check_eq("charge enable released", io.charge_enable_is_on(), 0);

    suite("io: an input already active at boot is dispatched");
    sim_reset();
    battery.initialise(&bms);
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    io.init();
    bms.init(&battery, &io, nullptr);
    check_eq("seeded on from the pin", io.ignition_is_on(), 1);
    io.poll_inputs();
    check_eq("initial state dispatched into drive",
             bms.get_state() == &state_drive, 1);
    sim_reset();
}
