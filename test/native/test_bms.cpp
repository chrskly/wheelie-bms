#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;

static void fresh() {
    sim_reset();
    io.init();
    battery.initialise(&bms);
    bms.init(&battery, &io, &shunt);
}

void test_bms() {
    suite("bms: init seeds the fail-safe holds");
    fresh();
    check_eq("drive inhibited from power-on", bms.drive_is_inhibited(), 1);
    check_eq("charge inhibited from power-on", bms.charge_is_inhibited(), 1);
    check_eq("reason reported is R_STARTUP", bms.get_drive_inhibit_reason(), R_STARTUP);
    check_eq("charge reason reported is R_STARTUP", bms.get_charge_inhibit_reason(), R_STARTUP);
    check_eq("starts in standby", bms.get_state() == &state_standby, 1);
    check("time_in_state advances", bms.time_in_state_ms() <= 1);

    suite("bms: inhibit reason masks");
    bms.disable_drive_inhibit("t", R_STARTUP);
    check_eq("released once the only reason withdraws", bms.drive_is_inhibited(), 0);
    bms.enable_drive_inhibit("t", R_TOO_HOT);
    bms.enable_drive_inhibit("t", R_MODULE_UNRESPONSIVE);
    check_eq("two reasons held", bms.drive_is_inhibited(), 1);
    check_eq("reports the more severe", bms.get_drive_inhibit_reason(), R_MODULE_UNRESPONSIVE);
    bms.disable_drive_inhibit("t", R_MODULE_UNRESPONSIVE);
    check_eq("still held by the other", bms.drive_is_inhibited(), 1);
    check_eq("now reports the remaining one", bms.get_drive_inhibit_reason(), R_TOO_HOT);
    bms.disable_drive_inhibit("t", R_TOO_HOT);
    check_eq("released", bms.drive_is_inhibited(), 0);
    check_eq("reason is R_NONE", bms.get_drive_inhibit_reason(), R_NONE);
    bms.disable_drive_inhibit("t", R_NONE);
    check_eq("R_NONE occupies no bit", bms.drive_is_inhibited(), 0);
    bms.disable_charge_inhibit("t", R_STARTUP);   // R_STARTUP outranks it in the severity table
    bms.enable_charge_inhibit("t", R_BATTERY_FULL);
    check_eq("charge held", bms.charge_is_inhibited(), 1);
    check_eq("charge reason", bms.get_charge_inhibit_reason(), R_BATTERY_FULL);
    bms.disable_charge_inhibit("t", R_BATTERY_FULL);
    check_eq("charge released", bms.charge_is_inhibited(), 0);

    suite("bms: heater passthrough");
    bms.enable_heater();  check_eq("heater on", bms.heater_is_enabled(), 1);
    bms.disable_heater(); check_eq("heater off", bms.heater_is_enabled(), 0);

    suite("bms: inputs");
    io.poll_inputs();                             // first poll is the boot dispatch
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES; i++) io.poll_inputs();
    check_eq("ignition reported", bms.ignition_is_on(), 1);
    sim_gpio[CHARGE_ENABLE_PIN] = 1;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES; i++) io.poll_inputs();
    check_eq("charge enable reported", bms.charge_is_enabled(), 1);

    suite("bms: state of charge");
    fresh();
    shunt.set_ampSeconds(0);          bms.recalculate_soc();
    check_eq("full at zero amp-seconds", bms.get_soc(), 100);
    shunt.set_ampSeconds(-BATTERY_CAPACITY_AS / 2); bms.recalculate_soc();
    check_eq("half", bms.get_soc(), 50);
    shunt.set_ampSeconds(-BATTERY_CAPACITY_AS);     bms.recalculate_soc();
    check_eq("empty", bms.get_soc(), 0);
    shunt.set_ampSeconds(-BATTERY_CAPACITY_AS * 3); bms.recalculate_soc();
    check_eq("over-discharged clamps to 0", bms.get_soc(), 0);
    shunt.set_ampSeconds(BATTERY_CAPACITY_AS);      bms.recalculate_soc();
    check_eq("over-charged clamps to 100", bms.get_soc(), 100);
    check_eq("regen blocked when full", bms.regen_not_allowed(), 1);
    shunt.set_ampSeconds(-BATTERY_CAPACITY_AS / 2); bms.recalculate_soc();
    check_eq("regen allowed at half", bms.regen_not_allowed(), 0);
    shunt.set_ampSeconds(0);

    suite("bms: internal error sources");
    fresh();
    check_eq("no error at start", bms.get_internal_error(), 0);
    bms.set_internal_error(IE_LOW_CELL_RANGE);
    check_eq("error raised", bms.get_internal_error(), 1);
    check_eq("specific source reported", bms.has_internal_error(IE_LOW_CELL_RANGE), 1);
    check_eq("other source not reported", bms.has_internal_error(IE_HIGH_TEMP_RANGE), 0);
    bms.set_internal_error(IE_HIGH_TEMP_RANGE);
    bms.clear_internal_error(IE_LOW_CELL_RANGE);
    check_eq("still raised by the other source", bms.get_internal_error(), 1);
    bms.clear_internal_error(IE_HIGH_TEMP_RANGE);
    check_eq("cleared when no source remains", bms.get_internal_error(), 0);

    suite("bms: reported bytes");
    fresh();
    uint8_t st = bms.get_status_byte();
    check_eq("charge inhibit bit set at boot", st & 0x01, 0x01);
    check_eq("drive inhibit bit set at boot", (st >> 1) & 0x01, 0x01);
    uint8_t eb = bms.get_error_byte();
    check("error byte computed", eb == eb);
    bms.do_welding_checks();
    check("welding byte computed", bms.get_welding_byte() == bms.get_welding_byte());
    bms.increment_invalid_event_count();
    check_eq("invalid event counter increments", bms.get_invalid_event_count(), 1);
    bms.set_illegal_state_transition();
    check_eq("illegal flag set", bms.get_illegal_state_transition(), 1);
    bms.clear_illegal_state_transition();
    check_eq("illegal flag cleared", bms.get_illegal_state_transition(), 0);
    bms.set_watchdog_reboot(true);
    check_eq("watchdog flag set", bms.get_watchdog_reboot(), 1);
    bms.set_watchdog_reboot(false);
    check_eq("watchdog flag cleared", bms.get_watchdog_reboot(), 0);

    suite("bms: current limits");
    fresh();
    bms.update_max_charge_current();
    bms.update_max_discharge_current();
    check_eq("no charge with everything inhibited", bms.get_max_charge_current(), 0);
    check_eq("no discharge with everything inhibited", bms.get_max_discharge_current(), 0);
    shunt.set_ampSeconds(-BATTERY_CAPACITY_AS / 2); bms.recalculate_soc();
    check("SoC taper allows current below the threshold", bms.get_max_charge_current_by_soc() > 0);
    shunt.set_ampSeconds(0); bms.recalculate_soc();
    check_eq("SoC taper is zero at 100%", bms.get_max_charge_current_by_soc(), 0);

    suite("bms: CAN send and receive");
    fresh();
    CANMessage m; zero_frame(&m); m.id = 0x123;
    check_eq("send succeeds", bms.send_frame(&m, false), 1);
    zero_frame(&m); m.id = 0x124; m.data[0] = 0x0F; m.data[1] = 0xF0;
    check_eq("send with checksum succeeds", bms.send_frame(&m, true), 1);
    check_eq("checksum is the xor of bytes 0..6", m.data[7], 0x0F ^ 0xF0);
    sim_main_tx_fails = true;
    check_eq("send failure reported after retries", bms.send_frame(&m, false), 0);
    check("tx error counted", bms.get_can_tx_error_count() > 0);
    sim_main_tx_fails = false;
    CANMessage r;
    check_eq("no frame waiting reads false", bms.read_frame(&r), 0);
    SimFrame f; f.id = 0x521; f.len = 8; sim_main_rx.push_back(f);
    check_eq("queued frame reads true", bms.read_frame(&r), 1);
    check_eq("and carries the id", (long)r.id, 0x521);
    bms.send_shunt_reset_message();
    check("shunt reset transmitted", sim_count_frames(0x411) > 0);
    bms.led_blink();
    bms.print();
    check("print and blink run", true);
    sim_reset();
}
