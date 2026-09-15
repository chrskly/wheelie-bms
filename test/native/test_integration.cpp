#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery;
void setup(); void loop();

static uint16_t u16(const SimFrame& f,int o){ return (uint16_t)(f.data[o] | (f.data[o+1]<<8)); }
static int16_t  i16(const SimFrame& f,int o){ return (int16_t)u16(f,o); }

void test_integration() {
    suite("integration: cold boot, everything off");
    sim_reset();
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_gpio[CHARGE_ENABLE_PIN] = 0;
    setup();
    check_eq("contactors inhibited at power-on", battery.number_of_active_packs(), 0);
    check_eq("drive inhibited at power-on", sim_gpio[DRIVE_INHIBIT_PIN], 1);
    check_eq("charge inhibited at power-on", sim_gpio[CHARGE_INHIBIT_PIN], 1);
    loop();                                     // the Arduino loop just yields
    check("loop() runs", true);

    sim_run_ms(2500);
    check_eq("still standby", bms.get_state() == &state_standby, 1);
    check_eq("contactor hold released once modules report", battery.number_of_active_packs(), NUM_PACKS);
    check_eq("drive inhibit released", sim_gpio[DRIVE_INHIBIT_PIN], 0);
    check_eq("heater off", sim_gpio[HEATER_ENABLE_PIN], 0);

    suite("integration: transmitted frames carry the right values");
    SimFrame f;
    check_eq("0x356 present", sim_last_frame(0x356, f), 1);
    check_eq("pack voltage 355.20 V", (long)u16(f,0), 35520);
    check_eq("temperature 25.0 C", (long)i16(f,4), 250);
    check_eq("0x351 present", sim_last_frame(0x351, f), 1);
    /* Derived rather than written out: these are the cell window scaled to the
     * whole pack in 0.1 V units, and hardcoding them meant moving
     * CELL_FULL_VOLTAGE / CELL_EMPTY_VOLTAGE broke a test that was not about
     * the window at all. */
    check_eq("charge voltage limit is the full pack at CELL_FULL_VOLTAGE", (long)u16(f,0),
             (long)CELL_FULL_VOLTAGE * CELLS_PER_MODULE * MODULES_PER_PACK / 100);
    check_eq("discharge voltage limit is the full pack at CELL_EMPTY_VOLTAGE", (long)u16(f,6),
             (long)CELL_EMPTY_VOLTAGE * CELLS_PER_MODULE * MODULES_PER_PACK / 100);
    check("discharge current advertised", u16(f,4) > 0);
    check_eq("0x355 present", sim_last_frame(0x355, f), 1);
    check_eq("SoC 100", (long)u16(f,0), 100);
    check("0x352 state frame sent", sim_count_frames(0x352) > 0);
    check("0x354 main error counters sent", sim_count_frames(0x354) > 0);
    check("0x357 pack error counters sent", sim_count_frames(0x357) > 0);
    check("0x35A alarm frame sent", sim_count_frames(0x35A) > 0);
    check("0x353 liveness sent", sim_count_frames(0x353) > 0);

    suite("integration: drive and back");
    sim_gpio[IGNITION_ENABLE_PIN] = 1; sim_run_ms(500);
    check_eq("entered drive", bms.get_state() == &state_drive, 1);
    check_eq("packs still connected in drive", battery.number_of_active_packs(), NUM_PACKS);
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_run_ms(500);
    check_eq("back to standby", bms.get_state() == &state_standby, 1);

    suite("integration: boot with the ignition already on");
    sim_reset();
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    setup(); sim_run_ms(2500);
    check_eq("entered drive from the boot dispatch", bms.get_state() == &state_drive, 1);
    check_eq("packs connected, not stranded", battery.number_of_active_packs(), NUM_PACKS);
    check_eq("discharge current advertised", sim_last_frame(0x351, f) && u16(f,4) > 0, 1);

    suite("integration: boot with the charger already connected");
    sim_reset();
    sim_gpio[CHARGE_ENABLE_PIN] = 1;
    setup(); sim_run_ms(2500);
    check_eq("drive-away protection armed", sim_gpio[DRIVE_INHIBIT_PIN], 1);

    suite("integration: charge cycle");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1000);
    check_eq("entered charging", bms.get_state() == &state_charging, 1);
    sim_set_temp(-20); sim_run_ms(1000);
    check_eq("too cold -> heating", bms.get_state() == &state_batteryHeating, 1);
    check_eq("heater on", sim_gpio[HEATER_ENABLE_PIN], 1);
    sim_set_temp(25); sim_run_ms(1000);
    check_eq("warmed -> charging", bms.get_state() == &state_charging, 1);
    check_eq("heater off", sim_gpio[HEATER_ENABLE_PIN], 0);
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(1000);
    check_eq("unplugged -> standby", bms.get_state() == &state_standby, 1);

    suite("integration: heating gives up when blind");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_set_temp(-20); sim_run_ms(1000);
    check_eq("heating", bms.get_state() == &state_batteryHeating, 1);
    sim_modules_answer(false); sim_run_ms(7000);
    check_eq("heater off once the data goes stale", sim_gpio[HEATER_ENABLE_PIN], 0);
    sim_modules_answer(true);

    suite("integration: module loss and recovery");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_modules_answer(false); sim_run_ms(7000);
    check_eq("critical fault on module loss", bms.get_state() == &state_criticalFault, 1);
    check_eq("drive inhibited", sim_gpio[DRIVE_INHIBIT_PIN], 1);
    sim_modules_answer(true); sim_run_ms(2000);
    check_eq("recovers", bms.get_state() != &state_criticalFault, 1);
    check_eq("drive inhibit released", sim_gpio[DRIVE_INHIBIT_PIN], 0);

    suite("integration: shunt loss");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_shunt_answers(false); sim_run_ms(5000);
    check_eq("critical fault on shunt loss", bms.get_state() == &state_criticalFault, 1);
    sim_shunt_answers(true); sim_run_ms(2000);
    check_eq("recovers", bms.get_state() != &state_criticalFault, 1);

    suite("integration: over temperature and recovery");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_set_temp(MAXIMUM_TEMPERATURE + 5); sim_run_ms(1500);
    check_eq("entered overTempFault", bms.get_state() == &state_overTempFault, 1);
    check_eq("both inhibits asserted",
             sim_gpio[DRIVE_INHIBIT_PIN] && sim_gpio[CHARGE_INHIBIT_PIN], 1);
    sim_set_temp(25); sim_run_ms(1500);
    check_eq("left the fault", bms.get_state() != &state_overTempFault, 1);
    check_eq("R_TOO_HOT withdrawn from drive", sim_gpio[DRIVE_INHIBIT_PIN], 0);
    check_eq("R_TOO_HOT withdrawn from charge", sim_gpio[CHARGE_INHIBIT_PIN], 0);

    suite("integration: empty battery");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_set_cells(CELL_EMPTY_VOLTAGE - 50); sim_run_ms(1500);
    check_eq("entered batteryEmpty", bms.get_state() == &state_batteryEmpty, 1);
    check_eq("drive inhibited", sim_gpio[DRIVE_INHIBIT_PIN], 1);
    sim_set_cells(3700); sim_run_ms(1500);
    check_eq("recovers to standby", bms.get_state() == &state_standby, 1);

    suite("integration: full battery inhibits charge");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_set_cells(CELL_FULL_VOLTAGE + 10); sim_run_ms(1500);
    check_eq("charge inhibited when full", sim_gpio[CHARGE_INHIBIT_PIN], 1);
    sim_set_cells(3700); sim_run_ms(1500);
    check_eq("charge released when no longer full", sim_gpio[CHARGE_INHIBIT_PIN], 0);

    suite("integration: cell balancing");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_set_cells(3960); sim_set_cell_override(0, 0, -100);
    sim_run_ms(70000);
    int onPolls = 0, offPolls = 0; uint16_t target = 0;
    for (auto& tx : sim_pack_tx[0]) if ((tx.id & 0xFF0) == 0x080) {
        if (tx.data[4] == MODULE_CMD_BALANCE_ON) { onPolls++; if (!target) target = (uint16_t)(tx.data[0] | (tx.data[1] << 8)); }
        else if (tx.data[4] == MODULE_CMD_BALANCE_OFF) offPolls++;
    }
    check("balancing armed", onPolls > 0);
    check("and also idles between bursts", offPolls > 0);
    check_eq("bleed target is the lowest cell + 5 mV", (long)target, 3865);

    suite("integration: dead cell");
    sim_reset();
    setup(); sim_run_ms(2000);
    sim_set_cell_override(0, 0, -(int)(3700 - DEAD_CELL_VOLTAGE + 50));
    sim_run_ms(2000);
    check_eq("affected pack isolated", battery.number_of_active_packs(), NUM_PACKS - 1);
    sim_set_cell_override(0, 0, 0); sim_run_ms(2000);
    check_eq("released once the cell recovers", battery.number_of_active_packs(), NUM_PACKS);

    suite("integration: main CAN receive error counting");
    sim_reset();
    setup(); sim_run_ms(500);
    sim_main_status_flags = 0x03;               // hardware + driver rx overflow
    sim_run_ms(200);
    sim_main_status_flags = 0x04;               // bus off
    sim_run_ms(200);
    sim_main_status_flags = 0;
    check("rx errors counted", bms.get_can_rx_error_count() > 0);
    sim_reset();
}
