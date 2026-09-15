#include "sim/unit.h"
#include "sim/sim.h"
#include "pack.h"
#include "battery.h"

extern Battery battery;

static BatteryPackConfig cfg(int id) {
    BatteryPackConfig c;
    c.id = id; c.canChipSelectPin = CS_PINS[id];
    c.contactorInhibitPin = INHIBIT_CONTACTOR_PINS[id];
    c.contactorFeedbackPin = CONTACTOR_FEEDBACK_PINS[id];
    c.numModules = MODULES_PER_PACK; c.numCellsPerModule = CELLS_PER_MODULE;
    c.numTemperatureSensorsPerModule = TEMPS_PER_MODULE;
    return c;
}
// Feed one pack a complete sweep directly through its decoders.
static void feed(BatteryPack& p, uint16_t mv, int8_t degC, int tempsFitted = 2) {
    for (int m = 0; m < MODULES_PER_PACK; m++) {
        CANMessage s; s.id = 0x100 | m; s.len = 8;
        for (int i=0;i<8;i++) s.data[i]=0;
        p.decode_voltages(&s);
        for (int g = 0; g < 6; g++) {
            CANMessage f; f.id = 0x100 | ((0x20 + g*0x10) & 0xF0) | m; f.len = 8;
            for (int i=0;i<8;i++) f.data[i]=0;
            for (int c = 0; c < 3; c++) { f.data[c*2]=(uint8_t)(mv&0xFF); f.data[c*2+1]=(uint8_t)((mv>>8)&0x3F); }
            p.decode_voltages(&f);
        }
        CANMessage t; t.id = 0x180 | m; t.len = 8;
        for (int i=0;i<TEMPS_PER_MODULE;i++) t.data[i] = (i<tempsFitted)?(uint8_t)(degC+40):0;
        p.decode_temperatures(&t);
    }
    p.process_voltage_update();
}

/* Overwrite one module's temperature slots, so a pack can be given a genuine
 * spread between its coldest and hottest sensor rather than one flat value. */
static void set_module_temperature(BatteryPack& p, int module, int8_t degC, int tempsFitted = 2) {
    CANMessage t; t.id = 0x180 | module; t.len = 8;
    for (int i=0;i<TEMPS_PER_MODULE;i++) t.data[i] = (i<tempsFitted)?(uint8_t)(degC+40):0;
    p.decode_temperatures(&t);
}

void test_pack() {
    sim_reset();
    suite("pack: init");
    BatteryPack p;
    check_eq("contactors inhibited before init", p.contactors_are_inhibited(), 1);
    BatteryPackConfig c = cfg(0); c.numModules = MODULES_PER_PACK + 99;
    p.init(c);
    check_eq("module count clamped", p.all_modules_populated(), 0);
    c = cfg(0); c.numModules = -1; p.init(c);
    check_eq("negative module count clamped", p.all_modules_populated(), 0);
    sim_pack_begin_error = 0x0040; p.init(cfg(0)); sim_pack_begin_error = 0;
    check("CAN begin error path exercised", true);
    p.init(cfg(0)); p.set_battery(&battery);
    check_eq("inhibited by CI_STARTUP after init", p.contactors_are_inhibited(), 1);

    suite("pack: voltage");
    check_eq("no data: voltage reads 0", (long)p.get_voltage(), 0);
    check_eq("no data: lowest sentinel", (long)p.get_lowest_cell_voltage(), NO_CELL_VOLTAGE_READING);
    check_eq("no data: highest is 0", (long)p.get_highest_cell_voltage(), 0);
    check_eq("no data: cell delta 0", (long)p.get_cell_delta(), 0);
    feed(p, 3700, 25);
    check_eq("all modules populated", p.all_modules_populated(), 1);
    check_eq("pack voltage summed", (long)p.get_voltage(), 3700L*CELLS_PER_MODULE*MODULES_PER_PACK);
    check_eq("CI_STARTUP withdrawn once populated", p.contactors_are_inhibited(), 0);
    check_eq("cell delta with uniform cells", (long)p.get_cell_delta(), 0);

    suite("pack: cell delta is millivolt-wide");
    p.init(cfg(0)); p.set_battery(&battery); feed(p, 3700, 25);
    CANMessage hi; hi.id = 0x120; hi.len = 8;
    for (int i=0;i<8;i++) hi.data[i]=0;
    uint16_t big = 4000;
    // frame 0x120 carries cells 0,1,2 -- set all three or the others read 0 mV
    for (int c = 0; c < 3; c++) { hi.data[c*2]=(uint8_t)(big&0xFF); hi.data[c*2+1]=(uint8_t)((big>>8)&0x3F); }
    p.decode_voltages(&hi); p.process_voltage_update();
    check_eq("300 mV delta survives (was truncated to 44)", (long)p.get_cell_delta(), 300);

    suite("pack: out-of-range module ids are rejected");
    uint16_t before = p.get_can_rx_error_count();
    CANMessage bad; bad.id = 0x12F; bad.len = 8;       // module 15, only 6 exist
    for (int i=0;i<8;i++) bad.data[i]=0;
    p.decode_voltages(&bad);
    CANMessage badT; badT.id = 0x18F; badT.len = 8;
    for (int i=0;i<8;i++) badT.data[i]=0;
    p.decode_temperatures(&badT);
    check("rx error counted for both", p.get_can_rx_error_count() >= before + 2);

    suite("pack: an unfitted temperature sensor is not -40C");
    p.init(cfg(0)); p.set_battery(&battery); feed(p, 3700, 25, 2);
    check_eq("lowest ignores the unfitted slots", p.get_lowest_temperature(), 25);
    check_eq("highest ignores the unfitted slots", p.get_highest_temperature(), 25);

    suite("pack: charge current by temperature");
    check_eq("below the table: no charge", (long)p.charge_current_for_temperature(-11), 0);
    check_eq("first table entry", (long)p.charge_current_for_temperature(-10), 3);
    check_eq("mid table", (long)p.charge_current_for_temperature(0), 7);
    check_eq("last table entry", (long)p.charge_current_for_temperature(39), 10);
    check_eq("above the table: no charge", (long)p.charge_current_for_temperature(40), 0);
    check_eq("no-data sentinel low", (long)p.charge_current_for_temperature(-126), 0);
    check_eq("no-data sentinel high", (long)p.charge_current_for_temperature(126), 0);
    check("warm pack allows charge", p.get_max_charge_current_by_temperature() > 0);
    feed(p, CELL_FULL_VOLTAGE, 25);
    check_eq("full pack: no charge", (long)p.get_max_charge_current_by_temperature(), 0);
    p.init(cfg(0)); p.set_battery(&battery); feed(p, 3700, MAXIMUM_TEMPERATURE + 5);
    check_eq("over-max temperature: no charge", (long)p.get_max_charge_current_by_temperature(), 0);
    p.init(cfg(0)); p.set_battery(&battery); feed(p, 3700, CHARGE_TEMPERATURE_MINIMUM - 5);
    check_eq("too cold: no charge", (long)p.get_max_charge_current_by_temperature(), 0);

    /* The table used to plateau at a flat 125 A, which on a 26 Ah pack is 4.8C
     * -- above what NMC/graphite accepts, and above the discharge limit, which
     * is backwards. It is now derived from CHARGE_C_RATE_PERCENT so it cannot
     * drift from the cells again. */
    suite("pack: charge current stays inside the C-rate envelope");
    check_eq("the plateau is CHARGE_C_RATE_PERCENT of 1C",
             (long)p.charge_current_for_temperature(25),
             (long)( ( PACK_CAPACITY_AH * CHARGE_C_RATE_PERCENT ) / 100 ));
    check("charging never exceeds the discharge rating",
          CHARGE_CURRENT_MAX_PER_PACK_A <= DISCHARGE_CURRENT_MAX_PER_PACK_A);
    for (int t = -10; t <= 39; t++) {
        if (p.charge_current_for_temperature((int8_t)t) > CHARGE_CURRENT_MAX_PER_PACK_A) {
            check("no table entry exceeds the plateau", false);
            break;
        }
    }
    check("no table entry exceeds the plateau", true);
    /* The old table stepped 6 A -> 13 A across freezing, more than doubling the
     * current for one degree at the temperature where plating risk is highest. */
    check("no cliff across 0C",
          p.charge_current_for_temperature(0) - p.charge_current_for_temperature(-1) <= 2);
    check_eq("still ~0.1C at the cold limit, per the published guidance",
             (long)p.charge_current_for_temperature(-10),
             (long)( ( PACK_CAPACITY_AH * 12 + 50 ) / 100 ));

    /* The cold end of the table is plating-limited and the hot end is
     * thermally limited, so the limit has to be the lower of the two ends. This
     * used to key the lookup on the hottest sensor alone, which let a pack
     * spanning -8C to +14C clear the too-cold guard (it tests the coldest) and
     * then take the +14C entry, 111 A, into a module entitled to 3 A. */
    suite("pack: charge current keys on both ends of the temperature spread");
    check_eq("the cold end wins", (long)p.charge_current_for_temperature_range(-8, 14), 3);
    check_eq("the hot end wins", (long)p.charge_current_for_temperature_range(20, 38), 10);
    check_eq("argument order does not matter",
             (long)p.charge_current_for_temperature_range(14, -8), 3);
    check_eq("a flat pack is just the table",
             (long)p.charge_current_for_temperature_range(25, 25),
             (long)CHARGE_CURRENT_MAX_PER_PACK_A);
    check_eq("either end off the table means no charge",
             (long)p.charge_current_for_temperature_range(-8, 45), 0);
    check_eq("no-data sentinels (lowest 126, highest -126) mean no charge",
             (long)p.charge_current_for_temperature_range(126, -126), 0);

    p.init(cfg(0)); p.set_battery(&battery);
    feed(p, 3700, 14); set_module_temperature(p, 0, -8);
    check_eq("the spread is set up as intended", p.get_lowest_temperature(), -8);
    check_eq("...at both ends", p.get_highest_temperature(), 14);
    check_eq("the coldest module governs, not the hottest",
             (long)p.get_max_charge_current_by_temperature(),
             (long)p.charge_current_for_temperature(-8));

    p.init(cfg(0)); p.set_battery(&battery);
    feed(p, 3700, 20); set_module_temperature(p, 0, 38);
    check_eq("and the hottest still governs at the top of the range",
             (long)p.get_max_charge_current_by_temperature(),
             (long)p.charge_current_for_temperature(38));

    suite("pack: contactor reason mask");
    p.init(cfg(0)); p.set_battery(&battery);
    p.disable_inhibit_contactor_close(CI_STARTUP);
    check_eq("released once startup withdraws", p.contactors_are_inhibited(), 0);
    p.enable_inhibit_contactor_close(CI_DEAD_CELL);
    p.enable_inhibit_contactor_close(CI_IMBALANCE);
    check_eq("held by two reasons", p.contactors_are_inhibited(), 1);
    p.disable_inhibit_contactor_close(CI_IMBALANCE);
    check_eq("still held by the dead-cell reason", p.contactors_are_inhibited(), 1);
    p.disable_inhibit_contactor_close(CI_DEAD_CELL);
    check_eq("released when no reason remains", p.contactors_are_inhibited(), 0);

    suite("pack: weld detection needs the contactor commanded open");
    sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 1;          // feedback says closed
    check_eq("not welded while permitted to close", p.contactors_are_welded(), 0);
    p.enable_inhibit_contactor_close(CI_IMBALANCE);
    check_eq("not welded before the settle time", p.contactors_are_welded(), 0);
    sim_now_us += (uint64_t)(WELD_CHECK_SETTLE_MS + 1) * 1000;
    check_eq("welded once settled and still closed", p.contactors_are_welded(), 1);
    sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 0;
    check_eq("not welded when feedback opens", p.contactors_are_welded(), 0);

    suite("pack: polling and balancing");
    p.init(cfg(0)); p.set_battery(&battery);
    sim_pack_tx[0].clear();
    for (int i = 0; i < MODULES_PER_PACK * 4; i++) p.request_data();
    check("poll frames transmitted", sim_pack_tx[0].size() > 0);
    bool sawEveryModule = true;
    for (int m = 0; m < MODULES_PER_PACK; m++) {
        bool seen = false;
        for (auto& f : sim_pack_tx[0]) if (f.id == (uint32_t)(0x080 | m)) seen = true;
        if (!seen) sawEveryModule = false;
    }
    check("every module polled across the sweeps", sawEveryModule);
    check_eq("balancing idle by default", p.balancing_is_active(), 0);
    check_eq("readings trusted when idle", p.voltage_readings_are_suspect(), 0);

    suite("pack: send failure path");
    sim_pack_tx_fails = true;
    p.request_data();
    sim_pack_tx_fails = false;
    check("tx error counted", p.get_can_tx_error_count() > 0);

    suite("pack: liveness and print");
    check_eq("module liveness bounds low", p.get_module_liveness(-1), 0);
    check_eq("module liveness bounds high", p.get_module_liveness(MODULES_PER_PACK), 0);
    p.print();
    check("print runs", true);
    p.poll_can();
    check("poll_can runs", true);
    sim_reset();
}
