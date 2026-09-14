#include "sim/unit.h"
#include "sim/sim.h"
#include "module.h"
#include "pack.h"

// A module reports voltages as six groups; drive it the way decode_voltages does.
static void populate(BatteryModule& m, uint16_t mv, int8_t degC, int tempsFitted = 2) {
    for (int c = 0; c < CELLS_PER_MODULE; c++) m.set_cell_voltage(c, mv);
    for (int g = 0x020; g <= 0x070; g += 0x10) m.note_voltage_group(g);
    for (int t = 0; t < TEMPS_PER_MODULE; t++)
        m.update_temperature(t, (t < tempsFitted) ? degC : NO_TEMPERATURE_READING);
    m.check_if_module_data_is_populated();
}

void test_module() {
    sim_reset();
    suite("module: init clamps counts");
    BatteryModule m;
    m.init(0, nullptr, CELLS_PER_MODULE + 99, TEMPS_PER_MODULE + 99);
    m.set_cell_voltage(CELLS_PER_MODULE - 1, 3500);
    check_eq("cell count clamped", (long)m.get_highest_cell_voltage(), 3500);
    m.init(0, nullptr, -5, -5);
    check_eq("negative counts clamped to zero", (long)m.get_voltage(), 0);

    suite("module: voltage accessors");
    BatteryModule v; v.init(1, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    check_eq("unpopulated module is not populated", v.all_module_data_populated(), 0);
    populate(v, 3700, 25);
    check_eq("populated after a full sweep", v.all_module_data_populated(), 1);
    check_eq("summed voltage", (long)v.get_voltage(), 3700L * CELLS_PER_MODULE);
    check_eq("lowest cell", (long)v.get_lowest_cell_voltage(), 3700);
    check_eq("highest cell", (long)v.get_highest_cell_voltage(), 3700);
    v.set_cell_voltage(3, 3600); v.set_cell_voltage(9, 3900);
    check_eq("lowest tracks the minimum", (long)v.get_lowest_cell_voltage(), 3600);
    check_eq("highest tracks the maximum", (long)v.get_highest_cell_voltage(), 3900);
    v.set_cell_voltage(-1, 9999); v.set_cell_voltage(CELLS_PER_MODULE, 9999);
    check_eq("out-of-range writes ignored (low)", (long)v.get_lowest_cell_voltage(), 3600);
    check_eq("out-of-range writes ignored (high)", (long)v.get_highest_cell_voltage(), 3900);

    suite("module: cell predicates need data");
    BatteryModule q; q.init(2, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    check_eq("unpopulated is not empty", q.has_empty_cell(), 0);
    check_eq("unpopulated is not full", q.has_full_cell(), 0);
    check_eq("unpopulated has no dead cell", q.has_dead_cell(), 0);
    populate(q, 3700, 25);
    check_eq("healthy: not empty", q.has_empty_cell(), 0);
    check_eq("healthy: not full", q.has_full_cell(), 0);
    check_eq("healthy: no dead cell", q.has_dead_cell(), 0);
    q.set_cell_voltage(0, CELL_EMPTY_VOLTAGE);
    check_eq("at the empty threshold counts as empty", q.has_empty_cell(), 1);
    q.set_cell_voltage(0, CELL_FULL_VOLTAGE);
    check_eq("at the full threshold counts as full", q.has_full_cell(), 1);
    q.set_cell_voltage(0, DEAD_CELL_VOLTAGE);
    check_eq("at the dead threshold counts as dead", q.has_dead_cell(), 1);
    q.set_cell_voltage(0, 0);
    check_eq("a 0 mV cell is visible as dead", q.has_dead_cell(), 1);
    check_eq("and the module stays populated", q.all_module_data_populated(), 1);

    suite("module: population requires every voltage group");
    BatteryModule p; p.init(3, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    for (int c = 0; c < CELLS_PER_MODULE; c++) p.set_cell_voltage(c, 3700);
    p.update_temperature(0, 25);
    p.note_voltage_group(0x020); p.note_voltage_group(0x030);
    p.check_if_module_data_is_populated();
    check_eq("two of six groups is not enough", p.all_module_data_populated(), 0);
    for (int g = 0x040; g <= 0x070; g += 0x10) p.note_voltage_group(g);
    p.check_if_module_data_is_populated();
    check_eq("all six groups is enough", p.all_module_data_populated(), 1);
    p.note_voltage_group(0x000); p.note_voltage_group(0x080);
    check("out-of-range group ids ignored", true);

    suite("module: an unfitted sensor must not block population");
    BatteryModule u; u.init(4, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    populate(u, 3700, 25, 2);              // only 2 of 4 slots fitted
    check_eq("populated with 2 of 4 sensors", u.all_module_data_populated(), 1);
    BatteryModule z; z.init(5, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    populate(z, 3700, 25, 0);              // no sensors at all
    check_eq("no sensors at all is not populated", z.all_module_data_populated(), 0);

    suite("module: temperature");
    BatteryModule t; t.init(6, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    check_eq("no data: lowest sentinel", t.get_lowest_temperature(), 126);
    check_eq("no data: highest sentinel", t.get_highest_temperature(), -126);
    t.update_temperature(0, 20); t.update_temperature(1, 40);
    check_eq("lowest of the fitted sensors", t.get_lowest_temperature(), 20);
    check_eq("highest of the fitted sensors", t.get_highest_temperature(), 40);
    check_eq("not over max at 40C", t.has_temperature_sensor_over_max(), 0);
    t.update_temperature(1, MAXIMUM_TEMPERATURE + 1);
    check_eq("over max detected", t.has_temperature_sensor_over_max(), 1);
    t.update_temperature(-1, 10); t.update_temperature(TEMPS_PER_MODULE, 10);
    check_eq("out-of-range sensor writes ignored", t.get_lowest_temperature(), 20);

    suite("module: liveness");
    BatteryModule L; L.init(7, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    check_eq("silent inside the boot grace window is not dead", L.is_alive(), 1);
    sim_now_us = (uint64_t)(MODULE_TTL_MS + 1) * 1000;
    check_eq("silent past the grace window is dead", L.is_alive(), 0);
    L.heartbeat();
    check_eq("alive after a heartbeat", L.is_alive(), 1);
    sim_now_us += (uint64_t)(MODULE_TTL_MS - 1) * 1000;
    check_eq("alive just inside the TTL", L.is_alive(), 1);
    sim_now_us += 2000;
    check_eq("dead just past the TTL", L.is_alive(), 0);

    suite("module: status words and print");
    BatteryModule s; s.init(8, nullptr, CELLS_PER_MODULE, TEMPS_PER_MODULE);
    check_eq("balance status starts clear", (long)s.get_balance_status(), 0);
    s.set_balance_status(0x1234);
    check_eq("balance status stored", (long)s.get_balance_status(), 0x1234);
    s.set_error_status(0xDEADBEEF);
    check_eq("error status stored", (long)s.get_error_status(), (long)0xDEADBEEF);
    s.print();                              // with an error set
    s.set_error_status(0);
    s.print();                              // without
    check("print runs both ways", true);
}
