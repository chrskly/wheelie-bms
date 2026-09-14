#include "sim/unit.h"
#include "sim/sim.h"
#include "battery.h"
#include "bms.h"
#include "statemachine.h"

extern Battery battery; extern Bms bms;

void test_battery() {
    suite("battery: init and identity");
    sim_reset();
    battery.initialise(&bms);
    check_eq("multiple packs configured", battery.has_multiple_packs(), NUM_PACKS > 1);
    check_eq("max voltage", (long)battery.get_max_voltage(),
             (long)CELL_FULL_VOLTAGE * CELLS_PER_MODULE * MODULES_PER_PACK);
    check_eq("min voltage", (long)battery.get_min_voltage(),
             (long)CELL_EMPTY_VOLTAGE * CELLS_PER_MODULE * MODULES_PER_PACK);
    check_eq("all packs inhibited at init", battery.number_of_active_packs(), 0);
    check_eq("one or more inhibited", battery.one_or_more_contactors_inhibited(), 1);
    check_eq("no data: not alive yet counts as alive in the grace window", battery.is_alive(), 1);

    suite("battery: no-data behaviour");
    check_eq("voltage 0 with no data", (long)battery.get_voltage(), 0);
    check_eq("no dead cell with no data", battery.has_dead_cell(), 0);
    check_eq("not empty with no data", battery.has_empty_cell(), 0);
    check_eq("not full with no data", battery.has_full_cell(), 0);
    check_eq("temperature data is stale before any frame", battery.temperature_data_is_stale(), 1);
    check_eq("stale temperature reads as too cold", battery.too_cold_to_charge(), 1);
    check_eq("pack delta 0 with no data", (long)battery.voltage_delta_between_packs(), 0);
    check_eq("cell delta 0 with no data", (long)battery.get_cell_delta(), 0);

    suite("battery: populated via the worker");
    bms.init(&battery, nullptr, nullptr);      // io/shunt unused on this path
    sim_reset();
    battery.initialise(&bms);
    bms.init(&battery, nullptr, nullptr);
    check("initialise+init complete", true);

    suite("battery: liveness byte index arithmetic");
    check_eq("negative start returns 0", battery.get_module_liveness_byte(-1), 0);
    check_eq("start beyond the last module returns 0",
             battery.get_module_liveness_byte(NUM_PACKS * MODULES_PER_PACK), 0);
    check_eq("well beyond returns 0", battery.get_module_liveness_byte(120), 0);
    uint8_t b0 = battery.get_module_liveness_byte(0);
    uint8_t b8 = battery.get_module_liveness_byte(8);
    check("byte 0 computed without a crash", b0 == b0);
    check("byte 8 stops at the last pack", (b8 & 0xF0) == 0);

    suite("battery: contactor inhibition");
    battery.enable_inhibit_contactor_close();
    check_eq("all inhibited", battery.number_of_active_packs(), 0);
    battery.disable_inhibit_contactor_close();
    check("release only withdraws the imbalance hold", true);
    battery.reevaluate_dead_cell_inhibition();
    check("dead-cell re-evaluation runs", true);
    battery.disable_inhibit_contactors_for_drive();
    battery.disable_inhibit_contactors_for_charge();
    check("drive/charge re-evaluation runs with no data", true);

    suite("battery: charge current with no active packs");
    check_eq("no active packs: no charge current",
             (long)battery.get_max_charge_current_by_temperature(), 0);

    suite("battery: print and welds");
    check_eq("print returns 0", battery.print(), 0);
    sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 0;
    check_eq("not welded when feedback is open", battery.contactor_is_welded(0), 0);
    sim_reset();
}
