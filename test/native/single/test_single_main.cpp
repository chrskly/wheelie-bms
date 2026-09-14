/*
 * The single-pack configuration, built with -DNUM_PACKS_CFG=1.
 *
 * The firmware carries explicit single-pack handling that the two-pack build
 * cannot reach: with nothing to isolate, a dead cell has to inhibit drive and
 * charge outright rather than opening one pack's contactors, and every
 * pack-to-pack comparison has to short-circuit. None of it had ever been
 * compiled in a configuration where it can run.
 */
#include "../sim/unit.h"
#include "../sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery; extern Io io;
void setup();

static void boot() {
    sim_reset();
    setup();
    sim_run_ms(3000);
}

int main() {
    static_assert(NUM_PACKS == 1, "this binary must be built with -DNUM_PACKS_CFG=1");

    suite("single-pack: the battery comes up and runs");
    boot();
    check_eq("only one pack", battery.has_multiple_packs(), 0);
    check_eq("battery is alive", battery.is_alive(), 1);
    check("battery voltage is reported", battery.get_voltage() > 0);
    check("state is standby", bms.get_state() == &state_standby);

    suite("single-pack: pack-to-pack comparisons short-circuit");
    check_eq("high pack is pack 0", battery.get_index_of_high_pack(), 0);
    check_eq("low pack is pack 0", battery.get_index_of_low_pack(), 0);
    check_eq("there is no delta", (long)battery.voltage_delta_between_packs(), 0);
    check_eq("so the packs are never imbalanced", battery.packs_are_imbalanced(), 0);

    suite("single-pack: contactor re-evaluation is a no-op");
    battery.enable_inhibit_contactor_close();
    battery.reevaluate_contactor_inhibition_for_drive();
    check_eq("drive re-evaluation leaves the hold alone",
             battery.one_or_more_contactors_inhibited(), 1);
    battery.reevaluate_contactor_inhibition_for_charge();
    check_eq("charge re-evaluation leaves the hold alone",
             battery.one_or_more_contactors_inhibited(), 1);
    battery.disable_inhibit_contactor_close();
    check_eq("and the hold can still be withdrawn",
             battery.one_or_more_contactors_inhibited(), 0);

    suite("single-pack: a dead cell inhibits outright");
    boot();
    check_eq("drive permitted to start", bms.drive_is_inhibited(), 0);
    check_eq("charge permitted to start", bms.charge_is_inhibited(), 0);
    sim_set_cell_override(0, 0, -(int)(3700 - DEAD_CELL_VOLTAGE + 50));
    sim_run_ms(3000);
    check_eq("the dead cell is seen", battery.has_dead_cell(), 1);
    /* With two packs this would open the affected pack's contactors and carry
     * on. With one pack there is nothing to isolate, so the only safe response
     * is to stop drive and charge. */
    check_eq("drive is inhibited", bms.drive_is_inhibited(), 1);
    check_eq("charge is inhibited", bms.charge_is_inhibited(), 1);

    suite("single-pack: and the inhibit is withdrawn when the cell recovers");
    sim_set_cell_override(0, 0, 0);
    sim_run_ms(3000);
    check_eq("the dead cell is gone", battery.has_dead_cell(), 0);
    check_eq("drive is permitted again", bms.drive_is_inhibited(), 0);
    check_eq("charge is permitted again", bms.charge_is_inhibited(), 0);

    suite("single-pack: the welding byte only covers the pack that exists");
    boot();
    check_eq("nothing welded at rest", bms.get_welding_byte(), 0x00);
    battery.enable_inhibit_contactor_close();
    sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 1;
    sim_run_ms(WELD_CHECK_SETTLE_MS + 1000);
    check_eq("pack 0 welded shows in bit 2", bms.get_welding_byte() & 0x04, 0x04);
    /* Bit 3 belongs to a pack that does not exist here. It used to be filled
     * from packContactorsWelded[1], one past the end of the array. */
    check_eq("bit 3 is clear, not read off the end of the array",
             (long)(bms.get_welding_byte() & 0x08), 0);
    sim_gpio[CONTACTOR_FEEDBACK_PINS[0]] = 0;
    sim_run_ms(1000);
    check_eq("clears when the feedback opens", bms.get_welding_byte(), 0x00);
    battery.disable_inhibit_contactor_close();

    suite("single-pack: normal operation still works");
    boot();
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    sim_run_ms(1000);
    check("ignition on -> drive", bms.get_state() == &state_drive);
    sim_gpio[IGNITION_ENABLE_PIN] = 0;
    sim_run_ms(1000);
    check("ignition off -> standby", bms.get_state() == &state_standby);
    sim_gpio[CHARGE_ENABLE_PIN] = 1;
    sim_run_ms(1000);
    check("charge on -> charging", bms.get_state() == &state_charging);
    sim_gpio[CHARGE_ENABLE_PIN] = 0;
    sim_run_ms(1000);
    check("charge off -> standby", bms.get_state() == &state_standby);

    return unit_summary();
}
