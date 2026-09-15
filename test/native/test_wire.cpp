/*
 * Properties of what the BMS puts on the bus and publishes to the web, checked
 * over a quiet, healthy run.
 *
 * Cadence matters on its own: a periodic frame that silently stops being sent
 * is indistinguishable, at the receiver, from a BMS that has died -- and
 * nothing else in the suite would notice, because every other test asserts on
 * the content of a frame it already knows arrived.
 */
#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "webstatus.h"
#include "statemachine.h"
#include <cstring>

extern Bms bms; extern Battery battery; extern Io io;
void setup();

struct Expect { uint32_t id; long periodMs; const char* what; };

void test_wire() {
    suite("wire: every periodic frame keeps to its cadence");
    sim_reset();
    setup();
    sim_run_ms(2000);
    sim_main_tx.clear();

    const long runMs = 20000;
    sim_run_ms(runMs);

    const Expect expected[] = {
        { 0x351, 1000, "charge/discharge limits" },
        { 0x352, 1000, "BMS state" },
        { 0x353, 5000, "module liveness" },
        { 0x354, 1000, "main CAN error counters" },
        { 0x355, 1000, "state of charge" },
        { 0x356, 1000, "voltage/current/temperature" },
        { 0x357, 1000, "pack CAN error counters" },
        { 0x35A, 1000, "alarms" },
    };
    for (const auto& e : expected) {
        const int seen = sim_count_frames(e.id);
        const int want = (int)(runMs / e.periodMs);
        /* Allow one frame either side for where the window falls. */
        const bool onCadence = seen >= want - 1 && seen <= want + 1;
        if (!onCadence)
            printf("   0x%03X (%s): saw %d frames in %ld ms, expected about %d\n",
                   e.id, e.what, seen, runMs, want);
        check("frame is sent at its expected rate", onCadence);
    }

    suite("wire: main-bus checksummed frames carry a correct checksum");
    {
        /* Only the main bus uses the simple XOR that send_frame() computes.
         * The pack module polls carry a CRC8 with a per-module final XOR
         * (BatteryPack::getcheck), which is verified by comparison against the
         * BMWPhevBMS reference rather than re-derived here -- a test that
         * recomputed it with the same algorithm would prove nothing. */
        int checked = 0, bad = 0;
        SimFrame f;
        if (sim_last_frame(0x353, f)) {
            uint8_t x = 0;
            for (int i = 0; i < 7; i++) x ^= f.data[i];
            checked++;
            if (x != f.data[7]) bad++;
        }
        check("a checksummed frame was seen", checked > 0);
        check_eq("its checksum is correct", bad, 0);
    }

    suite("wire: the temperature thresholds agree with their tables");
    {
        /* Two DIFFERENT thresholds, deliberately: chargeCurrentMax[] stops
         * charging above 39 C as a policy, while MAXIMUM_TEMPERATURE (50 C) is
         * the fault that stops everything. What must agree is the fault
         * boundary itself -- too_hot() latches at >= MAXIMUM_TEMPERATURE, and
         * has_temperature_sensor_over_max(), the safety check inside the
         * charge-current path, used to use a strict > and so was one degree
         * more permissive than the fault it backs up. */
        sim_reset();
        sim_set_shunt_amp_seconds(-(int32_t)(BATTERY_CAPACITY_AS / 2));
        sim_set_temp(30);
        setup();
        sim_run_ms(3000);
        check_eq("30 C is not too hot", battery.too_hot(), 0);
        check("and a charge current is offered",
              battery.get_max_charge_current_by_temperature() > 0);

        sim_set_temp(39);
        sim_run_ms(3000);
        check("39 C is the top of the charging band",
              battery.get_max_charge_current_by_temperature() > 0);

        sim_set_temp(40);
        sim_run_ms(3000);
        check_eq("40 C is still not a fault", battery.too_hot(), 0);
        check_eq("but the table offers no charge current",
                 battery.get_max_charge_current_by_temperature(), 0);

        sim_set_temp(MAXIMUM_TEMPERATURE - 1);
        sim_run_ms(3000);
        check_eq("one below the limit is not too hot", battery.too_hot(), 0);

        sim_set_temp(MAXIMUM_TEMPERATURE);
        sim_run_ms(3000);
        check_eq("at the limit, too hot", battery.too_hot(), 1);
        check_eq("and no charge current", battery.get_max_charge_current_by_temperature(), 0);
        sim_set_temp(25);
        sim_run_ms(3000);
    }

    suite("wire: alarms distinguish 'no alarm' from 'not evaluated'");
    {
        sim_reset();
        sim_set_shunt_amp_seconds(-(int32_t)(BATTERY_CAPACITY_AS / 2));
        setup();
        sim_run_ms(3000);
        SimFrame a;
        sim_main_tx.clear();
        sim_run_ms(1200);
        check_eq("an alarm frame was sent", sim_last_frame(0x35A, a), 1);

        /* Healthy battery: every field this firmware evaluates must read 10
         * (explicitly no alarm), never 00, which means "not evaluated". */
        check_eq("general alarm reads explicitly-clear", a.data[0] & 0x03, 0x02);
        check_eq("overvolt reads explicitly-clear",      a.data[0] & 0x0C, 0x08);
        check_eq("undervolt reads explicitly-clear",     a.data[0] & 0x30, 0x20);
        check_eq("high temp reads explicitly-clear",     a.data[0] & 0xC0, 0x80);
        check_eq("cell delta alarm reads explicitly-clear", a.data[3] & 0x03, 0x02);
        check_eq("cell delta warning reads explicitly-clear", a.data[7] & 0x03, 0x02);

        /* Fields this firmware genuinely does not compute stay 00. */
        check_eq("high current is reported as not evaluated", a.data[1] & 0xC0, 0x00);
        check_eq("short circuit is reported as not evaluated", a.data[2] & 0x30, 0x00);

        // An actual alarm reads 01.
        sim_set_cells(CELL_EMPTY_VOLTAGE - 100);
        sim_run_ms(3000);
        sim_main_tx.clear();
        sim_run_ms(1200);
        check_eq("an alarm frame was sent", sim_last_frame(0x35A, a), 1);
        check_eq("undervolt now reads active", a.data[0] & 0x30, 0x10);
        check_eq("undervolt warning reads active", a.data[4] & 0x30, 0x10);
        sim_set_cells(3700);
        sim_run_ms(3000);
    }

    suite("wire: the firmware reports its version");
    {
        sim_reset();
        setup();
        sim_main_tx.clear();
        sim_run_ms(6000);                       // 0x35F is a 5 s frame
        SimFrame f;
        check_eq("a battery-info frame was sent", sim_last_frame(0x35F, f), 1);
        check_eq("it carries the model id", (long)(f.data[0] | (f.data[1] << 8)),
                 (long)BATTERY_MODEL_ID);
        check_eq("and the firmware version", (long)(f.data[2] | (f.data[3] << 8)),
                 (long)VERSION_U16);
        check_eq("and the nameplate capacity in Ah",
                 (long)(f.data[4] | (f.data[5] << 8)), (long)(BATTERY_CAPACITY_AS / 3600));

        WebSnapshot s;
        static char buf[WEB_JSON_BUFFER_BYTES];
        sim_run_ms(WEB_SNAPSHOT_INTERVAL_MS + 1000);
        check_eq("a snapshot is available", webstatus_read(s), 1);
        const size_t n = webstatus_render_json(s, buf, sizeof buf);
        check("the snapshot renders", n > 0);
        check("and names the firmware version", strstr(buf, "\"version\":\"" VERSION_STRING "\"") != nullptr);
    }

    suite("wire: the shunt cross-check stays quiet on consistent readings");
    {
        sim_reset();
        sim_set_shunt_amp_seconds(-(int32_t)(BATTERY_CAPACITY_AS / 2));
        sim_set_shunt_amps(-40000);              // 40 A discharge, watts follow
        setup();
        sim_run_ms(SHUNT_IMPLAUSIBLE_MS + 5000);
        check_eq("no implausibility reported while discharging",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 0);

        sim_set_shunt_amps(40000);               // 40 A charge
        sim_run_ms(SHUNT_IMPLAUSIBLE_MS + 5000);
        check_eq("nor while charging the right way round",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 0);
    }

    suite("wire: a backwards shunt is caught while charging");
    {
        /* The one case where the BMS independently knows which way current
         * should be flowing. A shunt fitted or decoded backwards satisfies
         * every internal consistency check and shows up only here. */
        sim_reset();
        sim_set_shunt_amp_seconds(-(int32_t)(BATTERY_CAPACITY_AS / 2));
        setup();
        sim_run_ms(2000);
        sim_gpio[CHARGE_ENABLE_PIN] = 1;
        sim_run_ms(2000);
        check("the BMS is charging", bms.get_state() == &state_charging);

        sim_set_shunt_amps(-40000);              // current flowing OUT while charging
        sim_run_ms(2000);
        check_eq("not reported immediately -- it must persist",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 0);
        sim_run_ms(SHUNT_IMPLAUSIBLE_MS + 3000);
        check_eq("but a sustained contradiction is reported",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 1);

        sim_set_shunt_amps(40000);               // put it the right way round
        sim_run_ms(3000);
        check_eq("and clears once the readings agree again",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 0);
        sim_gpio[CHARGE_ENABLE_PIN] = 0;
        sim_run_ms(2000);
    }

    suite("wire: power disagreeing with current x voltage is caught");
    {
        sim_reset();
        sim_set_shunt_amp_seconds(-(int32_t)(BATTERY_CAPACITY_AS / 2));
        setup();
        sim_run_ms(2000);
        sim_set_shunt_amps(-40000);              // 40 A out of a ~355 V pack
        sim_set_shunt_watts(9999);               // nothing like -14 kW
        sim_run_ms(SHUNT_IMPLAUSIBLE_MS + 5000);
        check_eq("an impossible power reading is reported",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 1);

        /* A dead shunt is a different fault and must not masquerade as this one. */
        sim_shunt_answers(false);
        sim_run_ms(SHUNT_TTL_MS + 3000);
        check_eq("a dead shunt clears it rather than reporting both",
                 bms.has_internal_error(IE_SHUNT_IMPLAUSIBLE), 0);
        sim_shunt_answers(true);
        sim_run_ms(2000);
    }

    suite("wire: pack CAN bus errors are detected and counted");
    {
        sim_reset();
        setup();
        sim_run_ms(2000);
        const uint16_t before = battery.get_can_rx_error_count_for_pack(0);
        /* RX0OVR: the MCP2515's receive buffer overflowed and dropped frames.
         * Nothing used to read this register at all. */
        sim_set_pack_eflg(0, 0x40);
        sim_run_ms(500);
        check("a receive overflow is counted",
              battery.get_can_rx_error_count_for_pack(0) > before);

        /* Latched flags must count once, not once per health check. */
        const uint16_t after = battery.get_can_rx_error_count_for_pack(0);
        sim_run_ms(3000);
        check_eq("a latched flag is counted once, not every cycle",
                 (long)battery.get_can_rx_error_count_for_pack(0), (long)after);

        // Clearing and re-raising counts again.
        sim_set_pack_eflg(0, 0x00);
        sim_run_ms(500);
        sim_set_pack_eflg(0, 0x80);            // RX1OVR
        sim_run_ms(500);
        check("a fresh overflow is counted again",
              battery.get_can_rx_error_count_for_pack(0) > after);

        /* The driver's own receive buffer filling means module replies were
         * dropped in software between drains -- the failure this pack bus is
         * actually prone to, and the one EFLG cannot report. */
        const uint16_t beforePeak = battery.get_can_rx_error_count_for_pack(0);
        sim_set_pack_rx_peak(0, 31);             // high, but not at the limit
        sim_run_ms(500);
        check_eq("a high-water mark below the limit is not an error",
                 (long)battery.get_can_rx_error_count_for_pack(0), (long)beforePeak);
        sim_set_pack_rx_peak(0, 32);             // at the buffer size
        sim_run_ms(500);
        check("reaching the buffer size is counted",
              battery.get_can_rx_error_count_for_pack(0) > beforePeak);
        const uint16_t afterPeak = battery.get_can_rx_error_count_for_pack(0);
        sim_run_ms(3000);
        check_eq("and a high-water mark that has not moved is not recounted",
                 (long)battery.get_can_rx_error_count_for_pack(0), (long)afterPeak);
        sim_set_pack_rx_peak(0, 0);

        // Error-passive is reported but is not a receive error either.
        sim_set_pack_eflg(0, 0x00); sim_run_ms(500);
        const uint16_t beforePassive = battery.get_can_rx_error_count_for_pack(0);
        sim_set_pack_eflg(0, 0x18);            // TXEP | RXEP
        sim_run_ms(500);
        check_eq("error-passive is not counted as a receive error",
                 (long)battery.get_can_rx_error_count_for_pack(0), (long)beforePassive);

        // Bus-off is reported but is not a receive error.
        const uint16_t beforeBusOff = battery.get_can_rx_error_count_for_pack(1);
        sim_set_pack_eflg(1, 0x20);            // TXBO
        sim_run_ms(500);
        check_eq("bus-off is not counted as a receive error",
                 (long)battery.get_can_rx_error_count_for_pack(1), (long)beforeBusOff);
        sim_set_pack_eflg(0, 0); sim_set_pack_eflg(1, 0);
    }

    suite("wire: no temperature data is reported as such, not as a reading");
    {
        sim_reset();
        sim_modules_answer(false);              // nothing ever reports
        setup();
        sim_run_ms(3000);
        check_eq("no temperature reading is claimed", battery.have_temperature_reading(), 0);
        /* Both sentinels must be caught. The lowest getter's "no data" value is
         * +126, at the opposite end from the highest getter's -126. */
        check("the lowest sensor reads its no-data sentinel",
              battery.get_lowest_sensor_temperature() == 126);
        check("the highest sensor reads its no-data sentinel",
              battery.get_highest_sensor_temperature() == -126);
        sim_main_tx.clear();
        sim_run_ms(1200);
        SimFrame tf;
        check_eq("a 0x356 was sent", sim_last_frame(0x356, tf), 1);
        const int16_t reported = (int16_t)(tf.data[4] | (tf.data[5] << 8));
        check("and it carries a physically possible temperature",
              reported >= MODULE_SENSOR_MINIMUM_C * 10 && reported <= MODULE_SENSOR_MAXIMUM_C * 10);
        sim_modules_answer(true);
    }

    suite("wire: a balance burst does not look like a pack imbalance");
    {
        /* Readings are deliberately discarded during a burst, so the packs
         * cannot be compared for its full 60 s. That must not decay into a
         * declared imbalance and hold every contactor open on packs that are
         * perfectly matched. */
        sim_reset();
        sim_set_cells(3960);                    // above CELL_BALANCE_VOLTAGE
        sim_set_cell_override(0, 0, -100);      // an internal spread in each pack,
        sim_set_cell_override(1, 0, -100);      // but the packs match each other
        setup();
        sim_run_ms(20000);
        check_eq("the packs match", (long)battery.voltage_delta_between_packs(), 0);
        int heldSamples = 0, uncomparableSamples = 0;
        for (int i = 0; i < 24; i++) {          // 120 s, two full duty cycles
            sim_run_ms(5000);
            if (!battery.pack_voltages_are_comparable()) uncomparableSamples++;
            if (bms.packs_are_imbalanced() || battery.one_or_more_contactors_inhibited())
                heldSamples++;
        }
        check("a burst did make the packs uncomparable", uncomparableSamples > 0);
        check_eq("but the contactors were never held over 120 s", heldSamples, 0);
        sim_set_cells(3700);
        for (int p = 0; p < NUM_PACKS; p++) sim_set_cell_override(p, 0, 0);
    }

    suite("wire: the limits frame reflects an inhibit in its very next send");
    {
        /* The exact form of the property the fuzzer can only sample: assert an
         * inhibit, then look at the first 0x351 built afterwards. It has to
         * carry 0, because the limits are recomputed at send time rather than
         * read from a cache refreshed on a different tick. */
        sim_reset();
        /* Half charged. The shunt's counter reads 0 at full, so leaving it
         * there makes the battery 100% and the charge limit legitimately 0. */
        sim_set_shunt_amp_seconds(-(int32_t)(BATTERY_CAPACITY_AS / 2));
        setup();
        sim_run_ms(3000);
        check("soc is mid-range", bms.get_soc() > 40 && bms.get_soc() < 60);
        check("a discharge limit is advertised while healthy",
              bms.get_max_discharge_current() > 0);
        check("a charge limit is advertised while healthy",
              bms.get_max_charge_current() > 0);

        /* Driven from a real condition, not a hand-set reason bit: the
         * reconciliation pass withdraws any reason whose condition is not
         * actually present, so a synthetic one is gone within 100 ms. */
        SimFrame f;
        sim_set_cells(CELL_EMPTY_VOLTAGE - 100);          // empty -> drive inhibited
        sim_run_ms(2000);
        check_eq("drive is inhibited", bms.drive_is_inhibited(), 1);
        check_eq("the cached limit is zero", bms.get_max_discharge_current(), 0);
        sim_main_tx.clear();
        sim_run_ms(1200);
        check_eq("a limits frame was sent", sim_last_frame(0x351, f), 1);
        check_eq("and it advertises zero discharge current",
                 (long)(f.data[4] | (f.data[5] << 8)), 0);

        sim_set_cells(CELL_FULL_VOLTAGE + 100);           // full -> charge inhibited
        sim_run_ms(2000);
        check_eq("charge is inhibited", bms.charge_is_inhibited(), 1);
        check_eq("the cached charge limit is zero", bms.get_max_charge_current(), 0);
        sim_main_tx.clear();
        sim_run_ms(1200);
        check_eq("a limits frame was sent", sim_last_frame(0x351, f), 1);
        check_eq("and it advertises zero charge current",
                 (long)(f.data[2] | (f.data[3] << 8)), 0);
        sim_set_cells(3700);
        sim_run_ms(2000);
    }

    suite("wire: the published snapshot is self-consistent");
    {
        sim_reset();
        setup();
        sim_run_ms(WEB_SNAPSHOT_INTERVAL_MS + 3000);
        WebSnapshot s;
        check_eq("a snapshot has been published", webstatus_read(s), 1);
        check_eq("it is marked valid", s.valid, 1);
        check("highest cell is not below lowest", s.highestCellVoltage >= s.lowestCellVoltage);
        check_eq("cell delta matches the two ends",
                 (long)s.cellDelta, (long)(s.highestCellVoltage - s.lowestCellVoltage));
        check("highest temperature is not below lowest", s.highestTemperature >= s.lowestTemperature);
        check("soc is a percentage", s.soc <= 100);
        check("active packs cannot exceed the packs that exist", s.activePacks <= NUM_PACKS);
        check_eq("the state name is populated", s.stateName != nullptr, 1);

        /* Whole-battery figures must agree with the per-pack figures they are
         * derived from, or the page shows two numbers that contradict. */
        uint16_t lowestOfPacks = 0xFFFF, highestOfPacks = 0;
        for (int p = 0; p < NUM_PACKS; p++) {
            if (s.packs[p].lowestCellVoltage < lowestOfPacks)  lowestOfPacks = s.packs[p].lowestCellVoltage;
            if (s.packs[p].highestCellVoltage > highestOfPacks) highestOfPacks = s.packs[p].highestCellVoltage;
        }
        check_eq("battery lowest cell matches the lowest pack", (long)s.lowestCellVoltage, (long)lowestOfPacks);
        check_eq("battery highest cell matches the highest pack", (long)s.highestCellVoltage, (long)highestOfPacks);

        for (int p = 0; p < NUM_PACKS; p++) {
            check("pack delta matches its own two ends",
                  s.packs[p].cellDelta == (uint16_t)(s.packs[p].highestCellVoltage
                                                     - s.packs[p].lowestCellVoltage));
            for (int m = 0; m < MODULES_PER_PACK; m++) {
                check("a populated module reports plausible cell voltages",
                      !s.packs[m < MODULES_PER_PACK ? p : p].modules[m].populated
                      || (s.packs[p].modules[m].cellVoltage[0] > 1000
                          && s.packs[p].modules[m].cellVoltage[0] < 5000));
            }
        }
    }
    sim_reset();
}
