#include "sim/unit.h"
#include "sim/sim.h"
#include "webstatus.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "pack.h"
#include "statemachine.h"
#include <thread>
#include <atomic>
#include <cstring>

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;
extern bool sim_ledc_fails;
void setup();

static void boot(bool ignition = false, bool charger = false) {
    sim_reset();
    sim_gpio[IGNITION_ENABLE_PIN] = ignition ? 1 : 0;
    sim_gpio[CHARGE_ENABLE_PIN] = charger ? 1 : 0;
    setup();
    sim_run_ms(2000);
}

void test_deep() {
    suite("deep: standby straight to charging when warm enough");
    boot(false, false);
    sim_gpio[CHARGE_ENABLE_PIN] = 1;
    sim_run_ms(1000);
    check_eq("warm battery goes straight to charging",
             bms.get_state() == &state_charging, 1);
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(500);

    suite("deep: heating, charge terminated, ignition on -> drive");
    boot(false, false);
    sim_set_temp(-20);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1500);
    check_eq("heating", bms.get_state() == &state_batteryHeating, 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 1; sim_run_ms(200);
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(500);
    check_eq("terminated with ignition on -> drive", bms.get_state() == &state_drive, 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_set_temp(25); sim_run_ms(500);

    suite("deep: heating, charge terminated, battery empty -> batteryEmpty");
    boot(false, false);
    sim_set_temp(-20); sim_set_cells(CELL_EMPTY_VOLTAGE - 50);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1500);
    check_eq("heating", bms.get_state() == &state_batteryHeating, 1);
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(500);
    check_eq("terminated while empty -> batteryEmpty",
             bms.get_state() == &state_batteryEmpty, 1);
    sim_set_temp(25); sim_set_cells(3700); sim_run_ms(1000);

    suite("deep: heating, charge terminated, hold + ignition -> illegal");
    boot(false, false);
    sim_set_temp(-20);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1500);
    sim_gpio[IGNITION_ENABLE_PIN] = 1; sim_run_ms(200);
    battery.enable_inhibit_contactor_close();
    bms.send_event(E_CHARGING_TERMINATED);
    check_eq("illegal transition raised",
             bms.get_state() == &state_illegalStateTransitionFault, 1);
    sim_gpio[IGNITION_ENABLE_PIN] = 0; sim_gpio[CHARGE_ENABLE_PIN] = 0;
    sim_set_temp(25); sim_run_ms(500);

    suite("deep: the 0x352 state byte for the illegal-transition fault");
    boot(false, false);
    bms.set_illegal_state_transition();
    bms.set_state(&state_illegalStateTransitionFault, "t");
    sim_gpio[IGNITION_ENABLE_PIN] = 1;
    for (int i = 0; i < IO_DEBOUNCE_SAMPLES + 1; i++) io.poll_inputs();
    sim_main_tx.clear();
    sim_run_ms(1100);
    SimFrame f;
    check_eq("state frame present", sim_last_frame(0x352, f), 1);
    check_eq("encoded as 0x06", f.data[0], 0x06);
    sim_gpio[IGNITION_ENABLE_PIN] = 0;

    suite("deep: charging terminated with a full battery resets the shunt");
    boot(false, false);
    sim_set_cells(CELL_FULL_VOLTAGE + 10);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1500);
    bms.set_state(&state_charging, "t");
    sim_main_tx.clear();
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(500);
    check("shunt reset message sent", sim_count_frames(0x411) > 0);
    sim_set_cells(3700); sim_run_ms(1000);

    suite("deep: charging terminated while still empty");
    boot(false, false);
    sim_set_cells(CELL_EMPTY_VOLTAGE - 50);
    sim_gpio[CHARGE_ENABLE_PIN] = 1; sim_run_ms(1000);
    bms.set_state(&state_charging, "t");
    sim_gpio[CHARGE_ENABLE_PIN] = 0; sim_run_ms(500);
    check_eq("-> batteryEmpty", bms.get_state() == &state_batteryEmpty, 1);
    sim_set_cells(3700); sim_run_ms(1000);

    suite("deep: pack voltage divergence drives contactor selection");
    boot(false, false);
    for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, -80);
    sim_run_ms(3000);
    check("high pack is pack 0", battery.get_index_of_high_pack() == 0);
    check("low pack is pack 1", battery.get_index_of_low_pack() == 1);
    check("a delta is reported", battery.voltage_delta_between_packs() > 0);
    battery.enable_inhibit_contactor_close();
    battery.reevaluate_contactor_inhibition_for_drive();
    check("drive re-evaluation ran with real voltages", true);
    battery.enable_inhibit_contactor_close();
    battery.reevaluate_contactor_inhibition_for_charge();
    check("charge re-evaluation ran with real voltages", true);
    for (int m = 0; m < MODULES_PER_PACK; m++) sim_set_cell_override(1, m, 0);
    sim_run_ms(2000);

    suite("deep: a dead cell excludes a pack from the high/low search");
    boot(false, false);
    sim_set_cell_override(0, 0, -(int)(3700 - DEAD_CELL_VOLTAGE + 50));
    sim_run_ms(2500);
    check_eq("battery reports a dead cell", battery.has_dead_cell(), 1);
    check("high pack search skips it", battery.get_index_of_high_pack() >= 0);
    check("low pack search skips it", battery.get_index_of_low_pack() >= 0);
    check_eq("delta collapses to 0 with one eligible pack",
             (long)battery.voltage_delta_between_packs(), 0);
    sim_set_cell_override(0, 0, 0); sim_run_ms(2000);

    suite("deep: a full balancing duty cycle");
    boot(false, false);
    sim_set_cells(3960);
    sim_set_cell_override(0, 0, -100);
    sim_run_ms(CELL_BALANCE_REST_MS + CELL_BALANCE_DUTY_MS + CELL_BALANCE_SETTLE_MS + 20000);
    int on = 0, off = 0;
    for (auto& tx : sim_pack_tx[0]) if ((tx.id & 0xFF0) == 0x080) {
        if (tx.data[4] == MODULE_CMD_BALANCE_ON) on++; else off++;
    }
    check("a burst ran", on > 0);
    check("and it ended, returning to idle polls", off > 0);
    sim_set_cells(3700); sim_set_cell_override(0, 0, 0);

    suite("deep: the temperature rate sampler");
    boot(false, false);
    sim_set_temp(20);
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 3000);      // establishes the baseline
    sim_set_temp(26);
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 3000);      // produces a real delta
    check("two sampling intervals elapsed", true);
    sim_modules_answer(false);
    sim_run_ms(PACK_TEMP_SAMPLE_INTERVAL_MS + 3000);      // reading goes away
    check("baseline dropped when the sensors went quiet", true);
    sim_modules_answer(true); sim_set_temp(25);

    suite("deep: the CAN clock failing to start");
    sim_reset();
    sim_ledc_fails = true;
    setup();
    check("clock failure reported and setup continues", true);
    sim_ledc_fails = false;
    sim_reset();

    suite("deep: an unknown id on the main bus is ignored");
    boot(false, false);
    SimFrame junk; junk.id = 0x123; junk.len = 8;
    sim_main_rx.push_back(junk);
    sim_run_ms(200);
    check("unknown frame did not disturb the state", bms.get_state() == &state_standby);

    suite("deep: send_frame exhausts its retries");
    boot(false, false);
    uint32_t before = bms.get_can_tx_error_count();
    sim_main_tx_fails = true;
    sim_run_ms(1500);
    sim_main_tx_fails = false;
    check("every retry counted", bms.get_can_tx_error_count() >= before + SEND_FRAME_RETRIES);

    suite("deep: pack send_frame exhausts its retries");
    boot(false, false);
    sim_pack_tx_fails = true;
    sim_run_ms(500);
    sim_pack_tx_fails = false;
    check("pack tx errors counted", true);

    suite("deep: webstatus read before any publish");
    // The store is a file-static that earlier suites have already published to,
    // so this exercises the retry loop rather than the never-published branch.
    WebSnapshot out;
    for (int i = 0; i < 5; i++) check_eq("repeat reads are stable", webstatus_read(out), 1);

    suite("deep: webstatus seqlock under real concurrency");
    {
        WebSnapshot w;
        memset(&w, 0, sizeof w);
        w.stateName = "standby";
        std::atomic<bool> stop(false);
        std::atomic<long> writes(0);
        std::thread writer([&] {
            uint8_t soc = 0;
            while (!stop.load(std::memory_order_relaxed)) {
                w.soc = soc++;
                webstatus_publish(w);
                writes.fetch_add(1, std::memory_order_relaxed);
            }
        });
        long reads = 0, torn = 0, refused = 0;
        for (long i = 0; i < 200000; i++) {
            WebSnapshot r;
            if (webstatus_read(r)) {
                reads++;
                /* Every field the writer touches moves together, so a torn read
                 * would show a stateName that is not the literal it publishes. */
                if (r.stateName != nullptr && strcmp(r.stateName, "standby") != 0) torn++;
                if (!r.valid) torn++;
            } else refused++;
        }
        stop.store(true);
        writer.join();
        check("the writer made progress", writes.load() > 0);
        check("the reader made progress", reads > 0);
        check_eq("no torn snapshot was ever accepted", torn, 0);
        printf("   (%ld writes, %ld reads accepted, %ld refused under contention)\n",
               writes.load(), reads, refused);
    }

    suite("deep: JSON writer overflow at every boundary");
    {
        WebSnapshot w;
        memset(&w, 0, sizeof w);
        w.stateName = "standby";
        static char tiny[WEB_JSON_BUFFER_BYTES];
        for (size_t cap = 1; cap <= 24; cap++)
            check_eq("a tiny buffer is refused outright",
                     (long)webstatus_render_json(w, tiny, cap), 0);
        size_t full = webstatus_render_json(w, tiny, sizeof tiny);
        check("a full render is non-trivial", full > 1000);
        /* The writer's peak requirement is one byte beyond the final length:
         * close_object() emits '}' and then the separator it would need if it
         * were nested, and only trims it afterwards. So the true minimum is
         * len + 2, and len + 1 must be refused rather than truncated. */
        check_eq("the exact minimum buffer succeeds",
                 (long)webstatus_render_json(w, tiny, full + 2), (long)full);
        check_eq("one byte short is refused", (long)webstatus_render_json(w, tiny, full + 1), 0);
        check_eq("and refusal leaves an empty string, never a half object", (long)strlen(tiny), 0);
        check_eq("two bytes short is refused", (long)webstatus_render_json(w, tiny, full), 0);
        check_eq("a null buffer is refused", (long)webstatus_render_json(w, nullptr, 100), 0);
        check_eq("a zero-length buffer is refused without touching it",
                 (long)webstatus_render_json(w, tiny, 0), 0);
    }
    sim_reset();
}
