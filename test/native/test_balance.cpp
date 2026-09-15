/*
 * Properties of the balancing duty cycle, observed from the pack bus.
 *
 * Balancing is the newest subsystem here and the one whose mistakes are least
 * visible: an over-long burst or a target below the lowest cell bleeds real
 * capacity away, and nothing upstream would report it.
 */
#include "sim/unit.h"
#include "sim/sim.h"
#include "bms.h"
#include "battery.h"
#include "statemachine.h"

extern Bms bms; extern Battery battery;
void setup();

void test_balance() {
    suite("balance: the duty cycle and the targets it sends");

    sim_reset();
    sim_set_cells(3960);                         // above CELL_BALANCE_VOLTAGE
    for (int p = 0; p < NUM_PACKS; p++)
        sim_set_cell_override(p, 0, -120);       // one low module per pack
    setup();

    const uint16_t lowestCell = 3960 - 120;
    const uint16_t expectedTarget = (uint16_t)(lowestCell + CELL_BALANCE_TARGET_OFFSET_MV);

    long armedPolls = 0, idlePolls = 0, wrongTarget = 0, targetBelowLowest = 0;
    long burstMs = 0, longestBurstMs = 0;
    bool wasArmed = false;

    /* Step in units small enough to time a burst, over two full duty cycles. */
    const long stepMs = 250;
    const long totalMs = 2 * (CELL_BALANCE_DUTY_MS + CELL_BALANCE_REST_MS) + 20000;
    for (long t = 0; t < totalMs; t += stepMs) {
        for (int p = 0; p < NUM_PACKS; p++) sim_pack_tx[p].clear();
        sim_run_ms(stepMs);

        bool armedThisStep = false;
        for (int p = 0; p < NUM_PACKS; p++)
            for (auto& tx : sim_pack_tx[p]) {
                if ((tx.id & 0xFF0) != 0x080) continue;
                const uint16_t target = (uint16_t)(tx.data[0] | (tx.data[1] << 8));
                if (tx.data[4] == MODULE_CMD_BALANCE_ON) {
                    armedPolls++;
                    armedThisStep = true;
                    if (target != expectedTarget) wrongTarget++;
                    /* A target below the lowest cell would tell every cell in
                     * the pack to bleed below the floor. */
                    if (target < lowestCell) targetBelowLowest++;
                } else {
                    idlePolls++;
                    // The inert target must be the one the modules ignore.
                    if (target != 0x10C7 && tx.data[4] != 0x20) wrongTarget++;
                }
            }

        if (armedThisStep) {
            burstMs += stepMs;
            if (burstMs > longestBurstMs) longestBurstMs = burstMs;
        } else {
            burstMs = 0;
        }
    }

    check("balancing armed at some point", armedPolls > 0);
    check("and idled at some point", idlePolls > 0);
    check_eq("every armed poll carried the lowest cell plus the offset", wrongTarget, 0);
    check_eq("no target was ever below the lowest cell", targetBelowLowest, 0);
    check("a burst never outlasts CELL_BALANCE_DUTY_MS",
          longestBurstMs <= CELL_BALANCE_DUTY_MS + 2 * stepMs);
    if (longestBurstMs > CELL_BALANCE_DUTY_MS + 2 * stepMs)
        printf("   longest burst was %ld ms, duty is %d ms\n", longestBurstMs, CELL_BALANCE_DUTY_MS);

    suite("balance: cells below the threshold are never balanced");
    sim_reset();
    sim_set_cells(3700);                         // below CELL_BALANCE_VOLTAGE
    for (int p = 0; p < NUM_PACKS; p++) sim_set_cell_override(p, 0, -120);
    setup();
    sim_run_ms(CELL_BALANCE_REST_MS + 30000);
    long armedBelowThreshold = 0;
    for (int p = 0; p < NUM_PACKS; p++)
        for (auto& tx : sim_pack_tx[p])
            if ((tx.id & 0xFF0) == 0x080 && tx.data[4] == MODULE_CMD_BALANCE_ON)
                armedBelowThreshold++;
    check_eq("nothing was armed", armedBelowThreshold, 0);

    suite("balance: a pack with no spread is not balanced");
    sim_reset();
    sim_set_cells(3960);                         // high, but perfectly even
    setup();
    sim_run_ms(CELL_BALANCE_REST_MS + 30000);
    long armedWithNoSpread = 0;
    for (int p = 0; p < NUM_PACKS; p++)
        for (auto& tx : sim_pack_tx[p])
            if ((tx.id & 0xFF0) == 0x080 && tx.data[4] == MODULE_CMD_BALANCE_ON)
                armedWithNoSpread++;
    check_eq("nothing was armed", armedWithNoSpread, 0);

    sim_set_cells(3700);
    for (int p = 0; p < NUM_PACKS; p++) sim_set_cell_override(p, 0, 0);
    sim_reset();
}
