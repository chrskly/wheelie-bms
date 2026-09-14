#include "sim/unit.h"
#include "sim/sim.h"
#include "webstatus.h"
#include "bms.h"
#include "battery.h"
#include "io.h"
#include "shunt.h"
#include "statemachine.h"
#include <cstring>
#include <cstdlib>

extern Bms bms; extern Battery battery; extern Io io; extern Shunt shunt;
void setup();

// Minimal structural JSON check: balanced braces/brackets, quotes closed.
static bool json_balanced(const char* s) {
    int curly = 0, square = 0; bool inStr = false;
    for (const char* c = s; *c; c++) {
        if (inStr) { if (*c == '\\') { if (c[1]) c++; } else if (*c == '"') inStr = false; continue; }
        if (*c == '"') inStr = true;
        else if (*c == '{') curly++;
        else if (*c == '}') curly--;
        else if (*c == '[') square++;
        else if (*c == ']') square--;
        if (curly < 0 || square < 0) return false;
    }
    return curly == 0 && square == 0 && !inStr;
}
static bool has_key(const char* json, const char* key) {
    char pat[64]; snprintf(pat, sizeof pat, "\"%s\":", key);
    return strstr(json, pat) != nullptr;
}

void test_webstatus() {
    static char buf[WEB_JSON_BUFFER_BYTES];

    suite("webstatus: render guards");
    WebSnapshot snap;
    memset(&snap, 0, sizeof snap);
    snap.stateName = "standby";
    check_eq("null buffer returns 0", (long)webstatus_render_json(snap, nullptr, sizeof buf), 0);
    check_eq("zero length returns 0", (long)webstatus_render_json(snap, buf, 0), 0);
    check_eq("a one-byte buffer returns 0", (long)webstatus_render_json(snap, buf, 1), 0);
    check_eq("a plainly-too-small buffer returns 0",
             (long)webstatus_render_json(snap, buf, 64), 0);

    suite("webstatus: a full render");
    size_t n = webstatus_render_json(snap, buf, sizeof buf);
    check("render produced output", n > 0);
    check_eq("length matches strlen", (long)n, (long)strlen(buf));
    check("JSON is balanced", json_balanced(buf));
    check("starts with an object", buf[0] == '{');
    check("ends with an object", buf[n-1] == '}');
    check("no trailing comma before a brace", strstr(buf, ",}") == nullptr);
    check("no trailing comma before a bracket", strstr(buf, ",]") == nullptr);
    check("no empty key", strstr(buf, "\"\":") == nullptr);

    suite("webstatus: expected keys are present");
    const char* keys[] = { "uptime","state","timeInState","soc","voltage","cellMin",
        "cellMax","cellDelta","tempMin","tempMax","batteryAlive","activePacks",
        "maxChargeCurrent","maxDischargeCurrent","heater","ignition","chargeEnable",
        "drive","charge","flags","fault","shunt","can","cfg","packs" };
    for (size_t i = 0; i < sizeof keys / sizeof keys[0]; i++)
        check("key present", has_key(buf, keys[i]), keys[i]);

    suite("webstatus: the state name is escaped, not emitted raw");
    snap.stateName = nullptr;
    webstatus_render_json(snap, buf, sizeof buf);
    check("null name renders as null", strstr(buf, "\"state\":null") != nullptr);
    snap.stateName = "has\"quote";
    webstatus_render_json(snap, buf, sizeof buf);
    check("a quote is refused", strstr(buf, "\"state\":null") != nullptr);
    check("and the document stays balanced", json_balanced(buf));
    snap.stateName = "has\\backslash";
    webstatus_render_json(snap, buf, sizeof buf);
    check("a backslash is refused", strstr(buf, "\"state\":null") != nullptr);
    snap.stateName = "has\nnewline";
    webstatus_render_json(snap, buf, sizeof buf);
    check("a control character is refused", strstr(buf, "\"state\":null") != nullptr);
    snap.stateName = "standby";

    suite("webstatus: values survive the round trip");
    snap.uptimeMs = 1234567ULL;
    snap.soc = 87;
    snap.voltage = 355200;
    snap.lowestCellVoltage = 3690;
    snap.highestCellVoltage = 3712;
    snap.lowestTemperature = -12;
    snap.highestTemperature = 41;
    snap.shuntAmps = -45000;
    snap.driveInhibitReasons = 0x0102;
    n = webstatus_render_json(snap, buf, sizeof buf);
    check("uptime rendered", strstr(buf, "\"uptime\":1234567") != nullptr);
    check("soc rendered", strstr(buf, "\"soc\":87") != nullptr);
    check("voltage rendered", strstr(buf, "\"voltage\":355200") != nullptr);
    check("negative temperature rendered", strstr(buf, "\"tempMin\":-12") != nullptr);
    check("negative shunt current rendered", strstr(buf, "\"amps\":-45000") != nullptr);
    check("reason bitmask rendered", strstr(buf, "\"reasons\":258") != nullptr);
    check("still balanced", json_balanced(buf));

    suite("webstatus: truncation never yields half a document");
    // Walk the buffer size down through the point where it stops fitting.
    size_t full = webstatus_render_json(snap, buf, sizeof buf);
    int zeros = 0, complete = 0;
    for (size_t cap = full + 2; cap > full - 400 && cap > 8; cap -= 7) {
        static char small[WEB_JSON_BUFFER_BYTES];
        size_t got = webstatus_render_json(snap, small, cap);
        if (got == 0) { zeros++; }
        else { complete++; check("any non-zero result is complete JSON", json_balanced(small)); }
    }
    check("some sizes were rejected outright", zeros > 0);
    check("and some still fitted", complete > 0);

    suite("webstatus: seqlock");
    WebSnapshot out;
    memset(&out, 0, sizeof out);
    snap.soc = 42;
    webstatus_publish(snap);
    check_eq("read succeeds after a publish", webstatus_read(out), 1);
    check_eq("valid flag set by the publisher", out.valid, 1);
    check_eq("payload carried across", out.soc, 42);
    snap.soc = 43;
    webstatus_publish(snap);
    check_eq("second publish read back", webstatus_read(out) && out.soc == 43, 1);
    for (int i = 0; i < 20; i++) { snap.soc = (uint8_t)(50 + i); webstatus_publish(snap); }
    check_eq("repeated publishes stay readable", webstatus_read(out) && out.soc == 69, 1);

    suite("webstatus: end-to-end from the running firmware");
    sim_reset();
    setup();
    sim_run_ms(3000);                       // long enough for the worker to publish
    WebSnapshot live;
    check_eq("worker published a snapshot", webstatus_read(live), 1);
    check_eq("state name filled in", strcmp(live.stateName, "standby") == 0, 1);
    check("uptime advanced", live.uptimeMs > 0);
    check_eq("battery voltage matches the packs", (long)live.voltage,
             (long)(3700L * CELLS_PER_MODULE * MODULES_PER_PACK));
    check_eq("cell min", (long)live.lowestCellVoltage, 3700);
    check_eq("cell max", (long)live.highestCellVoltage, 3700);
    check_eq("temperature", live.highestTemperature, 25);
    check_eq("active packs", live.activePacks, NUM_PACKS);
    check_eq("battery alive", live.batteryAlive, 1);
    check_eq("shunt alive", live.shuntAlive, 1);
    check_eq("soc", live.soc, 100);
    check_eq("per-pack voltage", (long)live.packs[0].voltage,
             (long)(3700L * CELLS_PER_MODULE * MODULES_PER_PACK));
    check_eq("per-module cell voltage", (long)live.packs[0].modules[0].cellVoltage[0], 3700);
    check_eq("per-module populated", live.packs[0].modules[0].populated, 1);
    check_eq("per-module alive", live.packs[0].modules[0].alive, 1);
    check_eq("unfitted sensor reported as no-reading",
             live.packs[0].modules[0].cellTemperature[TEMPS_PER_MODULE-1], NO_TEMPERATURE_READING);
    n = webstatus_render_json(live, buf, sizeof buf);
    check("live snapshot renders", n > 0);
    check("live JSON balanced", json_balanced(buf));
    check("live JSON fits the configured buffer", n < WEB_JSON_BUFFER_BYTES);
    printf("   (live /api/status payload is %u bytes of a %u byte buffer)\n",
           (unsigned)n, (unsigned)WEB_JSON_BUFFER_BYTES);

    suite("webstatus: snapshot tracks a fault");
    sim_modules_answer(false);
    sim_run_ms(7000);
    check_eq("fault state captured", webstatus_read(live) &&
             strcmp(live.stateName, "criticalFault") == 0, 1);
    check_eq("battery reported not alive", live.batteryAlive, 0);
    check_eq("drive inhibited in the snapshot", live.driveInhibited, 1);
    check("a drive inhibit reason is recorded", live.driveInhibitReasons != 0);
    sim_modules_answer(true);
    sim_reset();
}
