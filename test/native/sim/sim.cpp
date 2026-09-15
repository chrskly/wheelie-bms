#include "sim.h"
#include <cstdint>
#include <cstring>

uint64_t sim_now_us = 0;
int sim_gpio[64] = {0};
int sim_gpio_mode[64] = {0};
std::deque<SimFrame>  sim_pack_rx[2];
std::vector<SimFrame> sim_pack_tx[2];
std::deque<SimFrame>  sim_main_rx;
std::vector<SimFrame> sim_main_tx;
long sim_ticks_remaining = 0;
SimTaskFn sim_task_fn = nullptr; void* sim_task_arg = nullptr;
SPIClass SPI; SerialStub Serial;
ACAN_ESP32 ACAN_ESP32::can;
bool sim_pack_tx_fails = false;
uint16_t sim_pack_begin_error = 0;
bool sim_main_tx_fails = false;
uint32_t sim_main_begin_error = 0;
uint32_t sim_main_status_flags = 0;
bool sim_wdt_init_fails = false;
bool sim_wdt_add_fails = false;
bool sim_task_create_fails = false;
uint8_t sim_pack_eflg[2] = {0, 0};
uint16_t sim_pack_rx_peak[2] = {0, 0};
uint32_t sim_ledc_result = 0;
bool sim_ledc_fails = false;
esp_reset_reason_t sim_reset_reason = ESP_RST_POWERON;
void sim_set_ledc_result(uint32_t r) { sim_ledc_result = r; }
void sim_set_reset_reason(esp_reset_reason_t r) { sim_reset_reason = r; }

int sim_port_for_cs(uint8_t cs) {
    for (int i = 0; i < NUM_PACKS; i++) if (CS_PINS[i] == (int)cs) return i;
    return -1;
}

static uint16_t s_cell_mv = 3700;
static int      s_override[2][MODULES_PER_PACK];
static int8_t   s_temp_c = 25;
static int      s_temps_fitted = 2;
static bool     s_modules_answer = true;
static bool     s_shunt_answers = true;
/* The simulated shunt reports a physically consistent triple by default:
 * watts follows from amps x volts, so Bms::check_shunt_plausibility() stays
 * quiet unless a test deliberately makes it disagree. */
static int32_t  s_shunt_amps = 0;                              // mA
static int32_t  s_shunt_voltage1 = 3700 * CELLS_PER_MODULE * MODULES_PER_PACK;  // mV
static int32_t  s_shunt_watts = 0;                             // W
static bool     s_shunt_watts_overridden = false;
static int32_t  s_amp_seconds = 0;
static long     s_tick = 0;

void sim_set_cells(uint16_t mv) { s_cell_mv = mv; }
void sim_set_cell_override(int p,int m,int d){ if(p>=0&&p<2&&m>=0&&m<MODULES_PER_PACK) s_override[p][m]=d; }
void sim_set_temp(int8_t c) { s_temp_c = c; }
void sim_set_temp_sensors_fitted(int n) { s_temps_fitted = n; }
void sim_modules_answer(bool on) { s_modules_answer = on; }
void sim_shunt_answers(bool on) { s_shunt_answers = on; }
void sim_set_shunt_amp_seconds(int32_t as) { s_amp_seconds = as; }

void sim_reset() {
    /* Back to t=0: a fresh power-on. Safe because init() on every firmware
     * object is a genuine reset -- if that stops being true, suites that run
     * after a long-running one will start seeing timestamps from the future. */
    sim_now_us = 0;
    s_tick = 0;
    memset(sim_gpio, 0, sizeof sim_gpio);
    memset(sim_gpio_mode, 0, sizeof sim_gpio_mode);
    memset(s_override, 0, sizeof s_override);
    for (int i=0;i<2;i++){ sim_pack_rx[i].clear(); sim_pack_tx[i].clear(); }
    sim_main_rx.clear(); sim_main_tx.clear();
    sim_task_fn = nullptr; sim_task_arg = nullptr;
    sim_pack_tx_fails = false; sim_pack_begin_error = 0;
    sim_wdt_init_fails = false; sim_wdt_add_fails = false; sim_task_create_fails = false;
    sim_pack_eflg[0] = 0; sim_pack_eflg[1] = 0;
    sim_pack_rx_peak[0] = 0; sim_pack_rx_peak[1] = 0;
    sim_ledc_result = 0; sim_ledc_fails = false; sim_reset_reason = ESP_RST_POWERON;
    sim_main_tx_fails = false; sim_main_begin_error = 0; sim_main_status_flags = 0;
    s_cell_mv = 3700; s_temp_c = 25; s_temps_fitted = 2;
    s_modules_answer = true; s_shunt_answers = true; s_amp_seconds = 0;
    s_shunt_amps = 0; s_shunt_watts = 0; s_shunt_watts_overridden = false;
    s_shunt_voltage1 = 3700 * CELLS_PER_MODULE * MODULES_PER_PACK;
}

static void push_module_reply(int port, int mod) {
    SimFrame s; s.id = 0x100 | mod; s.len = 8;
    sim_pack_rx[port].push_back(s);
    uint16_t mv = (uint16_t)((int)s_cell_mv + s_override[port][mod]);
    for (int grp = 0; grp < 6; grp++) {
        SimFrame f; f.id = 0x100 | ((0x20 + grp*0x10) & 0xF0) | mod; f.len = 8;
        for (int c = 0; c < 3; c++) {
            f.data[c*2]   = (uint8_t)(mv & 0xFF);
            f.data[c*2+1] = (uint8_t)((mv >> 8) & 0x3F);
        }
        sim_pack_rx[port].push_back(f);
    }
    SimFrame t; t.id = 0x180 | mod; t.len = 8;
    for (int i = 0; i < TEMPS_PER_MODULE; i++)
        t.data[i] = (i < s_temps_fitted) ? (uint8_t)(s_temp_c + 40) : 0;
    sim_pack_rx[port].push_back(t);
}

static void push_shunt() {
    for (uint32_t id = 0x521; id <= 0x528; id++) {
        SimFrame f; f.id = id; f.len = 8;
        int32_t v = 0;
        switch (id) {
            case 0x521: v = s_shunt_amps;     break;
            case 0x522: v = s_shunt_voltage1; break;
            case 0x526: v = s_shunt_watts;    break;
            case 0x527: v = s_amp_seconds;    break;
            default:    v = 0;                break;
        }
        f.data[2] = (uint8_t)(v & 0xFF);       f.data[3] = (uint8_t)((v >> 8) & 0xFF);
        f.data[4] = (uint8_t)((v >> 16)&0xFF); f.data[5] = (uint8_t)((v >> 24)&0xFF);
        sim_main_rx.push_back(f);
    }
}

void sim_on_tick() {
    s_tick++;
    if (s_modules_answer && (s_tick % 18) == 0)
        for (int p = 0; p < NUM_PACKS; p++)
            for (int m = 0; m < MODULES_PER_PACK; m++) push_module_reply(p, m);
    if (s_shunt_answers && (s_tick % 20) == 0) push_shunt();
}

void sim_run_ms(long ms) {
    sim_ticks_remaining = ms / BMS_WORKER_TICK_MS;
    if (sim_ticks_remaining <= 0) sim_ticks_remaining = 1;
    try { for (;;) { if (!sim_task_fn) return; sim_task_fn(sim_task_arg); } }
    catch (SimStop&) {}
}

bool sim_last_frame(uint32_t id, SimFrame& out) {
    for (int i = (int)sim_main_tx.size()-1; i >= 0; i--)
        if (sim_main_tx[i].id == id) { out = sim_main_tx[i]; return true; }
    return false;
}
int sim_count_frames(uint32_t id) {
    int n = 0; for (auto& f : sim_main_tx) if (f.id == id) n++; return n;
}

void sim_set_pack_eflg(int pack, uint8_t eflg) { if (pack>=0&&pack<2) sim_pack_eflg[pack]=eflg; }

static void recompute_shunt_watts() {
    if (!s_shunt_watts_overridden)
        s_shunt_watts = (int32_t)(((int64_t)s_shunt_amps * (int64_t)s_shunt_voltage1) / 1000000);
}
void sim_set_shunt_amps(int32_t mA) { s_shunt_amps = mA; recompute_shunt_watts(); }
void sim_set_shunt_voltage(int32_t mV) { s_shunt_voltage1 = mV; recompute_shunt_watts(); }
void sim_set_shunt_watts(int32_t w) { s_shunt_watts = w; s_shunt_watts_overridden = true; }

void sim_set_pack_rx_peak(int pack, uint16_t peak) { if (pack>=0&&pack<2) sim_pack_rx_peak[pack]=peak; }
