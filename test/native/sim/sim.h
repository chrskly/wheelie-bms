#pragma once
/* Test-side control surface over the simulated hardware. */
#include <Arduino.h>
#include <ACAN_ESP32.h>
#include "simcore.h"
#include "settings.h"
#include <esp_system.h>

void     sim_reset();                       // clean slate between suites
void     sim_run_ms(long ms);               // advance the worker by wall-clock ms
void     sim_set_cells(uint16_t mv);        // what every module reports
void     sim_set_cell_override(int pack, int module, int deltaMv);
void     sim_set_temp(int8_t degC);
void     sim_set_temp_sensors_fitted(int n);
void     sim_modules_answer(bool on);
void     sim_shunt_answers(bool on);
void     sim_set_shunt_amp_seconds(int32_t as);
void     sim_set_ledc_result(uint32_t r);
void     sim_set_reset_reason(esp_reset_reason_t r);
bool     sim_last_frame(uint32_t id, SimFrame& out);
int      sim_count_frames(uint32_t id);
