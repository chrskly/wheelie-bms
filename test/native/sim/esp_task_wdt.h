#pragma once
#define ESP_OK 0
extern bool sim_wdt_init_fails;
extern bool sim_wdt_add_fails;
inline int esp_task_wdt_init(uint32_t, bool) { return sim_wdt_init_fails ? -1 : ESP_OK; }
inline int esp_task_wdt_add(void*) { return sim_wdt_add_fails ? -1 : ESP_OK; }
inline void esp_task_wdt_reset() {}
