#pragma once
typedef enum { ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_TASK_WDT, ESP_RST_INT_WDT, ESP_RST_WDT } esp_reset_reason_t;
extern esp_reset_reason_t sim_reset_reason;
inline esp_reset_reason_t esp_reset_reason() { return sim_reset_reason; }
