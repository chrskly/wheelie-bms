#pragma once
#include "simcore.h"
inline int64_t esp_timer_get_time() { return (int64_t)sim_now_us; }
