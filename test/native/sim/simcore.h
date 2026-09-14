#pragma once
#include <stdint.h>
#include <deque>
#include <vector>
struct SimFrame { uint32_t id=0; uint8_t len=8; uint8_t data[8]={0,0,0,0,0,0,0,0}; };
extern uint64_t sim_now_us;
extern int      sim_gpio[64];
extern int      sim_gpio_mode[64];
extern std::deque<SimFrame>  sim_pack_rx[2];
extern std::vector<SimFrame> sim_pack_tx[2];
extern std::deque<SimFrame>  sim_main_rx;
extern std::vector<SimFrame> sim_main_tx;
extern long sim_ticks_remaining;
struct SimStop {};
