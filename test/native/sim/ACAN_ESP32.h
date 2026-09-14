#pragma once
#include <ACAN2515.h>
typedef int gpio_num_t;
#define GPIO_NUM_16 16
#define GPIO_NUM_17 17
class ACAN_ESP32_Settings { public: ACAN_ESP32_Settings(uint32_t) {}
  gpio_num_t mRxPin=0, mTxPin=0; };
extern bool sim_main_tx_fails;
extern uint32_t sim_main_begin_error;
extern uint32_t sim_main_status_flags;
class ACAN_ESP32 { public: uint32_t begin(const ACAN_ESP32_Settings&) { return sim_main_begin_error; }
  bool tryToSend(const CANMessage& m) { if (sim_main_tx_fails) return false; SimFrame f; f.id=m.id; f.len=m.len;
      for(int i=0;i<8;i++) f.data[i]=m.data[i]; sim_main_tx.push_back(f); return true; }
  bool receive(CANMessage& m) { if(sim_main_rx.empty()) return false;
      SimFrame f=sim_main_rx.front(); sim_main_rx.pop_front();
      m.id=f.id; m.len=f.len; for(int i=0;i<8;i++) m.data[i]=f.data[i]; return true; }
  uint32_t statusFlags() const { return sim_main_status_flags; }
  bool recoverFromBusOff() const { return true; }
  static ACAN_ESP32 can; };
