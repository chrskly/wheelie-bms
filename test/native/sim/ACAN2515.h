#pragma once
#include <Arduino.h>
class CANMessage { public: uint32_t id=0; bool ext=false; bool rtr=false;
  uint8_t idx=0; uint8_t len=0; uint8_t data[8]={0,0,0,0,0,0,0,0}; };
class ACAN2515Settings { public: ACAN2515Settings(uint32_t,uint32_t) {}
  enum RequestedMode { NormalMode }; RequestedMode mRequestedMode=NormalMode; };
int sim_port_for_cs(uint8_t cs);
extern bool sim_pack_tx_fails;      // make tryToSend() report failure
extern uint16_t sim_pack_begin_error;  // make begin() report an error code
class ACAN2515 { int port;
 public: ACAN2515(uint8_t cs, SPIClass&, uint8_t) : port(sim_port_for_cs(cs)) {}
  uint16_t begin(const ACAN2515Settings&, void(*)()) { return sim_pack_begin_error; }
  bool tryToSend(const CANMessage& m) { if (sim_pack_tx_fails) return false; SimFrame f; f.id=m.id; f.len=m.len;
      for(int i=0;i<8;i++) f.data[i]=m.data[i];
      if(port>=0&&port<2) sim_pack_tx[port].push_back(f); return true; }
  bool receive(CANMessage& m) {
      if(port<0||port>=2||sim_pack_rx[port].empty()) return false;
      SimFrame f=sim_pack_rx[port].front(); sim_pack_rx[port].pop_front();
      m.id=f.id; m.len=f.len; for(int i=0;i<8;i++) m.data[i]=f.data[i]; return true; }
  bool available() { return port>=0&&port<2&&!sim_pack_rx[port].empty(); }
  void poll() {} };
