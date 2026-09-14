#include "sim/unit.h"
#include "sim/sim.h"
#include "shunt.h"

void test_shunt() {
    sim_reset();
    suite("shunt: accessors");
    Shunt s;
    check_eq("amps default", (long)s.get_amps(), 0);
    s.set_amps(-45000);      check_eq("amps", (long)s.get_amps(), -45000);
    s.set_voltage1(350123);  check_eq("voltage1", (long)s.get_voltage1(), 350123);
    s.set_voltage2(12500);   check_eq("voltage2", (long)s.get_voltage2(), 12500);
    s.set_voltage3(349000);  check_eq("voltage3", (long)s.get_voltage3(), 349000);
    s.set_temperature(31);   check_eq("temperature", (long)s.get_temperature(), 31);
    s.set_watts(-16000);     check_eq("watts", (long)s.get_watts(), -16000);
    s.set_ampSeconds(-9360); check_eq("ampSeconds", (long)s.get_ampSeconds(), -9360);
    s.set_wattHours(-7400);  check_eq("wattHours", (long)s.get_wattHours(), -7400);

    suite("shunt: liveness");
    check_eq("not dead at boot", s.is_dead(), 0);
    sim_now_us = (uint64_t)(SHUNT_TTL_MS - 1) * 1000;
    check_eq("not dead just inside the TTL", s.is_dead(), 0);
    sim_now_us = (uint64_t)(SHUNT_TTL_MS + 1) * 1000;
    check_eq("dead past the TTL", s.is_dead(), 1);
    s.heartbeat();
    check_eq("heartbeat revives it", s.is_dead(), 0);
    sim_now_us += (uint64_t)(SHUNT_TTL_MS + 1) * 1000;
    check_eq("dead again after the TTL", s.is_dead(), 1);
}
