#include "sim/unit.h"
#include "sim/sim.h"
#include "util.h"
#include "CRC8.h"

void test_util() {
    suite("util: clock");
    sim_reset();
    check_eq("clock starts at 0 after a reset", (long)get_clock_ms(), 0);
    sim_now_us = 1500000;
    check_eq("clock reports milliseconds", (long)get_clock_ms(), 1500);
    sim_now_us = 0;

    suite("util: zero_frame / print_frame");
    CANMessage f; f.id = 0x123; f.len = 3;
    for (int i = 0; i < 8; i++) f.data[i] = 0xAA;
    zero_frame(&f);
    check_eq("id cleared", (long)f.id, 0);
    check_eq("len set to 8", f.len, 8);
    long sum = 0; for (int i = 0; i < 8; i++) sum += f.data[i];
    check_eq("payload cleared", sum, 0);
    f.id = 0x7FF; f.data[0] = 1;
    print_frame(&f);                       // exercises the printer
    check("print_frame runs", true);

    suite("util: put_u16_le");
    zero_frame(&f);
    put_u16_le(&f, 0, 0x1234);
    check_eq("LSB first", f.data[0], 0x34);
    check_eq("MSB second", f.data[1], 0x12);
    put_u16_le(&f, 6, 0xBEEF);
    check_eq("offset 6 LSB", f.data[6], 0xEF);
    check_eq("offset 6 MSB", f.data[7], 0xBE);
    f.data[7] = 0x11;
    put_u16_le(&f, 7, 0x1234);             // would run past the end
    check_eq("offset 7 refused", f.data[7], 0x11);
    put_u16_le(&f, -1, 0x1234);
    check("negative offset refused", true);
    put_u16_le(nullptr, 0, 0x1234);
    check("null frame refused", true);

    suite("util: put_i16_le");
    zero_frame(&f);
    put_i16_le(&f, 2, (int16_t)-1234);
    check_eq("signed round trip", (int16_t)(f.data[2] | (f.data[3] << 8)), -1234);
    put_i16_le(&f, 0, 32767);
    check_eq("max positive", (int16_t)(f.data[0] | (f.data[1] << 8)), 32767);

    suite("util: put_u32_le");
    zero_frame(&f);
    put_u32_le(&f, 0, 0xDEADBEEF);
    check_eq("byte 0", f.data[0], 0xEF);
    check_eq("byte 1", f.data[1], 0xBE);
    check_eq("byte 2", f.data[2], 0xAD);
    check_eq("byte 3", f.data[3], 0xDE);
    put_u32_le(&f, 4, 0x01020304);
    check_eq("offset 4 byte 7", f.data[7], 0x01);
    f.data[5] = 0x55;
    put_u32_le(&f, 5, 0xFFFFFFFF);         // would run past the end
    check_eq("offset 5 refused", f.data[5], 0x55);
    put_u32_le(&f, -2, 0);
    check("negative offset refused", true);
    put_u32_le(nullptr, 0, 0);
    check("null frame refused", true);

    suite("CRC8");
    CRC8 c8;
    c8.begin();
    uint8_t msg[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t a = c8.get_crc8(msg, 4, 0x00);
    uint8_t b = c8.get_crc8(msg, 4, 0x00);
    check("deterministic", a == b);
    uint8_t withXor = c8.get_crc8(msg, 4, 0xFF);
    check("final xor changes the result", a != withXor);
    msg[0] = 0x02;
    uint8_t changed = c8.get_crc8(msg, 4, 0x00);
    check("input change changes the result", a != changed);
    check_eq("zero length returns the seed xor final", c8.get_crc8(msg, 0, 0x00), 0xFF);
    check_eq("zero length honours final xor", c8.get_crc8(msg, 0, 0x0F), 0xF0);
}
