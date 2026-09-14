#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string>
#include "simcore.h"
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define CHANGE 1
inline void pinMode(uint8_t p, uint8_t m) { if (p<64) sim_gpio_mode[p]=m; }
inline void digitalWrite(uint8_t p, uint8_t v) { if (p<64) sim_gpio[p]=v; }
inline int  digitalRead(uint8_t p) { return (p<64) ? sim_gpio[p] : 0; }
inline void delay(unsigned long ms) { sim_now_us += (uint64_t)ms*1000; }
inline unsigned long millis() { return (unsigned long)(sim_now_us/1000); }
inline unsigned long micros() { return (unsigned long)sim_now_us; }
extern uint32_t sim_ledc_result;   // != 0 forces a specific (wrong) frequency
extern bool sim_ledc_fails;        // makes ledcSetup report outright failure
inline uint32_t ledcSetup(uint8_t, uint32_t f, uint8_t) {
    if (sim_ledc_fails) return 0;
    return sim_ledc_result ? sim_ledc_result : f;
}
inline void ledcAttachPin(uint8_t, uint8_t) {}
inline void ledcWrite(uint8_t, uint32_t) {}
inline int digitalPinToInterrupt(uint8_t p) { return p; }
inline void attachInterrupt(int, void(*)(), int) {}
struct SPIClass { void begin(int8_t=-1,int8_t=-1,int8_t=-1,int8_t=-1) {} };
extern SPIClass SPI;
struct SerialStub { void begin(unsigned long) {} };
extern SerialStub Serial;
typedef uint32_t TickType_t; typedef int BaseType_t; typedef void* TaskHandle_t;
#define pdPASS 1
#define pdTRUE 1
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(x) (x)
typedef void (*SimTaskFn)(void*);
extern SimTaskFn sim_task_fn; extern void* sim_task_arg;
extern bool sim_task_create_fails;
inline BaseType_t xTaskCreate(void(*fn)(void*), const char*, uint32_t, void* arg, uint32_t, TaskHandle_t*)
{ if (sim_task_create_fails) return 0; sim_task_fn = fn; sim_task_arg = arg; return pdPASS; }
inline TickType_t xTaskGetTickCount() { return (TickType_t)(sim_now_us/1000); }
void sim_on_tick();   // harness hook, runs once per worker tick
inline void vTaskDelayUntil(TickType_t* last, TickType_t inc) {
    sim_on_tick();
    sim_now_us += (uint64_t)inc*1000; *last += inc;
    if (--sim_ticks_remaining <= 0) throw SimStop{};
}
inline void vTaskDelay(TickType_t t) { sim_now_us += (uint64_t)t*1000; }
