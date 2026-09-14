#pragma once
#include <cstdio>
#include <cstring>
extern int g_pass, g_fail;
extern const char* g_suite;
void suite(const char* name);
void check(const char* what, bool ok, const char* detail = "");
void check_eq(const char* what, long got, long want);
int  unit_summary();
