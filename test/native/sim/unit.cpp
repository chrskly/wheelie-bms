#include "unit.h"
int g_pass = 0, g_fail = 0;
const char* g_suite = "";
void suite(const char* name) { g_suite = name; printf("\n--- %s ---\n", name); }
void check(const char* what, bool ok, const char* detail) {
    if (ok) { g_pass++; }
    else { g_fail++; printf("   FAIL [%s] %s %s\n", g_suite, what, detail); }
}
void check_eq(const char* what, long got, long want) {
    if (got == want) { g_pass++; }
    else { g_fail++; printf("   FAIL [%s] %s : got %ld want %ld\n", g_suite, what, got, want); }
}
int unit_summary() {
    printf("\n=========================================\n");
    printf(" %d passed, %d failed\n", g_pass, g_fail);
    printf("=========================================\n");
    return g_fail ? 1 : 0;
}
