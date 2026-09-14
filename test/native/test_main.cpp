#include "sim/unit.h"
#include "webstatus.h"
void test_util(); void test_module(); void test_shunt(); void test_led();
void test_pack(); void test_battery(); void test_io(); void test_bms();
void test_statemachine(); void test_integration(); void test_edge(); void test_webstatus();
void test_deep(); void test_matrix(); void test_final();

/* Must run before anything publishes a snapshot: the store is a file-static and
 * the "nothing published yet" branch is only reachable once per process. */
static void test_webstatus_before_first_publish() {
    suite("webstatus: a read before the worker has ever published");
    WebSnapshot out;
    out.valid = true;                       // must be left untouched
    check_eq("read is refused", webstatus_read(out), 0);
    check_eq("and out is not written to", out.valid, 1);
}

int main() {
    test_webstatus_before_first_publish();
    test_util(); test_module(); test_shunt(); test_led();
    test_pack(); test_battery(); test_io(); test_bms();
    test_statemachine(); test_integration(); test_edge(); test_webstatus();
    test_deep(); test_matrix(); test_final();
    return unit_summary();
}
