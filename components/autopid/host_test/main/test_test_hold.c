/**
 * @file test_test_hold.c
 * @brief The test route and an app's hold on the chip (GitHub issue #1078):
 *        a test that takes the chip waits as the poller and can.send do,
 *        a J1939 group's test does not, and the sentence is can.send's.
 */
#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "autopid_private.h"

static void test_test_route_waits_for_an_app(void)
{
    /* an app's monitor runs: obd_chip reports 0 ms idle (car 2026-11-24:
       a filter test STOPPED the app's ATMA) */
    TEST_ASSERT_NOT_NULL(ap_sched_test_refusal(0, true));

    /* the app's last command, inside and at the end of the 10 s */
    TEST_ASSERT_NOT_NULL(ap_sched_test_refusal(AP_CLIENT_YIELD_MS - 1, true));
    TEST_ASSERT_NULL(ap_sched_test_refusal(AP_CLIENT_YIELD_MS, true));

    /* no app ever */
    TEST_ASSERT_NULL(ap_sched_test_refusal(UINT32_MAX, true));

    /* a J1939 group's test reads the listener's store: no chip, no wait */
    TEST_ASSERT_NULL(ap_sched_test_refusal(0, false));
    TEST_ASSERT_NULL(ap_sched_test_refusal(UINT32_MAX, false));

    /* the poller's own rule, the same window */
    for (uint32_t ms = 0; ms <= AP_CLIENT_YIELD_MS + 1; ms += 2500)
    {
        TEST_ASSERT_EQUAL(ap_sched_client_hold(ms),
                          ap_sched_test_refusal(ms, true) != NULL);
    }
}

static void test_test_route_sentence(void)
{
    const char *s = ap_sched_test_refusal(0, true);

    /* the words can.send's 409 starts with, and how the app lets go */
    TEST_ASSERT_EQUAL_STRING(AP_CLIENT_BUSY_TEXT, s);
    TEST_ASSERT_NOT_NULL(strstr(s, "an app is using the OBD chip"));
    TEST_ASSERT_NOT_NULL(strstr(s, "10 s after its last command"));
    TEST_ASSERT_NOT_NULL(strstr(s, "monitor it opened ends"));

    /* can.send's own tail is not the test route's */
    TEST_ASSERT_NULL(strstr(s, "Bus can"));
}

void run_test_hold_tests(void)
{
    RUN_TEST(test_test_route_waits_for_an_app);
    RUN_TEST(test_test_route_sentence);
}
