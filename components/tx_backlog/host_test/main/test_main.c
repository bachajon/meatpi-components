/** Unity runner for the tx_backlog host suite (the ring and the pool). */
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

extern void run_backlog_tests(void);

void app_main(void)
{
    UNITY_BEGIN();
    run_backlog_tests();
    UNITY_END();
}
