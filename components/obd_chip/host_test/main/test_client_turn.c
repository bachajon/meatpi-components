/* How the chip changes hands between an app and the poller
   (obd_chip_client.c, 2026-11-25, GitHub issue #1078): the connection that
   wrote last and has gone, the poller's turn an app's bytes wait for, and
   what an app is not shown of the firmware's own traffic. The tests drive
   the clock themselves (microseconds). */
#include <string.h>

#include "unity.h"

#include "obd_chip.h"
#include "obd_chip_private.h"

#define S(x) ((int64_t)(x) * 1000000) /* seconds -> microseconds */
#define YIELD_MS 10000                /* the poller's yield time */

static const char TCP[1], WS[1]; /* two connections: addresses only */

/* an app's bytes with the chip free: noted, and out at once */
static void app_from(const void *who, const char *text, int64_t now)
{
    obd_client_write_from(who, (const uint8_t *)text, strlen(text), now);
    obd_client_sent((const uint8_t *)text, strlen(text));
}

static void chip(const char *text)
{
    obd_client_rx((const uint8_t *)text, strlen(text));
}

/* the app that opened the monitor leaves: the hold is over, the idle time
   runs from its last byte, and the poller has the chip back at ten seconds */
void test_client_gone_ends_the_monitor_of_the_one_that_wrote_last(void)
{
    obd_chip_client_stats_t st;

    obd_client_test_reset();
    app_from(TCP, "ATMA\r", S(100));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(104)));

    TEST_ASSERT_TRUE(obd_client_gone(TCP));
    TEST_ASSERT_EQUAL_UINT32(4000, obd_client_idle_ms(S(104)));
    TEST_ASSERT_EQUAL_UINT32(10000, obd_client_idle_ms(S(110)));

    /* told once: a second report of the same connection finds nothing */
    TEST_ASSERT_FALSE(obd_client_gone(TCP));
    obd_client_get_stats(&st);
    TEST_ASSERT_EQUAL_UINT32(1, st.gone);
}

/* another connection leaving is nobody's business: the web terminal's tab
   is closed while an app on TCP sniffs the bus */
void test_client_gone_of_another_connection_changes_nothing(void)
{
    obd_chip_client_stats_t st;

    obd_client_test_reset();
    app_from(WS, "ATI\r", S(90));
    chip("ELM327 v2.3\r\r>");
    app_from(TCP, "ATMA\r", S(100));

    TEST_ASSERT_FALSE(obd_client_gone(WS));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(150)));

    /* a connection that was not named (a translated bridge) is never the
       one, and neither is "nobody" */
    TEST_ASSERT_FALSE(obd_client_gone(NULL));
    obd_client_test_reset();
    app_from(NULL, "ATMA\r", S(100));
    TEST_ASSERT_FALSE(obd_client_gone(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(150)));
    obd_client_get_stats(&st);
    TEST_ASSERT_EQUAL_UINT32(0, st.gone);
}

/* a connection that leaves with no monitor open: nothing ends, the clock
   stays where its last byte put it; and its bare CR is not remembered for
   whoever comes next */
void test_client_gone_without_a_monitor(void)
{
    obd_client_test_reset();
    app_from(TCP, "ATMA\r", S(100));
    app_from(TCP, " ", S(105));
    chip("STOPPED\r\r>");
    TEST_ASSERT_FALSE(obd_client_gone(TCP));
    TEST_ASSERT_EQUAL_UINT32(3000, obd_client_idle_ms(S(108)));

    app_from(WS, "\r", S(120)); /* repeats the chip's last line: not ours to hold */
    TEST_ASSERT_EQUAL_UINT32(5000, obd_client_idle_ms(S(125)));
}

/* the poller's turn: refused while an app drives the chip, by its last
   byte or by its monitor; granted when the app has been silent */
void test_client_turn_is_refused_while_an_app_drives_the_chip(void)
{
    obd_client_test_reset();
    TEST_ASSERT_TRUE(obd_client_turn_begin(S(1), YIELD_MS)); /* no app ever */
    TEST_ASSERT_TRUE(obd_client_turn());
    obd_client_turn_end();
    TEST_ASSERT_FALSE(obd_client_turn());

    app_from(TCP, "ATI\r", S(10));
    chip("ELM327 v2.3\r\r>");
    TEST_ASSERT_FALSE(obd_client_turn_begin(S(15), YIELD_MS));
    TEST_ASSERT_FALSE(obd_client_turn());
    TEST_ASSERT_FALSE(obd_client_turn_begin(S(20) - 1000, YIELD_MS));
    TEST_ASSERT_TRUE(obd_client_turn_begin(S(20), YIELD_MS));
    obd_client_turn_end();

    app_from(TCP, "ATMA\r", S(30));
    TEST_ASSERT_FALSE(obd_client_turn_begin(S(100), YIELD_MS)); /* its monitor */
    TEST_ASSERT_TRUE(obd_client_gone(TCP));
    TEST_ASSERT_TRUE(obd_client_turn_begin(S(100), YIELD_MS));
    obd_client_turn_end();
}

/* an app's bytes and a turn: the write that comes during a turn sees it
   (and waits), and stamps the clock, so the pass behind it gets no turn */
void test_client_a_write_during_a_turn_sees_it_and_keeps_the_next_one_off(void)
{
    obd_chip_client_stats_t st;

    obd_client_test_reset();
    TEST_ASSERT_TRUE(obd_client_turn_begin(S(50), YIELD_MS));

    /* 0.3 s into the poller's row: the bytes are noted and wait */
    obd_client_write_from(TCP, (const uint8_t *)"ATZ\r", 4, S(50) + 300000);
    TEST_ASSERT_TRUE(obd_client_turn());

    /* what the chip says meanwhile is still the poller's: the rest of its
       row is not the app's to see (bench 2026-11-25: the prompt of the
       poller's last answer came out in front of the app's banner) */
    TEST_ASSERT_TRUE(obd_client_mute(true));
    chip("OK\r\r>");
    TEST_ASSERT_TRUE(obd_client_mute(true));

    obd_client_turn_end();                      /* the row is over */
    TEST_ASSERT_FALSE(obd_client_turn());
    obd_client_turn_waited(420, false);
    obd_client_sent((const uint8_t *)"ATZ\r", 4); /* now they go out */
    TEST_ASSERT_FALSE(obd_client_mute(true));   /* its answer is its own */
    chip("ELM327 v2.3\r\r>");
    TEST_ASSERT_TRUE(obd_client_mute(true));

    /* the poller's next pass: an app is there */
    TEST_ASSERT_FALSE(obd_client_turn_begin(S(51), YIELD_MS));

    obd_client_turn_waited(3000, true); /* one that gave up */
    obd_client_get_stats(&st);
    TEST_ASSERT_EQUAL_UINT32(2, st.turn_waits);
    TEST_ASSERT_EQUAL_UINT32(3000, st.turn_wait_max_ms);
    TEST_ASSERT_EQUAL_UINT32(1, st.turn_forced);
}

/* what an app is not shown: the chip's answers while a claim is held, the
   poller's lines for a connected and silent app */
void test_client_mute_keeps_the_firmwares_answers_from_an_app(void)
{
    obd_client_test_reset();
    TEST_ASSERT_FALSE(obd_client_mute(false)); /* nobody asked: the app's */
    TEST_ASSERT_TRUE(obd_client_mute(true));   /* the poller's request   */

    app_from(TCP, "ATI\r", S(1));
    TEST_ASSERT_FALSE(obd_client_mute(false)); /* its answer, no claim   */
    chip("ELM327 v2.3\r\r>");

    /* ten seconds on the poller is back: its answers are not the app's */
    TEST_ASSERT_TRUE(obd_client_mute(true));
}

/* the exception: a command of the app is under way beside a request of
   the firmware's (the app's wait was given up, or a tool began while the
   app's command ran). The app sees what became of it, up to the prompt,
   and nothing behind it */
void test_client_mute_not_while_an_apps_own_command_is_under_way(void)
{
    obd_client_test_reset();
    app_from(TCP, "0100\r", S(1));
    TEST_ASSERT_FALSE(obd_client_mute(true)); /* asked ahead of the chunk */
    chip("STOPPED\r\r>");                     /* the prompt ends it      */
    TEST_ASSERT_TRUE(obd_client_mute(true));

    /* a byte that submits nothing (a stop byte, half a line) is no command */
    app_from(TCP, " ", S(2));
    TEST_ASSERT_TRUE(obd_client_mute(true));
    app_from(TCP, "AT", S(3));
    TEST_ASSERT_TRUE(obd_client_mute(true));
    app_from(TCP, "I\r", S(3));
    TEST_ASSERT_FALSE(obd_client_mute(true));

    /* a monitor the app left at its cap: the poller's first word stops
       it, and that STOPPED is the app's to see */
    obd_client_test_reset();
    app_from(TCP, "ATMA\r", S(10));
    TEST_ASSERT_FALSE(obd_client_mute(true));
    chip("STOPPED\r\r>");
    TEST_ASSERT_TRUE(obd_client_mute(true));
}
