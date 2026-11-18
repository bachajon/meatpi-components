/**
 * @file test_backlog.c
 * @brief Host tests for tx_backlog's pure part: the ring (order around the
 *        end of its storage, a unit whole or not at all, the stall clock)
 *        and the pool. The first four came from socket_manager's suite
 *        with the code (2026-11-18).
 */
#include <string.h>

#include "unity.h"

#include "tx_backlog.h"

/* ---- the ring ---------------------------------------------------------------------- */

/* everything that waits, in order, the way the sender takes it: run by run */
static size_t backlog_drain(tx_backlog_t *b, char *out)
{
    const uint8_t *p = NULL;
    size_t total = 0;
    size_t n;

    while ((n = tx_backlog_peek(b, &p)) > 0)
    {
        memcpy(out + total, p, n);
        total += n;
        tx_backlog_took(b, n);
    }

    return total;
}

static void test_backlog_keeps_order_around_the_end(void)
{
    uint8_t store[16];
    char out[32];
    tx_backlog_t b = { .buf = store, .cap = sizeof(store) };
    const uint8_t *p = NULL;

    tx_backlog_clear(&b);
    TEST_ASSERT_EQUAL(0, tx_backlog_peek(&b, &p));
    TEST_ASSERT_NULL(p);

    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"abcdefgh", 8));
    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"ijkl", 4));
    TEST_ASSERT_EQUAL(12, tx_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_MEMORY("abcdefghijkl", p, 12);

    /* the socket took ten: two wait at 10 and 11. Fourteen more fill the
       storage: four behind them, ten around the end. Nothing is moved */
    tx_backlog_took(&b, 10);
    TEST_ASSERT_EQUAL(2, tx_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_PTR(store + 10, p);
    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"0123456789ABCD", 14));
    TEST_ASSERT_FALSE(tx_backlog_put(&b, (const uint8_t *)"x", 1)); /* full */

    /* the first run ends where the storage ends, the rest is the next run */
    TEST_ASSERT_EQUAL(6, tx_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_MEMORY("kl0123", p, 6);
    tx_backlog_took(&b, 4);                      /* a partial send */
    TEST_ASSERT_EQUAL(2, tx_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL(12, backlog_drain(&b, out));
    TEST_ASSERT_EQUAL_MEMORY("23456789ABCD", out, 12);

    /* everything left: empty again, the next bytes start at the front */
    TEST_ASSERT_EQUAL(0, tx_backlog_peek(&b, &p));
    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"xy", 2));
    TEST_ASSERT_EQUAL(2, tx_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_PTR(store, p);
}

static void test_backlog_many_laps_lose_and_repeat_nothing(void)
{
    /* a client that stays behind: bytes in and out for many laps of a
       storage whose size no chunk length divides */
    uint8_t store[37];
    char out[64];
    tx_backlog_t b = { .buf = store, .cap = sizeof(store) };
    uint8_t next_in = 0;
    uint8_t next_out = 0;

    tx_backlog_clear(&b);

    for (int round = 0; round < 500; round++)
    {
        uint8_t chunk[11];
        size_t len = 1 + (size_t)(round % 11);
        const uint8_t *p = NULL;

        for (size_t i = 0; i < len; i++)
        {
            chunk[i] = (uint8_t)(next_in + i);
        }

        if (tx_backlog_put(&b, chunk, len))
        {
            next_in = (uint8_t)(next_in + len);
        }

        /* the socket takes a few bytes of the first run */
        size_t n = tx_backlog_peek(&b, &p);
        size_t take = (n < (size_t)(1 + round % 7)) ? n : (size_t)(1 + round % 7);

        for (size_t i = 0; i < take; i++)
        {
            TEST_ASSERT_EQUAL_UINT8(next_out, p[i]);
            next_out++;
        }

        tx_backlog_took(&b, take);
    }

    size_t left = backlog_drain(&b, out);

    for (size_t i = 0; i < left; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(next_out, (uint8_t)out[i]);
        next_out++;
    }

    TEST_ASSERT_EQUAL_UINT8(next_in, next_out);
}

static void test_backlog_takes_a_chunk_whole_or_not_at_all(void)
{
    uint8_t store[8];
    tx_backlog_t b = { .buf = store, .cap = sizeof(store) };
    const uint8_t *p = NULL;

    tx_backlog_clear(&b);
    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"t1230\r", 6));

    /* three more do not fit: nothing of them is stored (a line is never
       cut), what waited is untouched */
    TEST_ASSERT_FALSE(tx_backlog_put(&b, (const uint8_t *)"t12", 3));
    TEST_ASSERT_EQUAL(6, tx_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_MEMORY("t1230\r", p, 6);
    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"ab", 2));
    TEST_ASSERT_FALSE(tx_backlog_put(&b, (const uint8_t *)"c", 1));

    /* a client that holds no storage (the pool is empty) stores nothing */
    tx_backlog_t none = { 0 };

    TEST_ASSERT_FALSE(tx_backlog_put(&none, (const uint8_t *)"a", 1));
    TEST_ASSERT_EQUAL(0, tx_backlog_peek(&none, &p));
}

static void test_backlog_stall_clock(void)
{
    tx_backlog_t b = { 0 };
    const int64_t T = 5000000; /* any moment after boot */

    tx_backlog_clear(&b);
    TEST_ASSERT_FALSE(tx_backlog_expired(&b, T, 10000));

    /* the socket refuses and nothing moves: the clock starts once */
    tx_backlog_attempt(&b, false, true, T);
    tx_backlog_attempt(&b, false, true, T + 4000000);
    TEST_ASSERT_FALSE(tx_backlog_expired(&b, T + 9999000, 10000));
    TEST_ASSERT_TRUE(tx_backlog_expired(&b, T + 10000000, 10000));

    /* one byte moved (a slow client is not a dead one): the clock stops,
       and a later stall counts from its own start */
    tx_backlog_attempt(&b, true, true, T + 9000000);
    TEST_ASSERT_FALSE(tx_backlog_expired(&b, T + 30000000, 10000));
    tx_backlog_attempt(&b, false, true, T + 30000000);
    TEST_ASSERT_FALSE(tx_backlog_expired(&b, T + 39000000, 10000));
    TEST_ASSERT_TRUE(tx_backlog_expired(&b, T + 40000000, 10000));

    /* a send the socket took whole also stops it */
    tx_backlog_attempt(&b, false, false, T + 41000000);
    TEST_ASSERT_FALSE(tx_backlog_expired(&b, T + 90000000, 10000));

    /* a new client in the slot starts clean */
    tx_backlog_attempt(&b, false, true, T);
    b.dropped = 3;
    tx_backlog_clear(&b);
    TEST_ASSERT_FALSE(tx_backlog_expired(&b, T + 90000000, 10000));
    TEST_ASSERT_EQUAL(0, b.dropped);
}

/* a frame's header and its payload go in together or not at all */
static void test_backlog_put2_is_one_unit(void)
{
    uint8_t store[16];
    tx_backlog_t b = { .buf = store, .cap = sizeof(store) };
    const uint8_t *p;
    char out[32];

    tx_backlog_clear(&b);
    TEST_ASSERT_TRUE(tx_backlog_put2(&b, (const uint8_t *)"\x82\x05", 2,
                                     (const uint8_t *)"hello", 5));
    TEST_ASSERT_EQUAL(7, tx_backlog_peek(&b, &p));

    /* 9 bytes are free: a header that fits with a payload that does not
       leaves nothing behind, not even the header */
    TEST_ASSERT_FALSE(tx_backlog_put2(&b, (const uint8_t *)"\x82\x0A", 2,
                                      (const uint8_t *)"0123456789", 10));
    TEST_ASSERT_EQUAL(7, tx_backlog_peek(&b, &p));

    /* either part may be empty */
    TEST_ASSERT_TRUE(tx_backlog_put2(&b, NULL, 0, (const uint8_t *)"ab", 2));
    TEST_ASSERT_TRUE(tx_backlog_put2(&b, (const uint8_t *)"cd", 2, NULL, 0));
    TEST_ASSERT_TRUE(tx_backlog_put2(&b, NULL, 0, NULL, 0));
    TEST_ASSERT_EQUAL(11, backlog_drain(&b, out));
    TEST_ASSERT_EQUAL_MEMORY("\x82\x05helloabcd", out, 11);

    /* around the end of the storage the two parts still follow each other */
    TEST_ASSERT_TRUE(tx_backlog_put(&b, (const uint8_t *)"0123456789AB", 12));
    tx_backlog_took(&b, 10);               /* two bytes wait, at 10 and 11 */
    TEST_ASSERT_TRUE(tx_backlog_put2(&b, (const uint8_t *)"HEAD", 4,
                                     (const uint8_t *)"payload!", 8));
    TEST_ASSERT_EQUAL(14, backlog_drain(&b, out));
    TEST_ASSERT_EQUAL_MEMORY("ABHEADpayload!", out, 14);

    /* no storage, nothing stored */
    tx_backlog_t none = { 0 };

    TEST_ASSERT_FALSE(tx_backlog_put2(&none, (const uint8_t *)"a", 1, NULL, 0));
}

/* ---- the pool ---------------------------------------------------------------------- */

/* The pace: units that keep coming sooner than the gap after the last send
   are a stream and wait for the owner's push; a unit after a pause, and the
   few that follow it, leave at once. */
static void test_pace_holds_a_stream_not_an_exchange(void)
{
    tx_backlog_t b;
    int64_t t = 1000000;

    memset(&b, 0, sizeof(b));
    tx_backlog_clear(&b);

    /* the first unit ever: no unit before it, it leaves */
    TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, t, 3000, 4));
    b.sent_us = t;

    /* an answer in four more pieces, 200 us apart: none is held */
    for (int i = 0; i < 4; i++)
    {
        t += 200;
        TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, t, 3000, 4));
        b.sent_us = t;
    }

    /* it goes on, a unit every 200 us: a stream. Each one waits while
       the gap since the last send is not over, and the one that comes
       when it is leaves (and takes the others with it): one send a gap,
       15 units to a send, for as long as the stream lasts */
    int sends = 0;
    int held = 0;

    for (int i = 0; i < 150; i++)
    {
        t += 200;

        if (tx_backlog_pace_hold(&b, t, 3000, 4))
        {
            held++;
        }
        else
        {
            sends++;
            b.sent_us = t;
        }
    }

    TEST_ASSERT_EQUAL(10, sends);
    TEST_ASSERT_EQUAL(140, held);

    /* the same when the owner's push did the sending in between: a unit
       right after it waits */
    b.sent_us = t;
    TEST_ASSERT_TRUE(tx_backlog_pace_hold(&b, t + 100, 3000, 4));
    t += 100;

    /* a late send does not end the stream: units kept coming, so the
       unit that follows a send is held again, not let through as the
       first of a new exchange */
    t += 200;
    b.sent_us = t - 5000;
    TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, t, 3000, 4)); /* the gap is over: it goes */
    b.sent_us = t;
    TEST_ASSERT_TRUE(tx_backlog_pace_hold(&b, t + 200, 3000, 4));
    t += 200;

    /* a pause of the gap with no unit: the next one leaves at once, and
       the count starts again */
    t += 3000;
    TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, t, 3000, 4));
    TEST_ASSERT_EQUAL(0, b.run);
    b.sent_us = t;
    TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, t + 10, 3000, 4));
    TEST_ASSERT_EQUAL(1, b.run);
}

static void test_pace_due_and_no_pace(void)
{
    tx_backlog_t b;

    memset(&b, 0, sizeof(b));
    tx_backlog_clear(&b);
    b.sent_us = 5000000;

    TEST_ASSERT_FALSE(tx_backlog_pace_due(&b, 5002999, 3000));
    TEST_ASSERT_TRUE(tx_backlog_pace_due(&b, 5003000, 3000));

    /* no pace: nothing is ever held, what waits is always due */
    b.unit_us = 5000000;
    TEST_ASSERT_TRUE(tx_backlog_pace_due(&b, 5000001, 0));

    for (int i = 0; i < 20; i++)
    {
        TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, 5000001 + i, 0, 4));
    }

    /* a unit that must not wait: the hold is off for it */
    b.sent_us = 7000000;
    b.unit_us = 7000000;
    b.run = 4;
    TEST_ASSERT_TRUE(tx_backlog_pace_hold(&b, 7000100, 3000, 4));
    tx_backlog_pace_now(&b);
    TEST_ASSERT_FALSE(tx_backlog_pace_hold(&b, 7000100, 3000, 4));
    TEST_ASSERT_TRUE(tx_backlog_pace_due(&b, 7000100, 3000));

    /* clear forgets the last send and the run */
    b.run = 3;
    tx_backlog_clear(&b);
    TEST_ASSERT_EQUAL(0, b.run);
    TEST_ASSERT_EQUAL(0, (int)b.sent_us);
}

static void test_pool_lends_each_buffer_once(void)
{
    static uint8_t mem[3 * 8];
    tx_backlog_pool_t pool = { .mem = mem, .buf_bytes = 8, .count = 3 };
    tx_backlog_t b[4] = { 0 };

    for (int i = 0; i < 4; i++)
    {
        tx_backlog_lend(&pool, &b[i]);
    }

    TEST_ASSERT_EQUAL_PTR(mem, b[0].buf);
    TEST_ASSERT_EQUAL_PTR(mem + 8, b[1].buf);
    TEST_ASSERT_EQUAL_PTR(mem + 16, b[2].buf);
    TEST_ASSERT_EQUAL(8, b[2].cap);
    TEST_ASSERT_NULL(b[3].buf);            /* the pool is empty: nothing lent */
    TEST_ASSERT_FALSE(tx_backlog_put(&b[3], (const uint8_t *)"x", 1));

    /* a backlog that holds one is not given a second */
    tx_backlog_lend(&pool, &b[1]);
    TEST_ASSERT_EQUAL_PTR(mem + 8, b[1].buf);
}

static void test_pool_takes_a_buffer_back_and_lends_it_again(void)
{
    static uint8_t mem[2 * 8];
    tx_backlog_pool_t pool = { .mem = mem, .buf_bytes = 8, .count = 2 };
    tx_backlog_t a = { 0 };
    tx_backlog_t b = { 0 };
    tx_backlog_t c = { 0 };
    const uint8_t *p;

    tx_backlog_lend(&pool, &a);
    tx_backlog_lend(&pool, &b);
    TEST_ASSERT_TRUE(tx_backlog_put(&a, (const uint8_t *)"abc", 3));

    tx_backlog_return(&pool, &a);
    TEST_ASSERT_NULL(a.buf);
    TEST_ASSERT_EQUAL(0, a.cap);
    TEST_ASSERT_EQUAL(0, tx_backlog_peek(&a, &p)); /* what waited is gone with it */

    tx_backlog_lend(&pool, &c);
    TEST_ASSERT_EQUAL_PTR(mem, c.buf);     /* the one that came back */
    TEST_ASSERT_EQUAL(0, tx_backlog_peek(&c, &p));

    /* returning twice, or what was never lent, changes nothing */
    tx_backlog_return(&pool, &a);
    tx_backlog_lend(&pool, &a);
    TEST_ASSERT_NULL(a.buf);
    TEST_ASSERT_EQUAL_PTR(mem + 8, b.buf);
}

static void test_pool_without_storage_lends_nothing(void)
{
    tx_backlog_pool_t pool = { 0 };
    tx_backlog_t b = { 0 };

    tx_backlog_lend(&pool, &b);
    TEST_ASSERT_NULL(b.buf);
    tx_backlog_lend(NULL, &b);
    TEST_ASSERT_NULL(b.buf);
    tx_backlog_return(&pool, &b);
    tx_backlog_return(NULL, &b);
    TEST_ASSERT_NULL(b.buf);
}

void run_backlog_tests(void)
{
    RUN_TEST(test_backlog_keeps_order_around_the_end);
    RUN_TEST(test_backlog_many_laps_lose_and_repeat_nothing);
    RUN_TEST(test_backlog_takes_a_chunk_whole_or_not_at_all);
    RUN_TEST(test_backlog_stall_clock);
    RUN_TEST(test_backlog_put2_is_one_unit);
    RUN_TEST(test_pace_holds_a_stream_not_an_exchange);
    RUN_TEST(test_pace_due_and_no_pace);
    RUN_TEST(test_pool_lends_each_buffer_once);
    RUN_TEST(test_pool_takes_a_buffer_back_and_lends_it_again);
    RUN_TEST(test_pool_without_storage_lends_nothing);
}
