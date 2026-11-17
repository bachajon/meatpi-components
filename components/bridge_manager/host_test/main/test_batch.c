/**
 * @file test_batch.c
 * @brief Host tests for a translated bridge's batch (bridge_manager_batch.c):
 *        the codec's outputs gathered into one endpoint send, none split,
 *        none sent twice.
 */
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "bridge_manager.h"
#include "bridge_manager_private.h"

/* ---- a destination that remembers what it was sent ------------------------------ */

#define SENDS_MAX 16

static char s_sent[SENDS_MAX][64];
static size_t s_sent_len[SENDS_MAX];
static int s_sends;
static bool s_refuse;

static esp_err_t fake_send(const uint8_t *data, size_t len)
{
    TEST_ASSERT_TRUE(s_sends < SENDS_MAX);
    TEST_ASSERT_TRUE(len <= sizeof(s_sent[0]));
    memcpy(s_sent[s_sends], data, len);
    s_sent_len[s_sends] = len;
    s_sends++;
    return s_refuse ? ESP_FAIL : ESP_OK;
}

static uint8_t s_store[16];
static uint32_t s_errors;

static bm_batch_t fresh_batch(void)
{
    bm_batch_t b =
    {
        .send = fake_send,
        .send_errors = &s_errors,
        .buf = s_store,
        .len = 0,
        .cap = sizeof(s_store),
    };

    s_sends = 0;
    s_refuse = false;
    s_errors = 0;
    return b;
}

static esp_err_t put(bm_batch_t *b, const char *text)
{
    return bm_batch_sink(b, (const uint8_t *)text, strlen(text));
}

/* ---- tests ------------------------------------------------------------------------ */

static void test_outputs_leave_as_one_send(void)
{
    bm_batch_t b = fresh_batch();

    bm_batch_flush(&b);                       /* nothing gathered: nothing sent */
    TEST_ASSERT_EQUAL(0, s_sends);

    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t1230\r"));
    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t4560\r"));
    TEST_ASSERT_EQUAL(0, s_sends);            /* still gathering */

    bm_batch_flush(&b);
    TEST_ASSERT_EQUAL(1, s_sends);
    TEST_ASSERT_EQUAL(12, s_sent_len[0]);
    TEST_ASSERT_EQUAL_MEMORY("t1230\rt4560\r", s_sent[0], 12);
    TEST_ASSERT_EQUAL(0, b.len);
}

static void test_an_output_that_does_not_fit_is_never_split(void)
{
    bm_batch_t b = fresh_batch();

    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t1230\r"));
    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t4560\r"));   /* 12 of 16 */

    /* six more do not fit behind twelve: the sink answers ESP_OK (the
       codec is never asked to run again), what was gathered leaves first,
       and the new line starts the next batch whole */
    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t7890\r"));
    TEST_ASSERT_EQUAL(1, s_sends);
    TEST_ASSERT_EQUAL_MEMORY("t1230\rt4560\r", s_sent[0], 12);
    TEST_ASSERT_EQUAL(6, b.len);

    bm_batch_flush(&b);
    TEST_ASSERT_EQUAL(2, s_sends);
    TEST_ASSERT_EQUAL(6, s_sent_len[1]);
    TEST_ASSERT_EQUAL_MEMORY("t7890\r", s_sent[1], 6);
}

static void test_an_output_larger_than_the_batch_goes_straight_through(void)
{
    bm_batch_t b = fresh_batch();
    const char *big = "T1234567880011223344556677\r";      /* 27 > 16 */

    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t1230\r"));
    TEST_ASSERT_EQUAL(ESP_OK, put(&b, big));

    /* in order: what waited, then the large one alone */
    TEST_ASSERT_EQUAL(2, s_sends);
    TEST_ASSERT_EQUAL_MEMORY("t1230\r", s_sent[0], 6);
    TEST_ASSERT_EQUAL(27, s_sent_len[1]);
    TEST_ASSERT_EQUAL_MEMORY(big, s_sent[1], 27);
    TEST_ASSERT_EQUAL(0, b.len);
}

static void test_every_output_reaches_the_destination_once_in_order(void)
{
    /* a codec call that emits many records while the batch fills up on
       the way: the case a second run of the codec would have doubled */
    bm_batch_t b = fresh_batch();
    char all[SENDS_MAX * 64];
    char want[256];
    size_t all_len = 0;
    size_t want_len = 0;

    for (int i = 0; i < 20; i++)
    {
        char rec[8];
        int n = snprintf(rec, sizeof(rec), "t%02d0\r", i);    /* 5 bytes each */

        TEST_ASSERT_EQUAL(ESP_OK, bm_batch_sink(&b, (const uint8_t *)rec, (size_t)n));
        memcpy(want + want_len, rec, (size_t)n);
        want_len += (size_t)n;
    }

    bm_batch_flush(&b);

    for (int i = 0; i < s_sends; i++)
    {
        TEST_ASSERT_EQUAL(0, s_sent_len[i] % 5);      /* whole records only */
        memcpy(all + all_len, s_sent[i], s_sent_len[i]);
        all_len += s_sent_len[i];
    }

    TEST_ASSERT_EQUAL(want_len, all_len);
    TEST_ASSERT_EQUAL_MEMORY(want, all, want_len);
    TEST_ASSERT_EQUAL(0, s_errors);
}

static void test_a_refusing_destination_is_counted_and_the_batch_moves_on(void)
{
    bm_batch_t b = fresh_batch();

    s_refuse = true;
    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t1230\r"));
    bm_batch_flush(&b);
    TEST_ASSERT_EQUAL(1, s_errors);
    TEST_ASSERT_EQUAL(0, b.len);              /* gone, as an unbatched send's */

    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "T1234567880011223344556677\r"));
    TEST_ASSERT_EQUAL(2, s_errors);           /* the straight-through path too */

    s_refuse = false;
    TEST_ASSERT_EQUAL(ESP_OK, put(&b, "t4560\r"));
    bm_batch_flush(&b);
    TEST_ASSERT_EQUAL(2, s_errors);
    TEST_ASSERT_EQUAL_MEMORY("t4560\r", s_sent[s_sends - 1], 6);
}

void run_batch_tests(void)
{
    RUN_TEST(test_outputs_leave_as_one_send);
    RUN_TEST(test_an_output_that_does_not_fit_is_never_split);
    RUN_TEST(test_an_output_larger_than_the_batch_goes_straight_through);
    RUN_TEST(test_every_output_reaches_the_destination_once_in_order);
    RUN_TEST(test_a_refusing_destination_is_counted_and_the_batch_moves_on);
}
