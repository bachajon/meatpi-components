/**
 * @file test_policy.c
 * @brief Host tests for socket_manager's pure policy layer: backoff
 *        progression, accept decisions, config parse + cross-item rules.
 */
#include <string.h>

#include "unity.h"

#include "socket_manager.h"
#include "socket_manager_private.h"

/* ---- backoff ------------------------------------------------------------------ */

static void test_backoff_progression_and_cap(void)
{
    uint32_t ms = smp_backoff_next_ms(0);

    TEST_ASSERT_EQUAL(1000, ms);
    ms = smp_backoff_next_ms(ms);
    TEST_ASSERT_EQUAL(2000, ms);
    ms = smp_backoff_next_ms(ms);
    TEST_ASSERT_EQUAL(4000, ms);
    ms = smp_backoff_next_ms(ms);
    TEST_ASSERT_EQUAL(8000, ms);
    ms = smp_backoff_next_ms(ms);
    TEST_ASSERT_EQUAL(8000, ms); /* capped */
}

/* ---- accept decision ------------------------------------------------------------ */

static void test_accept_until_max_then_reject(void)
{
    TEST_ASSERT_EQUAL(SMP_ACCEPT, smp_accept_decision(0, 2));
    TEST_ASSERT_EQUAL(SMP_ACCEPT, smp_accept_decision(1, 2));
    TEST_ASSERT_EQUAL(SMP_REJECT, smp_accept_decision(2, 2));
    TEST_ASSERT_EQUAL(SMP_REJECT, smp_accept_decision(3, 2));
}

/* ---- parse -------------------------------------------------------------------- */

static void test_parse_fills_code_defaults(void)
{
    cJSON *item = cJSON_Parse(
        "{\"name\":\"obd0\",\"proto\":\"tcp\",\"port\":35000}");
    smp_server_cfg_t cfg;

    TEST_ASSERT_EQUAL(ESP_OK, smp_parse_server(item, &cfg));
    TEST_ASSERT_EQUAL_STRING("obd0", cfg.name);
    TEST_ASSERT_FALSE(cfg.is_udp);
    TEST_ASSERT_EQUAL(35000, cfg.port);
    TEST_ASSERT_EQUAL(2, cfg.max_clients);   /* code default */
    TEST_ASSERT_EQUAL(30, cfg.keepalive_s);  /* code default */
    TEST_ASSERT_FALSE(cfg.enabled);          /* code default */
    cJSON_Delete(item);
}

static void test_parse_rejects_bad_name(void)
{
    cJSON *item = cJSON_Parse(
        "{\"name\":\"OBD-0!\",\"proto\":\"tcp\",\"port\":1}");
    smp_server_cfg_t cfg;

    TEST_ASSERT_NOT_EQUAL(ESP_OK, smp_parse_server(item, &cfg));
    cJSON_Delete(item);
}

/* ---- cross-item validation ------------------------------------------------------- */

static esp_err_t validate(const char *json, char *err, size_t err_len)
{
    cJSON *arr = cJSON_Parse(json);
    esp_err_t r = smp_validate_servers(arr, err, err_len);

    cJSON_Delete(arr);
    return r;
}

static void test_valid_server_set_accepted(void)
{
    char err[96] = "";

    TEST_ASSERT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"obd0\",\"proto\":\"tcp\",\"port\":35000,"
          "\"enabled\":true},"
         "{\"name\":\"udp0\",\"proto\":\"udp\",\"port\":17,"
          "\"enabled\":true}]", err, sizeof(err)));
}

static void test_duplicate_name_rejected(void)
{
    char err[96] = "";

    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":1},"
         "{\"name\":\"a\",\"proto\":\"udp\",\"port\":2}]",
        err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "duplicate"));
}

static void test_enabled_port_clash_rejected(void)
{
    char err[96] = "";

    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":23,\"enabled\":true},"
         "{\"name\":\"b\",\"proto\":\"tcp\",\"port\":23,\"enabled\":true}]",
        err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "port"));
}

static void test_disabled_port_park_allowed(void)
{
    char err[96] = "";

    /* a disabled server may park the same port an enabled one uses */
    TEST_ASSERT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":23,\"enabled\":true},"
         "{\"name\":\"b\",\"proto\":\"tcp\",\"port\":23,\"enabled\":false}]",
        err, sizeof(err)));
}

static void test_max_clients_bounds_enforced(void)
{
    char err[96] = "";

    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":1,"
          "\"max_clients\":9}]", err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "max_clients"));
}

static void test_httpd_port_rejected(void)
{
    char err[96] = "";

    /* :80 = httpd; lwip SO_REUSEADDR would let both LISTEN and split
       the SYNs nondeterministically (live find, 2026-07-26) */
    TEST_ASSERT_NOT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":80,"
          "\"enabled\":true}]", err, sizeof(err)));
    TEST_ASSERT_NOT_NULL(strstr(err, "web server"));

    /* disabled park + udp:80 both stay legal */
    TEST_ASSERT_EQUAL(ESP_OK, validate(
        "[{\"name\":\"a\",\"proto\":\"tcp\",\"port\":80,"
          "\"enabled\":false},"
         "{\"name\":\"b\",\"proto\":\"udp\",\"port\":80,"
          "\"enabled\":true}]", err, sizeof(err)));
}

/* ---- a client's backlog --------------------------------------------------------- */

/* everything that waits, in order, the way the sender takes it: run by run */
static size_t backlog_drain(smp_backlog_t *b, char *out)
{
    const uint8_t *p = NULL;
    size_t total = 0;
    size_t n;

    while ((n = smp_backlog_peek(b, &p)) > 0)
    {
        memcpy(out + total, p, n);
        total += n;
        smp_backlog_took(b, n);
    }

    return total;
}

static void test_backlog_keeps_order_around_the_end(void)
{
    uint8_t store[16];
    char out[32];
    smp_backlog_t b = { .buf = store, .cap = sizeof(store) };
    const uint8_t *p = NULL;

    smp_backlog_clear(&b);
    TEST_ASSERT_EQUAL(0, smp_backlog_peek(&b, &p));
    TEST_ASSERT_NULL(p);

    TEST_ASSERT_TRUE(smp_backlog_put(&b, (const uint8_t *)"abcdefgh", 8));
    TEST_ASSERT_TRUE(smp_backlog_put(&b, (const uint8_t *)"ijkl", 4));
    TEST_ASSERT_EQUAL(12, smp_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_MEMORY("abcdefghijkl", p, 12);

    /* the socket took ten: two wait at 10 and 11. Fourteen more fill the
       storage: four behind them, ten around the end. Nothing is moved */
    smp_backlog_took(&b, 10);
    TEST_ASSERT_EQUAL(2, smp_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_PTR(store + 10, p);
    TEST_ASSERT_TRUE(smp_backlog_put(&b, (const uint8_t *)"0123456789ABCD", 14));
    TEST_ASSERT_FALSE(smp_backlog_put(&b, (const uint8_t *)"x", 1)); /* full */

    /* the first run ends where the storage ends, the rest is the next run */
    TEST_ASSERT_EQUAL(6, smp_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_MEMORY("kl0123", p, 6);
    smp_backlog_took(&b, 4);                      /* a partial send */
    TEST_ASSERT_EQUAL(2, smp_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL(12, backlog_drain(&b, out));
    TEST_ASSERT_EQUAL_MEMORY("23456789ABCD", out, 12);

    /* everything left: empty again, the next bytes start at the front */
    TEST_ASSERT_EQUAL(0, smp_backlog_peek(&b, &p));
    TEST_ASSERT_TRUE(smp_backlog_put(&b, (const uint8_t *)"xy", 2));
    TEST_ASSERT_EQUAL(2, smp_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_PTR(store, p);
}

static void test_backlog_many_laps_lose_and_repeat_nothing(void)
{
    /* a client that stays behind: bytes in and out for many laps of a
       storage whose size no chunk length divides */
    uint8_t store[37];
    char out[64];
    smp_backlog_t b = { .buf = store, .cap = sizeof(store) };
    uint8_t next_in = 0;
    uint8_t next_out = 0;

    smp_backlog_clear(&b);

    for (int round = 0; round < 500; round++)
    {
        uint8_t chunk[11];
        size_t len = 1 + (size_t)(round % 11);
        const uint8_t *p = NULL;

        for (size_t i = 0; i < len; i++)
        {
            chunk[i] = (uint8_t)(next_in + i);
        }

        if (smp_backlog_put(&b, chunk, len))
        {
            next_in = (uint8_t)(next_in + len);
        }

        /* the socket takes a few bytes of the first run */
        size_t n = smp_backlog_peek(&b, &p);
        size_t take = (n < (size_t)(1 + round % 7)) ? n : (size_t)(1 + round % 7);

        for (size_t i = 0; i < take; i++)
        {
            TEST_ASSERT_EQUAL_UINT8(next_out, p[i]);
            next_out++;
        }

        smp_backlog_took(&b, take);
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
    smp_backlog_t b = { .buf = store, .cap = sizeof(store) };
    const uint8_t *p = NULL;

    smp_backlog_clear(&b);
    TEST_ASSERT_TRUE(smp_backlog_put(&b, (const uint8_t *)"t1230\r", 6));

    /* three more do not fit: nothing of them is stored (a line is never
       cut), what waited is untouched */
    TEST_ASSERT_FALSE(smp_backlog_put(&b, (const uint8_t *)"t12", 3));
    TEST_ASSERT_EQUAL(6, smp_backlog_peek(&b, &p));
    TEST_ASSERT_EQUAL_MEMORY("t1230\r", p, 6);
    TEST_ASSERT_TRUE(smp_backlog_put(&b, (const uint8_t *)"ab", 2));
    TEST_ASSERT_FALSE(smp_backlog_put(&b, (const uint8_t *)"c", 1));

    /* a client that holds no storage (the pool is empty) stores nothing */
    smp_backlog_t none = { 0 };

    TEST_ASSERT_FALSE(smp_backlog_put(&none, (const uint8_t *)"a", 1));
    TEST_ASSERT_EQUAL(0, smp_backlog_peek(&none, &p));
}

static void test_backlog_stall_clock(void)
{
    smp_backlog_t b = { 0 };
    const int64_t T = 5000000; /* any moment after boot */

    smp_backlog_clear(&b);
    TEST_ASSERT_FALSE(smp_backlog_expired(&b, T, 10000));

    /* the socket refuses and nothing moves: the clock starts once */
    smp_backlog_attempt(&b, false, true, T);
    smp_backlog_attempt(&b, false, true, T + 4000000);
    TEST_ASSERT_FALSE(smp_backlog_expired(&b, T + 9999000, 10000));
    TEST_ASSERT_TRUE(smp_backlog_expired(&b, T + 10000000, 10000));

    /* one byte moved (a slow client is not a dead one): the clock stops,
       and a later stall counts from its own start */
    smp_backlog_attempt(&b, true, true, T + 9000000);
    TEST_ASSERT_FALSE(smp_backlog_expired(&b, T + 30000000, 10000));
    smp_backlog_attempt(&b, false, true, T + 30000000);
    TEST_ASSERT_FALSE(smp_backlog_expired(&b, T + 39000000, 10000));
    TEST_ASSERT_TRUE(smp_backlog_expired(&b, T + 40000000, 10000));

    /* a send the socket took whole also stops it */
    smp_backlog_attempt(&b, false, false, T + 41000000);
    TEST_ASSERT_FALSE(smp_backlog_expired(&b, T + 90000000, 10000));

    /* a new client in the slot starts clean */
    smp_backlog_attempt(&b, false, true, T);
    b.dropped = 3;
    smp_backlog_clear(&b);
    TEST_ASSERT_FALSE(smp_backlog_expired(&b, T + 90000000, 10000));
    TEST_ASSERT_EQUAL(0, b.dropped);
}

void run_policy_tests(void)
{
    RUN_TEST(test_backlog_keeps_order_around_the_end);
    RUN_TEST(test_backlog_many_laps_lose_and_repeat_nothing);
    RUN_TEST(test_backlog_takes_a_chunk_whole_or_not_at_all);
    RUN_TEST(test_backlog_stall_clock);
    RUN_TEST(test_backoff_progression_and_cap);
    RUN_TEST(test_accept_until_max_then_reject);
    RUN_TEST(test_parse_fills_code_defaults);
    RUN_TEST(test_parse_rejects_bad_name);
    RUN_TEST(test_valid_server_set_accepted);
    RUN_TEST(test_duplicate_name_rejected);
    RUN_TEST(test_enabled_port_clash_rejected);
    RUN_TEST(test_disabled_port_park_allowed);
    RUN_TEST(test_max_clients_bounds_enforced);
    RUN_TEST(test_httpd_port_rejected);
}
