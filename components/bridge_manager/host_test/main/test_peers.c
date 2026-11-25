/* The peers at the far end of a bridge (bridge_manager_peers.c, 2026-11-25,
   GitHub issue #1078): an endpoint that counts who is connected to it, one
   that wants to know where its bytes come from, and the report that the
   far end has nobody left. Stub endpoints, no FreeRTOS. */
#include <string.h>

#include "unity.h"

#include "bridge_manager.h"
#include "bridge_manager_peers.h"

/* ---- stubs -------------------------------------------------------------------------- */

static int s_clients;        /* what the counting endpoint reports        */
static int s_plain_sends;    /* send() calls                              */
static int s_from_sends;     /* send_from() calls                         */
static const void *s_from;   /* the last send_from's source               */
static int s_gone;           /* far_gone() calls                          */
static const void *s_gone_from;

static esp_err_t plain_send(const uint8_t *d, size_t l)
{
    (void)d;
    (void)l;
    s_plain_sends++;
    return ESP_OK;
}

static esp_err_t from_send(const void *from, const uint8_t *d, size_t l)
{
    (void)d;
    (void)l;
    s_from_sends++;
    s_from = from;
    return ESP_ERR_INVALID_STATE; /* its own result comes back */
}

static int count_peers(void)
{
    return s_clients;
}

static void told_gone(const void *from)
{
    s_gone++;
    s_gone_from = from;
}

/* a TCP server: counts its clients */
static const bridge_endpoint_t TCP =
{
    .name = "obd0", .send = plain_send, .peers = count_peers,
};
/* a UART: cannot tell */
static const bridge_endpoint_t UART = { .name = "usb_obd", .send = plain_send };
/* the chip's jack: wants the source and the report */
static const bridge_endpoint_t CHIP =
{
    .name = "obd", .send = plain_send, .send_from = from_send,
    .far_gone = told_gone,
};

static void fresh(void)
{
    s_clients = 0;
    s_plain_sends = 0;
    s_from_sends = 0;
    s_from = NULL;
    s_gone = 0;
    s_gone_from = NULL;
}

/* ---- tests ------------------------------------------------------------------------ */

static void test_send_names_the_source_for_an_endpoint_that_asks(void)
{
    const uint8_t b[] = "ATMA\r";

    fresh();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      bm_peers_send(&CHIP, &TCP, b, sizeof(b) - 1));
    TEST_ASSERT_EQUAL(1, s_from_sends);
    TEST_ASSERT_EQUAL(0, s_plain_sends);
    TEST_ASSERT_EQUAL_PTR(&TCP, s_from);

    /* the other way: the server has no send_from, send() it is */
    TEST_ASSERT_EQUAL(ESP_OK, bm_peers_send(&TCP, &CHIP, b, sizeof(b) - 1));
    TEST_ASSERT_EQUAL(1, s_plain_sends);
    TEST_ASSERT_EQUAL(1, s_from_sends);
}

static void test_an_end_that_empties_is_reported_once(void)
{
    bm_peers_t st = { 0 };

    fresh();
    bm_peers_poll(&st, &TCP, &CHIP); /* nobody yet: nothing to report */
    TEST_ASSERT_EQUAL(0, s_gone);

    s_clients = 1;                   /* the app connects */
    bm_peers_poll(&st, &TCP, &CHIP);
    bm_peers_poll(&st, &TCP, &CHIP);
    TEST_ASSERT_EQUAL(0, s_gone);

    s_clients = 0;                   /* and leaves */
    bm_peers_poll(&st, &TCP, &CHIP);
    TEST_ASSERT_EQUAL(1, s_gone);
    TEST_ASSERT_EQUAL_PTR(&TCP, s_gone_from);

    bm_peers_poll(&st, &TCP, &CHIP); /* still nobody: not told again */
    bm_peers_poll(&st, &TCP, &CHIP);
    TEST_ASSERT_EQUAL(1, s_gone);

    s_clients = 2;                   /* two apps, one leaves: somebody is left */
    bm_peers_poll(&st, &TCP, &CHIP);
    s_clients = 1;
    bm_peers_poll(&st, &TCP, &CHIP);
    TEST_ASSERT_EQUAL(1, s_gone);
    s_clients = 0;                   /* the last one */
    bm_peers_poll(&st, &TCP, &CHIP);
    TEST_ASSERT_EQUAL(2, s_gone);
}

static void test_the_bridge_may_be_written_either_way_round(void)
{
    bm_peers_t st = { 0 };

    fresh();
    s_clients = 1;
    bm_peers_poll(&st, &CHIP, &TCP); /* a = the chip, b = the server */
    s_clients = 0;
    bm_peers_poll(&st, &CHIP, &TCP);
    TEST_ASSERT_EQUAL(1, s_gone);
    TEST_ASSERT_EQUAL_PTR(&TCP, s_gone_from);
}

static void test_an_end_that_cannot_count_is_never_reported(void)
{
    bm_peers_t st = { 0 };

    fresh();

    for (int i = 0; i < 5; i++)
    {
        bm_peers_poll(&st, &UART, &CHIP);
    }

    TEST_ASSERT_EQUAL(0, s_gone);

    /* and an end that does not want to know is not told: two servers */
    s_clients = 1;
    bm_peers_poll(&st, &TCP, &UART);
    s_clients = 0;
    bm_peers_poll(&st, &TCP, &UART);
    TEST_ASSERT_EQUAL(0, s_gone);
}

void run_peers_tests(void)
{
    RUN_TEST(test_send_names_the_source_for_an_endpoint_that_asks);
    RUN_TEST(test_an_end_that_empties_is_reported_once);
    RUN_TEST(test_the_bridge_may_be_written_either_way_round);
    RUN_TEST(test_an_end_that_cannot_count_is_never_reported);
}
