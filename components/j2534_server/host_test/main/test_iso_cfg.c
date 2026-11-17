/**
 * @file test_iso_cfg.c
 * @brief SET_CONFIG and GET_CONFIG of an ISO15765 channel (j2534_iso_cfg.c):
 *        the separation time's two codings, a list taken whole or not at
 *        all, the ranges of SAE J2534-1, what is passed over, the answer a
 *        GET makes, and the byte-exact example of J2534_WIRE_PROTOCOL.md
 *        section 12. See ../README.md. test_main.c runs the cases through
 *        run_iso_cfg_tests().
 */
#include <string.h>

#include "unity.h"
#include "j2534_iso_cfg.h"
#include "j2534_proto.h"

#define BS    J2534_CFG_ISO15765_BS
#define STMIN J2534_CFG_ISO15765_STMIN
#define OK    J2534_STATUS_NOERROR
#define BAD   J2534_ERR_INVALID_IOCTL_VALUE

typedef struct
{
    uint32_t param;
    uint32_t value;
} pair_t;

static uint8_t s_body[4 + (J2534_ISO_CFG_MAX + 2) * 8];
static uint8_t s_out[4 + J2534_ISO_CFG_MAX * 8];

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/* A list of n pairs in s_body; returns its length. */
static uint32_t list(uint32_t n, const pair_t *pairs)
{
    put_u32(s_body, n);

    for (uint32_t i = 0; i < n; i++)
    {
        put_u32(s_body + 4 + i * 8, pairs[i].param);
        put_u32(s_body + 8 + i * 8, pairs[i].value);
    }

    return 4u + n * 8u;
}

/* SET_CONFIG of one list on *flow. */
static uint32_t set(uint32_t n, const pair_t *pairs, j2534_iso_flow_t *flow,
                    bool *changed)
{
    return j2534_iso_cfg_set(s_body, list(n, pairs), flow, changed);
}

static void test_iso_stmin_codes(void)
{
    uint32_t us = 7;

    TEST_ASSERT_TRUE(j2534_iso_stmin_to_us(0, &us));
    TEST_ASSERT_EQUAL_UINT32(0, us);
    TEST_ASSERT_TRUE(j2534_iso_stmin_to_us(1, &us));
    TEST_ASSERT_EQUAL_UINT32(1000, us);
    TEST_ASSERT_TRUE(j2534_iso_stmin_to_us(0x7F, &us));
    TEST_ASSERT_EQUAL_UINT32(127000, us);
    TEST_ASSERT_TRUE(j2534_iso_stmin_to_us(0xF1, &us));
    TEST_ASSERT_EQUAL_UINT32(100, us);
    TEST_ASSERT_TRUE(j2534_iso_stmin_to_us(0xF9, &us));
    TEST_ASSERT_EQUAL_UINT32(900, us);

    /* the reserved values of ISO 15765-2, and past the byte */
    const uint32_t reserved[] = { 0x80, 0xF0, 0xFA, 0xFF, 0x100, 0xFFFFFFFFu };

    us = 4242;

    for (unsigned i = 0; i < sizeof(reserved) / sizeof(reserved[0]); i++)
    {
        TEST_ASSERT_FALSE(j2534_iso_stmin_to_us(reserved[i], &us));
        TEST_ASSERT_EQUAL_UINT32(4242, us);
    }

    /* every value the device takes reads back as it was set */
    for (uint32_t v = 0; v <= 0xFF; v++)
    {
        if (j2534_iso_stmin_to_us(v, &us))
        {
            TEST_ASSERT_EQUAL_UINT32(v, j2534_iso_us_to_stmin(us));
        }
    }
}

static void test_iso_cfg_set_takes_the_list(void)
{
    j2534_iso_flow_t flow = { J2534_ISO_DEFAULT_BS, J2534_ISO_DEFAULT_ST_US };
    bool changed = false;

    /* what a channel opens with: SAE J2534-1's 0 and 0 */
    TEST_ASSERT_EQUAL_UINT8(0, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(0, flow.st_us);

    /* the old pace, as a tool asks for it */
    const pair_t old_pace[] = { { BS, 8 }, { STMIN, 1 } };

    TEST_ASSERT_EQUAL(OK, set(2, old_pace, &flow, &changed));
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL_UINT8(8, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(1000, flow.st_us);

    const pair_t both[] = { { BS, 0 }, { STMIN, 0 } };

    TEST_ASSERT_EQUAL(OK, set(2, both, &flow, &changed));
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL_UINT8(0, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(0, flow.st_us);

    /* one of the two alone leaves the other */
    const pair_t st_only[] = { { STMIN, 5 } };
    const pair_t bs_only[] = { { BS, 0xFF } };

    TEST_ASSERT_EQUAL(OK, set(1, st_only, &flow, &changed));
    TEST_ASSERT_EQUAL_UINT8(0, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(5000, flow.st_us);
    TEST_ASSERT_EQUAL(OK, set(1, bs_only, &flow, &changed));
    TEST_ASSERT_EQUAL_UINT8(0xFF, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(5000, flow.st_us);

    /* the list udsoncan's J2534 connection sends */
    const pair_t udsoncan[] =
    {
        { J2534_CFG_DATA_RATE, 500000 }, { J2534_CFG_LOOPBACK, 0 },
        { BS, 0x20 }, { STMIN, 0 },
    };

    TEST_ASSERT_EQUAL(OK, set(4, udsoncan, &flow, &changed));
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL_UINT8(0x20, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(0, flow.st_us);

    /* a list without the two is acknowledged and says so */
    const pair_t others[] = { { J2534_CFG_DATA_RATE, 250000 }, { J2534_CFG_BS_TX, 4 } };

    TEST_ASSERT_EQUAL(OK, set(2, others, &flow, &changed));
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_EQUAL_UINT8(0x20, flow.bs);

    /* the same parameter twice: the last one holds */
    const pair_t twice[] = { { BS, 1 }, { BS, 2 } };

    TEST_ASSERT_EQUAL(OK, set(2, twice, &flow, &changed));
    TEST_ASSERT_EQUAL_UINT8(2, flow.bs);
}

static void test_iso_cfg_set_refuses_whole(void)
{
    const j2534_iso_flow_t was = { 8, 1000 };
    j2534_iso_flow_t flow = was;
    bool changed = true;

    const pair_t bs_over[] = { { BS, 0x100 } };
    const pair_t st_80[] = { { STMIN, 0x80 } };
    const pair_t st_fa[] = { { STMIN, 0xFA } };

    TEST_ASSERT_EQUAL(BAD, set(1, bs_over, &flow, &changed));
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_EQUAL(BAD, set(1, st_80, &flow, &changed));
    TEST_ASSERT_EQUAL(BAD, set(1, st_fa, &flow, &changed));

    /* a good pair in front of a refused one is not taken either */
    const pair_t good_bad[] = { { BS, 4 }, { STMIN, 0x80 } };

    TEST_ASSERT_EQUAL(BAD, set(2, good_bad, &flow, &changed));
    TEST_ASSERT_EQUAL_MEMORY(&was, &flow, sizeof(was));

    /* not a list: a count without its pairs, two stray bytes */
    const pair_t one[] = { { BS, 0 } };
    uint32_t len = list(1, one);

    put_u32(s_body, 2);
    TEST_ASSERT_EQUAL(BAD, j2534_iso_cfg_set(s_body, len, &flow, &changed));
    TEST_ASSERT_EQUAL(BAD, j2534_iso_cfg_set(s_body, 2, &flow, &changed));

    /* one pair too many, and the most it takes (all parameter 0: passed over) */
    memset(s_body, 0, sizeof(s_body));
    put_u32(s_body, J2534_ISO_CFG_MAX + 1);
    len = 4u + (J2534_ISO_CFG_MAX + 1) * 8u;
    TEST_ASSERT_EQUAL(BAD, j2534_iso_cfg_set(s_body, len, &flow, &changed));
    put_u32(s_body, J2534_ISO_CFG_MAX);
    len = 4u + J2534_ISO_CFG_MAX * 8u;
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_set(s_body, len, &flow, &changed));
    TEST_ASSERT_EQUAL_MEMORY(&was, &flow, sizeof(was));

    /* the id alone, as every driver up to 1.8 sends it: acknowledged,
       nothing taken */
    changed = true;
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_set(NULL, 0, &flow, &changed));
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_set(s_body, 0, &flow, &changed));
    TEST_ASSERT_EQUAL_MEMORY(&was, &flow, sizeof(was));
    TEST_ASSERT_EQUAL(J2534_ERR_NULL_PARAMETER,
                      j2534_iso_cfg_set(s_body, 12, NULL, &changed));
}

static void test_iso_cfg_get_answers_what_it_holds(void)
{
    const j2534_iso_flow_t flow = { 3, 300 };
    uint32_t out_len = 99;
    uint32_t len;

    const pair_t three[] = { { STMIN, 0xEE }, { J2534_CFG_DATA_RATE, 0 }, { BS, 0xEE } };

    len = list(3, three);
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_get(s_body, len, &flow, s_out,
                                            sizeof(s_out), &out_len));
    TEST_ASSERT_EQUAL_UINT32(4 + 2 * 8, out_len);
    TEST_ASSERT_EQUAL_UINT32(2, get_u32(s_out));
    TEST_ASSERT_EQUAL_UINT32(STMIN, get_u32(s_out + 4));
    TEST_ASSERT_EQUAL_UINT32(0xF3, get_u32(s_out + 8));
    TEST_ASSERT_EQUAL_UINT32(BS, get_u32(s_out + 12));
    TEST_ASSERT_EQUAL_UINT32(3, get_u32(s_out + 16));

    /* nothing the channel holds: an empty list, still an answer */
    const pair_t loopback[] = { { J2534_CFG_LOOPBACK, 0 } };

    len = list(1, loopback);
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_get(s_body, len, &flow, s_out,
                                            sizeof(s_out), &out_len));
    TEST_ASSERT_EQUAL_UINT32(4, out_len);
    TEST_ASSERT_EQUAL_UINT32(0, get_u32(s_out));

    /* the id alone: acknowledged without an answer */
    out_len = 99;
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_get(NULL, 0, &flow, s_out,
                                            sizeof(s_out), &out_len));
    TEST_ASSERT_EQUAL_UINT32(0, out_len);

    /* not a list, and an answer that does not fit */
    const pair_t two[] = { { BS, 0 }, { STMIN, 0 } };

    len = list(2, two);
    TEST_ASSERT_EQUAL(BAD, j2534_iso_cfg_get(s_body, len - 1, &flow, s_out,
                                             sizeof(s_out), &out_len));
    TEST_ASSERT_EQUAL_UINT32(0, out_len);
    TEST_ASSERT_EQUAL(J2534_ERR_INVALID_MSG,
                      j2534_iso_cfg_get(s_body, len, &flow, s_out, 19, &out_len));
    TEST_ASSERT_EQUAL(J2534_ERR_INVALID_MSG,
                      j2534_iso_cfg_get(s_body, len, &flow, NULL, 0, &out_len));
}

/* The byte-exact example of J2534_WIRE_PROTOCOL.md section 12 (the
 * standard, section 6c: a protocol document's examples are host-test
 * vectors): the bodies behind `ioctl_id` and `body_len`, and the bytes
 * behind the ACK's status word. */
static void test_iso_cfg_wire_vector(void)
{
    static const uint8_t SET_BODY[] =
    {
        0x02, 0x00, 0x00, 0x00,                         /* two pairs     */
        0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* BS 0          */
        0x1F, 0x00, 0x00, 0x00, 0xF2, 0x00, 0x00, 0x00, /* STMIN 200 us  */
    };
    static const uint8_t GET_BODY[] =
    {
        0x02, 0x00, 0x00, 0x00,
        0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    j2534_iso_flow_t flow = { J2534_ISO_DEFAULT_BS, J2534_ISO_DEFAULT_ST_US };
    uint32_t out_len = 0;
    bool changed = false;

    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_set(SET_BODY, sizeof(SET_BODY), &flow,
                                            &changed));
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_EQUAL_UINT8(0, flow.bs);
    TEST_ASSERT_EQUAL_UINT32(200, flow.st_us);
    TEST_ASSERT_EQUAL(OK, j2534_iso_cfg_get(GET_BODY, sizeof(GET_BODY), &flow,
                                            s_out, sizeof(s_out), &out_len));
    TEST_ASSERT_EQUAL_UINT32(sizeof(SET_BODY), out_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(SET_BODY, s_out, sizeof(SET_BODY));
}

void run_iso_cfg_tests(void)
{
    RUN_TEST(test_iso_stmin_codes);
    RUN_TEST(test_iso_cfg_set_takes_the_list);
    RUN_TEST(test_iso_cfg_set_refuses_whole);
    RUN_TEST(test_iso_cfg_get_answers_what_it_holds);
    RUN_TEST(test_iso_cfg_wire_vector);
}
