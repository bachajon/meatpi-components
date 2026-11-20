/**
 * @file test_main.c
 * @brief The vendored isotp-c link as this firmware builds it, two links
 *        back to back: every message size from 1 to 4095 bytes in both
 *        directions, with one flow control for the whole message (BS 0)
 *        and with one every eight frames, in normal addressing and in
 *        ISO 15765-2 extended addressing (the local change of 2026-11-19,
 *        PROVENANCE.md). Every frame either link sends is checked against
 *        the standard's layout, not only the bytes that arrive.
 *
 * Normal addressing is also pinned: the frames of its whole sweep hash to
 * the value the library gave BEFORE extended addressing was added (taken
 * 2026-11-20 from the copy on main at eb15b34 and from the changed one:
 * the same). A change that moves one byte of a normal-addressing frame
 * fails the first case.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "isotp.h"

#define MAXP      4095          /* the largest message of a 12-bit length */
#define LOG_CAP   1024          /* frames kept per transfer (683 at most)  */
#define Q_CAP     1024          /* frames in flight between the two links  */
#define NORMAL_TRACE_HASH 0x80ae981061e3e1a4ull

typedef struct node
{
    IsoTpLink    link;
    struct node *peer;
    uint8_t      txbuf[MAXP + 8];
    uint8_t      rxbuf[MAXP + 8];
    int          ae_on;
    uint8_t      send_ae;
    uint8_t      tag;                 /* 0: the first node, 1: the second */
    int          frames;              /* sent in the running transfer     */
    uint8_t      log[LOG_CAP][9];     /* [0] = size, then the bytes       */
} node_t;

typedef struct
{
    node_t *to;
    uint8_t d[8];
    uint8_t n;
} qframe_t;

static qframe_t s_q[Q_CAP];
static int s_qh, s_qt;
static uint32_t s_now = 1000;
static uint64_t s_hash;
static int s_hash_on;
static long s_fail, s_checks, s_frames;
static node_t s_a, s_b, s_c;
static uint8_t s_p[MAXP];
static uint8_t s_out[MAXP + 8];

static void fail(const char *fmt, ...)
{
    va_list ap;

    if (s_fail++ < 10)
    {
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
        printf("\n");
    }
}

#define CHECK(c, ...) do { s_checks++; if (!(c)) { fail(__VA_ARGS__); } } while (0)

static void hash_bytes(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        s_hash = (s_hash ^ p[i]) * 1099511628211ull;
    }
}

/* ---- what the library asks of its user -------------------------------- */

uint32_t isotp_user_get_us(void)
{
    return s_now;
}

void isotp_user_debug(const char *message, ...)
{
    (void)message;
}

int isotp_user_send_can(const uint32_t id, const uint8_t *data,
                        const uint8_t size, void *arg)
{
    node_t *n = arg;
    qframe_t *f = &s_q[s_qh % Q_CAP];

    (void)id;
    s_frames++;
    CHECK(size >= 1 && size <= 8, "a frame of %u bytes", size);

    if (size > 8)
    {
        return ISOTP_RET_ERROR;
    }

    if (n->frames < LOG_CAP)
    {
        n->log[n->frames][0] = size;
        memcpy(&n->log[n->frames][1], data, size);
    }

    n->frames++;

    if (s_hash_on)
    {
        hash_bytes(&n->tag, 1);
        hash_bytes(&size, 1);
        hash_bytes(data, size);
    }

    f->to = n->peer;
    f->n = size;
    memcpy(f->d, data, size);
    s_qh++;
    CHECK(s_qh - s_qt <= Q_CAP, "the frame queue of the test ran over");
    return ISOTP_RET_OK;
}

/* ---- the two links ---------------------------------------------------- */

static void node_init(node_t *n, node_t *peer, uint8_t tag, int ae_on,
                      uint8_t send_ae, uint8_t recv_ae, uint8_t bs,
                      uint32_t st_us)
{
    memset(&n->link, 0, sizeof(n->link));
    isotp_init_link(&n->link, 0x7E0, n->txbuf, sizeof(n->txbuf), n->rxbuf,
                    sizeof(n->rxbuf));
    n->link.user_send_can_arg = n;
    isotp_set_flow_control(&n->link, bs, st_us);
    n->peer = peer;
    n->tag = tag;
    n->ae_on = ae_on;
    n->send_ae = send_ae;
    n->frames = 0;

    if (ae_on)
    {
        isotp_set_addr_ext(&n->link, send_ae, recv_ae);
    }
}

static void pump(node_t *a, node_t *b)
{
    while (s_qt != s_qh)
    {
        qframe_t f = s_q[s_qt % Q_CAP];

        s_qt++;
        isotp_on_can_message(&f.to->link, f.d, f.n);
    }

    isotp_poll(&a->link);
    isotp_poll(&b->link);
    s_now += 100;
}

/* the data frames of one message, against ISO 15765-2 */
static void check_data_frames(const node_t *tx, const uint8_t *p, uint32_t n)
{
    int off = tx->ae_on ? 1 : 0;
    uint32_t sf_max = 7u - (uint32_t)off;
    uint32_t ff_data = 6u - (uint32_t)off;
    uint32_t at = 0;

    for (int i = 0; i < tx->frames && i < LOG_CAP; i++)
    {
        const uint8_t *d = &tx->log[i][1];
        uint8_t size = tx->log[i][0];

        if (off)
        {
            CHECK(d[0] == tx->send_ae, "n=%u frame %d: first byte %02X, want "
                  "the address extension %02X", (unsigned)n, i, d[0],
                  tx->send_ae);
        }

        if (n <= sf_max)
        {
            CHECK(size == off + 1 + n && d[off] == n &&
                  memcmp(d + off + 1, p, n) == 0,
                  "ae=%d n=%u: single frame wrong (size %u, PCI %02X)", off,
                  (unsigned)n, size, d[off]);
            at = n;
        }
        else if (i == 0)
        {
            CHECK(size == 8 && d[off] == (0x10 | (n >> 8)) &&
                  d[off + 1] == (n & 0xFF) &&
                  memcmp(d + off + 2, p, ff_data) == 0,
                  "ae=%d n=%u: first frame wrong (size %u, PCI %02X %02X)",
                  off, (unsigned)n, size, d[off], d[off + 1]);
            at = ff_data;
        }
        else
        {
            uint32_t take = (n - at > sf_max) ? sf_max : n - at;

            CHECK(size == off + 1 + take && d[off] == (0x20 | (i & 0x0F)) &&
                  memcmp(d + off + 1, p + at, take) == 0,
                  "ae=%d n=%u: consecutive frame %d wrong (size %u, PCI %02X)",
                  off, (unsigned)n, i, size, d[off]);
            at += take;
        }
    }

    CHECK(at == n, "ae=%d n=%u: the frames carried %u bytes", off,
          (unsigned)n, (unsigned)at);
}

/* one message from tx to rx: it arrives, and every frame is as written */
static void xfer(node_t *tx, node_t *rx, const uint8_t *p, uint32_t n,
                 uint8_t bs)
{
    uint32_t got = 0;
    int off = tx->ae_on ? 1 : 0;
    uint32_t sf_max = 7u - (uint32_t)off;
    uint32_t ff_data = 6u - (uint32_t)off;
    int rc;

    tx->frames = 0;
    rx->frames = 0;
    rc = isotp_send(&tx->link, p, n);
    CHECK(rc == ISOTP_RET_OK, "send %u: rc %d", (unsigned)n, rc);

    for (int i = 0; i < 200000; i++)
    {
        pump(tx, rx);

        if (tx->link.send_status != ISOTP_SEND_STATUS_INPROGRESS &&
            rx->link.receive_status == ISOTP_RECEIVE_STATUS_FULL &&
            s_qt == s_qh)
        {
            break;
        }
    }

    rc = isotp_receive(&rx->link, s_out, sizeof(s_out), &got);
    CHECK(rc == ISOTP_RET_OK && got == n && memcmp(s_out, p, n) == 0,
          "ae=%d n=%u bs=%u: received rc %d, %u bytes", off, (unsigned)n, bs,
          rc, (unsigned)got);

    uint32_t cfs = (n <= sf_max) ? 0 : (n - ff_data + sf_max - 1u) / sf_max;
    uint32_t want_fc = (n <= sf_max) ? 0
                                     : (bs == 0 ? 1u : (cfs + bs - 1u) / bs);

    CHECK((uint32_t)tx->frames == 1u + cfs, "ae=%d n=%u: %d data frames, "
          "want %u", off, (unsigned)n, tx->frames, (unsigned)(1u + cfs));
    CHECK((uint32_t)rx->frames == want_fc, "ae=%d n=%u bs=%u: %d flow "
          "controls, want %u", off, (unsigned)n, bs, rx->frames,
          (unsigned)want_fc);
    check_data_frames(tx, p, n);

    for (int i = 0; i < rx->frames && i < LOG_CAP; i++)
    {
        const uint8_t *d = &rx->log[i][1];
        uint8_t size = rx->log[i][0];

        CHECK(size == off + 3 && d[off] == 0x30 && d[off + 1] == bs &&
              d[off + 2] == 0 && (!off || d[0] == rx->send_ae),
              "ae=%d n=%u: flow control wrong (size %u: %02X %02X %02X %02X)",
              off, (unsigned)n, size, d[0], d[1], d[2], d[3]);
    }
}

/* every size, both directions; the address extensions differ by direction */
static void sweep(int ae_on, uint8_t bs)
{
    node_init(&s_a, &s_b, 0, ae_on, 0xE9, 0x5F, bs, 0);
    node_init(&s_b, &s_a, 1, ae_on, 0x5F, 0xE9, bs, 0);

    for (uint32_t n = 1; n <= MAXP; n++)
    {
        for (uint32_t i = 0; i < n; i++)
        {
            s_p[i] = (uint8_t)(n * 31u + i * 7u + 3u);
        }

        xfer(&s_a, &s_b, s_p, n, bs);
        xfer(&s_b, &s_a, s_p, n, bs);
    }
}

static void fresh(void)
{
    s_fail = 0;
    s_qh = 0;
    s_qt = 0;
    s_hash_on = 0;
}

/* ---- the cases -------------------------------------------------------- */

static void test_normal_addressing_every_size_and_its_trace(void)
{
    fresh();
    s_hash = 1469598103934665603ull;    /* FNV-1a, 64 bit */
    s_hash_on = 1;
    sweep(0, 0);
    sweep(0, 8);
    s_hash_on = 0;
    TEST_ASSERT_EQUAL_MESSAGE(0, s_fail, "a frame or a message was wrong: "
                              "the first ones are printed above");
    TEST_ASSERT_EQUAL_HEX64_MESSAGE(NORMAL_TRACE_HASH, s_hash,
                                    "a normal-addressing frame differs from "
                                    "the library before extended addressing");
}

static void test_extended_addressing_every_size(void)
{
    fresh();
    sweep(1, 0);
    sweep(1, 8);
    TEST_ASSERT_EQUAL_MESSAGE(0, s_fail, "a frame or a message was wrong: "
                              "the first ones are printed above");
}

/* The frames of the largest message, by count: 6 + 585 x 7 bytes in normal
   addressing, 5 + 682 x 6 behind an address extension. */
static void test_frame_counts_of_the_largest_message(void)
{
    for (uint32_t i = 0; i < MAXP; i++)
    {
        s_p[i] = (uint8_t)(i * 13u + 1u);
    }

    fresh();
    node_init(&s_a, &s_b, 0, 0, 0, 0, 0, 0);
    node_init(&s_b, &s_a, 1, 0, 0, 0, 0, 0);
    xfer(&s_a, &s_b, s_p, MAXP, 0);
    TEST_ASSERT_EQUAL(586, s_a.frames);
    TEST_ASSERT_EQUAL(1, s_b.frames);

    node_init(&s_a, &s_b, 0, 1, 0xE9, 0x5F, 0, 0);
    node_init(&s_b, &s_a, 1, 1, 0x5F, 0xE9, 0, 0);
    xfer(&s_a, &s_b, s_p, MAXP, 0);
    TEST_ASSERT_EQUAL(683, s_a.frames);
    TEST_ASSERT_EQUAL(1, s_b.frames);
    TEST_ASSERT_EQUAL(0, s_fail);
}

/* A link takes only the frames led by its peer's address extension: two
   links listen on one CAN id the way two sessions of a gateway do (b to
   E9, c to 5F), and the same frames are shown to both. */
static void test_extended_addressing_takes_only_its_own(void)
{
    static const uint8_t SF_E9[] = { 0xE9, 0x02, 0x50, 0x03 };
    static const uint8_t SF_99[] = { 0x99, 0x02, 0x50, 0x03 };
    /* a single frame of 7 bytes does not fit behind an address extension */
    static const uint8_t SF_LONG[] = { 0xE9, 0x07, 1, 2, 3, 4, 5, 6 };
    /* a first frame of 6 bytes: that is a single frame's length */
    static const uint8_t FF_SHORT[] = { 0xE9, 0x10, 0x06, 1, 2, 3, 4, 5 };
    static const uint8_t NORMAL_SF[] = { 0x02, 0x50, 0x03 };
    static const uint8_t ONLY_AE[] = { 0xE9 };
    uint32_t got = 0;

    fresh();
    node_init(&s_a, &s_b, 0, 1, 0xE9, 0x5F, 0, 0);
    node_init(&s_b, &s_a, 1, 1, 0x5F, 0xE9, 0, 0);
    node_init(&s_c, &s_a, 1, 1, 0x11, 0x5F, 0, 0);

    isotp_on_can_message(&s_b.link, SF_99, sizeof(SF_99));
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_b.link.receive_status);
    isotp_on_can_message(&s_b.link, NORMAL_SF, sizeof(NORMAL_SF));
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_b.link.receive_status);
    isotp_on_can_message(&s_b.link, ONLY_AE, sizeof(ONLY_AE));
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_b.link.receive_status);
    isotp_on_can_message(&s_b.link, SF_LONG, sizeof(SF_LONG));
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_b.link.receive_status);
    isotp_on_can_message(&s_b.link, FF_SHORT, sizeof(FF_SHORT));
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_b.link.receive_status);
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_INPROGRESS,
                          s_b.link.receive_status);
    TEST_ASSERT_EQUAL_MESSAGE(0, s_b.frames, "a refused frame was answered");

    isotp_on_can_message(&s_b.link, SF_E9, sizeof(SF_E9));
    isotp_on_can_message(&s_c.link, SF_E9, sizeof(SF_E9));
    TEST_ASSERT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_b.link.receive_status);
    TEST_ASSERT_NOT_EQUAL(ISOTP_RECEIVE_STATUS_FULL, s_c.link.receive_status);
    TEST_ASSERT_EQUAL(ISOTP_RET_OK,
                      isotp_receive(&s_b.link, s_out, sizeof(s_out), &got));
    TEST_ASSERT_EQUAL_UINT32(2, got);
    TEST_ASSERT_EQUAL_HEX8(0x50, s_out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03, s_out[1]);
    s_qt = s_qh;
}

/* The receiver's flow control is what isotp_set_flow_control() set (the
   local change of 2026-11-17): the block size as given, the separation
   time in ISO 15765-2's coding (whole milliseconds, or 0xF1..0xF9 for
   100..900 us), behind the address extension when there is one. */
static void test_flow_control_as_set(void)
{
    static const struct
    {
        uint8_t  bs;
        uint32_t st_us;
        uint8_t  st_code;
    } SET[] =
    {
        { 0, 0, 0x00 }, { 4, 2000, 0x02 }, { 3, 300, 0xF3 },
        { 255, 127000, 0x7F },
    };
    /* a first frame that names 20 bytes, in each addressing */
    static const uint8_t FF[] = { 0x10, 0x14, 1, 2, 3, 4, 5, 6 };
    static const uint8_t FF_AE[] = { 0xE9, 0x10, 0x14, 1, 2, 3, 4, 5 };

    for (size_t i = 0; i < sizeof(SET) / sizeof(SET[0]); i++)
    {
        fresh();
        node_init(&s_a, &s_b, 0, 0, 0, 0, 0, 0);
        node_init(&s_b, &s_a, 1, 0, 0, 0, SET[i].bs, SET[i].st_us);
        isotp_on_can_message(&s_b.link, FF, sizeof(FF));
        TEST_ASSERT_EQUAL(1, s_b.frames);
        TEST_ASSERT_EQUAL_UINT8(3, s_b.log[0][0]);
        TEST_ASSERT_EQUAL_HEX8(0x30, s_b.log[0][1]);
        TEST_ASSERT_EQUAL_HEX8(SET[i].bs, s_b.log[0][2]);
        TEST_ASSERT_EQUAL_HEX8(SET[i].st_code, s_b.log[0][3]);

        node_init(&s_b, &s_a, 1, 1, 0x5F, 0xE9, SET[i].bs, SET[i].st_us);
        isotp_on_can_message(&s_b.link, FF_AE, sizeof(FF_AE));
        TEST_ASSERT_EQUAL(1, s_b.frames);
        TEST_ASSERT_EQUAL_UINT8(4, s_b.log[0][0]);
        TEST_ASSERT_EQUAL_HEX8(0x5F, s_b.log[0][1]);
        TEST_ASSERT_EQUAL_HEX8(0x30, s_b.log[0][2]);
        TEST_ASSERT_EQUAL_HEX8(SET[i].bs, s_b.log[0][3]);
        TEST_ASSERT_EQUAL_HEX8(SET[i].st_code, s_b.log[0][4]);
        s_qt = s_qh;
    }
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_normal_addressing_every_size_and_its_trace);
    RUN_TEST(test_extended_addressing_every_size);
    RUN_TEST(test_frame_counts_of_the_largest_message);
    RUN_TEST(test_extended_addressing_takes_only_its_own);
    RUN_TEST(test_flow_control_as_set);
    UNITY_END();
}
