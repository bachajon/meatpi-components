/**
 * @file test_main.c
 * @brief Host suite for the pure GVRET codec: decode (CAN→F1 00 record with
 *        XOR checksum + ext bit31), encode BUILD_CAN_FRAME → CAN (incl.
 *        fragmentation), and handshake replies (GET_DEV_INFO / TIME_SYNC /
 *        KEEPALIVE / GET_NUMBUSES) delivered through the reply channel; the
 *        TIME_SYNC answer carries the device clock; what the codec refuses
 *        is reported to the caller (the bridge's codec_errors).
 */
#include <string.h>

#include "unity.h"

#include "bridge_manager.h"
#include "can_frame_wire.h"

#include "translator_gvret_private.h"

extern const bridge_translator_t translator_gvret_desc;

/* the codec ctx begins with a bridge_reply_hdr_t (wants_reply). Give the ctx
 * plenty of room and bind the reply header to a collector. */
static uint8_t g_ctx[256];

/* far-sink (decode output = GVRET records; encode output = CAN-wire) */
static uint8_t g_far[64]; static size_t g_farlen;
static can_core_frame_t g_frame; static int g_nframes;
static esp_err_t far_raw(void *a, const uint8_t *o, size_t l)
{
    (void)a;

    if (g_farlen + l <= sizeof(g_far))
    {
        memcpy(g_far + g_farlen, o, l);
        g_farlen += l;
    }

    return ESP_OK;
}
static esp_err_t far_frame(void *a, const uint8_t *o, size_t l)
{ (void)a; if (can_wire_decode(o, l, &g_frame)) g_nframes++; return ESP_OK; }

/* reply channel collector */
static uint8_t g_reply[64]; static size_t g_replylen;
static esp_err_t reply_sink(void *a, const uint8_t *o, size_t l)
{
    (void)a;

    if (g_replylen + l <= sizeof(g_reply))
    {
        memcpy(g_reply + g_replylen, o, l);
        g_replylen += l;
    }

    return ESP_OK;
}

static void init_ctx(void)
{
    translator_gvret_desc.ctx_init(g_ctx);
    bridge_reply_hdr_t *h = (bridge_reply_hdr_t *)g_ctx;
    h->reply = reply_sink;
    h->reply_arg = NULL;
    g_farlen = 0; g_replylen = 0; g_nframes = 0;
}

/* ---- decode ------------------------------------------------------------ */

void test_decode_frame_record(void)
{
    init_ctx();
    can_core_frame_t f = { .id = 0x100, .ext = false, .dlc = 2,
                             .data = { 0xAA, 0xBB } };
    uint8_t wire[CAN_WIRE_MAX];
    size_t n = can_wire_encode(&f, wire);
    translator_gvret_desc.decode(g_ctx, wire, n, far_raw, NULL);
    /* F1 00 <ts:4> <id:4> <dlc> <data:2> <chk> = 2+4+4+1+2+1 = 14 */
    TEST_ASSERT_EQUAL(14, g_farlen);
    TEST_ASSERT_EQUAL_HEX8(0xF1, g_far[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_far[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_far[6]);  /* id LE 0x100 */
    TEST_ASSERT_EQUAL_HEX8(0x01, g_far[7]);
    TEST_ASSERT_EQUAL_HEX8(2, g_far[10]);    /* dlc */
    TEST_ASSERT_EQUAL_HEX8(0xAA, g_far[11]);
    /* checksum = XOR of the first 13 bytes */
    uint8_t chk = 0; for (int i = 0; i < 13; i++) chk ^= g_far[i];
    TEST_ASSERT_EQUAL_HEX8(chk, g_far[13]);
}

void test_decode_ext_bit31(void)
{
    init_ctx();
    can_core_frame_t f = { .id = 0x18FF0001, .ext = true, .dlc = 0 };
    uint8_t wire[CAN_WIRE_MAX];
    size_t n = can_wire_encode(&f, wire);
    translator_gvret_desc.decode(g_ctx, wire, n, far_raw, NULL);
    uint32_t id = (uint32_t)g_far[6] | ((uint32_t)g_far[7] << 8) |
                  ((uint32_t)g_far[8] << 16) | ((uint32_t)g_far[9] << 24);
    TEST_ASSERT_TRUE((id & (1u << 31)) != 0);     /* ext bit set */
    TEST_ASSERT_EQUAL_HEX32(0x18FF0001, id & 0x7FFFFFFF);
}

/* ---- encode: BUILD_CAN_FRAME ------------------------------------------- */

void test_encode_build_frame(void)
{
    init_ctx();
    /* F1 00 <id:4 LE> <bus:1> <dlc:1> <data> <chk:1> */
    uint8_t cmd[] = { 0xF1, 0x00, 0x00, 0x01, 0x00, 0x00, /* id 0x100 */
                      0x00,                                /* bus */
                      0x03,                                /* dlc 3 */
                      0xDE, 0xAD, 0xBE,                    /* data */
                      0x00 };                              /* checksum (ignored) */
    translator_gvret_desc.encode(g_ctx, cmd, sizeof(cmd), far_frame, NULL);
    TEST_ASSERT_EQUAL(1, g_nframes);
    TEST_ASSERT_EQUAL_HEX32(0x100, g_frame.id);
    TEST_ASSERT_EQUAL(3, g_frame.dlc);
    TEST_ASSERT_EQUAL_HEX8(0xBE, g_frame.data[2]);
}

void test_encode_build_frame_fragmented(void)
{
    init_ctx();
    uint8_t cmd[] = { 0xF1, 0x00, 0x34, 0x12, 0x00, 0x80, /* id bytes LE; 0x80
                                                             = bit31 ext flag */
                      0x00, 0x01, 0x99, 0x00 };
    for (size_t i = 0; i < sizeof(cmd); i++)
        translator_gvret_desc.encode(g_ctx, &cmd[i], 1, far_frame, NULL);
    TEST_ASSERT_EQUAL(1, g_nframes);
    TEST_ASSERT_TRUE(g_frame.ext);         /* bit31 → extended */
    TEST_ASSERT_EQUAL_HEX32(0x1234, g_frame.id); /* bit31 masked off */
    TEST_ASSERT_EQUAL_HEX8(0x99, g_frame.data[0]);
}

/* ---- encode: handshake replies ---------------------------------------- */

void test_reply_dev_info(void)
{
    init_ctx();
    uint8_t cmd[] = { 0xF1, 0x07 }; /* GET_DEV_INFO */
    translator_gvret_desc.encode(g_ctx, cmd, sizeof(cmd), far_frame, NULL);
    TEST_ASSERT_EQUAL(8, g_replylen);
    TEST_ASSERT_EQUAL_HEX8(0xF1, g_reply[0]);
    TEST_ASSERT_EQUAL_HEX8(7, g_reply[1]);
    TEST_ASSERT_EQUAL(0, g_nframes); /* no CAN frame */
}

void test_reply_keepalive_and_numbuses(void)
{
    init_ctx();
    uint8_t cmd[] = { 0xF1, 0x09, 0xF1, 0x0C }; /* keepalive then get_numbuses */
    translator_gvret_desc.encode(g_ctx, cmd, sizeof(cmd), far_frame, NULL);
    /* F1 09 DE AD (4) + F1 0C 01 (3) = 7 */
    TEST_ASSERT_EQUAL(7, g_replylen);
    TEST_ASSERT_EQUAL_HEX8(0xDE, g_reply[2]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, g_reply[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_reply[6]);
}

void test_reply_time_sync(void)
{
    init_ctx();
    uint8_t cmd[] = { 0xF1, 0x01 };
    translator_gvret_desc.encode(g_ctx, cmd, sizeof(cmd), far_frame, NULL);
    TEST_ASSERT_EQUAL(6, g_replylen);
    TEST_ASSERT_EQUAL_HEX8(0xF1, g_reply[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_reply[1]);
}

/* ---- TIME_SYNC carries the device clock -------------------------------- */

static uint32_t g_now_us;

static uint32_t fake_clock_us(void)
{
    return g_now_us;
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

void test_reply_time_sync_carries_the_clock(void)
{
    /* SavvyCAN lines its own clock up with this answer and reads the
       timestamps of the frame records against it. Until 2026-11-18 the
       answer came from a counter of the encode ctx that nothing advances:
       0 at every request */
    static const uint8_t cmd[] = { 0xF1, 0x01 };

    init_ctx();
    gvret_clock_us_hook = fake_clock_us;
    g_now_us = 0x12345678u;
    translator_gvret_desc.encode(g_ctx, cmd, sizeof(cmd), far_frame, NULL);
    gvret_clock_us_hook = NULL;

    TEST_ASSERT_EQUAL(6, g_replylen);
    TEST_ASSERT_EQUAL_HEX8(0xF1, g_reply[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_reply[1]);
    TEST_ASSERT_EQUAL_HEX32(0x12345678u, rd32(&g_reply[2]));
}

void test_reply_time_sync_runs_with_the_frame_clock(void)
{
    /* a frame received at ts_us, a request a quarter of a second later:
       the record carries the first time, the answer the second, on one
       clock */
    static const uint8_t cmd[] = { 0xF1, 0x01 };
    static uint8_t dec_ctx[256];
    can_core_frame_t f = { .id = 0x100, .dlc = 0, .ts_us = 5000000u };
    uint8_t wire[CAN_WIRE_MAX];
    size_t n = can_wire_encode(&f, wire);

    init_ctx();
    translator_gvret_desc.ctx_init(dec_ctx);
    translator_gvret_desc.decode(dec_ctx, wire, n, far_raw, NULL);
    TEST_ASSERT_EQUAL_HEX32(5000000u, rd32(&g_far[2]));

    gvret_clock_us_hook = fake_clock_us;
    g_now_us = 5250000u;
    translator_gvret_desc.encode(g_ctx, cmd, sizeof(cmd), far_frame, NULL);
    gvret_clock_us_hook = NULL;

    TEST_ASSERT_EQUAL(6, g_replylen);
    TEST_ASSERT_EQUAL_UINT32(250000u, rd32(&g_reply[2]) - rd32(&g_far[2]));
}

/* ---- what the codec refuses is reported -------------------------------- */

void test_encode_reports_what_is_not_gvret(void)
{
    /* the pump counts a codec call that does not return ESP_OK in the
       bridge's codec_errors. Until 2026-11-18 every refusal returned
       ESP_OK and the counter stayed 0 whatever arrived */
    /* SavvyCAN's connect: binary mode twice, then a keepalive */
    static const uint8_t hello[] = { 0xE7, 0xE7, 0xF1, 0x09 };
    /* another protocol's client; F1 and a byte that is no command */
    static const uint8_t other[] = { 'A', 'T', 'Z', '\r' };
    static const uint8_t unknown[] = { 0xF1, 0x7F };
    /* SETUP_CANBUS, its 8 bytes, and the zero SavvyCAN ends it with */
    static const uint8_t setup[] = { 0xF1, 0x05, 0x20, 0xA1, 0x07, 0xC0,
                                     0x00, 0x00, 0x00, 0x00, 0x00 };

    init_ctx();
    TEST_ASSERT_EQUAL(ESP_OK,
                      translator_gvret_desc.encode(g_ctx, hello, sizeof(hello),
                                                   far_frame, NULL));
    TEST_ASSERT_EQUAL(ESP_OK,
                      translator_gvret_desc.encode(g_ctx, setup, sizeof(setup),
                                                   far_frame, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      translator_gvret_desc.encode(g_ctx, other, sizeof(other),
                                                   far_frame, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      translator_gvret_desc.encode(g_ctx, unknown,
                                                   sizeof(unknown), far_frame,
                                                   NULL));
    TEST_ASSERT_EQUAL(0, g_nframes);
}

void test_encode_refused_bytes_keep_the_frame_behind_them(void)
{
    /* a stray byte in front of a BUILD_CAN_FRAME in one chunk: the chunk is
       reported and the frame still reaches the bus */
    static const uint8_t mixed[] = { 0x55,
                                     0xF1, 0x00, 0x00, 0x01, 0x00, 0x00,
                                     0x00, 0x01, 0xAB, 0x00 };

    init_ctx();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      translator_gvret_desc.encode(g_ctx, mixed, sizeof(mixed),
                                                   far_frame, NULL));
    TEST_ASSERT_EQUAL(1, g_nframes);
    TEST_ASSERT_EQUAL_HEX32(0x100, g_frame.id);
    TEST_ASSERT_EQUAL_HEX8(0xAB, g_frame.data[0]);
}

void test_decode_reports_a_malformed_chunk(void)
{
    /* bytes that are no wire frame (a bridge row whose side `a` is not the
       can jack): reported, and the frame in front of them still goes out */
    can_core_frame_t f = { .id = 0x100, .dlc = 1, .data = { 0x11 } };
    uint8_t wire[2 * CAN_WIRE_MAX];
    size_t n = can_wire_encode(&f, wire);

    init_ctx();
    TEST_ASSERT_EQUAL(ESP_OK, translator_gvret_desc.decode(g_ctx, wire, n,
                                                           far_raw, NULL));
    TEST_ASSERT_EQUAL(13, g_farlen);    /* 2 + ts 4 + id 4 + dlc + 1 + chk */

    g_farlen = 0;
    memset(wire + n, 0xFF, 5);          /* a tail shorter than a header */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      translator_gvret_desc.decode(g_ctx, wire, n + 5, far_raw,
                                                   NULL));
    TEST_ASSERT_EQUAL(13, g_farlen);
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_decode_frame_record);
    RUN_TEST(test_decode_ext_bit31);
    RUN_TEST(test_encode_build_frame);
    RUN_TEST(test_encode_build_frame_fragmented);
    RUN_TEST(test_reply_dev_info);
    RUN_TEST(test_reply_keepalive_and_numbuses);
    RUN_TEST(test_reply_time_sync);
    RUN_TEST(test_reply_time_sync_carries_the_clock);
    RUN_TEST(test_reply_time_sync_runs_with_the_frame_clock);
    RUN_TEST(test_encode_reports_what_is_not_gvret);
    RUN_TEST(test_encode_refused_bytes_keep_the_frame_behind_them);
    RUN_TEST(test_decode_reports_a_malformed_chunk);
    UNITY_END();
}
