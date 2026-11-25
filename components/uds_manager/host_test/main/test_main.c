/**
 * @file test_main.c
 * @brief Host tests for uds_proto (response predicates, NRC decode,
 *        hex⇄bytes round-trips, an answer's hex in pieces), the AT
 *        response parser, the AT transaction and its target setup against
 *        a scripted chip, the native path's "is the answer still
 *        arriving", and the DTC codec.
 *        Expected output: 36 Tests 0 Failures 0 Ignored.
 */
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "uds_proto.h"
#include "uds_transport.h"

void setUp(void) {}
void tearDown(void) {}

static void test_positive_response(void)
{
    const uint8_t r[] = { 0x62, 0xF1, 0x90, 0x01, 0x02 };
    TEST_ASSERT_TRUE(uds_is_positive_for(0x22, r, sizeof(r)));
    TEST_ASSERT_FALSE(uds_is_negative(r, sizeof(r)));
    TEST_ASSERT_FALSE(uds_is_pending(r, sizeof(r)));
    TEST_ASSERT_EQUAL_UINT8(0, uds_nrc_of(r, sizeof(r)));
    /* wrong SID doesn't match */
    TEST_ASSERT_FALSE(uds_is_positive_for(0x19, r, sizeof(r)));
}

static void test_negative_response(void)
{
    const uint8_t r[] = { 0x7F, 0x22, 0x31 }; /* requestOutOfRange */
    TEST_ASSERT_TRUE(uds_is_negative(r, sizeof(r)));
    TEST_ASSERT_FALSE(uds_is_positive_for(0x22, r, sizeof(r)));
    TEST_ASSERT_EQUAL_UINT8(0x31, uds_nrc_of(r, sizeof(r)));
    TEST_ASSERT_EQUAL_STRING("requestOutOfRange", uds_nrc_name(0x31));
}

static void test_pending_response(void)
{
    const uint8_t r[] = { 0x7F, 0x31, 0x78 };
    TEST_ASSERT_TRUE(uds_is_negative(r, sizeof(r)));
    TEST_ASSERT_TRUE(uds_is_pending(r, sizeof(r)));
    /* a different NRC is negative but not pending */
    const uint8_t r2[] = { 0x7F, 0x31, 0x33 };
    TEST_ASSERT_FALSE(uds_is_pending(r2, sizeof(r2)));
    TEST_ASSERT_EQUAL_STRING("securityAccessDenied", uds_nrc_name(0x33));
}

static void test_short_and_null_inputs(void)
{
    const uint8_t r[] = { 0x7F, 0x22 }; /* too short for an NRC */
    TEST_ASSERT_FALSE(uds_is_negative(r, sizeof(r)));
    TEST_ASSERT_FALSE(uds_is_negative(NULL, 0));
    TEST_ASSERT_FALSE(uds_is_positive_for(0x22, NULL, 0));
    TEST_ASSERT_EQUAL_STRING("unknown", uds_nrc_name(0x01));
}

static void test_response_matches(void)
{
    /* the answer to OUR request: positive or a negative naming our SID */
    const uint8_t pos[] = { 0x62, 0xF1, 0x90, 0x31 };
    const uint8_t neg[] = { 0x7F, 0x22, 0x31 };
    /* autopid's mode-01 reply landing in a UDS window (bench 2026-09-16) */
    const uint8_t stray[] = { 0x41, 0x0C, 0x0C, 0x80 };
    const uint8_t tp[] = { 0x7E, 0x00 };

    TEST_ASSERT_TRUE(uds_response_matches(0x22, pos, sizeof(pos)));
    TEST_ASSERT_TRUE(uds_response_matches(0x22, neg, sizeof(neg)));
    TEST_ASSERT_TRUE(uds_response_matches(0x3E, tp, sizeof(tp)));
    TEST_ASSERT_FALSE(uds_response_matches(0x3E, stray, sizeof(stray)));
    TEST_ASSERT_FALSE(uds_response_matches(0x22, stray, sizeof(stray)));
    TEST_ASSERT_FALSE(uds_response_matches(0x10, neg, sizeof(neg)));
    TEST_ASSERT_FALSE(uds_response_matches(0x22, NULL, 0));
}

static void test_hex_to_bytes_ok(void)
{
    uint8_t out[8];
    size_t n = 0;

    TEST_ASSERT_TRUE(uds_hex_to_bytes("22F190", out, sizeof(out), &n));
    TEST_ASSERT_EQUAL_size_t(3, n);
    TEST_ASSERT_EQUAL_UINT8(0x22, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0xF1, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x90, out[2]);

    /* separators + 0x prefix + mixed case */
    TEST_ASSERT_TRUE(uds_hex_to_bytes("0x10 03-1a:F1", out, sizeof(out), &n));
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_EQUAL_UINT8(0x10, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x03, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x1A, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0xF1, out[3]);
}

static void test_hex_to_bytes_rejects(void)
{
    uint8_t out[2];
    size_t n = 0;

    TEST_ASSERT_FALSE(uds_hex_to_bytes("22F", out, sizeof(out), &n)); /* odd */
    TEST_ASSERT_FALSE(uds_hex_to_bytes("22GG", out, sizeof(out), &n)); /* junk */
    TEST_ASSERT_FALSE(uds_hex_to_bytes("112233", out, sizeof(out), &n)); /* overflow */
}

static void test_bytes_to_hex(void)
{
    const uint8_t b[] = { 0x62, 0xF1, 0x90 };
    char s[16];

    size_t n = uds_bytes_to_hex(b, sizeof(b), s, sizeof(s));
    TEST_ASSERT_EQUAL_STRING("62 F1 90", s);
    TEST_ASSERT_EQUAL_size_t(8, n);

    /* round-trip */
    uint8_t back[4];
    size_t bn = 0;
    TEST_ASSERT_TRUE(uds_hex_to_bytes(s, back, sizeof(back), &bn));
    TEST_ASSERT_EQUAL_size_t(3, bn);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(b, back, 3);

    /* too small a buffer refuses cleanly */
    char tiny[4];
    TEST_ASSERT_EQUAL_size_t(0, uds_bytes_to_hex(b, sizeof(b), tiny,
                                                 sizeof(tiny)));
    TEST_ASSERT_EQUAL_STRING("", tiny);
}

static void test_hex_pieces_are_the_whole_answer(void)
{
    /* the console command prints an answer of any length through a small
       buffer (2026-11-26: a 201-byte answer ended after its 84th byte):
       the pieces, one after another, are the one-shot text, to the byte */
    static uint8_t msg[4096];
    static char whole[3 * 4096 + 1];
    static char joined[3 * 4096 + 1];
    static const size_t SIZES[] = { 1, 2, 41, 42, 43, 84, 85, 128, 129, 201,
                                    4095, 4096 };
    static const size_t CAPS[] = { 4, 5, 6, 7, 97, 128 };

    for (size_t i = 0; i < sizeof(msg); i++)
    {
        msg[i] = (uint8_t)(i * 7 + 3);
    }

    for (size_t s = 0; s < sizeof(SIZES) / sizeof(SIZES[0]); s++)
    {
        size_t want = uds_bytes_to_hex(msg, SIZES[s], whole, sizeof(whole));

        for (size_t c = 0; c < sizeof(CAPS) / sizeof(CAPS[0]); c++)
        {
            char piece[128];
            size_t off = 0;
            size_t o = 0;
            size_t n;

            while ((n = uds_hex_piece(msg, SIZES[s], &off, piece,
                                      CAPS[c])) > 0)
            {
                TEST_ASSERT_TRUE(n < CAPS[c]);            /* room for the NUL */
                TEST_ASSERT_EQUAL_size_t(n, strlen(piece));
                memcpy(joined + o, piece, n);
                o += n;
            }

            joined[o] = '\0';
            TEST_ASSERT_EQUAL_size_t(SIZES[s], off);
            TEST_ASSERT_EQUAL_size_t(want, o);
            TEST_ASSERT_EQUAL_STRING(whole, joined);
        }
    }

    /* a buffer too small for one byte, and the end of the message */
    char tiny[3] = "xx";
    size_t off = 0;

    TEST_ASSERT_EQUAL_size_t(0, uds_hex_piece(msg, 4, &off, tiny, sizeof(tiny)));
    TEST_ASSERT_EQUAL_STRING("", tiny);
    TEST_ASSERT_EQUAL_size_t(0, off);

    char piece[16];

    off = 4;
    TEST_ASSERT_EQUAL_size_t(0, uds_hex_piece(msg, 4, &off, piece,
                                              sizeof(piece)));
    TEST_ASSERT_EQUAL_STRING("", piece);
}

static void test_at_parse_single_frame(void)
{
    /* MIC/ELM headers-off single frame + prompt */
    uint8_t out[16];
    size_t n = 0;

    TEST_ASSERT_TRUE(uds_at_parse_response("62 F1 90 01 02 03\r\r>",
                                           out, sizeof(out), &n));
    TEST_ASSERT_EQUAL_size_t(6, n);
    TEST_ASSERT_EQUAL_UINT8(0x62, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x03, out[5]);
}

static void test_at_parse_multiline_isotp(void)
{
    /* ISO-TP multiline: length line + indexed frames (headers off) */
    uint8_t out[32];
    size_t n = 0;

    const char *r =
        "014\r0: 62 F1 90 31 32 33\r1: 34 35 36 37 38 39 41\r"
        "2: 42 43 44 45 46 47 48\r\r>";
    TEST_ASSERT_TRUE(uds_at_parse_response(r, out, sizeof(out), &n));
    TEST_ASSERT_EQUAL_size_t(0x14, n); /* length prefix "014" = 20 bytes */
    TEST_ASSERT_EQUAL_UINT8(0x62, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x48, out[19]);
}

static void test_at_parse_pending_lines_then_final(void)
{
    /* with the response-count digit the MIC prints every 7F xx 78 it rode
       out, then the final answer (bench 2026-09-16, erase routine) */
    uint8_t out[16];
    size_t n = 0;
    uint8_t pend = 0;

    const char *r = "7F 31 78\r7F 31 78\r7F 31 78\r71 01 FF 00 00\r\r>";
    TEST_ASSERT_TRUE(uds_at_parse_response_ex(r, out, sizeof(out), &n, &pend));
    TEST_ASSERT_EQUAL_size_t(5, n);
    TEST_ASSERT_EQUAL_UINT8(0x71, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[4]);
    TEST_ASSERT_EQUAL_UINT8(3, pend);

    /* the plain entry point still returns the final, pendings dropped */
    TEST_ASSERT_TRUE(uds_at_parse_response(r, out, sizeof(out), &n));
    TEST_ASSERT_EQUAL_size_t(5, n);

    /* only pendings, no final: nothing to return */
    TEST_ASSERT_FALSE(uds_at_parse_response_ex("7F 31 78\r7F 31 78\r\r>", out,
                                               sizeof(out), &n, &pend));
    TEST_ASSERT_EQUAL_UINT8(2, pend);

    /* a real negative response is NOT a pending: it is the answer */
    TEST_ASSERT_TRUE(uds_at_parse_response_ex("7F 22 31\r\r>", out, sizeof(out),
                                              &n, &pend));
    TEST_ASSERT_EQUAL_size_t(3, n);
    TEST_ASSERT_EQUAL_UINT8(0x31, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0, pend);
}

static void test_at_parse_pending_then_multiframe_final(void)
{
    uint8_t out[32];
    size_t n = 0;
    uint8_t pend = 0;

    const char *r =
        "7F 22 78\r7F 22 78\r"
        "014\r0: 62 F1 90 31 32 33\r1: 34 35 36 37 38 39 41\r"
        "2: 42 43 44 45 46 47 48\r\r>";
    TEST_ASSERT_TRUE(uds_at_parse_response_ex(r, out, sizeof(out), &n, &pend));
    TEST_ASSERT_EQUAL_size_t(0x14, n);
    TEST_ASSERT_EQUAL_UINT8(0x62, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x48, out[19]);
    TEST_ASSERT_EQUAL_UINT8(2, pend);
}

static void test_at_parse_error_and_empty(void)
{
    uint8_t out[16];
    size_t n = 0;

    TEST_ASSERT_FALSE(uds_at_parse_response("NO DATA\r\r>", out,
                                            sizeof(out), &n));
    TEST_ASSERT_FALSE(uds_at_parse_response("7F 22 31 ERROR", out,
                                            sizeof(out), &n));
    TEST_ASSERT_FALSE(uds_at_parse_response("\r\r>", out, sizeof(out), &n));
}

static void test_at_parse_trims_padding(void)
{
    /* 11 bytes announced, 13 printed: the last frame's padding is not
       part of the message (bench 2026-10-02, 19 02 08 with two DTCs) */
    uint8_t out[32];
    size_t n = 0;
    size_t announced = 0;

    const char *r =
        "00B\r0: 59 02 FF 04 20 00\r1: 09 01 71 00 09 00 00\r\r>";

    TEST_ASSERT_TRUE(uds_at_parse_response(r, out, sizeof(out), &n));
    TEST_ASSERT_EQUAL_size_t(11, n);
    TEST_ASSERT_EQUAL_UINT8(0x59, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x71, out[8]);
    TEST_ASSERT_EQUAL_UINT8(0x09, out[10]);

    /* the worker reports the announced size; a single frame reports 0 */
    TEST_ASSERT_TRUE(uds_at_parse_response_len(r, out, sizeof(out), &n, NULL,
                                               &announced));
    TEST_ASSERT_EQUAL_size_t(11, n);
    TEST_ASSERT_EQUAL_size_t(11, announced);

    TEST_ASSERT_TRUE(uds_at_parse_response_len("62 F1 90 01\r\r>", out,
                                               sizeof(out), &n, NULL,
                                               &announced));
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_EQUAL_size_t(0, announced);

    /* indexed lines without a length line: taken as printed */
    TEST_ASSERT_TRUE(uds_at_parse_response_len(
        "0: 59 02 FF 04 20 00\r1: 09 01 71 00 09 00 00\r\r>", out,
        sizeof(out), &n, NULL, &announced));
    TEST_ASSERT_EQUAL_size_t(13, n);
    TEST_ASSERT_EQUAL_size_t(0, announced);
}

static void test_at_parse_cut_multiframe(void)
{
    /* the response-count digit "1" stops the chip after the first frame
       (bench 2026-10-02): 6 of 11 bytes are not a message */
    uint8_t out[32];
    size_t n = 0;
    size_t announced = 0;
    uint8_t pend = 0;

    const char *r = "00B\r0: 59 02 FF 04 20 00\r\r>";

    TEST_ASSERT_TRUE(uds_at_parse_response_len(r, out, sizeof(out), &n, &pend,
                                               &announced));
    TEST_ASSERT_EQUAL_size_t(6, n);
    TEST_ASSERT_EQUAL_size_t(11, announced);

    TEST_ASSERT_FALSE(uds_at_parse_response_ex(r, out, sizeof(out), &n,
                                               &pend));
    TEST_ASSERT_EQUAL_size_t(0, n);
    TEST_ASSERT_FALSE(uds_at_parse_response(r, out, sizeof(out), &n));

    /* the same behind two responsePending lines (pendings do not count
       against the digit) */
    const char *p = "7F 22 78\r7F 22 78\r014\r0: 62 F1 90 31 57 43\r\r>";

    TEST_ASSERT_TRUE(uds_at_parse_response_len(p, out, sizeof(out), &n, &pend,
                                               &announced));
    TEST_ASSERT_EQUAL_size_t(6, n);
    TEST_ASSERT_EQUAL_size_t(20, announced);
    TEST_ASSERT_EQUAL_UINT8(2, pend);
    TEST_ASSERT_FALSE(uds_at_parse_response_ex(p, out, sizeof(out), &n,
                                               &pend));
}

static void test_at_lines_and_first_digit(void)
{
    /* first frame 6 bytes, 7 per consecutive frame */
    TEST_ASSERT_EQUAL_UINT8(1, uds_at_lines_for(1));
    TEST_ASSERT_EQUAL_UINT8(1, uds_at_lines_for(7));
    TEST_ASSERT_EQUAL_UINT8(2, uds_at_lines_for(8));
    TEST_ASSERT_EQUAL_UINT8(2, uds_at_lines_for(11));
    TEST_ASSERT_EQUAL_UINT8(2, uds_at_lines_for(13));
    TEST_ASSERT_EQUAL_UINT8(3, uds_at_lines_for(14));
    TEST_ASSERT_EQUAL_UINT8(3, uds_at_lines_for(20));
    TEST_ASSERT_EQUAL_UINT8(4, uds_at_lines_for(21));
    TEST_ASSERT_EQUAL_UINT8(15, uds_at_lines_for(6 + 14 * 7));
    TEST_ASSERT_EQUAL_UINT8(16, uds_at_lines_for(6 + 14 * 7 + 1));
    TEST_ASSERT_EQUAL_UINT8(255, uds_at_lines_for(4095));

    /* reads go out with "1"; the services that must not be sent twice
       go out without a digit */
    TEST_ASSERT_EQUAL_CHAR('1', uds_at_first_digit(0x22));
    TEST_ASSERT_EQUAL_CHAR('1', uds_at_first_digit(0x19));
    TEST_ASSERT_EQUAL_CHAR('1', uds_at_first_digit(0x3E));
    TEST_ASSERT_EQUAL_CHAR('1', uds_at_first_digit(0x2E));
    TEST_ASSERT_EQUAL_CHAR('\0', uds_at_first_digit(0x27));
    TEST_ASSERT_EQUAL_CHAR('\0', uds_at_first_digit(0x29));
    TEST_ASSERT_EQUAL_CHAR('\0', uds_at_first_digit(0x31));
    TEST_ASSERT_EQUAL_CHAR('\0', uds_at_first_digit(0x38));
    TEST_ASSERT_EQUAL_CHAR('\0', uds_at_first_digit(0x84));
}

/* ---- the AT transaction against a scripted chip ---------------------------- */
#define FAKE_MAX 4

static const char *s_script[FAKE_MAX]; /* the chip's answer per request   */
static char s_sent[FAKE_MAX][48];      /* the request lines as they went  */
static int s_calls;

static esp_err_t fake_req(const char *cmd, char *resp, size_t resp_len,
                          uint32_t timeout_ms)
{
    (void)timeout_ms;

    if (s_calls >= FAKE_MAX || s_script[s_calls] == NULL)
    {
        return ESP_ERR_TIMEOUT;
    }

    snprintf(s_sent[s_calls], sizeof(s_sent[0]), "%s", cmd);
    snprintf(resp, resp_len, "%s", s_script[s_calls]);
    s_calls++;
    return ESP_OK;
}

static void fake_reset(const char *a, const char *b, const char *c)
{
    memset(s_sent, 0, sizeof(s_sent));
    s_script[0] = a;
    s_script[1] = b;
    s_script[2] = c;
    s_script[3] = NULL;
    s_calls = 0;
}

static const uds_addr_t FAKE_ADDR = { .tx_id = 0x7E0, .rx_id = 0x7E8 };

static void test_at_transceive_single_frame_one_request(void)
{
    const uint8_t req[] = { 0x22, 0xF4, 0x0C };
    uint8_t out[32];
    size_t n = 0;
    uint8_t pend = 9;

    fake_reset("62 F4 0C 0C 80\r\r>", NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, req,
                                           sizeof(req), out, sizeof(out), &n,
                                           150, 5000, &pend, true));
    TEST_ASSERT_EQUAL(1, s_calls);
    TEST_ASSERT_EQUAL_STRING("22F40C1", s_sent[0]);
    TEST_ASSERT_EQUAL_size_t(5, n);
    TEST_ASSERT_EQUAL_UINT8(0, pend);
}

static void test_at_transceive_asks_again_for_a_cut_answer(void)
{
    /* bench 2026-10-02: "1902081" came back as the first frame only and
       was handed over as a 6-byte answer */
    const uint8_t req[] = { 0x19, 0x02, 0x08 };
    uint8_t out[32];
    size_t n = 0;
    uint8_t pend = 9;

    fake_reset("00B\r0: 59 02 FF 04 20 00\r\r>",
               "00B\r0: 59 02 FF 04 20 00\r1: 09 01 71 00 09 00 00\r\r>",
               NULL);
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, req,
                                           sizeof(req), out, sizeof(out), &n,
                                           150, 5000, &pend, true));
    TEST_ASSERT_EQUAL(2, s_calls);
    TEST_ASSERT_EQUAL_STRING("1902081", s_sent[0]);
    TEST_ASSERT_EQUAL_STRING("1902082", s_sent[1]);
    TEST_ASSERT_EQUAL_size_t(11, n);
    TEST_ASSERT_EQUAL_UINT8(0x09, out[10]);
    TEST_ASSERT_EQUAL_UINT8(0, pend);

    /* behind responsePending lines, and the 3-line digit */
    const uint8_t vin[] = { 0x22, 0xF1, 0x90 };

    fake_reset("7F 22 78\r7F 22 78\r014\r0: 62 F1 90 31 57 43\r\r>",
               "7F 22 78\r014\r0: 62 F1 90 31 57 43\r"
               "1: 41 4E 30 46 57 30 50\r2: 30 30 30 30 30 30 31\r\r>",
               NULL);
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, vin,
                                           sizeof(vin), out, sizeof(out), &n,
                                           150, 5000, &pend, true));
    TEST_ASSERT_EQUAL(2, s_calls);
    TEST_ASSERT_EQUAL_STRING("22F1901", s_sent[0]);
    TEST_ASSERT_EQUAL_STRING("22F1903", s_sent[1]);
    TEST_ASSERT_EQUAL_size_t(20, n);
    TEST_ASSERT_EQUAL_UINT8(0x31, out[19]);
    TEST_ASSERT_EQUAL_UINT8(1, pend); /* of the answer that was kept */
}

static void test_at_transceive_long_answer_goes_without_digit(void)
{
    /* 105 bytes = 16 lines: more than one hex digit can ask for. Each
       consecutive line starts with its own number; the 16th carries one
       message byte and six of padding. */
    const uint8_t req[] = { 0x22, 0xF1, 0x00 };
    static char full[16 * 32 + 16];
    static uint8_t out[128];
    size_t n = 0;
    size_t o = 0;

    o += (size_t)snprintf(full + o, sizeof(full) - o,
                          "069\r0: 62 F1 00 A0 A1 A2\r");

    for (int line = 1; line < 16; line++)
    {
        o += (size_t)snprintf(full + o, sizeof(full) - o,
                              "%X: %02X 11 12 13 14 15 16\r", line & 0xF,
                              line);
    }

    snprintf(full + o, sizeof(full) - o, "\r>");

    fake_reset("069\r0: 62 F1 00 A0 A1 A2\r\r>", full, NULL);
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, req,
                                           sizeof(req), out, sizeof(out), &n,
                                           150, 5000, NULL, true));
    TEST_ASSERT_EQUAL(2, s_calls);
    TEST_ASSERT_EQUAL_STRING("22F1001", s_sent[0]);
    TEST_ASSERT_EQUAL_STRING("22F100", s_sent[1]);
    TEST_ASSERT_EQUAL_size_t(105, n);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x0F, out[104]); /* line F, its only byte */
}

static void test_at_transceive_never_repeats_a_state_changing_service(void)
{
    /* requestSeed: no digit, so a long seed arrives whole in ONE request */
    const uint8_t seed[] = { 0x27, 0x01 };
    uint8_t out[32];
    size_t n = 0;

    fake_reset("00A\r0: 67 01 11 22 33 44\r1: 55 66 77 88 00 00 00\r\r>",
               NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, seed,
                                           sizeof(seed), out, sizeof(out), &n,
                                           150, 5000, NULL, true));
    TEST_ASSERT_EQUAL(1, s_calls);
    TEST_ASSERT_EQUAL_STRING("2701", s_sent[0]);
    TEST_ASSERT_EQUAL_size_t(10, n);
    TEST_ASSERT_EQUAL_UINT8(0x88, out[9]);

    /* and when the chip stops early anyway: an error, not a second start */
    const uint8_t routine[] = { 0x31, 0x01, 0xFF, 0x00 };

    n = 7;
    fake_reset("00A\r0: 71 01 FF 00 01 02\r\r>",
               "00A\r0: 71 01 FF 00 01 02\r1: 03 04 05 06 00 00 00\r\r>",
               NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, routine,
                                           sizeof(routine), out, sizeof(out),
                                           &n, 150, 5000, NULL, true));
    TEST_ASSERT_EQUAL(1, s_calls);
    TEST_ASSERT_EQUAL_STRING("3101FF00", s_sent[0]);
    TEST_ASSERT_EQUAL_size_t(0, n);
}

static void test_at_setup_follows_the_bus_bitrate(void)
{
    /* the setup pins the protocol of the bitrate the bus runs at: a
       request pinned to the other one destroys a live bus's traffic */
    TEST_ASSERT_EQUAL_CHAR('6', uds_at_protocol(false));
    TEST_ASSERT_EQUAL_CHAR('7', uds_at_protocol(true));

    uds_at_set_can_kbps(250);
    TEST_ASSERT_EQUAL_CHAR('8', uds_at_protocol(false));
    TEST_ASSERT_EQUAL_CHAR('9', uds_at_protocol(true));

    /* the first line of a target setup carries it */
    const uint8_t req[] = { 0x3E, 0x00 };
    const uds_addr_t ext = { .tx_id = 0x18DA58F1, .rx_id = 0x18DAF158,
                             .ext_id = true };
    uint8_t out[8];
    size_t n = 0;

    fake_reset("OK\r\r>", NULL, NULL);
    (void)uds_at_transceive_ex(fake_req, &ext, req, sizeof(req), out,
                               sizeof(out), &n, 150, 5000, NULL, false);
    TEST_ASSERT_EQUAL_STRING("ATTP9", s_sent[0]);

    fake_reset("OK\r\r>", NULL, NULL);
    (void)uds_at_transceive_ex(fake_req, &FAKE_ADDR, req, sizeof(req), out,
                               sizeof(out), &n, 150, 5000, NULL, false);
    TEST_ASSERT_EQUAL_STRING("ATTP8", s_sent[0]);

    /* anything but 250 means 500, the bitrate it always used */
    uds_at_set_can_kbps(125);
    TEST_ASSERT_EQUAL_CHAR('6', uds_at_protocol(false));
    uds_at_set_can_kbps(500);
    fake_reset("OK\r\r>", NULL, NULL);
    (void)uds_at_transceive_ex(fake_req, &FAKE_ADDR, req, sizeof(req), out,
                               sizeof(out), &n, 150, 5000, NULL, false);
    TEST_ASSERT_EQUAL_STRING("ATTP6", s_sent[0]);
}

/* a chip that takes every setup line and answers the request: every line
   that went to it is kept */
#define SETUP_MAX 24

static char s_setup[SETUP_MAX][24];
static int s_setup_n;

static esp_err_t fake_setup_req(const char *cmd, char *resp, size_t resp_len,
                                uint32_t timeout_ms)
{
    (void)timeout_ms;

    if (s_setup_n < SETUP_MAX)
    {
        snprintf(s_setup[s_setup_n], sizeof(s_setup[0]), "%s", cmd);
    }

    s_setup_n++;
    snprintf(resp, resp_len, "%s",
             (cmd[0] == 'A' && cmd[1] == 'T') ? "OK\r\r>" : "7E 00\r\r>");
    return ESP_OK;
}

static int setup_line(const char *cmd)
{
    for (int i = 0; i < s_setup_n && i < SETUP_MAX; i++)
    {
        if (strcmp(s_setup[i], cmd) == 0)
        {
            return i;
        }
    }

    return -1;
}

static void test_at_setup_puts_back_what_another_requester_left(void)
{
    /* bench 2026-11-26: an AutoPID row's `ATFCSH710;ATFCSD300000;ATFCSM1`
       stayed in effect under the UDS setup, the chip sent its flow control
       to 0x710 and the ECU on 7E0 sent its first frame and nothing more.
       The setup undoes what a row or an app may have left, before the
       request goes out. */
    static const char *const WANT[] =
    {
        "ATTP6", "ATSH7E0", "ATCRA7E8", "ATH0", "ATCAF1", "ATS1",
        "ATCEA", "ATFCSM0", "ATCFC1", "ATAT1", "ATV0", "ATE0", "ATR1",
    };
    const uint8_t req[] = { 0x3E, 0x00 };
    uint8_t out[8];
    size_t n = 0;

    uds_at_set_can_kbps(500);
    s_setup_n = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_setup_req, &FAKE_ADDR, req,
                                           sizeof(req), out, sizeof(out), &n,
                                           250, 5000, NULL, false));
    TEST_ASSERT_TRUE(s_setup_n <= SETUP_MAX);

    int request_at = setup_line("3E001");

    TEST_ASSERT_EQUAL_INT_MESSAGE(s_setup_n - 1, request_at,
                                  "the request is the last line");
    TEST_ASSERT_EQUAL_size_t(2, n);

    for (size_t i = 0; i < sizeof(WANT) / sizeof(WANT[0]); i++)
    {
        int at = setup_line(WANT[i]);

        TEST_ASSERT_TRUE_MESSAGE(at >= 0 && at < request_at, WANT[i]);
    }

    /* the chip's own reply wait from p2 (250 ms / 4 ms) closes the setup */
    TEST_ASSERT_EQUAL_STRING("ATST3E", s_setup[request_at - 1]);

    /* a steady conversation still costs one line */
    s_setup_n = 0;
    TEST_ASSERT_EQUAL(ESP_OK,
                      uds_at_transceive_ex(fake_setup_req, &FAKE_ADDR, req,
                                           sizeof(req), out, sizeof(out), &n,
                                           250, 5000, NULL, true));
    TEST_ASSERT_EQUAL_INT(1, s_setup_n);
    TEST_ASSERT_EQUAL_STRING("3E001", s_setup[0]);
}

static void test_isotp_answer_still_arriving(void)
{
    /* bench 2026-11-26, native ISO-TP: an answer of 1024 bytes takes longer
       than P2 (250 ms) to arrive and the request ended in a timeout with
       the ECU in the middle of it. When the wait for the START of the
       answer runs out, the transport asks whether frames are coming in. */

    /* the ECU is sending: its last frame 1 ms ago, the request 250 ms ago */
    TEST_ASSERT_TRUE(uds_isotp_answer_arriving(1, 250));
    TEST_ASSERT_TRUE(uds_isotp_answer_arriving(999, 4000));

    /* a silent ECU: the last frame on its id is older than the request
       (the answer to the one before), or there never was one */
    TEST_ASSERT_FALSE(uds_isotp_answer_arriving(900, 250));
    TEST_ASSERT_FALSE(uds_isotp_answer_arriving(250, 250));
    TEST_ASSERT_FALSE(uds_isotp_answer_arriving(UINT32_MAX, 250));
    TEST_ASSERT_FALSE(uds_isotp_answer_arriving(UINT32_MAX, UINT32_MAX));

    /* an answer that stopped in the middle: N_Cr without a frame */
    TEST_ASSERT_FALSE(uds_isotp_answer_arriving(UDS_ISOTP_N_CR_MS, 4000));
    TEST_ASSERT_FALSE(uds_isotp_answer_arriving(1500, 4000));
}

static void test_at_transceive_gives_up_on_a_chip_that_keeps_cutting(void)
{
    const uint8_t req[] = { 0x19, 0x02, 0x08 };
    uint8_t out[32];
    size_t n = 7;
    const char *cut = "00B\r0: 59 02 FF 04 20 00\r\r>";

    fake_reset(cut, cut, cut);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE,
                      uds_at_transceive_ex(fake_req, &FAKE_ADDR, req,
                                           sizeof(req), out, sizeof(out), &n,
                                           150, 5000, NULL, true));
    TEST_ASSERT_EQUAL(3, s_calls);
    TEST_ASSERT_EQUAL_size_t(0, n); /* never a 6-byte "answer" */

    /* the second time it is asked without a digit: frames left over from
       the answer before cannot use up a line count that is not there */
    TEST_ASSERT_EQUAL_STRING("1902081", s_sent[0]);
    TEST_ASSERT_EQUAL_STRING("1902082", s_sent[1]);
    TEST_ASSERT_EQUAL_STRING("190208", s_sent[2]);
}

static void test_at_parse_the_largest_message_fits_its_size(void)
{
    /* bench 2026-11-26: 4095 bytes, the largest ISO-TP message, are 586
       frames whose last carries one byte and six of padding: 4101 bytes on
       the lines. The padding counted against the buffer before it was cut
       off, and the one message that fills the 4096 bytes every caller has
       was refused (ESP_ERR_INVALID_RESPONSE on the route). */
    static char text[586 * 26 + 32];
    static uint8_t out[4096];
    size_t o = 0;
    size_t n = 0;
    size_t announced = 0;
    size_t sent = 0;

    o += (size_t)snprintf(text + o, sizeof(text) - o, "FFF \r0:");

    for (; sent < 6; sent++)
    {
        o += (size_t)snprintf(text + o, sizeof(text) - o, " %02X",
                              (unsigned)(sent & 0xFF));
    }

    for (int line = 1; line <= 585; line++)
    {
        o += (size_t)snprintf(text + o, sizeof(text) - o, " \r%X:",
                              line & 0xF);

        for (int b = 0; b < 7; b++, sent++)
        {
            /* behind the 4095th byte: the padding */
            o += (size_t)snprintf(text + o, sizeof(text) - o, " %02X",
                                  (sent < 4095) ? (unsigned)(sent & 0xFF)
                                                : 0xAAu);
        }
    }

    snprintf(text + o, sizeof(text) - o, " \r\r>");
    TEST_ASSERT_EQUAL_size_t(4101, sent);

    TEST_ASSERT_TRUE(uds_at_parse_response_len(text, out, sizeof(out), &n,
                                               NULL, &announced));
    TEST_ASSERT_EQUAL_size_t(4095, announced);
    TEST_ASSERT_EQUAL_size_t(4095, n);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[0]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(4094 & 0xFF), out[4094]);

    /* a buffer smaller than the message is still an overflow */
    TEST_ASSERT_FALSE(uds_at_parse_response_len(text, out, 4094, &n, NULL,
                                                &announced));
}

static void test_at_wait_behind_a_cut_answer(void)
{
    /* bench 2026-11-26 (a capture of the bus): the chip stops printing at
       its line count but has sent its flow control, the ECU sends the rest
       of the answer, and a request 0.9 ms later is cut by those frames.
       The wait is for the frames still owed: 2 ms each and 10 on top. */

    /* the VIN, 20 bytes = 3 frames, the first one printed: 2 owed */
    TEST_ASSERT_EQUAL_UINT32(14, uds_at_tail_wait_ms(20, 6, 0));
    /* 8 bytes = 2 frames, 13 bytes = 2 frames, 14 bytes = 3 */
    TEST_ASSERT_EQUAL_UINT32(12, uds_at_tail_wait_ms(8, 6, 0));
    TEST_ASSERT_EQUAL_UINT32(12, uds_at_tail_wait_ms(13, 6, 0));
    TEST_ASSERT_EQUAL_UINT32(14, uds_at_tail_wait_ms(14, 6, 0));
    /* two frames printed of the VIN's three */
    TEST_ASSERT_EQUAL_UINT32(12, uds_at_tail_wait_ms(20, 13, 0));
    /* nothing owed (a message that was whole): the base alone */
    TEST_ASSERT_EQUAL_UINT32(10, uds_at_tail_wait_ms(20, 20, 0));
    TEST_ASSERT_EQUAL_UINT32(10, uds_at_tail_wait_ms(7, 6, 0));

    /* the largest message, 4095 bytes = 586 frames: under the cap */
    TEST_ASSERT_EQUAL_UINT32(1180, uds_at_tail_wait_ms(4095, 6, 0));

    /* cut again: four times as patient, and capped */
    TEST_ASSERT_EQUAL_UINT32(56, uds_at_tail_wait_ms(20, 6, 1));
    TEST_ASSERT_EQUAL_UINT32(3000, uds_at_tail_wait_ms(4095, 6, 1));
    TEST_ASSERT_EQUAL_UINT32(3000, uds_at_tail_wait_ms(4095, 6, 2));
}

/* ---- UDS DTC codec (uds_dtc_codec.c: TASK_dtc §12) ----------------------- */
#include "uds_dtc.h"

void test_dtc_requests(void)
{
    uint8_t b[4];

    TEST_ASSERT_EQUAL(3, uds_dtc_req_count(0x08, b));
    TEST_ASSERT_EQUAL_HEX8(0x19, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0x08, b[2]);

    TEST_ASSERT_EQUAL(3, uds_dtc_req_by_status(0x04, b));
    TEST_ASSERT_EQUAL_HEX8(0x02, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0x04, b[2]);

    TEST_ASSERT_EQUAL(2, uds_dtc_req_supported(b));
    TEST_ASSERT_EQUAL_HEX8(0x0A, b[1]);

    /* clear: all groups + one specific DTC (the true per-code clear) */
    TEST_ASSERT_EQUAL(4, uds_dtc_req_clear(NULL, b));
    TEST_ASSERT_EQUAL_HEX8(0x14, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, b[3]);

    uint8_t one[3] = { 0x04, 0x20, 0x08 };

    TEST_ASSERT_EQUAL(4, uds_dtc_req_clear(one, b));
    TEST_ASSERT_EQUAL_HEX8(0x04, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0x20, b[2]);
    TEST_ASSERT_EQUAL_HEX8(0x08, b[3]);
}

void test_dtc_parse_list_shapes(void)
{
    uds_dtc_t out[8];
    uint8_t mask = 0;

    /* two records (the ECU-sim shape: FTB 0, status from dtc_status) */
    const uint8_t two[] = { 0x59, 0x02, 0xFF,
                            0x04, 0x20, 0x00, 0x08,
                            0x01, 0x71, 0x00, 0x0C };

    TEST_ASSERT_EQUAL(2, uds_dtc_parse_list(two, sizeof(two), &mask,
                                            out, 8));
    TEST_ASSERT_EQUAL_HEX8(0xFF, mask);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[0].hi);
    TEST_ASSERT_EQUAL_HEX8(0x20, out[0].mid);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[0].ftb);
    TEST_ASSERT_EQUAL_HEX8(0x08, out[0].status);
    TEST_ASSERT_EQUAL_HEX8(0x0C, out[1].status);

    /* empty list (header only) */
    const uint8_t none[] = { 0x59, 0x02, 0xFF };

    TEST_ASSERT_EQUAL(0, uds_dtc_parse_list(none, sizeof(none), &mask,
                                            out, 8));

    /* truncated trailing record: ignored, earlier ones kept */
    const uint8_t trunc[] = { 0x59, 0x0A, 0xFF,
                              0x04, 0x20, 0x08, 0x08,
                              0x01, 0x71 };

    TEST_ASSERT_EQUAL(1, uds_dtc_parse_list(trunc, sizeof(trunc), &mask,
                                            out, 8));
    TEST_ASSERT_EQUAL_HEX8(0x08, out[0].ftb); /* sub-type carried      */

    /* out-cap: keep what fits */
    TEST_ASSERT_EQUAL(1, uds_dtc_parse_list(two, sizeof(two), &mask,
                                            out, 1));

    /* wrong service / wrong sub / NULL */
    const uint8_t neg[] = { 0x7F, 0x19, 0x31 };

    TEST_ASSERT_EQUAL(-1, uds_dtc_parse_list(neg, sizeof(neg), &mask,
                                             out, 8));

    const uint8_t sub[] = { 0x59, 0x04, 0x00 };

    TEST_ASSERT_EQUAL(-1, uds_dtc_parse_list(sub, sizeof(sub), &mask,
                                             out, 8));
    TEST_ASSERT_EQUAL(-1, uds_dtc_parse_list(NULL, 0, &mask, out, 8));
}

void test_dtc_parse_count_and_clear(void)
{
    uint8_t mask = 0;
    uint16_t count = 0;
    const uint8_t ok[] = { 0x59, 0x01, 0xFF, 0x01, 0x00, 0x02 };

    TEST_ASSERT_TRUE(uds_dtc_parse_count(ok, sizeof(ok), &mask, &count));
    TEST_ASSERT_EQUAL_HEX8(0xFF, mask);
    TEST_ASSERT_EQUAL(2, count);

    const uint8_t big[] = { 0x59, 0x01, 0xFF, 0x01, 0x01, 0x2C };

    TEST_ASSERT_TRUE(uds_dtc_parse_count(big, sizeof(big), NULL,
                                         &count));
    TEST_ASSERT_EQUAL(300, count);
    TEST_ASSERT_FALSE(uds_dtc_parse_count(ok, 5, &mask, &count));

    const uint8_t cleared[] = { 0x54 };
    const uint8_t nrc[] = { 0x7F, 0x14, 0x31 };

    TEST_ASSERT_TRUE(uds_dtc_clear_ok(cleared, 1));
    TEST_ASSERT_FALSE(uds_dtc_clear_ok(nrc, 3));
    TEST_ASSERT_FALSE(uds_dtc_clear_ok(NULL, 0));
}

void test_dtc_format_suffix_rules(void)
{
    char code[10];

    /* FTB 0 -> NO suffix (identical to the OBD code: stable diffs
       when `auto` flips protocols between scans) */
    uds_dtc_format(0x04, 0x20, 0x00, code);
    TEST_ASSERT_EQUAL_STRING("P0420", code);

    uds_dtc_format(0x04, 0x20, 0x08, code);
    TEST_ASSERT_EQUAL_STRING("P0420-08", code);

    /* every letter + high nibbles */
    uds_dtc_format(0x41, 0x23, 0x00, code);
    TEST_ASSERT_EQUAL_STRING("C0123", code);
    uds_dtc_format(0x9A, 0xBC, 0xFF, code);
    TEST_ASSERT_EQUAL_STRING("B1ABC-FF", code);
    uds_dtc_format(0xFF, 0xFF, 0x01, code);
    TEST_ASSERT_EQUAL_STRING("U3FFF-01", code);
}

void test_dtc_unformat_roundtrip(void)
{
    uint8_t hi, mid, ftb;

    TEST_ASSERT_TRUE(uds_dtc_unformat("P0420", &hi, &mid, &ftb));
    TEST_ASSERT_EQUAL_HEX8(0x04, hi);
    TEST_ASSERT_EQUAL_HEX8(0x20, mid);
    TEST_ASSERT_EQUAL_HEX8(0x00, ftb);

    TEST_ASSERT_TRUE(uds_dtc_unformat("P0420-08", &hi, &mid, &ftb));
    TEST_ASSERT_EQUAL_HEX8(0x08, ftb);

    TEST_ASSERT_TRUE(uds_dtc_unformat("u3fff-ff", &hi, &mid, &ftb));
    TEST_ASSERT_EQUAL_HEX8(0xFF, hi);
    TEST_ASSERT_EQUAL_HEX8(0xFF, mid);
    TEST_ASSERT_EQUAL_HEX8(0xFF, ftb);

    /* round-trip every shape */
    for (int i = 0; i < 256; i += 37)
    {
        char code[10];
        uint8_t h2, m2, f2;

        uds_dtc_format((uint8_t)i, (uint8_t)(255 - i), (uint8_t)i, code);
        TEST_ASSERT_TRUE(uds_dtc_unformat(code, &h2, &m2, &f2));
        TEST_ASSERT_EQUAL_HEX8((uint8_t)i, h2);
        TEST_ASSERT_EQUAL_HEX8((uint8_t)(255 - i), m2);
        TEST_ASSERT_EQUAL_HEX8((uint8_t)i, f2);
    }

    TEST_ASSERT_FALSE(uds_dtc_unformat("X0420", &hi, &mid, &ftb));
    TEST_ASSERT_FALSE(uds_dtc_unformat("P042", &hi, &mid, &ftb));
    TEST_ASSERT_FALSE(uds_dtc_unformat("P0420-8", &hi, &mid, &ftb));
    TEST_ASSERT_FALSE(uds_dtc_unformat("P0420-088", &hi, &mid, &ftb));
    TEST_ASSERT_FALSE(uds_dtc_unformat("P0420x", &hi, &mid, &ftb));
    TEST_ASSERT_FALSE(uds_dtc_unformat(NULL, &hi, &mid, &ftb));
}

/* ---- WWH-OBD (ISO 27145-3) / SAE J1979-2: 19 42, 19 55, 14 FFFF33 ------------- */

void test_wwh_requests(void)
{
    uint8_t req[5];

    TEST_ASSERT_EQUAL_size_t(5, uds_wwh_req_by_mask(
                                    UDS_WWH_FGID_EMISSIONS,
                                    UDS_DTC_STATUS_CONFIRMED,
                                    UDS_WWH_SEVERITY_CLASSES, req));

    const uint8_t confirmed[] = { 0x19, 0x42, 0x33, 0x08, 0x1E };

    TEST_ASSERT_EQUAL_HEX8_ARRAY(confirmed, req, 5);

    TEST_ASSERT_EQUAL_size_t(3, uds_wwh_req_permanent(
                                    UDS_WWH_FGID_EMISSIONS, req));

    const uint8_t permanent[] = { 0x19, 0x55, 0x33 };

    TEST_ASSERT_EQUAL_HEX8_ARRAY(permanent, req, 3);

    TEST_ASSERT_EQUAL_size_t(4, uds_wwh_req_clear(UDS_WWH_FGID_EMISSIONS,
                                                  req));

    const uint8_t clear[] = { 0x14, 0xFF, 0xFF, 0x33 };

    TEST_ASSERT_EQUAL_HEX8_ARRAY(clear, req, 4);
}

void test_wwh_parse_by_mask_captured(void)
{
    /* bench capture 2026-10-03 (19 42 33 0C 1E, format 04): P0420
       confirmed, class A; P2463-1F pending, class B1; the chip printed
       the last frame's padding, which is not a record */
    const uint8_t resp[] =
    {
        0x59, 0x42, 0x33, 0xFF, 0xFF, 0x04,
        0x02, 0x04, 0x20, 0x00, 0x08,
        0x04, 0x24, 0x63, 0x1F, 0x04,
    };
    uds_wwh_dtc_t rec[4];
    uint8_t fmt = 0;
    char text[UDS_DTC_TEXT_LEN];

    TEST_ASSERT_EQUAL(2, uds_wwh_parse_by_mask(resp, sizeof(resp), 0x33,
                                               &fmt, rec, 4));
    TEST_ASSERT_EQUAL_HEX8(UDS_DTC_FORMAT_J2012_04, fmt);
    TEST_ASSERT_EQUAL_HEX8(0x02, rec[0].severity);
    TEST_ASSERT_EQUAL_HEX8(0x08, rec[0].status);
    uds_wwh_dtc_text(fmt, &rec[0], text);
    TEST_ASSERT_EQUAL_STRING("P0420", text);
    TEST_ASSERT_EQUAL_HEX8(0x04, rec[1].severity);
    TEST_ASSERT_EQUAL_HEX8(0x04, rec[1].status);
    uds_wwh_dtc_text(fmt, &rec[1], text);
    TEST_ASSERT_EQUAL_STRING("P2463-1F", text);

    /* an ECU with no code answers the header alone */
    const uint8_t none[] = { 0x59, 0x42, 0x33, 0xFF, 0xFF, 0x04 };

    TEST_ASSERT_EQUAL(0, uds_wwh_parse_by_mask(none, sizeof(none), 0x33,
                                               &fmt, rec, 4));

    /* a record cut short is not a record; room for one keeps one */
    TEST_ASSERT_EQUAL(1, uds_wwh_parse_by_mask(resp, sizeof(resp) - 2, 0x33,
                                               &fmt, rec, 4));
    TEST_ASSERT_EQUAL(1, uds_wwh_parse_by_mask(resp, sizeof(resp), 0x33,
                                               &fmt, rec, 1));

    /* not this answer: another group, another sub-function, a negative
       response, too short, nothing */
    const uint8_t nrc[] = { 0x7F, 0x19, 0x31 };

    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_by_mask(resp, sizeof(resp), 0xD0,
                                                &fmt, rec, 4));
    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_permanent(resp, sizeof(resp), 0x33,
                                                  &fmt, rec, 4));
    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_by_mask(nrc, sizeof(nrc), 0x33,
                                                &fmt, rec, 4));
    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_by_mask(resp, 5, 0x33, &fmt, rec,
                                                4));
    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_by_mask(NULL, 0, 0x33, &fmt, rec,
                                                4));
}

void test_wwh_parse_permanent_and_bounds(void)
{
    /* 59 55 33 <avail> <format> then four bytes a record, no severity */
    const uint8_t resp[] =
    {
        0x59, 0x55, 0x33, 0xFF, 0x04,
        0x04, 0x20, 0x00, 0x08,
        0x01, 0x71, 0x00, 0x08,
        0xAA,                       /* a stray byte: not a record */
    };
    uds_wwh_dtc_t rec[4];
    uint8_t fmt = 0;
    char text[UDS_DTC_TEXT_LEN];

    TEST_ASSERT_EQUAL(2, uds_wwh_parse_permanent(resp, sizeof(resp), 0x33,
                                                 &fmt, rec, 4));
    TEST_ASSERT_EQUAL_HEX8(UDS_DTC_FORMAT_J2012_04, fmt);
    TEST_ASSERT_EQUAL_HEX8(0x00, rec[0].severity);
    uds_wwh_dtc_text(fmt, &rec[1], text);
    TEST_ASSERT_EQUAL_STRING("P0171", text);

    const uint8_t none[] = { 0x59, 0x55, 0x33, 0xFF, 0x04 };

    TEST_ASSERT_EQUAL(0, uds_wwh_parse_permanent(none, sizeof(none), 0x33,
                                                 &fmt, rec, 4));
    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_permanent(none, 4, 0x33, &fmt, rec,
                                                  4));
    TEST_ASSERT_EQUAL(-1, uds_wwh_parse_by_mask(resp, sizeof(resp), 0x33,
                                                &fmt, rec, 4));

    /* counting without a buffer is not offered: no room, no records */
    TEST_ASSERT_EQUAL(0, uds_wwh_parse_permanent(resp, sizeof(resp), 0x33,
                                                 NULL, NULL, 0));
}

void test_wwh_dtc_text_formats(void)
{
    char text[UDS_DTC_TEXT_LEN];

    /* SAE J1939-73: SPN 3226 (0x0C9A), FMI 4 -> 9A 0C 04 */
    const uds_wwh_dtc_t nox = { { 0x9A, 0x0C, 0x04 }, 0x08, 0x02 };

    uds_wwh_dtc_text(UDS_DTC_FORMAT_J1939, &nox, text);
    TEST_ASSERT_EQUAL_STRING("SPN3226-4", text);

    /* the largest SPN and FMI: 19 and 5 bits */
    const uds_wwh_dtc_t top = { { 0xFF, 0xFF, 0xFF }, 0x08, 0x02 };

    uds_wwh_dtc_text(UDS_DTC_FORMAT_J1939, &top, text);
    TEST_ASSERT_EQUAL_STRING("SPN524287-31", text);

    /* SPN 110 (coolant temperature), FMI 0 */
    const uds_wwh_dtc_t coolant = { { 0x6E, 0x00, 0x00 }, 0x08, 0x02 };

    uds_wwh_dtc_text(UDS_DTC_FORMAT_J1939, &coolant, text);
    TEST_ASSERT_EQUAL_STRING("SPN110-0", text);

    /* the same bytes read as J2012: every other format id is J2012 text */
    uds_wwh_dtc_text(UDS_DTC_FORMAT_J2012_04, &nox, text);
    TEST_ASSERT_EQUAL_STRING("B1A0C-04", text);
    uds_wwh_dtc_text(0x00, &coolant, text);
    TEST_ASSERT_EQUAL_STRING("C2E00", text);
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_positive_response);
    RUN_TEST(test_negative_response);
    RUN_TEST(test_pending_response);
    RUN_TEST(test_short_and_null_inputs);
    RUN_TEST(test_response_matches);
    RUN_TEST(test_hex_to_bytes_ok);
    RUN_TEST(test_hex_to_bytes_rejects);
    RUN_TEST(test_bytes_to_hex);
    RUN_TEST(test_hex_pieces_are_the_whole_answer);
    RUN_TEST(test_at_parse_single_frame);
    RUN_TEST(test_at_parse_multiline_isotp);
    RUN_TEST(test_at_parse_pending_lines_then_final);
    RUN_TEST(test_at_parse_pending_then_multiframe_final);
    RUN_TEST(test_at_parse_error_and_empty);
    RUN_TEST(test_at_parse_trims_padding);
    RUN_TEST(test_at_parse_cut_multiframe);
    RUN_TEST(test_at_lines_and_first_digit);
    RUN_TEST(test_at_transceive_single_frame_one_request);
    RUN_TEST(test_at_transceive_asks_again_for_a_cut_answer);
    RUN_TEST(test_at_transceive_long_answer_goes_without_digit);
    RUN_TEST(test_at_transceive_never_repeats_a_state_changing_service);
    RUN_TEST(test_at_transceive_gives_up_on_a_chip_that_keeps_cutting);
    RUN_TEST(test_at_setup_follows_the_bus_bitrate);
    RUN_TEST(test_at_setup_puts_back_what_another_requester_left);
    RUN_TEST(test_isotp_answer_still_arriving);
    RUN_TEST(test_at_wait_behind_a_cut_answer);
    RUN_TEST(test_at_parse_the_largest_message_fits_its_size);
    RUN_TEST(test_dtc_requests);
    RUN_TEST(test_dtc_parse_list_shapes);
    RUN_TEST(test_dtc_parse_count_and_clear);
    RUN_TEST(test_dtc_format_suffix_rules);
    RUN_TEST(test_dtc_unformat_roundtrip);
    RUN_TEST(test_wwh_requests);
    RUN_TEST(test_wwh_parse_by_mask_captured);
    RUN_TEST(test_wwh_parse_permanent_and_bounds);
    RUN_TEST(test_wwh_dtc_text_formats);
    UNITY_END();
}
