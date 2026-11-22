/**
 * @file test_send.c
 * @brief can.send (autopid_send.h, GitHub issue #1072): a request's JSON
 *        into frames, what is refused and with which sentence, where the
 *        chip is taken for a frame, the slot definitions with 29-bit ids
 *        and the single-wire wake mode, the words the chip is given (and
 *        that every one of them is undone by the restore), and how the
 *        chip's reply reads.
 */
#include <string.h>

#include "unity.h"

#include "cJSON.h"

#include "autopid_bus.h"
#include "autopid_private.h"
#include "autopid_send.h"

static ap_send_req_t s_req;
static char s_err[200];

static bool parse(const char *json)
{
    cJSON *o = cJSON_Parse(json);

    TEST_ASSERT_NOT_NULL_MESSAGE(o, json);
    s_err[0] = '\0';

    bool ok = ap_send_parse(o, &s_req, s_err, sizeof(s_err));

    cJSON_Delete(o);
    return ok;
}

void test_send_hex(void)
{
    uint8_t b[8];
    size_t n = 99;

    TEST_ASSERT_TRUE(ap_send_hex("80 01 FF", b, sizeof(b), &n));
    TEST_ASSERT_EQUAL(3, n);
    TEST_ASSERT_EQUAL_HEX8(0x80, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, b[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, b[2]);

    TEST_ASSERT_TRUE(ap_send_hex("8001ff", b, sizeof(b), &n));
    TEST_ASSERT_EQUAL(3, n);
    TEST_ASSERT_EQUAL_HEX8(0xFF, b[2]);

    TEST_ASSERT_TRUE(ap_send_hex("0x80,0x01, 0xfF", b, sizeof(b), &n));
    TEST_ASSERT_EQUAL(3, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, b[1]);

    /* nothing is no bytes, not an error */
    TEST_ASSERT_TRUE(ap_send_hex("", b, sizeof(b), &n));
    TEST_ASSERT_EQUAL(0, n);
    TEST_ASSERT_TRUE(ap_send_hex(NULL, b, sizeof(b), &n));
    TEST_ASSERT_EQUAL(0, n);

    TEST_ASSERT_TRUE(ap_send_hex("00FFFFFFFFFF0000", b, sizeof(b), &n));
    TEST_ASSERT_EQUAL(8, n);

    /* half a byte, a letter that is no digit, a ninth byte */
    TEST_ASSERT_FALSE(ap_send_hex("800", b, sizeof(b), &n));
    TEST_ASSERT_FALSE(ap_send_hex("8 0", b, sizeof(b), &n));
    TEST_ASSERT_FALSE(ap_send_hex("8G", b, sizeof(b), &n));
    TEST_ASSERT_FALSE(ap_send_hex("010203040506070809", b, sizeof(b), &n));
}

void test_send_parse_one_frame(void)
{
    /* the request of the issue: a 29-bit frame on single-wire CAN */
    TEST_ASSERT_TRUE_MESSAGE(
        parse("{\"bus\":\"sw\",\"id\":\"1024E097\",\"data\":\"80 01 FF\"}"),
        s_err);
    TEST_ASSERT_EQUAL(AP_SEND_SW, s_req.where);
    TEST_ASSERT_EQUAL(1, s_req.n);
    TEST_ASSERT_EQUAL_HEX32(0x1024E097, s_req.f[0].id);
    TEST_ASSERT_TRUE(s_req.f[0].ext);   /* above 7FF: 29 bits by itself */
    TEST_ASSERT_FALSE(s_req.f[0].wake);
    TEST_ASSERT_EQUAL(3, s_req.f[0].len);
    TEST_ASSERT_EQUAL_HEX8(0x80, s_req.f[0].data[0]);
    TEST_ASSERT_EQUAL(0, s_req.rate);

    /* the wake frame: 11 bits, the transceiver's high-voltage mode */
    TEST_ASSERT_TRUE_MESSAGE(
        parse("{\"bus\":\"sw\",\"id\":\"100\",\"data\":\"00\",\"wake\":true}"),
        s_err);
    TEST_ASSERT_EQUAL_HEX32(0x100, s_req.f[0].id);
    TEST_ASSERT_FALSE(s_req.f[0].ext);
    TEST_ASSERT_TRUE(s_req.f[0].wake);

    /* the id as a number, with the prefix, with ext said out loud */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"id\":2015,\"data\":\"0100\"}"));
    TEST_ASSERT_EQUAL_HEX32(0x7DF, s_req.f[0].id);
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ch\",\"id\":\"0x7df\",\"data\":\"01\"}"));
    TEST_ASSERT_EQUAL_HEX32(0x7DF, s_req.f[0].id);
    TEST_ASSERT_EQUAL(AP_SEND_CH, s_req.where);
    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"123\",\"ext\":true,"
                           "\"data\":\"01\"}"));
    TEST_ASSERT_TRUE(s_req.f[0].ext);   /* a 29-bit id that is a small one */
    TEST_ASSERT_EQUAL(AP_SEND_OBD, s_req.where);

    /* WiCAN's own controller sends a frame without data; the chip cannot */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"can\",\"id\":\"321\"}"));
    TEST_ASSERT_EQUAL(AP_SEND_CAN, s_req.where);
    TEST_ASSERT_EQUAL(0, s_req.f[0].len);
    TEST_ASSERT_TRUE(parse("{\"bus\":\"can\",\"id\":\"321\",\"data\":\"\"}"));

    /* a bus at another speed, as a row names it; 0 = its own */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"bus_kbps\":500,\"id\":\"1\","
                           "\"data\":\"01\"}"));
    TEST_ASSERT_EQUAL(1, s_req.rate);
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"bus_kbps\":125,\"id\":\"1\","
                           "\"data\":\"01\"}"));
    TEST_ASSERT_EQUAL(0, s_req.rate);
    TEST_ASSERT_TRUE(parse("{\"bus\":\"sw\",\"bus_kbps\":0,\"id\":\"1\","
                           "\"data\":\"01\"}"));
    TEST_ASSERT_EQUAL(0, s_req.rate);
}

void test_send_parse_refusals(void)
{
    /* no bus, a bus this firmware does not know */
    TEST_ASSERT_FALSE(parse("{\"id\":\"7DF\",\"data\":\"01\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "bus is one of"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"hs\",\"id\":\"7DF\",\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"SW\",\"id\":\"7DF\",\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("[1,2]"));

    /* the id */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"data\":\"01\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "frame 1: id"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DG\",\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"\",\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"20000000\","
                            "\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"123456789\","
                            "\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":-1,\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":1.5,\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":true,\"data\":\"01\"}"));

    /* an id over 11 bits that is said to be an 11-bit one */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"800\",\"ext\":false,"
                            "\"data\":\"01\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "does not fit 11 bits"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"ext\":1,"
                            "\"data\":\"01\"}"));

    /* the data */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"0\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\","
                            "\"data\":\"010203040506070809\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":7}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "without data"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"obd\",\"id\":\"7DF\",\"data\":\"\"}"));

    /* the data's sentence says which rule it broke (2026-11-21: one
       sentence said all three) */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"0\"}"));
    TEST_ASSERT_EQUAL_STRING("frame 1: data has half a byte: two hex digits "
                             "a byte (\"80 01 FF\")", s_err);
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"8 0\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "half a byte"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"8G\"}"));
    TEST_ASSERT_EQUAL_STRING("frame 1: data is hex digits (\"80 01 FF\") and "
                             "has another character", s_err);
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\","
                            "\"data\":\"010203040506070809\"}"));
    TEST_ASSERT_EQUAL_STRING("frame 1: data is 8 bytes at most", s_err);

    /* the wake mode is the single-wire transceiver's, nobody else's */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"100\",\"data\":\"00\","
                            "\"wake\":true}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "bus sw alone"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"can\",\"id\":\"100\",\"data\":\"00\","
                            "\"wake\":true}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"id\":\"100\",\"data\":\"00\","
                            "\"wake\":\"yes\"}"));
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"id\":\"100\",\"data\":\"00\","
                           "\"wake\":false}"));

    /* a speed: for the three buses that have one to choose */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"obd\",\"bus_kbps\":250,\"id\":\"1\","
                            "\"data\":\"01\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "bus_kbps"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"can\",\"bus_kbps\":250,\"id\":\"1\","
                            "\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"bus_kbps\":333,\"id\":\"1\","
                            "\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"bus_kbps\":\"125\",\"id\":\"1\","
                            "\"data\":\"01\"}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"bus_kbps\":-5,\"id\":\"1\","
                            "\"data\":\"01\"}"));

    /* a key the request does not know (2026-11-21): refused, with the keys
       it does. It was ignored and the request answered ok: a gap_ms beside
       frames, an at_ms or a repeat in a frame, each sent as if not there */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"01\","
                            "\"repeat\":3}"));
    TEST_ASSERT_EQUAL_STRING("unknown key \"repeat\" (bus, bus_kbps, id, ext, "
                             "data, wake, gap_ms)", s_err);
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"ID\":\"7DF\",\"data\":\"01\"}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "unknown key \"ID\""));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"gap_ms\":300,"
                            "\"frames\":[{\"id\":\"100\",\"data\":\"00\"}]}"));
    TEST_ASSERT_EQUAL_STRING("gap_ms is a key of each frame in frames", s_err);
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"note\":\"x\","
                            "\"frames\":[{\"id\":\"100\",\"data\":\"00\"}]}"));
    TEST_ASSERT_EQUAL_STRING("unknown key \"note\" (bus, bus_kbps, frames)",
                             s_err);
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"frames\":["
                            "{\"id\":\"100\",\"data\":\"00\"},"
                            "{\"id\":\"621\",\"data\":\"00\",\"at_ms\":300}]}"));
    TEST_ASSERT_EQUAL_STRING("frame 2: unknown key \"at_ms\" (id, ext, data, "
                             "wake, gap_ms)", s_err);
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"frames\":["
                            "{\"bus\":\"sw\",\"id\":\"100\",\"data\":\"00\"}]}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "frame 1: unknown key \"bus\""));

    /* every key it knows, in both forms */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"sw\",\"bus_kbps\":0,\"id\":\"100\","
                           "\"ext\":false,\"data\":\"00\",\"wake\":true,"
                           "\"gap_ms\":300}"));
    TEST_ASSERT_TRUE(parse("{\"bus\":\"sw\",\"bus_kbps\":0,\"frames\":["
                           "{\"id\":\"100\",\"ext\":false,\"data\":\"00\","
                           "\"wake\":true,\"gap_ms\":300}]}"));
}

void test_send_parse_sequence(void)
{
    /* the whole remote start of the issue in one request: the wake frame,
       the network frame, the command, and its second half 360 ms later */
    TEST_ASSERT_TRUE_MESSAGE(parse(
        "{\"bus\":\"sw\",\"frames\":["
        "{\"id\":\"100\",\"data\":\"00\",\"wake\":true},"
        "{\"id\":\"621\",\"data\":\"00 FF FF FF FF FF 00 00\"},"
        "{\"id\":\"1024E097\",\"data\":\"80 01 FF\",\"gap_ms\":360},"
        "{\"id\":\"1024E097\",\"data\":\"00 00 FF\"}]}"), s_err);
    TEST_ASSERT_EQUAL(4, s_req.n);
    TEST_ASSERT_TRUE(s_req.f[0].wake);
    TEST_ASSERT_FALSE(s_req.f[1].wake);
    TEST_ASSERT_EQUAL(8, s_req.f[1].len);
    TEST_ASSERT_TRUE(s_req.f[2].ext);
    TEST_ASSERT_EQUAL(360, s_req.f[2].gap_ms);
    TEST_ASSERT_EQUAL(0, s_req.f[3].gap_ms);
    TEST_ASSERT_EQUAL_HEX8(0x00, s_req.f[3].data[0]);

    /* one form or the other */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"id\":\"100\",\"data\":\"00\","
                            "\"frames\":[{\"id\":\"100\",\"data\":\"00\"}]}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "not both"));

    /* the list: 1 to 8 objects */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"frames\":[]}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"frames\":{}}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"sw\",\"frames\":[7]}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "frame 1"));
    TEST_ASSERT_FALSE(parse(
        "{\"bus\":\"ms\",\"frames\":["
        "{\"id\":\"1\",\"data\":\"01\"},{\"id\":\"1\",\"data\":\"01\"},"
        "{\"id\":\"1\",\"data\":\"01\"},{\"id\":\"1\",\"data\":\"01\"},"
        "{\"id\":\"1\",\"data\":\"01\"},{\"id\":\"1\",\"data\":\"01\"},"
        "{\"id\":\"1\",\"data\":\"01\"},{\"id\":\"1\",\"data\":\"01\"},"
        "{\"id\":\"1\",\"data\":\"01\"}]}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "1 to 8"));

    /* the sentence names the frame that is wrong */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"frames\":["
                            "{\"id\":\"1\",\"data\":\"01\"},"
                            "{\"id\":\"1\",\"data\":\"0\"}]}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "frame 2"));

    /* a gap: 2 s at most, 5 s for all of them */
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"frames\":["
                            "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":2001},"
                            "{\"id\":\"1\",\"data\":\"01\"}]}"));
    TEST_ASSERT_FALSE(parse("{\"bus\":\"ms\",\"frames\":["
                            "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":-1}]}"));
    TEST_ASSERT_FALSE(parse(
        "{\"bus\":\"ms\",\"frames\":["
        "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":2000},"
        "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":2000},"
        "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":1001},"
        "{\"id\":\"1\",\"data\":\"01\"}]}"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "add up"));
    TEST_ASSERT_TRUE(parse(
        "{\"bus\":\"ms\",\"frames\":["
        "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":2000},"
        "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":2000},"
        "{\"id\":\"1\",\"data\":\"01\",\"gap_ms\":1000},"
        "{\"id\":\"1\",\"data\":\"01\"}]}"));
}

void test_send_the_car_decides_obd(void)
{
    /* no car yet and the protocol on Automatic: nowhere to send */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"7DF\",\"data\":\"0100\"}"));
    TEST_ASSERT_FALSE(ap_send_check_car(&s_req, '0', s_err, sizeof(s_err)));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "not known yet"));
    TEST_ASSERT_FALSE(ap_send_check_car(&s_req, '\0', s_err, sizeof(s_err)));

    /* a CAN car: 11 and 29 bits both, on any of its protocols */
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, '6', s_err, sizeof(s_err)));
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, '9', s_err, sizeof(s_err)));
    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"18DB33F1\","
                           "\"data\":\"0100\"}"));
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, '6', s_err, sizeof(s_err)));
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, 'A', s_err, sizeof(s_err)));

    /* the chip's user protocols are no car's OBD protocol */
    TEST_ASSERT_FALSE(ap_send_check_car(&s_req, 'B', s_err, sizeof(s_err)));

    /* a car before CAN: three header bytes for the id, 1 to 7 data bytes */
    TEST_ASSERT_FALSE(ap_send_check_car(&s_req, '3', s_err, sizeof(s_err)));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "three header bytes"));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "ISO 9141-2"));
    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"686AF1\","
                           "\"data\":\"0100\"}"));

    for (const char *p = "12345"; *p != '\0'; p++)
    {
        TEST_ASSERT_TRUE_MESSAGE(
            ap_send_check_car(&s_req, *p, s_err, sizeof(s_err)), s_err);
        TEST_ASSERT_EQUAL(0, ap_send_key(&s_req, 0, *p)); /* at home */
    }

    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"686AF1\","
                           "\"data\":\"0102030405060708\"}"));
    TEST_ASSERT_FALSE(ap_send_check_car(&s_req, '2', s_err, sizeof(s_err)));
    TEST_ASSERT_NOT_NULL(strstr(s_err, "1 to 7 data bytes"));
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, '6', s_err, sizeof(s_err)));

    /* the other buses do not ask the car */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"0100\"}"));
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, '0', s_err, sizeof(s_err)));
    TEST_ASSERT_TRUE(parse("{\"bus\":\"can\",\"id\":\"7DF\",\"data\":\"0100\"}"));
    TEST_ASSERT_TRUE(ap_send_check_car(&s_req, '0', s_err, sizeof(s_err)));
}

void test_send_where_the_chip_goes(void)
{
    char word[40];
    char chain[AP_INIT_LEN];

    /* the buses of a row, with the id length of the frame */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"id\":\"7DF\",\"data\":\"01\"}"));
    TEST_ASSERT_EQUAL(AP_BUS_KEY(AP_BUS_MS, 0), ap_send_key(&s_req, 0, '6'));
    TEST_ASSERT_TRUE(parse("{\"bus\":\"ms\",\"id\":\"18DB33F1\","
                           "\"data\":\"01\"}"));

    uint16_t key = ap_send_key(&s_req, 0, '6');

    TEST_ASSERT_EQUAL(AP_BUS_KEY(AP_BUS_MS, 0) | AP_BUS_KEY_EXT, key);
    TEST_ASSERT_EQUAL(AP_BUS_MS, AP_BUS_OF(key)); /* the bits are no bus */
    TEST_ASSERT_TRUE(ap_bus_define(key, word, sizeof(word)) > 0);
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN B,01,04,MS_CAN", word);

    TEST_ASSERT_TRUE(parse("{\"bus\":\"ch\",\"bus_kbps\":250,\"id\":\"7DF\","
                           "\"data\":\"01\"}"));
    key = ap_send_key(&s_req, 0, '6');
    TEST_ASSERT_EQUAL(AP_BUS_KEY(AP_BUS_CH, 2), key);
    (void)ap_bus_define(key, word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN C,81,02,CH_CAN", word);

    /* single-wire CAN: normal, and the wake mode for the frame that asks */
    TEST_ASSERT_TRUE(parse(
        "{\"bus\":\"sw\",\"frames\":["
        "{\"id\":\"100\",\"data\":\"00\",\"wake\":true},"
        "{\"id\":\"621\",\"data\":\"00\"},"
        "{\"id\":\"1024E097\",\"data\":\"80 01 FF\"},"
        "{\"id\":\"1024E097\",\"data\":\"80\",\"wake\":true}]}"));
    key = ap_send_key(&s_req, 0, '6');
    TEST_ASSERT_EQUAL(AP_BUS_KEY(AP_BUS_SW, 0) | AP_BUS_KEY_WAKE, key);
    TEST_ASSERT_EQUAL(AP_BUS_SW, AP_BUS_OF(key));
    (void)ap_bus_define(key, word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN F,81,0F,SW_CAN,2", word);
    (void)ap_bus_chain(key, chain, sizeof(chain));
    TEST_ASSERT_EQUAL_STRING(
        "ATS1;ATH1;ATST96;VTSET_CAN F,81,0F,SW_CAN,2;ATTPF;ATSH7DF;ATCRA",
        chain);

    key = ap_send_key(&s_req, 1, '6');
    TEST_ASSERT_EQUAL(AP_BUS_KEY(AP_BUS_SW, 0), key); /* a row's own place */
    (void)ap_bus_define(key, word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN F,81,0F,SW_CAN,3", word);

    (void)ap_bus_define(ap_send_key(&s_req, 2, '6'), word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN F,01,0F,SW_CAN,3", word);
    (void)ap_bus_define(ap_send_key(&s_req, 3, '6'), word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN F,01,0F,SW_CAN,2", word);

    /* the car's OBD protocol on a CAN car: a slot on the HS pins at that
       protocol's bit rate, 11 or 29 bits by the frame, not by the protocol */
    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"7DF\",\"data\":\"01\"}"));
    key = ap_send_key(&s_req, 0, '6');
    TEST_ASSERT_EQUAL(AP_BUS_KEY(AP_BUS_HS, 0), key);
    (void)ap_bus_define(key, word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN D,81,01,HS_CAN", word);
    TEST_ASSERT_EQUAL(key, ap_send_key(&s_req, 0, '7'));
    (void)ap_bus_define(ap_send_key(&s_req, 0, '8'), word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN D,81,02,HS_CAN", word);
    TEST_ASSERT_EQUAL(500000, ap_bus_rate_bps(AP_BUS_HS, 0));
    TEST_ASSERT_EQUAL(250000, ap_bus_rate_bps(AP_BUS_HS, 2));
    TEST_ASSERT_EQUAL_CHAR('D', ap_bus_slot(AP_BUS_HS));

    TEST_ASSERT_TRUE(parse("{\"bus\":\"obd\",\"id\":\"18DB33F1\","
                           "\"data\":\"01\"}"));
    (void)ap_bus_define(ap_send_key(&s_req, 0, '6'), word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN D,01,01,HS_CAN", word);
    (void)ap_bus_define(ap_send_key(&s_req, 0, '9'), word, sizeof(word));
    TEST_ASSERT_EQUAL_STRING("VTSET_CAN D,01,02,HS_CAN", word);
    (void)ap_bus_chain(ap_send_key(&s_req, 0, '9'), chain, sizeof(chain));
    TEST_ASSERT_EQUAL_STRING(
        "ATS1;ATH1;ATST96;VTSET_CAN D,01,02,HS_CAN;ATTPD;ATSH7DF;ATCRA",
        chain);

    /* no ROW may name that slot, and a row's wake mode does not exist */
    ap_row_bus_t b = AP_BUS_OBD;

    TEST_ASSERT_FALSE(ap_bus_parse("hs", &b));
    TEST_ASSERT_EQUAL(0, ap_bus_define(AP_BUS_KEY(AP_BUS_OBD, 0), word,
                                       sizeof(word)));
    TEST_ASSERT_EQUAL(0, ap_bus_define(AP_BUS_KEY(AP_BUS_SLOTS, 0), word,
                                       sizeof(word)));
    TEST_ASSERT_EQUAL(0, ap_bus_define(AP_BUS_KEY(AP_BUS_MS, 0), word, 10));
}

void test_send_words_are_ram_and_undone(void)
{
    ap_send_frame_t f = { .id = 0x7DF, .len = 3, .data = { 0x80, 0x01, 0xFF } };
    char words[40];
    char undo[AP_RESTORE_LEN];

    /* the setup: every word of it is one the restore knows how to undo */
    TEST_ASSERT_EQUAL_STRING("ATCAF0;ATV1;ATAT0;ATST19", ap_send_setup(false));
    TEST_ASSERT_EQUAL_STRING("ATAT0;ATST19", ap_send_setup(true));
    TEST_ASSERT_EQUAL_HEX(AP_SETS_RAW | AP_SETS_DLC | AP_SETS_TIMING |
                          AP_SETS_PRINT, ap_chain_sets(ap_send_setup(false)));
    TEST_ASSERT_EQUAL_HEX(AP_SETS_TIMING | AP_SETS_PRINT,
                          ap_chain_sets(ap_send_setup(true)));
    (void)ap_chain_restore(ap_chain_sets(ap_send_setup(false)), "", undo,
                           sizeof(undo));
    TEST_ASSERT_NOT_NULL(strstr(undo, "ATCAF1"));
    TEST_ASSERT_NOT_NULL(strstr(undo, "ATV0"));
    TEST_ASSERT_NOT_NULL(strstr(undo, "ATAT1"));

    /* the id: 11 bits, 29 bits as the classic pair, three bytes before CAN */
    TEST_ASSERT_TRUE(ap_send_header(&f, false, words, sizeof(words)) > 0);
    TEST_ASSERT_EQUAL_STRING("ATSH7DF", words);
    f.id = 0x1024E097;
    f.ext = true;
    (void)ap_send_header(&f, false, words, sizeof(words));
    TEST_ASSERT_EQUAL_STRING("ATCP10;ATSH24E097", words);
    TEST_ASSERT_EQUAL_HEX(AP_SETS_PRIORITY | AP_SETS_HEADER,
                          ap_chain_sets(words));
    (void)ap_chain_restore(ap_chain_sets(words), "", undo, sizeof(undo));
    TEST_ASSERT_NOT_NULL(strstr(undo, "ATCP18"));
    f.id = 0x123;
    (void)ap_send_header(&f, false, words, sizeof(words));
    TEST_ASSERT_EQUAL_STRING("ATCP00;ATSH000123", words);
    f.id = 0x686AF1;
    (void)ap_send_header(&f, true, words, sizeof(words));
    TEST_ASSERT_EQUAL_STRING("ATSH686AF1", words);
    TEST_ASSERT_EQUAL(0, ap_send_header(&f, true, words, 6));

    /* the data, as the chip takes it */
    TEST_ASSERT_EQUAL(6, ap_send_data(&f, words, sizeof(words)));
    TEST_ASSERT_EQUAL_STRING("8001FF", words);
    f.len = 8;
    memset(f.data, 0xAB, sizeof(f.data));
    TEST_ASSERT_EQUAL(16, ap_send_data(&f, words, sizeof(words)));
    TEST_ASSERT_EQUAL_STRING("ABABABABABABABAB", words);

    /* nothing here is a word the chip stores, and nothing is refused on a
       row of another bus (no protocol word, no reset) */
    const char *all[] = { ap_send_setup(false), ap_send_setup(true),
                          "ATCP10;ATSH24E097", "ATSH7DF", "ATSH686AF1" };

    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++)
    {
        TEST_ASSERT_FALSE_MESSAGE(ap_chain_store_word(all[i], NULL, 0), all[i]);
        TEST_ASSERT_TRUE_MESSAGE(ap_bus_chain_allowed(all[i]), all[i]);
    }
}

void test_send_what_the_chip_said(void)
{
    /* on the bus: nobody answered (what a send expects), or somebody did */
    TEST_ASSERT_EQUAL(AP_SEND_SENT, ap_send_classify("NO DATA\r\r>"));
    TEST_ASSERT_EQUAL(AP_SEND_SENT, ap_send_classify("7E8 06 41 00 18 3F\r"));
    TEST_ASSERT_EQUAL(AP_SEND_SENT,
                      ap_send_classify("BUS INIT: OK\r48 6B 10 41 00 BE\r"));
    TEST_ASSERT_EQUAL(AP_SEND_SENT, ap_send_classify("BUS INIT: OK\rNO DATA"));

    /* CAN: it went out and nobody acknowledged it */
    TEST_ASSERT_EQUAL(AP_SEND_NO_ACK, ap_send_classify("CAN ERROR\r\r>"));

    /* refused */
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("?\r\r>"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("BUS INIT: ...ERROR"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("UNABLE TO CONNECT"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("BUS BUSY"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("BUS ERROR"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("FB ERROR"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("BUFFER FULL"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("STOPPED"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify(""));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify("\r\r>"));
    TEST_ASSERT_EQUAL(AP_SEND_ERROR, ap_send_classify(NULL));

    TEST_ASSERT_EQUAL_STRING("sent", ap_send_result_word(AP_SEND_SENT));
    TEST_ASSERT_EQUAL_STRING("no_ack", ap_send_result_word(AP_SEND_NO_ACK));
    TEST_ASSERT_EQUAL_STRING("error", ap_send_result_word(AP_SEND_ERROR));
    TEST_ASSERT_EQUAL_STRING("not_sent",
                             ap_send_result_word(AP_SEND_NOT_SENT));
    TEST_ASSERT_EQUAL_STRING("sw", ap_send_where_word(AP_SEND_SW));
    TEST_ASSERT_EQUAL_STRING("can", ap_send_where_word(AP_SEND_CAN));
}

/* 7DF 02 01 00 00 00 00 00 on a 2012 Chevrolet Volt (2026-11-21): its
   seven ECUs' answers as the chip printed them, the order of that request
   (it varies from one to the next). `reply` was cut at 46 characters, in
   the middle of the second answer. The reply is whole now, and `replies`
   is one answer a line. */
void test_send_reply_lines(void)
{
    static const char *const ECU[] =
    {
        "7E8 06 41 00 BE 7F B8 13 AA", "7EB 06 41 00 80 40 00 01 AA",
        "7EF 06 41 00 00 00 00 01 AA", "7EC 06 41 00 80 00 00 01 AA",
        "7E9 06 41 00 80 00 00 01 AA", "7EA 06 41 00 80 00 00 01 AA",
        "7ED 06 41 00 00 00 00 01 AA",
    };
    const size_t n_ecu = sizeof(ECU) / sizeof(ECU[0]);
    char resp[256] = "";
    char joined[256] = "";
    char text[256];
    char line[64];

    /* as the chip prints it: a space before each line end, the prompt */
    for (size_t i = 0; i < n_ecu; i++)
    {
        strcat(resp, ECU[i]);
        strcat(resp, " \r");
        strcat(joined, (i > 0) ? " " : "");
        strcat(joined, ECU[i]);
    }

    strcat(resp, "\r>");

    /* `reply`: whole, one line */
    TEST_ASSERT_EQUAL_UINT(strlen(joined),
                           ap_send_reply_join(resp, text, sizeof(text)));
    TEST_ASSERT_EQUAL_STRING(joined, text);

    /* what the 48-byte field kept: the second answer cut off */
    TEST_ASSERT_EQUAL_UINT(46, ap_send_reply_join(resp, text, 48));
    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 7F B8 13 AA 7EB 06 41 00 80 40",
                             text);
    TEST_ASSERT_EQUAL_UINT(195, strlen(joined)); /* the whole reply */

    /* `replies`: one answer a line, every one of them, in order */
    const char *p = resp;
    size_t got = 0;

    while ((p = ap_send_reply_next(p, line, sizeof(line))) != NULL)
    {
        TEST_ASSERT_TRUE(got < n_ecu);
        TEST_ASSERT_EQUAL_STRING(ECU[got], line);
        got++;
    }

    TEST_ASSERT_EQUAL_UINT(n_ecu, got);
    TEST_ASSERT_EQUAL_STRING("", line);

    /* nobody answered: one line; the prompt and blank lines are none */
    p = ap_send_reply_next("NO DATA\r\r>", line, sizeof(line));
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_STRING("NO DATA", line);
    TEST_ASSERT_NULL(ap_send_reply_next(p, line, sizeof(line)));
    TEST_ASSERT_NULL(ap_send_reply_next("\r\r>", line, sizeof(line)));
    TEST_ASSERT_EQUAL_STRING("", line);
    TEST_ASSERT_NULL(ap_send_reply_next("", line, sizeof(line)));
    TEST_ASSERT_NULL(ap_send_reply_next(NULL, line, sizeof(line)));
    TEST_ASSERT_EQUAL_UINT(0, ap_send_reply_join(NULL, text, sizeof(text)));
    TEST_ASSERT_EQUAL_STRING("", text);

    /* before CAN: the init line and the answer are two lines */
    p = ap_send_reply_next("BUS INIT: OK\r48 6B 10 41 00 BE\r", line,
                           sizeof(line));
    TEST_ASSERT_EQUAL_STRING("BUS INIT: OK", line);
    p = ap_send_reply_next(p, line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING("48 6B 10 41 00 BE", line);
    TEST_ASSERT_NULL(ap_send_reply_next(p, line, sizeof(line)));

    /* a line longer than the buffer is cut there, and the walk goes on
       with the next line */
    char small[8];

    p = ap_send_reply_next("7E8 06 41 00\r7E9\r", small, sizeof(small));
    TEST_ASSERT_EQUAL_STRING("7E8 06", small);
    p = ap_send_reply_next(p, small, sizeof(small));
    TEST_ASSERT_EQUAL_STRING("7E9", small);
    TEST_ASSERT_NULL(ap_send_reply_next(p, small, sizeof(small)));
}

void run_send_tests(void)
{
    RUN_TEST(test_send_hex);
    RUN_TEST(test_send_parse_one_frame);
    RUN_TEST(test_send_parse_refusals);
    RUN_TEST(test_send_parse_sequence);
    RUN_TEST(test_send_the_car_decides_obd);
    RUN_TEST(test_send_where_the_chip_goes);
    RUN_TEST(test_send_words_are_ram_and_undone);
    RUN_TEST(test_send_what_the_chip_said);
    RUN_TEST(test_send_reply_lines);
}
