/**
 * @file test_iso_ae.c
 * @brief ISO15765 extended addressing on a J2534 channel, the pure part
 *        (j2534_iso_ae.c): what a FLOW_CONTROL filter asks for, the message
 *        layout [id][address extension][payload] both ways, the single
 *        frame behind an address extension, the first frame questions of
 *        the receive path. Six cases, run from test_main.c.
 *
 * The ids and address extensions are the ones of the pull request that
 * brought the feature (Toyota's gateway: requests on 0x750, answers on
 * 0x758, one address extension per ECU behind it).
 */
#include <string.h>

#include "unity.h"

#include "j2534_iso_ae.h"
#include "j2534_proto.h"
#include "test_suites.h"

/* ~4 KB each: never on a stack */
static j2534_msg_t s_pattern, s_flow, s_msg;
static uint8_t s_big[J2534_MAX_DATA];

static void msg(j2534_msg_t *m, uint32_t tx_flags, const uint8_t *data,
                uint32_t n)
{
    memset(m, 0, sizeof(*m));
    m->protocol_id = J2534_PROT_ISO15765;
    m->tx_flags = tx_flags;
    m->data_size = n;
    memcpy(m->data, data, n);
}

static void test_ae_filter(void)
{
    static const uint8_t RX4[] = { 0x00, 0x00, 0x07, 0xE8 };
    static const uint8_t TX4[] = { 0x00, 0x00, 0x07, 0xE0 };
    static const uint8_t RX5[] = { 0x00, 0x00, 0x07, 0x58, 0x5F };
    static const uint8_t TX5[] = { 0x00, 0x00, 0x07, 0x50, 0xE9 };
    const uint32_t AE = J2534_TX_ISO15765_ADDR_TYPE;
    j2534_iso_ae_t ae = { .on = true, .tx = 0x11, .rx = 0x22 };

    /* normal addressing: the two ids, no flag */
    msg(&s_pattern, 0, RX4, 4);
    msg(&s_flow, J2534_TX_ISO15765_FRAME_PAD, TX4, 4);
    TEST_ASSERT_TRUE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    TEST_ASSERT_FALSE(ae.on);

    /* extended: the fifth byte of the pattern is what the ECU's frames
       start with, the fifth byte of the flow-control message what ours do */
    msg(&s_pattern, AE, RX5, 5);
    msg(&s_flow, AE | J2534_TX_ISO15765_FRAME_PAD, TX5, 5);
    TEST_ASSERT_TRUE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    TEST_ASSERT_TRUE(ae.on);
    TEST_ASSERT_EQUAL_HEX8(0x5F, ae.rx);
    TEST_ASSERT_EQUAL_HEX8(0xE9, ae.tx);

    /* the flag on one of the two is enough */
    msg(&s_pattern, 0, RX5, 5);
    ae.on = false;
    TEST_ASSERT_TRUE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    TEST_ASSERT_TRUE(ae.on);
    msg(&s_pattern, AE, RX5, 5);
    msg(&s_flow, 0, TX5, 5);
    ae.on = false;
    TEST_ASSERT_TRUE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    TEST_ASSERT_TRUE(ae.on);

    /* five bytes without the flag are a normal filter (the fifth unused) */
    msg(&s_pattern, 0, RX5, 5);
    TEST_ASSERT_TRUE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    TEST_ASSERT_FALSE(ae.on);

    /* refused, and the caller's record untouched: the flag with a message
       of four bytes, an id cut short, a message missing */
    ae = (j2534_iso_ae_t){ .on = true, .tx = 0x11, .rx = 0x22 };
    msg(&s_pattern, AE, RX4, 4);
    msg(&s_flow, AE, TX5, 5);
    TEST_ASSERT_FALSE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    msg(&s_pattern, AE, RX5, 5);
    msg(&s_flow, AE, TX4, 4);
    TEST_ASSERT_FALSE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    msg(&s_pattern, 0, RX4, 3);
    msg(&s_flow, 0, TX4, 4);
    TEST_ASSERT_FALSE(j2534_iso_filter_ae(&s_pattern, &s_flow, &ae));
    TEST_ASSERT_FALSE(j2534_iso_filter_ae(NULL, &s_flow, &ae));
    TEST_ASSERT_FALSE(j2534_iso_filter_ae(&s_pattern, NULL, &ae));
    TEST_ASSERT_FALSE(j2534_iso_filter_ae(&s_pattern, &s_flow, NULL));
    TEST_ASSERT_TRUE(ae.on);
    TEST_ASSERT_EQUAL_HEX8(0x11, ae.tx);
    TEST_ASSERT_EQUAL_HEX8(0x22, ae.rx);
}

static void test_ae_split(void)
{
    static const uint8_t REQ[] = { 0x00, 0x00, 0x07, 0x50, 0xE9, 0x10, 0x03 };
    static const uint8_t REQ29[] = { 0x18, 0xDA, 0x10, 0xF1, 0x40, 0x3E };
    const uint32_t AE = J2534_TX_ISO15765_ADDR_TYPE;
    uint32_t id = 0;
    uint8_t ae = 0;
    const uint8_t *pl = NULL;
    size_t n = 99;

    msg(&s_msg, AE, REQ, sizeof(REQ));
    TEST_ASSERT_TRUE(j2534_iso_ae_split(&s_msg, &id, &ae, &pl, &n));
    TEST_ASSERT_EQUAL_HEX32(0x750, id);
    TEST_ASSERT_EQUAL_HEX8(0xE9, ae);
    TEST_ASSERT_EQUAL(2, (int)n);
    TEST_ASSERT_EQUAL_PTR(s_msg.data + 5, pl);

    /* the id as written: the channel masks it to 11 or 29 bits */
    msg(&s_msg, AE | J2534_TX_CAN_29BIT_ID, REQ29, sizeof(REQ29));
    TEST_ASSERT_TRUE(j2534_iso_ae_split(&s_msg, &id, &ae, &pl, &n));
    TEST_ASSERT_EQUAL_HEX32(0x18DA10F1, id);
    TEST_ASSERT_EQUAL_HEX8(0x40, ae);
    TEST_ASSERT_EQUAL(1, (int)n);

    /* the id and the address extension alone: an empty payload, which the
       channel refuses */
    msg(&s_msg, AE, REQ, 5);
    TEST_ASSERT_TRUE(j2534_iso_ae_split(&s_msg, &id, &ae, &pl, &n));
    TEST_ASSERT_EQUAL(0, (int)n);

    /* not this layout: no flag, or no room for the address extension */
    msg(&s_msg, 0, REQ, sizeof(REQ));
    TEST_ASSERT_FALSE(j2534_iso_ae_split(&s_msg, &id, &ae, &pl, &n));
    msg(&s_msg, AE, REQ, 4);
    TEST_ASSERT_FALSE(j2534_iso_ae_split(&s_msg, &id, &ae, &pl, &n));
    msg(&s_msg, AE, REQ, sizeof(REQ));
    s_msg.data_size = J2534_MAX_DATA + 1;
    TEST_ASSERT_FALSE(j2534_iso_ae_split(&s_msg, &id, &ae, &pl, &n));
    TEST_ASSERT_FALSE(j2534_iso_ae_split(NULL, &id, &ae, &pl, &n));

    /* and the normal layout's split still refuses a flagged message */
    msg(&s_msg, AE, REQ, sizeof(REQ));
    TEST_ASSERT_FALSE(j2534_iso_split(&s_msg, &id, &pl, &n));
}

static void test_ae_single_frame(void)
{
    static const uint8_t REQ[] = { 0x10, 0x03 };
    static const uint8_t SIX[] = { 0x27, 0x01, 0xA1, 0xA2, 0xA3, 0xA4 };
    static const uint8_t SEVEN[] = { 1, 2, 3, 4, 5, 6, 7 };
    static const uint8_t BARE[] = { 0xE9, 0x02, 0x10, 0x03 };
    static const uint8_t PADDED[] =
    {
        0xE9, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA,
    };
    static const uint8_t FULL[] =
    {
        0xE9, 0x06, 0x27, 0x01, 0xA1, 0xA2, 0xA3, 0xA4,
    };
    uint8_t fr[8];

    memset(fr, 0x55, sizeof(fr));
    TEST_ASSERT_EQUAL_UINT8(4, j2534_iso_ae_single_frame(0xE9, REQ, 2, false, fr));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(BARE, fr, 4);
    TEST_ASSERT_EQUAL_HEX8(0x55, fr[4]);    /* nothing written behind it */

    TEST_ASSERT_EQUAL_UINT8(8, j2534_iso_ae_single_frame(0xE9, REQ, 2, true, fr));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(PADDED, fr, 8);

    /* six bytes fill the frame, padded or not */
    TEST_ASSERT_EQUAL_UINT8(8, j2534_iso_ae_single_frame(0xE9, SIX, 6, false, fr));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(FULL, fr, 8);
    TEST_ASSERT_EQUAL_UINT8(8, j2534_iso_ae_single_frame(0xE9, SIX, 6, true, fr));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(FULL, fr, 8);

    /* seven do not fit behind an address extension; nothing is nothing */
    TEST_ASSERT_EQUAL_UINT8(0, j2534_iso_ae_single_frame(0xE9, SEVEN, 7, true, fr));
    TEST_ASSERT_EQUAL_UINT8(0, j2534_iso_ae_single_frame(0xE9, REQ, 0, true, fr));
    TEST_ASSERT_EQUAL_UINT8(0, j2534_iso_ae_single_frame(0xE9, NULL, 2, true, fr));
    TEST_ASSERT_EQUAL_UINT8(0, j2534_iso_ae_single_frame(0xE9, REQ, 2, true, NULL));
}

static void test_ae_join_and_indication(void)
{
    static const uint8_t ANS[] = { 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4 };
    static const uint8_t WANT[] =
    {
        0x00, 0x00, 0x07, 0x58, 0x5F, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4,
    };
    static const uint8_t TXDONE[] = { 0x00, 0x00, 0x07, 0x50, 0xE9 };
    static const uint8_t SOM29[] = { 0x18, 0xDA, 0xF1, 0x10, 0x5F };

    /* a received message: [id][address extension][payload] */
    memset(&s_msg, 0xEE, sizeof(s_msg));
    TEST_ASSERT_TRUE(j2534_iso_ae_join(&s_msg, 0x758, false, 0x5F, ANS,
                                       sizeof(ANS), 4321));
    TEST_ASSERT_EQUAL_UINT32(J2534_PROT_ISO15765, s_msg.protocol_id);
    TEST_ASSERT_EQUAL_HEX32(J2534_RX_ISO15765_ADDR_TYPE, s_msg.rx_status);
    TEST_ASSERT_EQUAL_HEX32(0, s_msg.tx_flags);
    TEST_ASSERT_EQUAL_UINT32(4321, s_msg.timestamp);
    TEST_ASSERT_EQUAL_UINT32(sizeof(WANT), s_msg.data_size);
    TEST_ASSERT_EQUAL_UINT32(sizeof(WANT), s_msg.extra_data_index);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(WANT, s_msg.data, sizeof(WANT));

    /* 29-bit ids beside it: both bits */
    TEST_ASSERT_TRUE(j2534_iso_ae_join(&s_msg, 0x18DAF110, true, 0x5F, ANS, 2, 0));
    TEST_ASSERT_EQUAL_HEX32(J2534_RX_ISO15765_ADDR_TYPE | J2534_RX_CAN_29BIT_ID,
                            s_msg.rx_status);
    TEST_ASSERT_EQUAL_UINT32(7, s_msg.data_size);

    /* the largest payload fits behind the five bytes, one more does not
       and leaves the message as it was */
    TEST_ASSERT_TRUE(j2534_iso_ae_join(&s_msg, 0x758, false, 0x5F, s_big,
                                       J2534_MAX_DATA - 5, 0));
    TEST_ASSERT_EQUAL_UINT32(J2534_MAX_DATA, s_msg.data_size);
    s_msg.data_size = 77;
    TEST_ASSERT_FALSE(j2534_iso_ae_join(&s_msg, 0x758, false, 0x5F, s_big,
                                        J2534_MAX_DATA - 4, 0));
    TEST_ASSERT_FALSE(j2534_iso_ae_join(&s_msg, 0x758, false, 0x5F, NULL, 1, 0));
    TEST_ASSERT_FALSE(j2534_iso_ae_join(NULL, 0x758, false, 0x5F, ANS, 1, 0));
    TEST_ASSERT_EQUAL_UINT32(77, s_msg.data_size);

    /* TxDone of a write to 0x750 / E9: five bytes, the three bits */
    j2534_iso_indication(&s_msg, 0x750, false,
                         J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION, 99);
    j2534_iso_ae_indication(&s_msg, 0xE9);
    TEST_ASSERT_EQUAL_HEX32(0x89, s_msg.rx_status);
    TEST_ASSERT_EQUAL_UINT32(5, s_msg.data_size);
    TEST_ASSERT_EQUAL_UINT32(5, s_msg.extra_data_index);
    TEST_ASSERT_EQUAL_UINT32(99, s_msg.timestamp);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(TXDONE, s_msg.data, 5);

    /* START_OF_MESSAGE on a 29-bit id */
    j2534_iso_indication(&s_msg, 0x18DAF110, true, J2534_RX_START_OF_MESSAGE, 0);
    j2534_iso_ae_indication(&s_msg, 0x5F);
    TEST_ASSERT_EQUAL_HEX32(0x182, s_msg.rx_status);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(SOM29, s_msg.data, 5);
    j2534_iso_ae_indication(NULL, 0x5F);    /* nothing to do, no fault */
}

static void test_pdu_of_first_frame(void)
{
    /* 20 bytes: 62 F1 90 and a VIN's start */
    static const uint8_t FF[] = { 0x10, 0x14, 0x62, 0xF1, 0x90, 0x31, 0x32, 0x33 };
    /* the same message behind an address extension: one data byte less */
    static const uint8_t FF_AE[] = { 0x5F, 0x10, 0x14, 0x62, 0xF1, 0x90, 0x31, 0x32 };
    /* 0x1020 bytes: the 32-bit length, two data bytes left */
    static const uint8_t FF_LONG[] = { 0x10, 0x00, 0x00, 0x00, 0x10, 0x20, 0x62, 0xF1 };
    static const uint8_t SF[] = { 0x03, 0x62, 0xF1, 0x90 };
    uint8_t pdu[20] = { 0x62, 0xF1, 0x90, 0x31, 0x32, 0x33, 0x34 };

    TEST_ASSERT_TRUE(j2534_iso_pdu_of_first_frame(pdu, 20, FF, 8));
    TEST_ASSERT_TRUE(j2534_iso_pdu_of_first_frame(pdu, 20, FF_AE + 1, 7));
    /* another length, or other bytes: an earlier message of the session */
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 19, FF, 8));
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 6, FF, 8));
    pdu[5] = 0x39;
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 20, FF, 8));
    /* that byte is one the shorter frame does not carry */
    TEST_ASSERT_TRUE(j2534_iso_pdu_of_first_frame(pdu, 20, FF_AE + 1, 7));
    pdu[4] = 0x39;
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 20, FF_AE + 1, 7));

    /* the 32-bit length form */
    memset(s_big, 0, sizeof(s_big));
    s_big[0] = 0x62;
    s_big[1] = 0xF1;
    TEST_ASSERT_TRUE(j2534_iso_pdu_of_first_frame(s_big, 0x1020, FF_LONG, 8));
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(s_big, 0x1021, FF_LONG, 8));
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(s_big, 0x1020, FF_LONG, 5));

    /* not a first frame, or nothing to look at */
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 3, SF, 4));
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 20, FF, 1));
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(NULL, 20, FF, 8));
    TEST_ASSERT_FALSE(j2534_iso_pdu_of_first_frame(pdu, 20, NULL, 8));
}

static void test_frame_count(void)
{
    /* normal: 7 in a single frame, 6 in a first frame, 7 in each after */
    TEST_ASSERT_EQUAL_UINT32(1, j2534_iso_frame_count(1, false));
    TEST_ASSERT_EQUAL_UINT32(1, j2534_iso_frame_count(7, false));
    TEST_ASSERT_EQUAL_UINT32(2, j2534_iso_frame_count(8, false));
    TEST_ASSERT_EQUAL_UINT32(2, j2534_iso_frame_count(13, false));
    TEST_ASSERT_EQUAL_UINT32(3, j2534_iso_frame_count(14, false));
    TEST_ASSERT_EQUAL_UINT32(586, j2534_iso_frame_count(4095, false));
    /* behind an address extension: 6, 5 and 6 */
    TEST_ASSERT_EQUAL_UINT32(1, j2534_iso_frame_count(6, true));
    TEST_ASSERT_EQUAL_UINT32(2, j2534_iso_frame_count(7, true));
    TEST_ASSERT_EQUAL_UINT32(2, j2534_iso_frame_count(11, true));
    TEST_ASSERT_EQUAL_UINT32(3, j2534_iso_frame_count(12, true));
    TEST_ASSERT_EQUAL_UINT32(683, j2534_iso_frame_count(4095, true));
    TEST_ASSERT_EQUAL_UINT32(J2534_ISO_SF_MAX - 1, J2534_ISO_SF_MAX_AE);
}

void run_iso_ae_tests(void)
{
    RUN_TEST(test_ae_filter);
    RUN_TEST(test_ae_split);
    RUN_TEST(test_ae_single_frame);
    RUN_TEST(test_ae_join_and_indication);
    RUN_TEST(test_pdu_of_first_frame);
    RUN_TEST(test_frame_count);
}
