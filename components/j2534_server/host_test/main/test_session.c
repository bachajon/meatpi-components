/**
 * @file test_session.c
 * @brief What a session's wire version decides (j2534_proto_session.c): the
 *        return code numbers, HELLO's version, the EVENT frame, the loss
 *        count. Six cases, run from test_main.c.
 *
 * The first case is the defect of 2026-11-16 as a test: on the header of
 * the day before it fails at its first line (ERR_DEVICE_IN_USE was 0x1A).
 */
#include <string.h>

#include "unity.h"

#include "j2534_proto.h"
#include "test_suites.h"

/* SAE J2534-1's return values (table 87 of the JAN2022 revision; the same
   numbers in the 04.04 generation, read off a second maker's interface). */
static void test_status_numbers_are_sae(void)
{
    TEST_ASSERT_EQUAL_HEX32(0x0E, J2534_ERR_DEVICE_IN_USE);
    TEST_ASSERT_EQUAL_HEX32(0x15, J2534_ERR_MSG_PROTOCOL_ID);
    TEST_ASSERT_EQUAL_HEX32(0x16, J2534_ERR_INVALID_FILTER_ID);
    TEST_ASSERT_EQUAL_HEX32(0x17, J2534_ERR_NO_FLOW_CONTROL);
    TEST_ASSERT_EQUAL_HEX32(0x18, J2534_ERR_NOT_UNIQUE);
    TEST_ASSERT_EQUAL_HEX32(0x19, J2534_ERR_INVALID_BAUDRATE);
    TEST_ASSERT_EQUAL_HEX32(0x1A, J2534_ERR_INVALID_DEVICE_ID);
    /* the ones that were right all along */
    TEST_ASSERT_EQUAL_HEX32(0x07, J2534_ERR_FAILED);
    TEST_ASSERT_EQUAL_HEX32(0x09, J2534_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_HEX32(0x0C, J2534_ERR_EXCEEDED_LIMIT);
    TEST_ASSERT_EQUAL_HEX32(0x10, J2534_ERR_BUFFER_EMPTY);
    TEST_ASSERT_EQUAL_HEX32(0x12, J2534_ERR_BUFFER_OVERFLOW);
}

static void test_hello_version(void)
{
    static const uint8_t ONE[] = { 0x01, 0x00, 0x00, 0x00 };
    static const uint8_t TWO[] = { 0x02, 0x00, 0x00, 0x00 };
    static const uint8_t NINE[] = { 0x09, 0x00, 0x00, 0x00, 0xEE };
    static const uint8_t ZERO[] = { 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t BIG[] = { 0x00, 0x00, 0x00, 0x80 };

    /* the first clients send no payload; drivers up to 1.9 send 1 */
    TEST_ASSERT_EQUAL_UINT32(1, j2534_hello_version(NULL, 0));
    TEST_ASSERT_EQUAL_UINT32(1, j2534_hello_version(ONE, 0));
    TEST_ASSERT_EQUAL_UINT32(1, j2534_hello_version(ONE, sizeof(ONE)));
    TEST_ASSERT_EQUAL_UINT32(1, j2534_hello_version(ZERO, sizeof(ZERO)));
    /* a payload too short to be a version word says nothing */
    TEST_ASSERT_EQUAL_UINT32(1, j2534_hello_version(TWO, 3));
    TEST_ASSERT_EQUAL_UINT32(1, j2534_hello_version(NULL, 4));
    /* driver 1.10 asks for 2; a later client gets this device's highest */
    TEST_ASSERT_EQUAL_UINT32(2, j2534_hello_version(TWO, sizeof(TWO)));
    TEST_ASSERT_EQUAL_UINT32(J2534_SESSION_VERSION,
                             j2534_hello_version(NINE, sizeof(NINE)));
    TEST_ASSERT_EQUAL_UINT32(J2534_SESSION_VERSION,
                             j2534_hello_version(BIG, sizeof(BIG)));
    TEST_ASSERT_EQUAL_UINT32(2, J2534_SESSION_VERSION);
}

static void test_status_wire_by_version(void)
{
    static const struct
    {
        uint32_t status;
        uint32_t v1;
    } MOVED[] =
    {
        { J2534_ERR_MSG_PROTOCOL_ID,   0x14 },
        { J2534_ERR_INVALID_FILTER_ID, 0x15 },
        { J2534_ERR_NO_FLOW_CONTROL,   0x16 },
        { J2534_ERR_NOT_UNIQUE,        0x17 },
        { J2534_ERR_INVALID_BAUDRATE,  0x18 },
        { J2534_ERR_INVALID_DEVICE_ID, 0x19 },
        { J2534_ERR_DEVICE_IN_USE,     0x1A },
    };
    static const uint32_t SAME[] =
    {
        J2534_STATUS_NOERROR, J2534_ERR_NOT_SUPPORTED,
        J2534_ERR_INVALID_CHANNEL_ID, J2534_ERR_INVALID_IOCTL_VALUE,
        J2534_ERR_FAILED, J2534_ERR_DEVICE_NOT_CONNECTED, J2534_ERR_TIMEOUT,
        J2534_ERR_INVALID_MSG, J2534_ERR_EXCEEDED_LIMIT,
        J2534_ERR_BUFFER_EMPTY, J2534_ERR_BUFFER_FULL,
        J2534_ERR_BUFFER_OVERFLOW, J2534_ERR_NO_CONNECTION_ESTABLISHED,
        0x13, 0x1B, 0xABCD,
    };

    for (size_t i = 0; i < sizeof(MOVED) / sizeof(MOVED[0]); i++)
    {
        /* version 1: the number every client before 2026-11-18 knows */
        TEST_ASSERT_EQUAL_HEX32(MOVED[i].v1,
                                j2534_status_wire(MOVED[i].status, 1));
        TEST_ASSERT_EQUAL_HEX32(MOVED[i].v1,
                                j2534_status_wire(MOVED[i].status, 0));
        /* version 2 and later: the standard's own */
        TEST_ASSERT_EQUAL_HEX32(MOVED[i].status,
                                j2534_status_wire(MOVED[i].status, 2));
        TEST_ASSERT_EQUAL_HEX32(MOVED[i].status,
                                j2534_status_wire(MOVED[i].status, 3));

        /* no two of them share a version 1 number */
        for (size_t k = 0; k < i; k++)
        {
            TEST_ASSERT_NOT_EQUAL(MOVED[k].v1, MOVED[i].v1);
        }
    }

    for (size_t i = 0; i < sizeof(SAME) / sizeof(SAME[0]); i++)
    {
        TEST_ASSERT_EQUAL_HEX32(SAME[i], j2534_status_wire(SAME[i], 1));
        TEST_ASSERT_EQUAL_HEX32(SAME[i], j2534_status_wire(SAME[i], 2));
    }
}

/* the refusal of a second tester, as each kind of client is answered */
static void test_refusal_by_version(void)
{
    uint8_t buf[24];
    static const uint8_t V2[] =
    {
        0x35, 0x4A, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
        0x0E, 0x00, 0x00, 0x00,
    };
    static const uint8_t ASK2[] = { 0x02, 0x00, 0x00, 0x00 };

    TEST_ASSERT_EQUAL(16, j2534_ack_encode(buf, sizeof(buf), 1, 0,
        j2534_status_wire(J2534_ERR_DEVICE_IN_USE,
                          j2534_hello_version(ASK2, sizeof(ASK2))), NULL));
    TEST_ASSERT_EQUAL_MEMORY(V2, buf, 16);

    TEST_ASSERT_EQUAL(16, j2534_ack_encode(buf, sizeof(buf), 1, 0,
        j2534_status_wire(J2534_ERR_DEVICE_IN_USE,
                          j2534_hello_version(NULL, 0)), NULL));
    TEST_ASSERT_EQUAL_HEX8(0x1A, buf[12]);
}

/* EVENT RX_LOST on channel 2: 909 messages (0x38D) lost in front of its
   next RX_MSG. The vector of J2534_WIRE_PROTOCOL.md section 13. */
static void test_event_encode_vector(void)
{
    uint8_t buf[J2534_EVENT_FRAME_SIZE + 4];
    static const uint8_t RX_LOST[] =
    {
        0x35, 0x4A, 0x01, 0x82, 0x00, 0x00, 0x02, 0x00, 0x08, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00, 0x8D, 0x03, 0x00, 0x00,
    };
    j2534_hdr_t h;

    memset(buf, 0xEE, sizeof(buf));
    TEST_ASSERT_EQUAL(20, J2534_EVENT_FRAME_SIZE);
    TEST_ASSERT_EQUAL(20, j2534_event_encode(buf, sizeof(buf), 2,
                                             J2534_EVENT_RX_LOST, 909));
    TEST_ASSERT_EQUAL_MEMORY(RX_LOST, buf, 20);
    TEST_ASSERT_EQUAL_HEX8(0xEE, buf[20]); /* nothing behind the frame */

    TEST_ASSERT_TRUE(j2534_hdr_decode(buf, J2534_HDR_SIZE, &h));
    TEST_ASSERT_EQUAL(J2534_MT_EVENT, h.type);
    TEST_ASSERT_EQUAL(0, h.seq);
    TEST_ASSERT_EQUAL(2, h.channel);
    TEST_ASSERT_EQUAL(8, h.length);

    TEST_ASSERT_EQUAL(0, j2534_event_encode(buf, 19, 2, J2534_EVENT_RX_LOST, 1));
    TEST_ASSERT_EQUAL(0, j2534_event_encode(NULL, 20, 2, J2534_EVENT_RX_LOST, 1));
}

static void test_lost_since(void)
{
    uint32_t seen = 0;

    TEST_ASSERT_EQUAL_UINT32(0, j2534_lost_since(0, &seen));
    TEST_ASSERT_EQUAL_UINT32(909, j2534_lost_since(909, &seen));
    TEST_ASSERT_EQUAL_UINT32(909, seen);
    TEST_ASSERT_EQUAL_UINT32(0, j2534_lost_since(909, &seen));   /* told once */
    TEST_ASSERT_EQUAL_UINT32(716, j2534_lost_since(1625, &seen));
    /* a new subscription counts from zero again: all of it is new */
    TEST_ASSERT_EQUAL_UINT32(3, j2534_lost_since(3, &seen));
    TEST_ASSERT_EQUAL_UINT32(3, seen);
    TEST_ASSERT_EQUAL_UINT32(0, j2534_lost_since(0, &seen));
    TEST_ASSERT_EQUAL_UINT32(0, seen);
    TEST_ASSERT_EQUAL_UINT32(0, j2534_lost_since(5, NULL));
}

void run_session_tests(void)
{
    RUN_TEST(test_status_numbers_are_sae);
    RUN_TEST(test_hello_version);
    RUN_TEST(test_status_wire_by_version);
    RUN_TEST(test_refusal_by_version);
    RUN_TEST(test_event_encode_vector);
    RUN_TEST(test_lost_since);
}
