/**
 * @file test_tx_slots.c
 * @brief Host suite for can_tx_slots: the transmit slot table of the CAN
 *        driver, and what a bus-off does to it (2026-11-18: the frame
 *        inside the controller at a bus-off gets no tx_done, and its slot
 *        stayed taken for good: one slot gone with every bus-off).
 */
#include "unity.h"

#include "can_tx_slots.h"

static int in_flight(const can_txs_t *t)
{
    return can_txs_in_flight(t);
}

void test_txs_take_until_full_then_room_again(void)
{
    can_txs_t t;

    can_txs_reset(&t);
    TEST_ASSERT_EQUAL_INT(-1, can_txs_in_controller(&t));

    for (int i = 0; i < CAN_TXS_SLOTS; i++)
    {
        TEST_ASSERT_TRUE(can_txs_room(&t));
        TEST_ASSERT_TRUE(can_txs_take(&t) >= 0);
    }

    TEST_ASSERT_FALSE(can_txs_room(&t));
    TEST_ASSERT_EQUAL_INT(-1, can_txs_take(&t));

    can_txs_release(&t, 5);
    TEST_ASSERT_TRUE(can_txs_room(&t));
    TEST_ASSERT_EQUAL_INT(5, can_txs_take(&t));
}

void test_txs_the_controller_holds_the_frame_handed_over_first(void)
{
    can_txs_t t;

    can_txs_reset(&t);

    int a = can_txs_take(&t);
    int b = can_txs_take(&t);
    int c = can_txs_take(&t);

    TEST_ASSERT_EQUAL_INT(a, can_txs_in_controller(&t));

    /* a lost attempt sent again keeps its place: still the first */
    t.tries[a]++;
    TEST_ASSERT_EQUAL_INT(a, can_txs_in_controller(&t));

    can_txs_release(&t, a);
    TEST_ASSERT_EQUAL_INT(b, can_txs_in_controller(&t));

    /* a freed slot taken again is the newest frame, whatever its index */
    int d = can_txs_take(&t);

    TEST_ASSERT_EQUAL_INT(a, d);
    TEST_ASSERT_EQUAL_INT(b, can_txs_in_controller(&t));
    can_txs_release(&t, b);
    TEST_ASSERT_EQUAL_INT(c, can_txs_in_controller(&t));
    can_txs_release(&t, c);
    TEST_ASSERT_EQUAL_INT(d, can_txs_in_controller(&t));
}

void test_txs_bus_off_gives_back_the_slot_of_the_frame_in_the_controller(void)
{
    can_txs_t t;

    can_txs_reset(&t);

    int a = can_txs_take(&t);
    int b = can_txs_take(&t);
    int c = can_txs_take(&t);

    /* a is inside the controller, b and c wait in the driver's queue: the
       bus-off ends a without a tx_done, b and c go out after the recovery */
    TEST_ASSERT_EQUAL_INT(a, can_txs_bus_off(&t));
    TEST_ASSERT_EQUAL_INT(2, in_flight(&t));
    TEST_ASSERT_NOT_EQUAL(0, t.tries[b]);
    TEST_ASSERT_NOT_EQUAL(0, t.tries[c]);

    /* the recovery: the driver sends b, then c, each with its tx_done */
    TEST_ASSERT_EQUAL_INT(b, can_txs_in_controller(&t));
    can_txs_release(&t, b);
    can_txs_release(&t, c);
    TEST_ASSERT_EQUAL_INT(0, in_flight(&t));
}

void test_txs_bus_off_with_no_frame_in_flight(void)
{
    can_txs_t t;

    can_txs_reset(&t);
    TEST_ASSERT_EQUAL_INT(-1, can_txs_bus_off(&t));
    TEST_ASSERT_EQUAL_INT(0, in_flight(&t));
}

void test_txs_bus_off_many_times_costs_no_slot(void)
{
    can_txs_t t;

    can_txs_reset(&t);

    /* the field case: a bus that faults again and again, a frame inside
       the controller each time. Until 2026-11-18 the sixteenth bus-off
       left no slot and nothing was transmitted any more */
    for (int n = 0; n < 4 * CAN_TXS_SLOTS; n++)
    {
        TEST_ASSERT_TRUE_MESSAGE(can_txs_room(&t), "no slot left after a bus-off");
        TEST_ASSERT_TRUE(can_txs_take(&t) >= 0);
        (void)can_txs_bus_off(&t);
        TEST_ASSERT_EQUAL_INT(0, in_flight(&t));
    }
}

void test_txs_order_survives_the_counter_wrap(void)
{
    can_txs_t t;

    can_txs_reset(&t);
    t.next = UINT32_MAX - 1;

    int a = can_txs_take(&t); /* order 0xFFFFFFFE */
    int b = can_txs_take(&t); /* order 0xFFFFFFFF */
    int c = can_txs_take(&t); /* order 0: the newest of the three */

    TEST_ASSERT_EQUAL_INT(a, can_txs_in_controller(&t));
    TEST_ASSERT_EQUAL_INT(a, can_txs_bus_off(&t));
    TEST_ASSERT_EQUAL_INT(b, can_txs_bus_off(&t));
    TEST_ASSERT_EQUAL_INT(c, can_txs_bus_off(&t));
    TEST_ASSERT_EQUAL_INT(-1, can_txs_bus_off(&t));
}

void test_txs_frames_in_flight_are_counted_before_a_reset(void)
{
    can_txs_t t;

    can_txs_reset(&t);
    TEST_ASSERT_EQUAL_INT(0, can_txs_in_flight(&t));

    int a = can_txs_take(&t);

    (void)can_txs_take(&t);
    (void)can_txs_take(&t);
    t.tries[a] = 3; /* a frame on its third attempt is one frame */
    TEST_ASSERT_EQUAL_INT(3, can_txs_in_flight(&t));

    can_txs_release(&t, a);
    TEST_ASSERT_EQUAL_INT(2, can_txs_in_flight(&t));

    /* a node restart: what it ends is known before the table is emptied */
    can_txs_reset(&t);
    TEST_ASSERT_EQUAL_INT(0, can_txs_in_flight(&t));
    TEST_ASSERT_TRUE(can_txs_room(&t));
}

void run_tx_slots_tests(void)
{
    RUN_TEST(test_txs_take_until_full_then_room_again);
    RUN_TEST(test_txs_the_controller_holds_the_frame_handed_over_first);
    RUN_TEST(test_txs_bus_off_gives_back_the_slot_of_the_frame_in_the_controller);
    RUN_TEST(test_txs_bus_off_with_no_frame_in_flight);
    RUN_TEST(test_txs_bus_off_many_times_costs_no_slot);
    RUN_TEST(test_txs_order_survives_the_counter_wrap);
    RUN_TEST(test_txs_frames_in_flight_are_counted_before_a_reset);
}
