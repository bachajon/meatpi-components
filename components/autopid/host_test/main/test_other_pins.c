/**
 * @file test_other_pins.c
 * @brief Host suite for the bus guard's word about the pins that are not CAN
 *        (autopid_bus_guard.c, ap_guard_other_pins_open): a park for the CAN
 *        pins says nothing about SAE J1850 and the K-line. The case is a car
 *        from the field (GitHub issue #1022), played on the bench with ECU
 *        sim v2 as a KWP car behind a 50 kbit/s bus on the CAN pins. The
 *        detection job is the one that asks (ap_guard_detect_ok); the
 *        poller and first contact stay parked. Run from test_main.c's
 *        app_main.
 */
#include <string.h>

#include "unity.h"

#include "autopid_private.h"

static const ap_bus_t SILENT = { .kind = AP_BUS_SILENT };
static const ap_bus_t UNKNOWN = { .kind = AP_BUS_UNKNOWN };
static const ap_bus_t LIVE_500 = { .kind = AP_BUS_LIVE, .kbps = 500 };
static const ap_bus_t LIVE_250 = { .kind = AP_BUS_LIVE, .kbps = 250 };
static const ap_bus_t UNREADABLE = { .kind = AP_BUS_UNREADABLE };

/* GitHub issue #1022 (2026-11-27), a 2008 Fiat Ducato: a body bus at 50
   kbit/s on pins 6 and 14, the engine ECU on the K-line. The guard parks the
   chip for the CAN pins, rightly, and that park stopped everything: "Detect
   my vehicle" was refused before its walk ("The car did not answer"), first
   contact never left the CAN pins, and only the protocol set to 5 by hand
   worked. SAE J1850 and the K-line are other pins: a park nobody asked for
   with a pinned CAN protocol leaves them open. */
void test_guard_a_park_for_can_leaves_the_other_pins_open(void)
{
    const ap_bus_t by_node = { AP_BUS_UNREADABLE, 0, 500 };

    /* a unit from the factory: no car in the store, the chip's search */
    TEST_ASSERT_EQUAL(AP_GUARD_PARK,
                      ap_guard_decide(&UNREADABLE, '0', false));
    TEST_ASSERT_TRUE(ap_guard_other_pins_open(&UNREADABLE, '0', false));
    TEST_ASSERT_TRUE(ap_guard_other_pins_open(&UNREADABLE, '\0', false));

    /* a unit that knows a CAN car and was plugged into this one */
    TEST_ASSERT_TRUE(ap_guard_other_pins_open(&UNREADABLE, '6', false));
    TEST_ASSERT_TRUE(ap_guard_other_pins_open(&UNREADABLE, '8', false));
    TEST_ASSERT_TRUE(ap_guard_other_pins_open(&by_node, '0', false));

    /* the setting pins a CAN protocol: the user's word, nothing is looked
       for elsewhere, on traffic that cannot be read and at the other rate */
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&UNREADABLE, '6', true));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&by_node, '8', true));
    TEST_ASSERT_EQUAL(AP_GUARD_PARK, ap_guard_decide(&LIVE_250, '6', true));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&LIVE_250, '6', true));

    /* no park, nothing to open: the walk asks on CAN as before */
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&SILENT, '0', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&UNKNOWN, '0', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&LIVE_500, '0', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&LIVE_500, '6', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(NULL, '0', false));
    /* a stored protocol at the other rate gives way to the search */
    TEST_ASSERT_EQUAL(AP_GUARD_SEARCH,
                      ap_guard_decide(&LIVE_250, '6', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&LIVE_250, '6', false));

    /* a car known on the K-line (or pinned there) is not parked at all */
    TEST_ASSERT_EQUAL(AP_GUARD_ALLOW,
                      ap_guard_decide(&UNREADABLE, '5', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&UNREADABLE, '5', false));
    TEST_ASSERT_FALSE(ap_guard_other_pins_open(&UNREADABLE, '5', true));
}

/* The park's sentence says where nothing is sent: the pins the park is about.
   It read "nothing is transmitted" until this change, and with a K-line start
   on pin 7 a moment later that would not be true. */
void test_guard_the_parks_sentence_names_the_can_pins(void)
{
    char s[176]; /* the size of the buffers that keep the sentence */

    size_t n = ap_guard_reason(&UNREADABLE, '0', false, s, sizeof(s));

    TEST_ASSERT_EQUAL_STRING("the vehicle bus carries traffic at a bitrate "
                             "that could not be read (not 250 or 500 "
                             "kbit/s): nothing is sent on CAN", s);
    TEST_ASSERT_EQUAL_UINT(strlen(s), n);

    /* the same words under a pinned CAN protocol */
    (void)ap_guard_reason(&UNREADABLE, '6', true, s, sizeof(s));
    TEST_ASSERT_NOT_NULL(strstr(s, "nothing is sent on CAN"));

    /* a pinned CAN protocol at the other bit rate: nothing at all, as before */
    (void)ap_guard_reason(&LIVE_250, '6', true, s, sizeof(s));
    TEST_ASSERT_NOT_NULL(strstr(s, "nothing is transmitted (set the protocol "
                                   "to Automatic)"));

    /* no park: a car known on the K-line behind such a bus is asked, and the
       status says what the bus is (it read "vehicle bus not probed", bench
       2026-11-27, with the bus probed and unreadable) */
    (void)ap_guard_reason(&UNREADABLE, '5', false, s, sizeof(s));
    TEST_ASSERT_EQUAL_STRING("vehicle bus traffic could not be read; "
                             "protocol 5 is asked on other pins", s);
    (void)ap_guard_reason(&UNREADABLE, '2', true, s, sizeof(s));
    TEST_ASSERT_NOT_NULL(strstr(s, "protocol 2 is asked on other pins"));
    /* the user's own CAN definition: on the CAN pins, and not this guard's */
    (void)ap_guard_reason(&UNREADABLE, 'B', true, s, sizeof(s));
    TEST_ASSERT_EQUAL_STRING("vehicle bus traffic could not be read; "
                             "protocol B is not judged", s);
    /* nothing known about the bus: as before */
    (void)ap_guard_reason(&UNKNOWN, '5', false, s, sizeof(s));
    TEST_ASSERT_EQUAL_STRING("vehicle bus not probed", s);
}

void run_other_pins_tests(void)
{
    RUN_TEST(test_guard_a_park_for_can_leaves_the_other_pins_open);
    RUN_TEST(test_guard_the_parks_sentence_names_the_can_pins);
}
