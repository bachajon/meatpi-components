/**
 * @file test_filter_type.c
 * @brief A CAN filter follows the switch of its table (2026-11-14):
 *        `"profile": true` marks a filter of the vehicle's set, gated by
 *        the vehicle-specific switch; every other filter is gated by the
 *        custom one. The line a test of a filter shows for a captured
 *        frame (ap_flt_frame_line), read back by the monitor-line parser.
 *        And the protocol a window opens on (ap_flt_window_proto): the
 *        one of the id's width, as the legacy firmware chose it. And the
 *        bus of a filter (GitHub issue #1078): since 2026-11-22 its own, a
 *        word this firmware does not know refused while the filter is on
 *        and switched off in a stored file.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "autopid_bus.h"
#include "autopid_private.h"

/* ~1 MB (the 2048-parameter pool): static, never on the task stack */
static ap_config_t s_cfg;
static ap_sched_t s_st;

#define PRM(n) "\"parameters\":[{\"name\":\"" n "\",\"expression\":\"B0\"}]"

static void load(const char *json)
{
    char err[96] = "";

    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ap_config_parse(json, &s_cfg, err, sizeof(err)),
                              err);
    memset(&s_st, 0, sizeof(s_st));
    ap_sched_reset(&s_st, &s_cfg, 0);
    s_st.type_enabled[AP_PID_STD] = true;
    s_st.type_enabled[AP_PID_CUSTOM] = true;
    s_st.type_enabled[AP_PID_SPECIFIC] = true;
}

void test_filter_type_from_the_profile_tag(void)
{
    /* the tag the web UI writes; a "name" on a filter is the page's label
       and is not the parser's business */
    load("{\"filters\":["
         "{\"frame_id\":644,\"profile\":true,\"name\":\"SPEED\"," PRM("a") "},"
         "{\"frame_id\":645,\"profile\":false," PRM("b") "},"
         "{\"frame_id\":646," PRM("c") "}]}");

    TEST_ASSERT_EQUAL(3, s_cfg.n_filters);
    TEST_ASSERT_EQUAL(AP_PID_SPECIFIC, s_cfg.filters[0].type);
    TEST_ASSERT_EQUAL(AP_PID_CUSTOM, s_cfg.filters[1].type);
    TEST_ASSERT_EQUAL(AP_PID_CUSTOM, s_cfg.filters[2].type);
}

void test_filter_type_follows_its_switch(void)
{
    /* entry 0 = the custom PID, 1 = the vehicle's filter, 2 = a custom one */
    load("{\"pids\":[{\"name\":\"p\",\"type\":\"custom\",\"cmd\":\"22F190\","
         PRM("p") "}],"
         "\"filters\":["
         "{\"frame_id\":644,\"profile\":true," PRM("a") "},"
         "{\"frame_id\":645," PRM("b") "}]}");

    TEST_ASSERT_EQUAL(1, s_cfg.n_pids);

    for (int i = 0; i < 3; i++)
    {
        TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, i));
    }

    /* "Poll vehicle-specific PIDs" off: its filter rests, the others go on */
    s_st.type_enabled[AP_PID_SPECIFIC] = false;
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 1));
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 2));

    /* "Poll custom PIDs" off: the custom PID and the custom filter rest */
    s_st.type_enabled[AP_PID_SPECIFIC] = true;
    s_st.type_enabled[AP_PID_CUSTOM] = false;
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 1));
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 2));

    /* the standard switch is about no filter */
    s_st.type_enabled[AP_PID_CUSTOM] = true;
    s_st.type_enabled[AP_PID_STD] = false;
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 1));
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 2));

    /* a held init (it carries a word the chip stores) keeps the rows that
       are sent behind it waiting; a window sends no init and goes on */
    s_st.type_enabled[AP_PID_STD] = true;
    s_st.type_held[AP_PID_CUSTOM] = true;
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 2));

    /* its own switch still wins */
    s_st.type_held[AP_PID_CUSTOM] = false;
    s_cfg.filters[1].enabled = false;
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 2));
}

void test_filter_type_gated_filter_is_never_picked(void)
{
    load("{\"filters\":["
         "{\"frame_id\":644,\"profile\":true," PRM("a") "},"
         "{\"frame_id\":645," PRM("b") "}]}");

    int64_t due = 0;

    s_st.type_enabled[AP_PID_SPECIFIC] = false;

    /* the custom filter is the only thing left to run, round after round */
    for (int round = 0; round < 6; round++)
    {
        int i = ap_sched_next(&s_st, &s_cfg, &due);

        TEST_ASSERT_EQUAL(1, i);
        ap_sched_ran(&s_st, &s_cfg, i, due + 1000, true);
    }

    /* both switches off: nothing to run */
    s_st.type_enabled[AP_PID_CUSTOM] = false;
    TEST_ASSERT_EQUAL(-1, ap_sched_next(&s_st, &s_cfg, &due));
}

void test_flt_frame_line(void)
{
    const uint8_t data[8] = { 0x02, 0x1E, 0x02, 0x1F, 0x0B, 0x7C, 0x4E, 0x20 };
    char line[64];
    uint8_t back[16];
    size_t n = 0;

    /* an 11-bit id: three digits, then the data bytes */
    TEST_ASSERT_EQUAL(27, ap_flt_frame_line(0x284, data, 8, line,
                                            sizeof(line)));
    TEST_ASSERT_EQUAL_STRING("284 02 1E 02 1F 0B 7C 4E 20", line);

    /* the parser of the monitor's lines reads it back: same bytes */
    TEST_ASSERT_TRUE(ap_filter_frame(line, strlen(line), 0x284, back,
                                     sizeof(back), &n));
    TEST_ASSERT_EQUAL(8, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data, back, 8);

    /* a 29-bit id: eight digits */
    TEST_ASSERT_EQUAL(14, ap_flt_frame_line(0x18DAF110, data, 2, line,
                                            sizeof(line)));
    TEST_ASSERT_EQUAL_STRING("18DAF110 02 1E", line);

    /* a frame without data: the id alone */
    TEST_ASSERT_EQUAL(3, ap_flt_frame_line(0x7FF, NULL, 0, line,
                                           sizeof(line)));
    TEST_ASSERT_EQUAL_STRING("7FF", line);

    /* a byte that does not fit is left out whole, never cut in two */
    TEST_ASSERT_EQUAL(9, ap_flt_frame_line(0x284, data, 8, line, 10));
    TEST_ASSERT_EQUAL_STRING("284 02 1E", line);
    TEST_ASSERT_EQUAL(9, ap_flt_frame_line(0x284, data, 8, line, 12));
    TEST_ASSERT_EQUAL_STRING("284 02 1E", line);

    /* no room for the id: nothing, and a terminated string */
    line[0] = 'x';
    TEST_ASSERT_EQUAL(0, ap_flt_frame_line(0x284, data, 8, line, 3));
    TEST_ASSERT_EQUAL_STRING("", line);
}

void test_flt_window_proto(void)
{
    /* the legacy firmware's rule (v4.50 send_can_filter_cmd): the window
       opens on the protocol of the id's width, the sibling at the same bit
       rate when the chip is on the other one */
    TEST_ASSERT_EQUAL_CHAR('7', ap_flt_window_proto('6', true));
    TEST_ASSERT_EQUAL_CHAR('6', ap_flt_window_proto('7', false));
    TEST_ASSERT_EQUAL_CHAR('9', ap_flt_window_proto('8', true));
    TEST_ASSERT_EQUAL_CHAR('8', ap_flt_window_proto('9', false));

    /* the width fits: the chip stays where it is */
    TEST_ASSERT_EQUAL_CHAR('\0', ap_flt_window_proto('6', false));
    TEST_ASSERT_EQUAL_CHAR('\0', ap_flt_window_proto('7', true));
    TEST_ASSERT_EQUAL_CHAR('\0', ap_flt_window_proto('8', false));
    TEST_ASSERT_EQUAL_CHAR('\0', ap_flt_window_proto('9', true));

    /* none of the four: the chip's search, a K-line or J1850 protocol,
       J1939, the user's own CAN definitions, nothing known */
    const char others[] = { '0', '1', '2', '3', '4', '5', 'A', 'B', 'C', '\0' };

    for (size_t i = 0; i < sizeof(others); i++)
    {
        TEST_ASSERT_EQUAL_CHAR('\0', ap_flt_window_proto(others[i], true));
        TEST_ASSERT_EQUAL_CHAR('\0', ap_flt_window_proto(others[i], false));
    }
}

/* A filter listens on its bus (GitHub issue #1078): until 2026-11-21 a
   `bus` on a filter was read by nobody, so a filter of 0x1026C0CB (a GMLAN
   single-wire frame) listened on the HS bus and never filled; on 2026-11-21
   the tables refused it. Since 2026-11-22 the filter names its bus and
   speed as a row does, its window opens there (29-bit: the slot defined for
   the id's width), and the scheduler groups it with that bus's rows, out of
   the HS bus guard's hands. A word this firmware does not know is refused
   while the filter is on; a stored file is switched off once, and the rest
   of it loads. */
void test_filter_on_its_bus(void)
{
    char err[96] = "";
    int off = -1;

    TEST_ASSERT_TRUE(ap_filter_bus_ok(NULL));
    TEST_ASSERT_TRUE(ap_filter_bus_ok(""));
    TEST_ASSERT_TRUE(ap_filter_bus_ok("obd"));
    TEST_ASSERT_TRUE(ap_filter_bus_ok("sw"));
    TEST_ASSERT_TRUE(ap_filter_bus_ok("ms"));
    TEST_ASSERT_TRUE(ap_filter_bus_ok("ch"));
    TEST_ASSERT_FALSE(ap_filter_bus_ok("hs"));
    TEST_ASSERT_FALSE(ap_filter_bus_ok("SW"));

    /* the Volt's climate power (29-bit, single-wire), an 11-bit frame on
       MS CAN, one on CH CAN at 250 kbit/s, the car's own: each where it
       says, 29-bit with the slot of its width */
    load("{\"filters\":["
         "{\"frame_id\":270975179,\"bus\":\"sw\",\"ext\":true," PRM("hvac") "},"
         "{\"frame_id\":1234,\"bus\":\"ms\"," PRM("door") "},"
         "{\"frame_id\":801,\"bus\":\"ch\",\"bus_kbps\":250," PRM("gw") "},"
         "{\"frame_id\":288,\"bus\":\"obd\"," PRM("odo") "},"
         "{\"frame_id\":289," PRM("b") "}]}");
    TEST_ASSERT_EQUAL(5, s_cfg.n_filters);
    TEST_ASSERT_EQUAL(AP_BUS_SW, s_cfg.filters[0].bus);
    TEST_ASSERT_TRUE(s_cfg.filters[0].is_extended);
    TEST_ASSERT_EQUAL_HEX16(AP_BUS_KEY(AP_BUS_SW, 0) | AP_BUS_KEY_EXT,
                            ap_filter_bus_key(&s_cfg.filters[0]));
    TEST_ASSERT_EQUAL_HEX16(AP_BUS_KEY(AP_BUS_MS, 0),
                            ap_filter_bus_key(&s_cfg.filters[1]));
    TEST_ASSERT_EQUAL(2, s_cfg.filters[2].bus_rate); /* 500 / 2 */
    TEST_ASSERT_EQUAL_HEX16(AP_BUS_KEY(AP_BUS_CH, 2),
                            ap_filter_bus_key(&s_cfg.filters[2]));
    TEST_ASSERT_EQUAL_HEX16(0, ap_filter_bus_key(&s_cfg.filters[3]));
    TEST_ASSERT_EQUAL_HEX16(0, ap_filter_bus_key(&s_cfg.filters[4]));

    /* the scheduler: a filter on another bus is that bus's (grouped with
       its rows, its own class); on the car's own bus as before */
    TEST_ASSERT_EQUAL_HEX16(AP_BUS_KEY(AP_BUS_SW, 0) | AP_BUS_KEY_EXT,
                            ap_sched_entry_bus(&s_cfg, 0));
    TEST_ASSERT_EQUAL(AP_CLASS_LISTEN_BUS, ap_sched_entry_class(&s_cfg, 0));
    TEST_ASSERT_TRUE(AP_CLASS_IS_CHIP(ap_sched_entry_class(&s_cfg, 0)));
    TEST_ASSERT_EQUAL(AP_CLASS_LISTEN_BUS, ap_sched_entry_class(&s_cfg, 2));
    TEST_ASSERT_EQUAL_HEX16(0, ap_sched_entry_bus(&s_cfg, 3));
    TEST_ASSERT_EQUAL(AP_CLASS_CHIP, ap_sched_entry_class(&s_cfg, 3));

    /* on, with a bus this firmware does not know, or a speed on the car's
       own bus: refused, whole in the parser's 96 bytes */
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ap_config_parse(
        "{\"filters\":[{\"frame_id\":270975179,\"bus\":\"hs\","
        PRM("hvac") "}]}", &s_cfg, err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("filter 1026C0CB: unknown bus (obd, ms, sw or ch)",
                             err);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ap_config_parse(
        "{\"filters\":[{\"frame_id\":288,\"bus_kbps\":250,"
        PRM("odo") "}]}", &s_cfg, err, sizeof(err)));
    TEST_ASSERT_EQUAL_STRING("filter 120: bus_kbps needs a bus other than obd "
                             "and a speed the chip can make", err);

    /* switched off, either may stay: the file loads, the filter rests */
    load("{\"filters\":["
         "{\"frame_id\":270975179,\"bus\":\"hs\",\"enabled\":false,"
         PRM("hvac") "},"
         "{\"frame_id\":288,\"bus_kbps\":250,\"enabled\":false," PRM("odo") "},"
         "{\"frame_id\":290,\"bus\":7," PRM("c") "}]}");
    TEST_ASSERT_EQUAL(3, s_cfg.n_filters);
    TEST_ASSERT_FALSE(s_cfg.filters[0].enabled);
    TEST_ASSERT_EQUAL(AP_BUS_OBD, s_cfg.filters[0].bus);
    TEST_ASSERT_EQUAL(0, s_cfg.filters[1].bus_rate);
    TEST_ASSERT_EQUAL(AP_BUS_OBD, s_cfg.filters[2].bus); /* not text: none */

    /* the loader's repair: a stored file with such a filter on loads with
       that filter off and every other one as it was; a single-wire filter
       is no longer one of them */
    static const char STORED[] =
        "{\"filters\":["
        "{\"frame_id\":270975179,\"bus\":\"sw\"," PRM("hvac") "},"
        "{\"frame_id\":275857472,\"bus\":\"hs\"," PRM("refused") "},"
        "{\"frame_id\":288,\"bus_kbps\":250," PRM("odo") "},"
        "{\"frame_id\":289,\"bus\":\"ms\",\"enabled\":false," PRM("b") "}]}";

    TEST_ASSERT_NOT_EQUAL(ESP_OK, ap_config_parse(STORED, &s_cfg, err,
                                                  sizeof(err)));

    char *fixed = ap_config_repair_filter_bus(STORED, &off);

    TEST_ASSERT_NOT_NULL(fixed);
    TEST_ASSERT_EQUAL(2, off);
    load(fixed);
    TEST_ASSERT_EQUAL(4, s_cfg.n_filters);
    TEST_ASSERT_TRUE(s_cfg.filters[0].enabled);
    TEST_ASSERT_EQUAL(AP_BUS_SW, s_cfg.filters[0].bus);
    TEST_ASSERT_FALSE(s_cfg.filters[1].enabled);
    TEST_ASSERT_FALSE(s_cfg.filters[2].enabled);
    TEST_ASSERT_FALSE(s_cfg.filters[3].enabled);

    /* once: the repaired text needs no second repair */
    int again = -1;

    TEST_ASSERT_NULL(ap_config_repair_filter_bus(fixed, &again));
    TEST_ASSERT_EQUAL(0, again);
    free(fixed);

    /* nothing to repair, and a text that is no JSON: left to the parser */
    TEST_ASSERT_NULL(ap_config_repair_filter_bus(
        "{\"filters\":[{\"frame_id\":288,\"bus\":\"sw\"," PRM("odo") "}]}",
        &off));
    TEST_ASSERT_EQUAL(0, off);
    TEST_ASSERT_NULL(ap_config_repair_filter_bus("{}", &off));
    TEST_ASSERT_NULL(ap_config_repair_filter_bus("nonsense", &off));
    TEST_ASSERT_EQUAL(0, off);
}

/* A filter on another bus listens through the voltage pause (2026-11-22, a
   2012 Volt): a remote start is decided on the single-wire bus while the
   12 V is low, and a refused one never lifts it, so a filter that waited
   for the pause saw neither. It sends nothing: the pause stops it as it
   stops the passive rows, under pause_mode "all" only, as v4.50 went on
   monitoring under disable_pid_requests. A row of that bus sends, and
   waits; a filter on the car's own bus keeps waiting for the HS guard. */
void test_filter_listens_through_the_pause(void)
{
    load("{\"pids\":["
         "{\"name\":\"seat\",\"cmd\":\"224102\",\"bus\":\"sw\","
         "\"parameters\":[{\"name\":\"seat\",\"expression\":\"B4\"}]}],"
         "\"filters\":["
         "{\"frame_id\":288," PRM("odo") "},"
         "{\"frame_id\":271007947,\"bus\":\"sw\"," PRM("rs") "}]}");
    TEST_ASSERT_EQUAL(AP_CLASS_CHIP_BUS, ap_sched_entry_class(&s_cfg, 0));
    TEST_ASSERT_EQUAL(AP_CLASS_CHIP, ap_sched_entry_class(&s_cfg, 1));
    TEST_ASSERT_EQUAL(AP_CLASS_LISTEN_BUS, ap_sched_entry_class(&s_cfg, 2));
    TEST_ASSERT_TRUE((AP_CLASS_ALL & AP_CLASS_BIT(AP_CLASS_LISTEN_BUS)) != 0);

    /* the voltage pause under requests_only: the poller offers the passive
       classes and the listening one; the filter alone is picked */
    unsigned pause = AP_CLASS_BIT(AP_CLASS_PASSIVE) |
                     AP_CLASS_BIT(AP_CLASS_BUS_TX) |
                     AP_CLASS_BIT(AP_CLASS_LISTEN_BUS);
    int64_t now = 1000000;

    TEST_ASSERT_EQUAL(2, ap_sched_next_of(&s_st, &s_cfg, pause, NULL));
    TEST_ASSERT_EQUAL(2, ap_sched_next_grouped(&s_st, &s_cfg, pause, 0, 0,
                                               now, NULL));

    /* awake, every class, all due at home: the car's own first, as the
       grouping asks before the chip goes away; on the filter's bus (the
       29-bit slot) the filter is picked as a row of that bus would be */
    TEST_ASSERT_EQUAL(1, ap_sched_next_grouped(&s_st, &s_cfg, AP_CLASS_ALL,
                                               0, 0, now, NULL));
    TEST_ASSERT_EQUAL(2, ap_sched_next_grouped(
        &s_st, &s_cfg, AP_CLASS_ALL, AP_BUS_KEY(AP_BUS_SW, 0) | AP_BUS_KEY_EXT,
        now, now, NULL));

    /* under pause_mode "all", or an app on the chip: nothing */
    TEST_ASSERT_EQUAL(-1, ap_sched_next_of(&s_st, &s_cfg,
                                           AP_CLASS_BIT(AP_CLASS_PASSIVE),
                                           NULL));
}

void run_filter_type_tests(void)
{
    RUN_TEST(test_filter_type_from_the_profile_tag);
    RUN_TEST(test_filter_type_follows_its_switch);
    RUN_TEST(test_filter_type_gated_filter_is_never_picked);
    RUN_TEST(test_flt_frame_line);
    RUN_TEST(test_flt_window_proto);
    RUN_TEST(test_filter_on_its_bus);
    RUN_TEST(test_filter_listens_through_the_pause);
}
