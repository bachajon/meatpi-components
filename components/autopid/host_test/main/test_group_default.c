/**
 * @file test_group_default.c
 * @brief A group's state at boot and after a PUT (GitHub issue #1058): the
 *        word the README writes ("enabled") and the word the web UI writes
 *        ("enabled_default") both keep a group off, through the
 *        scheduler's reset and a rule's undo.
 */
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "autopid_private.h"

/* ~1 MB (the 2048-parameter pool): static, never on the task stack */
static ap_config_t s_cfg;
static ap_sched_t s_st;

/* one group with the given words, one row in it */
static void parse_group(const char *words)
{
    char json[256];
    char err[96] = "";

    snprintf(json, sizeof(json),
             "{\"groups\":[{\"name\":\"body\"%s}],"
             "\"pids\":[{\"name\":\"door\",\"cmd\":\"22F190\","
             "\"group\":\"body\",\"parameters\":["
             "{\"name\":\"door\",\"expression\":\"B4\"}]}]}",
             words);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK,
                              ap_config_parse(json, &s_cfg, err, sizeof(err)),
                              err);
    TEST_ASSERT_EQUAL_STRING("body", s_cfg.groups[0].name);
}

static void test_group_default_words(void)
{
    /* the words the web UI and the setup page write: the issue's case */
    parse_group(",\"enabled_default\":false");
    TEST_ASSERT_FALSE(s_cfg.groups[0].enabled_default);
    parse_group(",\"enabled_default\":true");
    TEST_ASSERT_TRUE(s_cfg.groups[0].enabled_default);

    /* the README's word, as before */
    parse_group(",\"enabled\":false");
    TEST_ASSERT_FALSE(s_cfg.groups[0].enabled_default);
    parse_group(",\"enabled\":true");
    TEST_ASSERT_TRUE(s_cfg.groups[0].enabled_default);

    /* neither: on, as before */
    parse_group("");
    TEST_ASSERT_TRUE(s_cfg.groups[0].enabled_default);

    /* both, one false: off, whichever it is */
    parse_group(",\"enabled\":true,\"enabled_default\":false");
    TEST_ASSERT_FALSE(s_cfg.groups[0].enabled_default);
    parse_group(",\"enabled\":false,\"enabled_default\":true");
    TEST_ASSERT_FALSE(s_cfg.groups[0].enabled_default);

    /* a word that is not a bool is ignored, as a row's "enabled" is */
    parse_group(",\"enabled_default\":0");
    TEST_ASSERT_TRUE(s_cfg.groups[0].enabled_default);

    /* the "default" group the parser adds is on */
    TEST_ASSERT_EQUAL(2, s_cfg.n_groups);
    TEST_ASSERT_EQUAL_STRING("default", s_cfg.groups[1].name);
    TEST_ASSERT_TRUE(s_cfg.groups[1].enabled_default);
}

static void test_group_default_through_the_scheduler(void)
{
    parse_group(",\"enabled_default\":false");

    /* what a boot and every PUT do (autopid.c): the row does not poll */
    memset(&s_st, 0, sizeof(s_st));
    s_st.type_enabled[0] = s_st.type_enabled[1] = s_st.type_enabled[2] =
        true;
    ap_sched_reset(&s_st, &s_cfg, 0);
    TEST_ASSERT_FALSE(s_st.group_enabled[0]);
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));

    /* a rule switches it on (the car in Ready): it polls */
    ap_sched_group_set(&s_st, 0, true, -1);
    TEST_ASSERT_TRUE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));

    /* the rule's undo puts back the configured state: off */
    ap_sched_group_restore(&s_st, &s_cfg, 0);
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));

    /* and the next reset leaves it off */
    ap_sched_group_set(&s_st, 0, true, -1);
    ap_sched_reset(&s_st, &s_cfg, 0);
    TEST_ASSERT_FALSE(ap_sched_entry_enabled(&s_st, &s_cfg, 0));
}

void run_group_default_tests(void)
{
    RUN_TEST(test_group_default_words);
    RUN_TEST(test_group_default_through_the_scheduler);
}
