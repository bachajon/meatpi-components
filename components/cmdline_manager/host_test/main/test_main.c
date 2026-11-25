/**
 * @file test_main.c
 * @brief Host tests for cmdline_manager's pure pieces: the line
 *        assembler (every transport chunking pattern, whole lines, split
 *        lines, CRLF, batches, oversize floods, must come out as clean
 *        lines) and the dispatcher's verdict line (what a client is owed
 *        after a command; issue #980: a console failure must not be
 *        silent), the formatter behind cmdline_printf (a text of any
 *        length is written whole) and the argument vector's size (no
 *        line that fits the assembler loses a word).
 *        Expected: 18 Tests 0 Failures 0 Ignored.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "cmdline_manager_private.h"

static cm_line_asm_t s_asm;
static char s_verdict[CM_VERDICT_BUF];

void setUp(void)
{
    cm_line_reset(&s_asm);
    memset(s_verdict, 'x', sizeof(s_verdict)); /* silence must write "" */
}

void tearDown(void)
{
}

/** Feed everything, collecting completed lines into @p out[]. */
static int feed_all(const char *bytes, const char *out[], int max)
{
    size_t len = strlen(bytes);
    size_t off = 0;
    int found = 0;
    static char store[8][256];

    while (off < len)
    {
        const char *line = NULL;

        off += cm_line_feed(&s_asm, (const uint8_t *)bytes + off,
                            len - off, &line);

        if (line != NULL && found < max)
        {
            snprintf(store[found], sizeof(store[found]), "%s", line);
            out[found] = store[found];
            found++;
        }
    }

    return found;
}

static void test_simple_line(void)
{
    const char *lines[4];

    TEST_ASSERT_EQUAL_INT(1, feed_all("version\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("version", lines[0]);
}

static void test_crlf_yields_one_line(void)
{
    const char *lines[4];

    /* the \r terminates; the \n tail must NOT become an empty line */
    TEST_ASSERT_EQUAL_INT(1, feed_all("status\r\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("status", lines[0]);
}

static void test_split_across_chunks(void)
{
    const char *line = NULL;

    /* transports chunk arbitrarily: "ver" now, "sion\n" later */
    cm_line_feed(&s_asm, (const uint8_t *)"ver", 3, &line);
    TEST_ASSERT_NULL(line);
    cm_line_feed(&s_asm, (const uint8_t *)"sion\n", 5, &line);
    TEST_ASSERT_NOT_NULL(line);
    TEST_ASSERT_EQUAL_STRING("version", line);
}

static void test_batch_of_lines_one_chunk(void)
{
    const char *lines[4];

    TEST_ASSERT_EQUAL_INT(3, feed_all("help\nrtc sync\r\nimu\n",
                                      lines, 4));
    TEST_ASSERT_EQUAL_STRING("help", lines[0]);
    TEST_ASSERT_EQUAL_STRING("rtc sync", lines[1]);
    TEST_ASSERT_EQUAL_STRING("imu", lines[2]);
}

static void test_empty_lines_skipped(void)
{
    const char *lines[4];

    /* a terminal sending bare newlines must not run empty commands */
    TEST_ASSERT_EQUAL_INT(1, feed_all("\n\r\n\nled\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("led", lines[0]);
}

static void test_overflow_discarded_then_recovers(void)
{
    static char flood[400];
    const char *lines[4];

    memset(flood, 'x', sizeof(flood) - 2);
    flood[sizeof(flood) - 2] = '\n';
    flood[sizeof(flood) - 1] = '\0';

    /* an oversize line (buf is 256) is dropped whole... */
    TEST_ASSERT_EQUAL_INT(0, feed_all(flood, lines, 4));
    /* ...and the assembler is clean for the next command */
    TEST_ASSERT_EQUAL_INT(1, feed_all("battery\n", lines, 4));
    TEST_ASSERT_EQUAL_STRING("battery", lines[0]);
}

/* ---- the dispatcher's verdict line --------------------------------------------- */

static void test_verdict_success_is_silent(void)
{
    /* the command ran and returned 0: its own output is the answer */
    TEST_ASSERT_EQUAL_INT(0, (int)cm_exec_verdict(ESP_OK, 0, "version",
                                                  s_verdict,
                                                  sizeof(s_verdict)));
    TEST_ASSERT_EQUAL_STRING("", s_verdict);
}

static void test_verdict_empty_line_is_silent(void)
{
    /* esp_console_run: ESP_ERR_INVALID_ARG = whitespace only, nothing ran */
    TEST_ASSERT_EQUAL_INT(0, (int)cm_exec_verdict(ESP_ERR_INVALID_ARG, 0,
                                                  "  ", s_verdict,
                                                  sizeof(s_verdict)));
    TEST_ASSERT_EQUAL_STRING("", s_verdict);
}

static void test_verdict_unknown_command(void)
{
    size_t n = cm_exec_verdict(ESP_ERR_NOT_FOUND, 0, "bogus 1 2",
                               s_verdict, sizeof(s_verdict));

    TEST_ASSERT_EQUAL_STRING("Unknown command: bogus 1 2\nTry: help\n",
                             s_verdict);
    TEST_ASSERT_EQUAL_INT((int)strlen(s_verdict), (int)n);
}

static void test_verdict_longest_unknown_line_fits(void)
{
    /* the assembler's longest line (255 chars) keeps both its lines:
     * cmdline_printf's 256-byte buffer used to cut the newline off */
    static char line[256];
    memset(line, 'q', sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';

    size_t n = cm_exec_verdict(ESP_ERR_NOT_FOUND, 0, line, s_verdict,
                               sizeof(s_verdict));

    TEST_ASSERT_EQUAL_INT(17 + 255 + 11, (int)n);
    TEST_ASSERT_EQUAL_STRING("\nTry: help\n", s_verdict + 17 + 255);
}

static void test_verdict_handler_failure_names_the_code(void)
{
    cm_exec_verdict(ESP_OK, 3, "rtc -x", s_verdict, sizeof(s_verdict));
    TEST_ASSERT_EQUAL_STRING("command failed (3)\n", s_verdict);
}

static void test_verdict_console_failure_is_printed(void)
{
    /* issue #980: esp_console_run failing on its own (no memory for the
     * argument vector, console not initialised) printed NOTHING, so the
     * command looked as if it had worked. The verdict names the error
     * and says the command did not run. */
    size_t n = cm_exec_verdict(ESP_ERR_NO_MEM, 0, "version", s_verdict,
                               sizeof(s_verdict));

    TEST_ASSERT_TRUE_MESSAGE(n > 0,
                             "a console failure printed nothing (issue #980)");
    TEST_ASSERT_NOT_NULL(strstr(s_verdict, "Internal error"));
    TEST_ASSERT_NOT_NULL(strstr(s_verdict, "ESP_ERR_NO_MEM"));
    TEST_ASSERT_NOT_NULL(strstr(s_verdict, "0x101"));
    TEST_ASSERT_NOT_NULL(strstr(s_verdict, "did not run"));
    TEST_ASSERT_EQUAL_CHAR('\n', s_verdict[n - 1]);

    cm_exec_verdict(ESP_ERR_INVALID_STATE, 0, "version", s_verdict,
                    sizeof(s_verdict));
    TEST_ASSERT_NOT_NULL(strstr(s_verdict, "ESP_ERR_INVALID_STATE"));
}

static void test_verdict_truncates_to_the_buffer(void)
{
    char small[8];
    size_t n = cm_exec_verdict(ESP_ERR_NOT_FOUND, 0, "bogus", small,
                               sizeof(small));

    TEST_ASSERT_EQUAL_INT(7, (int)n);
    TEST_ASSERT_EQUAL_STRING("Unknown", small);
}

/* ---- the formatter behind cmdline_printf ----------------------------------------- */

static char s_printed[16384]; /* what the console's output received */
static size_t s_printed_len;
static int s_writes;
static int s_allocs;
static int s_releases;
static bool s_no_memory;

static void port_write(const char *data, size_t len)
{
    s_writes++;

    if (s_printed_len + len < sizeof(s_printed))
    {
        memcpy(s_printed + s_printed_len, data, len);
        s_printed_len += len;
        s_printed[s_printed_len] = '\0';
    }
}

static void *port_alloc(size_t size)
{
    s_allocs++;
    return s_no_memory ? NULL : malloc(size);
}

static void port_release(void *block)
{
    s_releases++;
    free(block);
}

static const cm_print_port_t PORT =
{
    .write   = port_write,
    .alloc   = port_alloc,
    .release = port_release,
};

static size_t print(const char *fmt, ...)
{
    char small[256]; /* cmdline_printf's stack buffer */
    va_list args;

    s_printed_len = 0;
    s_printed[0] = '\0';
    s_writes = 0;
    s_allocs = 0;
    s_releases = 0;

    va_start(args, fmt);
    size_t n = cm_vprint(&PORT, small, sizeof(small), fmt, args);
    va_end(args);
    return n;
}

/** "00 01 02 .." for @p count bytes, as the uds command prints an answer. */
static const char *hex_run(size_t count)
{
    static char hex[3 * 4096 + 1];
    size_t o = 0;

    for (size_t i = 0; i < count; i++)
    {
        o += (size_t)snprintf(hex + o, sizeof(hex) - o, "%s%02X",
                              i ? " " : "", (unsigned)(i & 0xFF));
    }

    return hex;
}

static void test_print_short_text_needs_no_block(void)
{
    s_no_memory = false;

    TEST_ASSERT_EQUAL_INT(12, (int)print("uptime %d s\n", 42));
    TEST_ASSERT_EQUAL_STRING("uptime 42 s\n", s_printed);
    TEST_ASSERT_EQUAL_INT(1, s_writes);
    TEST_ASSERT_EQUAL_INT(0, s_allocs);
}

static void test_print_the_longest_text_of_the_small_buffer(void)
{
    /* 255 characters fit a 256-byte buffer; the 256th needs the block */
    static char text[257];

    s_no_memory = false;
    memset(text, 'a', 255);
    text[255] = '\0';
    TEST_ASSERT_EQUAL_INT(255, (int)print("%s", text));
    TEST_ASSERT_EQUAL_INT(0, s_allocs);
    TEST_ASSERT_EQUAL_INT(255, (int)s_printed_len);

    text[255] = 'b';
    text[256] = '\0';
    TEST_ASSERT_EQUAL_INT(256, (int)print("%s", text));
    TEST_ASSERT_EQUAL_INT(1, s_allocs);
    TEST_ASSERT_EQUAL_INT(1, s_releases);
    TEST_ASSERT_EQUAL_STRING(text, s_printed);
}

static void test_print_a_uds_answer_line_is_whole(void)
{
    /* the report of 2026-11-26: `uds` printed a 201-byte answer with one
     * cmdline_printf, and the line ended after its 84th byte, without the
     * transport and the time behind it */
    const char *hex = hex_run(201);
    size_t want = 3 + strlen(hex) + strlen("  [isotp, 57 ms");

    s_no_memory = false;
    TEST_ASSERT_EQUAL_INT((int)want,
                          (int)print("<- %s  [%s, %lu ms", hex, "isotp", 57ul));
    TEST_ASSERT_EQUAL_INT((int)want, (int)s_printed_len);
    TEST_ASSERT_EQUAL_INT(1, s_writes);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(s_printed, " C7 C8  [isotp, 57 ms"),
                                 "the end of the answer line is missing");
    TEST_ASSERT_EQUAL_INT(1, s_releases);
}

static void test_print_without_memory_writes_what_fits(void)
{
    /* no block to be had: the head of the text, as before, never a crash */
    const char *hex = hex_run(201);

    s_no_memory = true;
    TEST_ASSERT_EQUAL_INT(255, (int)print("<- %s", hex));
    TEST_ASSERT_EQUAL_INT(255, (int)s_printed_len);
    TEST_ASSERT_EQUAL_INT(0, s_releases);
    TEST_ASSERT_EQUAL_INT(0, memcmp(s_printed, "<- 00 01 02", 11));
    s_no_memory = false;
}

/* ---- the argument vector ----------------------------------------------------------- */

static void test_argument_vector_holds_every_word_of_a_line(void)
{
    /* the console keeps one word less than its vector's size and drops the
     * rest without a message. The line with the most words that fits the
     * assembler is "a a a .. a": 128 one-letter words in 255 characters. */
    size_t longest = sizeof(s_asm.buf) - 1;
    size_t most_words = (longest + 1) / 2;

    TEST_ASSERT_EQUAL_INT(128, (int)most_words);
    TEST_ASSERT_TRUE_MESSAGE((size_t)CM_ARGS_MAX - 1 >= most_words,
                             "a line that fits would lose its last words");

    /* a UDS request of the command's 64 bytes, each typed on its own */
    TEST_ASSERT_TRUE((size_t)CM_ARGS_MAX - 1 >= 5 + 1 + 64);
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_simple_line);
    RUN_TEST(test_crlf_yields_one_line);
    RUN_TEST(test_split_across_chunks);
    RUN_TEST(test_batch_of_lines_one_chunk);
    RUN_TEST(test_empty_lines_skipped);
    RUN_TEST(test_overflow_discarded_then_recovers);
    RUN_TEST(test_verdict_success_is_silent);
    RUN_TEST(test_verdict_empty_line_is_silent);
    RUN_TEST(test_verdict_unknown_command);
    RUN_TEST(test_verdict_longest_unknown_line_fits);
    RUN_TEST(test_verdict_handler_failure_names_the_code);
    RUN_TEST(test_verdict_console_failure_is_printed);
    RUN_TEST(test_verdict_truncates_to_the_buffer);
    RUN_TEST(test_print_short_text_needs_no_block);
    RUN_TEST(test_print_the_longest_text_of_the_small_buffer);
    RUN_TEST(test_print_a_uds_answer_line_is_whole);
    RUN_TEST(test_print_without_memory_writes_what_fits);
    RUN_TEST(test_argument_vector_holds_every_word_of_a_line);
    UNITY_END();
}
