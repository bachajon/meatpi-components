/* Host tests for the `since` parse of the export routes
   (dl_export_parse_since in data_logger_cursor.c, 2026-11-21): the
   cursor's colon percent-encoded, the plain epoch, the refusals, and the
   resume an encoded cursor used to lose. Kept apart from test_main.c,
   which sits near the 700-line rule. */
#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "data_logger_private.h"

/* `since` as the query string carries it (2026-11-21): the cursor's colon
 * may arrive percent-encoded ("%3A", "%3a": Python's urlencode,
 * encodeURIComponent) and must read as the cursor; anything that is not
 * an epoch or a cursor is refused, not guessed at */
void test_export_since_parse(void)
{
    int64_t e = -1;
    long off = -1;
    bool exact = true;

    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813:30721", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT64(1795220813LL, e);
    TEST_ASSERT_EQUAL_INT(30721, off);
    TEST_ASSERT_TRUE(exact);

    e = -1;
    off = -1;
    exact = false;
    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813%3A30721", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT64(1795220813LL, e);
    TEST_ASSERT_EQUAL_INT(30721, off);
    TEST_ASSERT_TRUE(exact);

    exact = false;
    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813%3a30721", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT(30721, off);
    TEST_ASSERT_TRUE(exact);

    /* an encoded digit is still a digit ("%31" = '1') */
    TEST_ASSERT_TRUE(dl_export_parse_since("%31795220813", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT64(1795220813LL, e);

    /* plain: seconds, and milliseconds read as seconds */
    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813", &e, &off, &exact));
    TEST_ASSERT_EQUAL_INT64(1795220813LL, e);
    TEST_ASSERT_EQUAL_INT(0, off);
    TEST_ASSERT_FALSE(exact);
    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813123", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT64(1795220813LL, e);
    TEST_ASSERT_FALSE(exact);

    /* a cursor's epoch is a file name: never divided */
    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813123:5", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT64(1795220813123LL, e);

    /* empty and 0: the oldest file */
    TEST_ASSERT_TRUE(dl_export_parse_since("", &e, &off, &exact));
    TEST_ASSERT_EQUAL_INT64(0, e);
    TEST_ASSERT_FALSE(exact);
    TEST_ASSERT_TRUE(dl_export_parse_since("0", &e, &off, &exact));
    TEST_ASSERT_EQUAL_INT64(0, e);

    /* refused, outputs cleared */
    static const char *const BAD[] =
    {
        "abc", "12:", ":5", "12:5:7", "-5", "+5", " 5", "5 ", "12x",
        "1795220813%3", "1795220813%3G30721", "%", "12%2C5", "12:-5",
        "1795220813:2147483648", "1234567890123456789",
        "123%00", "123%005", "%00123",
    };

    for (size_t i = 0; i < sizeof(BAD) / sizeof(BAD[0]); i++)
    {
        e = -1;
        off = -1;
        exact = true;
        TEST_ASSERT_FALSE_MESSAGE(dl_export_parse_since(BAD[i], &e, &off,
                                                        &exact), BAD[i]);
        TEST_ASSERT_EQUAL_INT64(0, e);
        TEST_ASSERT_EQUAL_INT(0, off);
        TEST_ASSERT_FALSE(exact);
    }

    /* the largest offset a 32-bit long holds is still a cursor */
    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813:2147483647", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT(2147483647L, off);
}

/* the loop itself (2026-11-21): a client that sent the _cursor it was
 * given, encoded, got the start of that file again (plain epoch, offset
 * 0) and the same _cursor back. Through the parse it resumes where the
 * cursor points, as the unencoded spelling does. */
void test_export_since_encoded_cursor_resumes(void)
{
    const int64_t epochs[] = { 1795100000, 1795220813, 1795300000 };
    int64_t e = 0;
    long off = 0;
    bool exact = false;

    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813%3A30721", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT(1, dl_export_start(epochs, 3, e, exact, &off));
    TEST_ASSERT_EQUAL_INT(30721, off);

    TEST_ASSERT_TRUE(dl_export_parse_since("1795220813:30721", &e, &off,
                                           &exact));
    TEST_ASSERT_EQUAL_INT(1, dl_export_start(epochs, 3, e, exact, &off));
    TEST_ASSERT_EQUAL_INT(30721, off);
}
