/*
 * This file is part of the MeatPi components project.
 *
 * Copyright (C) 2022-2026 MeatPi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/* An app's hold on the chip (obd_chip_client.c, 2026-11-21, GitHub issue
   #1078): the activity clock AutoPID yields to, and a monitor the app
   opened, which writes nothing while it runs and was cut ten seconds in.
   The tests drive the clock themselves (microseconds). */
#include <string.h>

#include "unity.h"

#include "obd_chip.h"
#include "obd_chip_private.h"

#define S(x) ((int64_t)(x) * 1000000) /* seconds -> microseconds */

static void app(const char *text, int64_t now)
{
    obd_client_write((const uint8_t *)text, strlen(text), now);
}

static void chip(const char *text)
{
    obd_client_rx((const uint8_t *)text, strlen(text));
}

/* no app ever wrote: never; a command: the time since it, as before */
void test_client_clock_as_before(void)
{
    obd_client_test_reset();
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, obd_client_idle_ms(S(5)));

    app("ATI\r", S(1));
    chip("ELM327 v2.3\r\r>");
    TEST_ASSERT_EQUAL_UINT32(10000, obd_client_idle_ms(S(11)));

    obd_client_touch(S(20)); /* the old entry point still counts */
    TEST_ASSERT_EQUAL_UINT32(3000, obd_client_idle_ms(S(23)));
}

/* ATMA holds the chip for the app while the frames come, up to the cap */
void test_client_monitor_holds_until_the_cap(void)
{
    obd_client_test_reset();
    app("ATMA\r", S(100));
    chip("7E8 03 41 0D 00\r10 26 C0 CB 00 01 76\r"); /* frames: no prompt */
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(110)));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(200)));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(219)));

    /* past OBD_CLIENT_MONITOR_HOLD_MS (120 s): idle since the ATMA, so the
       poller takes the chip back as it did after 10 s before */
    TEST_ASSERT_EQUAL_UINT32(121000, obd_client_idle_ms(S(221)));
}

/* the app stops its monitor with a byte (a space): the clock runs from it */
void test_client_monitor_ends_at_the_apps_next_byte(void)
{
    obd_client_test_reset();
    app("ATMA\r", S(0) + 1);
    app(" ", S(30));
    TEST_ASSERT_EQUAL_UINT32(5000, obd_client_idle_ms(S(35)));
}

/* the chip's prompt says the monitor ended (BUFFER FULL, a stop) */
void test_client_monitor_ends_at_the_prompt(void)
{
    obd_client_test_reset();
    app("ATMA\r", S(50));
    chip("7E8 03 41 0D 00\rBUFFER FULL\r\r>");
    TEST_ASSERT_EQUAL_UINT32(15000, obd_client_idle_ms(S(65)));
}

/* how apps type it: byte by byte, CR LF, spaces and lower case, the other
   monitor words; and what opens none */
void test_client_monitor_words(void)
{
    static const char *const OPEN[] =
    {
        "ATMA\r", "ATMA\r\n", "atma\r", "AT MA\r", "ATMT7E8\r", "ATMR 10\r",
        "STMA\r",
    };
    static const char *const NONE[] =
    {
        "ATI\r", "0100\r", "ATMA", "ATM1\r", "ATMAXXXXXXXXXXXXXXXXXXXXXXXX\r",
    };

    for (size_t i = 0; i < sizeof(OPEN) / sizeof(OPEN[0]); i++)
    {
        obd_client_test_reset();
        app(OPEN[i], S(1));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, obd_client_idle_ms(S(60)),
                                         OPEN[i]);
    }

    for (size_t i = 0; i < sizeof(NONE) / sizeof(NONE[0]); i++)
    {
        obd_client_test_reset();
        app(NONE[i], S(1));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(59000, obd_client_idle_ms(S(60)),
                                         NONE[i]);
    }

    /* a terminal sends a byte at a time */
    obd_client_test_reset();
    app("A", S(1));
    app("T", S(1));
    app("M", S(1));
    app("A", S(1));
    app("\r", S(2));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(60)));
}

/* a bare CR stops the monitor and the chip runs the last command again:
   a monitor again behind ATMA, none behind ATI */
void test_client_bare_cr_repeats_the_last_line(void)
{
    obd_client_test_reset();
    app("ATMA\r", S(1));
    app("\r", S(10));
    TEST_ASSERT_EQUAL_UINT32(0, obd_client_idle_ms(S(60)));

    obd_client_test_reset();
    app("ATI\r", S(1));
    chip("ELM327 v2.3\r\r>");
    app("\r", S(10));
    TEST_ASSERT_EQUAL_UINT32(50000, obd_client_idle_ms(S(60)));
}
