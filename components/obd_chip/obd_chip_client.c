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

/**
 * @file obd_chip_client.c
 * @brief A bridged app's hold on the chip (2026-11-21, GitHub issue
 *        #1078). PURE apart from a spinlock: the host tests drive it
 *        with their own clock.
 *
 *        AutoPID yields the chip while an app (TCP 35000, the web
 *        terminal, Bluetooth, USB) drives it, and takes it back
 *        AP_CLIENT_YIELD_MS (10 s) after the app's last byte. A monitor
 *        (ATMA and the other words of obd_parse_is_monitor_cmd) writes
 *        nothing for as long as it runs, so an app sniffing a bus was
 *        cut off ten seconds in: the poller's next command stopped the
 *        monitor (STOPPED) and the app read the poller's answers from
 *        then on. Now a monitor the app opened counts as activity until
 *        it ends: the app's next byte stops it (any byte but LF, as on
 *        the chip), the chip's prompt says it ended, and
 *        OBD_CLIENT_MONITOR_HOLD_MS caps it, so an app that went away in
 *        the middle of one holds the chip that long at most (nothing
 *        here knows when an app disconnects). A bare CR repeats the last
 *        command on the chip, so it opens a monitor again behind one.
 */
#include "obd_chip.h"

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "obd_chip_private.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED; /* internal: spinlock */

static int64_t s_touch_us;     /* the app's last byte; 0 = never          */
static int64_t s_monitor_us;   /* its monitor opened then; 0 = none open  */
static char    s_line[24];     /* the line it is typing                   */
static uint8_t s_n;
static bool    s_long;         /* that line outgrew s_line: no monitor    */
static bool    s_last_monitor; /* its last line was one (a bare CR)       */

static int64_t stamp(int64_t now_us)
{
    return (now_us != 0) ? now_us : 1; /* 0 means never */
}

void obd_client_touch(int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);
    s_touch_us = stamp(now_us);
    portEXIT_CRITICAL(&s_lock);
}

void obd_client_write(const uint8_t *data, size_t len, int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);
    s_touch_us = stamp(now_us);

    for (size_t i = 0; data != NULL && i < len; i++)
    {
        char c = (char)data[i];

        if (c == '\n' || c == '\0')
        {
            continue; /* the chip runs nothing on these */
        }

        s_monitor_us = 0; /* any other byte stops a monitor */

        if (c != '\r')
        {
            if (s_n < sizeof(s_line) - 1)
            {
                s_line[s_n++] = c;
            }
            else
            {
                s_long = true;
            }

            continue;
        }

        bool monitor = s_last_monitor; /* a bare CR repeats the last line */

        if (s_n > 0 || s_long)
        {
            s_line[s_n] = '\0';
            monitor = !s_long && obd_parse_is_monitor_cmd(s_line);
            s_last_monitor = monitor;
        }

        s_n = 0;
        s_long = false;

        if (monitor)
        {
            s_monitor_us = stamp(now_us);
        }
    }

    portEXIT_CRITICAL(&s_lock);
}

void obd_client_rx(const uint8_t *data, size_t len)
{
    /* the chip's prompt: whatever ran has ended, a monitor too */
    if (data == NULL || memchr(data, '>', len) == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_monitor_us = 0;
    portEXIT_CRITICAL(&s_lock);
}

uint32_t obd_client_idle_ms(int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);
    int64_t touch = s_touch_us;
    int64_t monitor = s_monitor_us;
    portEXIT_CRITICAL(&s_lock);

    if (monitor != 0 &&
        now_us - monitor < (int64_t)OBD_CLIENT_MONITOR_HOLD_MS * 1000)
    {
        return 0; /* the app's monitor runs: it is driving the chip */
    }

    if (touch == 0)
    {
        return UINT32_MAX; /* never */
    }

    int64_t idle = (now_us - touch) / 1000;

    return (idle < 0) ? 0 : (idle > UINT32_MAX) ? UINT32_MAX
                                                : (uint32_t)idle;
}

#ifdef OBD_CHIP_HOST_TEST
void obd_client_test_reset(void)
{
    portENTER_CRITICAL(&s_lock);
    s_touch_us = 0;
    s_monitor_us = 0;
    s_n = 0;
    s_long = false;
    s_last_monitor = false;
    portEXIT_CRITICAL(&s_lock);
}
#endif
