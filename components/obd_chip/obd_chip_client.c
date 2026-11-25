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
 *
 *        Three more things about the way the chip changes hands
 *        (2026-11-25, the same issue, measured on the bench):
 *
 *        - Who wrote last. The bridge names the connection its bytes come
 *          from, and says when that connection has nobody left: the hold
 *          of a monitor ends then (obd_client_gone), and the cap is left
 *          for a transport that cannot tell (the USB serial jack) and for
 *          an app that hangs while it is connected.
 *        - The poller's turn. An app's first command landed in the request
 *          the poller had on the wire: the chip answered STOPPED and ran
 *          nothing, the app's own reset included (25 of 25), and the app
 *          went on with the header and the receive filter of the poller's
 *          last row. The poller now takes a turn for each pass
 *          (obd_client_turn_begin, refused while an app drives the chip)
 *          and the app's bytes wait until a turn in progress has ended.
 *        - What an app is not shown. The chip's answers to the firmware's
 *          own requests went to every bridge, so an app that stayed
 *          connected and silent received the poller's lines. They are
 *          kept from the apps now (obd_client_mute), but for an app whose
 *          own command is under way: it is to see what became of it.
 */
#include "obd_chip.h"

#include <stdint.h>
#include <string.h>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

#include "obd_chip_private.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED; /* internal: spinlock */

/* PSRAM (standard §2): read and written by the bridges' tasks and the RX
   task, never with the cache off */
static struct
{
    int64_t touch_us;     /* the app's last byte; 0 = never               */
    int64_t monitor_us;   /* its monitor opened then; 0 = none open       */
    char    line[24];     /* the line it is typing                        */
    uint8_t n;
    bool    too_long;     /* that line outgrew `line`: no monitor         */
    bool    last_monitor; /* its last line was one (a bare CR repeats it) */
    const void *writer;   /* who wrote last (the bridge's far end); NULL =
                             not told                                     */
    bool    cmd;          /* a command of the app is under way: a CR went
                             OUT to the chip (obd_client_sent) and no
                             prompt came since                            */
    bool    turn;         /* a background turn is in progress             */
    obd_chip_client_stats_t stats;
} s EXT_RAM_BSS_ATTR;

static int64_t stamp(int64_t now_us)
{
    return (now_us != 0) ? now_us : 1; /* 0 means never */
}

void obd_client_touch(int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);
    s.touch_us = stamp(now_us);
    portEXIT_CRITICAL(&s_lock);
}

void obd_client_write_from(const void *who, const uint8_t *data, size_t len,
                           int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);
    s.touch_us = stamp(now_us);
    s.writer = who;

    for (size_t i = 0; data != NULL && i < len; i++)
    {
        char c = (char)data[i];

        if (c == '\n' || c == '\0')
        {
            continue; /* the chip runs nothing on these */
        }

        s.monitor_us = 0; /* any other byte stops a monitor */

        if (c != '\r')
        {
            if (s.n < sizeof(s.line) - 1)
            {
                s.line[s.n++] = c;
            }
            else
            {
                s.too_long = true;
            }

            continue;
        }

        bool monitor = s.last_monitor; /* a bare CR repeats the last line */

        if (s.n > 0 || s.too_long)
        {
            s.line[s.n] = '\0';
            monitor = !s.too_long && obd_parse_is_monitor_cmd(s.line);
            s.last_monitor = monitor;
        }

        s.n = 0;
        s.too_long = false;

        if (monitor)
        {
            s.monitor_us = stamp(now_us);
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
    s.monitor_us = 0;
    s.cmd = false;
    portEXIT_CRITICAL(&s_lock);
}

bool obd_client_gone(const void *who)
{
    bool ended = false;

    portENTER_CRITICAL(&s_lock);

    /* only the connection that wrote last can be the one whose monitor
       runs: any other app's byte would have stopped it */
    if (who != NULL && s.writer == who)
    {
        ended = (s.monitor_us != 0);
        s.monitor_us = 0;
        s.last_monitor = false; /* nobody is left to repeat it */
        s.n = 0;
        s.too_long = false;
        s.writer = NULL;

        if (ended)
        {
            s.stats.gone++;
        }
    }

    portEXIT_CRITICAL(&s_lock);
    return ended;
}

/** The idle time from the two stamps (see obd_client_idle_ms). */
static uint32_t idle_of(int64_t touch, int64_t monitor, int64_t now_us)
{
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

uint32_t obd_client_idle_ms(int64_t now_us)
{
    portENTER_CRITICAL(&s_lock);
    int64_t touch = s.touch_us;
    int64_t monitor = s.monitor_us;
    portEXIT_CRITICAL(&s_lock);

    return idle_of(touch, monitor, now_us);
}

/* ---- the poller's turn ---------------------------------------------------------- */

bool obd_client_turn_begin(int64_t now_us, uint32_t yield_ms)
{
    bool granted;

    /* one lock for the app's stamp and the turn: an app's write is either
       seen here (no turn), or it sees the turn and waits for its end */
    portENTER_CRITICAL(&s_lock);
    granted = (idle_of(s.touch_us, s.monitor_us, now_us) >= yield_ms);
    s.turn = granted;
    portEXIT_CRITICAL(&s_lock);
    return granted;
}

void obd_client_turn_end(void)
{
    portENTER_CRITICAL(&s_lock);
    s.turn = false;
    portEXIT_CRITICAL(&s_lock);
}

bool obd_client_turn(void)
{
    portENTER_CRITICAL(&s_lock);
    bool turn = s.turn;
    portEXIT_CRITICAL(&s_lock);
    return turn;
}

void obd_client_turn_waited(uint32_t waited_ms, bool forced)
{
    portENTER_CRITICAL(&s_lock);
    s.stats.turn_waits++;

    if (waited_ms > s.stats.turn_wait_max_ms)
    {
        s.stats.turn_wait_max_ms = waited_ms;
    }

    if (forced)
    {
        s.stats.turn_forced++;
    }

    portEXIT_CRITICAL(&s_lock);
}

/* ---- what an app is not shown ---------------------------------------------------- */

void obd_client_sent(const uint8_t *data, size_t len)
{
    /* a CR submits whatever is on the chip's line. Marked when the bytes go
       out and not when they arrive: while they wait for the poller's turn
       to end, what the chip still says is the poller's (bench 2026-11-25:
       the prompt of the poller's last answer came out in front of the
       app's own) */
    if (data == NULL || memchr(data, '\r', len) == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s.cmd = true;
    portEXIT_CRITICAL(&s_lock);
}

bool obd_client_mute(bool claimed)
{
    if (!claimed)
    {
        return false; /* no request of the firmware's: the chip talks to the app */
    }

    portENTER_CRITICAL(&s_lock);
    bool cmd = s.cmd;
    portEXIT_CRITICAL(&s_lock);

    /* an app's own command is under way beside the firmware's request
       (it would not wait any longer, or a tool began while it ran): the
       app is to see what became of it, STOPPED included */
    return !cmd;
}

void obd_client_get_stats(obd_chip_client_stats_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s.stats;
    portEXIT_CRITICAL(&s_lock);
}

#ifdef OBD_CHIP_HOST_TEST
void obd_client_test_reset(void)
{
    portENTER_CRITICAL(&s_lock);
    s.touch_us = 0;
    s.monitor_us = 0;
    s.n = 0;
    s.too_long = false;
    s.last_monitor = false;
    s.writer = NULL;
    s.cmd = false;
    s.turn = false;
    memset(&s.stats, 0, sizeof(s.stats));
    portEXIT_CRITICAL(&s_lock);
}
#endif
