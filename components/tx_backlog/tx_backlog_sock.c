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
 * @file tx_backlog_sock.c
 * @brief The socket side of tx_backlog: one push that never blocks. What
 *        waits goes first, then the new unit; what the socket does not take
 *        waits in the backlog, a unit that no longer fits is dropped.
 *
 * Device only (lwIP). The owner holds its lock around every call for one
 * socket, so one sender writes to a socket at a time.
 */
#include "tx_backlog.h"

#include <errno.h>

#include "lwip/sockets.h"

/** A send's result as bytes moved: >= 0, or -1 for an error that ends the
 *  connection. "Would block" is 0 bytes, not an error. */
static int send_some(int fd, int flags, const uint8_t *data, size_t len)
{
    int n = send(fd, data, len, flags);

    if (n >= 0)
    {
        return n;
    }

    return (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS ||
            errno == ENOMEM || errno == EINTR) ? 0 : -1;
}

tx_backlog_result_t tx_backlog_sock_push(int fd, int send_flags,
                                         tx_backlog_t *b,
                                         tx_backlog_pool_t *pool,
                                         const uint8_t *head, size_t head_len,
                                         const uint8_t *data, size_t len,
                                         int64_t now_us, uint32_t *bytes_out)
{
    const uint8_t *front;
    size_t waiting;
    uint32_t sent = 0;
    bool moved = false;
    bool refused = false;
    bool started = false; /* a part of the new unit is on the wire */
    bool held = false;    /* the new unit waits for the pace, not for the client */
    uint32_t gap_us = (pool != NULL) ? pool->pace_us : 0;
    uint8_t pace_after = (pool != NULL) ? pool->pace_after : 0;
    tx_backlog_result_t res = TX_BACKLOG_SENT;

    /* A client whose last attempt moved nothing is not asked again for every
       new unit: the unit queues behind what waits, and the owner's own push
       (a tick, the socket's writable event: a call with no new unit) asks.
       Each refused send is a round trip to the TCP/IP task. On the bench one
       silent WebSocket client beside a bus of 1,100 frames a second cost
       2,000 of them a second, and the web server's task, below both in
       priority, read another client's frames in bursts up to 1.5 s apart.
       Only while something waits: the owner's push visits a client that has
       a backlog, so one without (the pool was empty) must be asked here. */
    if (b->stalled && b->len > 0 && head_len + len > 0)
    {
        refused = true;
    }
    else if (head_len + len > 0)
    {
        /* a stream's units leave together (the pace, tx_backlog.h): this
           one waits for the owner's next push, if there is a buffer for
           it to wait in */
        if (tx_backlog_pace_hold(b, now_us, gap_us, pace_after))
        {
            if (b->buf == NULL)
            {
                tx_backlog_lend(pool, b);
            }

            held = b->buf != NULL;
            refused = held;
        }
    }
    else if (!b->stalled && !tx_backlog_pace_due(b, now_us, gap_us))
    {
        return TX_BACKLOG_SENT; /* what waits is a stream's: not its turn yet */
    }

    /* A unit that follows waiting bytes joins them when there is room, so
       that both leave in one send below: as two sends they were two TCP
       segments each time (a paced stream: what had gathered, then the unit
       that released it). One that does not fit goes behind them the old
       way. */
    if (!refused && b->len > 0 && head_len + len > 0 &&
        tx_backlog_put2(b, head, head_len, data, len))
    {
        head_len = 0;
        len = 0;
    }

    /* what waits goes first, or the stream would leave out of order */
    while (!refused && (waiting = tx_backlog_peek(b, &front)) > 0)
    {
        int n = send_some(fd, send_flags, front, waiting);

        if (n < 0)
        {
            res = TX_BACKLOG_ERROR;
            break;
        }

        tx_backlog_took(b, (size_t)n);
        sent += (uint32_t)n;
        moved = moved || n > 0;
        refused = (size_t)n < waiting;
    }

    if (res != TX_BACKLOG_ERROR && !refused && head_len > 0)
    {
        int n = send_some(fd, send_flags, head, head_len);

        if (n < 0)
        {
            res = TX_BACKLOG_ERROR;
        }
        else
        {
            sent += (uint32_t)n;
            moved = moved || n > 0;
            started = n > 0;
            refused = (size_t)n < head_len;
            head += n;
            head_len -= (size_t)n;
        }
    }

    if (res != TX_BACKLOG_ERROR && !refused && len > 0)
    {
        int n = send_some(fd, send_flags, data, len);

        if (n < 0)
        {
            res = TX_BACKLOG_ERROR;
        }
        else
        {
            sent += (uint32_t)n;
            moved = moved || n > 0;
            started = started || n > 0;
            refused = (size_t)n < len;
            data += n;
            len -= (size_t)n;
        }
    }

    if (bytes_out != NULL)
    {
        *bytes_out += sent;
    }

    if (sent > 0)
    {
        b->sent_us = now_us;
    }

    if (res == TX_BACKLOG_ERROR)
    {
        return res;
    }

    if (head_len + len > 0)
    {
        if (b->buf == NULL)
        {
            tx_backlog_lend(pool, b);
        }

        if (!tx_backlog_put2(b, head, head_len, data, len))
        {
            b->dropped++;
            res = started ? TX_BACKLOG_CUT : TX_BACKLOG_DROPPED;
        }
    }

    if (b->len == 0)
    {
        tx_backlog_return(pool, b);
    }

    if (!held)
    {
        tx_backlog_attempt(b, moved, refused, now_us);
    }

    return res;
}
