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
 * @file tx_backlog.c
 * @brief The pure part of tx_backlog: the ring, its stall clock and the pool
 *        that lends the rings their storage. No IDF, no lwIP: the host tests
 *        compile this file alone.
 */
#include "tx_backlog.h"

#include <string.h>

/* ---- the ring ------------------------------------------------------------------- */

void tx_backlog_clear(tx_backlog_t *b)
{
    b->off = 0;
    b->len = 0;
    b->stalled = false;
    b->stalled_us = 0;
    b->dropped = 0;
    b->sent_us = 0;
    b->unit_us = 0;
    b->run = 0;
}

/* A ring: the waiting bytes run from off, around the end of the storage.
 * Nothing is ever moved. A first cut closed the gap in front with a memmove
 * of the whole backlog, which a client that stays behind paid on every
 * chunk, under its owner's lock: up to 64 KB of PSRAM, 1,100 times a
 * second. */
bool tx_backlog_put(tx_backlog_t *b, const uint8_t *data, size_t len)
{
    if (b->buf == NULL || len > (size_t)(b->cap - b->len))
    {
        return false;
    }

    if (len == 0)
    {
        return true;
    }

    uint32_t tail = (b->off + b->len) % b->cap;
    size_t first = (len < (size_t)(b->cap - tail)) ? len : (size_t)(b->cap - tail);

    memcpy(b->buf + tail, data, first);
    memcpy(b->buf, data + first, len - first);
    b->len += (uint32_t)len;
    return true;
}

bool tx_backlog_put2(tx_backlog_t *b, const uint8_t *head, size_t head_len,
                     const uint8_t *data, size_t len)
{
    if (b->buf == NULL || head_len + len > (size_t)(b->cap - b->len))
    {
        return false;
    }

    /* both fit, so neither of these can refuse */
    (void)tx_backlog_put(b, head, head_len);
    (void)tx_backlog_put(b, data, len);
    return true;
}

size_t tx_backlog_peek(const tx_backlog_t *b, const uint8_t **out)
{
    if (b->len == 0)
    {
        *out = NULL;
        return 0;
    }

    *out = b->buf + b->off;
    return (b->len < b->cap - b->off) ? b->len : b->cap - b->off;
}

void tx_backlog_took(tx_backlog_t *b, size_t n)
{
    if (n >= b->len)
    {
        b->off = 0;
        b->len = 0;
        return;
    }

    b->off = (b->off + (uint32_t)n) % b->cap;
    b->len -= (uint32_t)n;
}

void tx_backlog_attempt(tx_backlog_t *b, bool moved, bool refused,
                        int64_t now_us)
{
    if (moved || !refused)
    {
        b->stalled = false;
    }
    else if (!b->stalled)
    {
        b->stalled = true;
        b->stalled_us = now_us;
    }
}

bool tx_backlog_expired(const tx_backlog_t *b, int64_t now_us,
                        uint32_t limit_ms)
{
    return b->stalled &&
           (now_us - b->stalled_us) >= (int64_t)limit_ms * 1000;
}

/* ---- the pace ---------------------------------------------------------------------- */

bool tx_backlog_pace_hold(tx_backlog_t *b, int64_t now_us, uint32_t gap_us,
                          uint8_t after)
{
    if (gap_us == 0)
    {
        return false;
    }

    bool pause = now_us - b->unit_us >= (int64_t)gap_us;

    b->unit_us = now_us;

    if (pause)
    {
        b->run = 0; /* nothing came for the gap: this is no stream now */
        return false;
    }

    if (b->run < after)
    {
        b->run++;
        return false;
    }

    /* a stream: this unit goes with the others, once the gap since the
       last send is over */
    return now_us - b->sent_us < (int64_t)gap_us;
}

bool tx_backlog_pace_due(const tx_backlog_t *b, int64_t now_us,
                         uint32_t gap_us)
{
    return gap_us == 0 || now_us - b->sent_us >= (int64_t)gap_us;
}

void tx_backlog_pace_now(tx_backlog_t *b)
{
    b->sent_us = 0;
    b->unit_us = 0;
    b->run = 0;
}

/* ---- the pool --------------------------------------------------------------------- */

void tx_backlog_lend(tx_backlog_pool_t *p, tx_backlog_t *b)
{
    if (p == NULL || p->mem == NULL || b->buf != NULL)
    {
        return;
    }

    for (int i = 0; i < p->count && i < TX_BACKLOG_POOL_MAX; i++)
    {
        if (!p->used[i])
        {
            p->used[i] = true;
            b->buf = p->mem + (size_t)i * p->buf_bytes;
            b->cap = p->buf_bytes;
            b->off = 0;
            b->len = 0;
            return;
        }
    }
}

void tx_backlog_return(tx_backlog_pool_t *p, tx_backlog_t *b)
{
    if (p != NULL && p->mem != NULL && b->buf != NULL)
    {
        for (int i = 0; i < p->count && i < TX_BACKLOG_POOL_MAX; i++)
        {
            if (b->buf == p->mem + (size_t)i * p->buf_bytes)
            {
                p->used[i] = false;
            }
        }
    }

    b->buf = NULL;
    b->cap = 0;
    b->off = 0;
    b->len = 0;
}
