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
 * @file socket_manager_tx.c
 * @brief One TCP client's send side: never block, never close a client for
 *        a short stall, never cut a chunk.
 *
 * The sockets are non-blocking. What a client's socket does not take waits
 * in a backlog (pure logic: socket_manager_policy.c) whose storage is lent
 * from a small PSRAM pool for as long as the client is behind. A chunk that
 * finds the backlog full, or no buffer left in the pool, is dropped whole
 * and counted. A client that moved no byte for SM_STALL_CLOSE_MS is gone.
 *
 * Why (a field report of 2026-11-17, reproduced on the bench): until then a
 * send was blocking with a 100 ms limit and a send that did not finish
 * closed the client. The slcan bridge writes one small chunk per CAN frame
 * and lwIP holds 57 unacknowledged writes (TCP_SND_QUEUELEN), 52 ms of a
 * 1,100 frames/s bus, so 150 ms without an ACK from a phone (a scan, power
 * save, a weak link) ended the session, about once a minute while driving.
 *
 * Every function here runs under the net lock of socket_manager_net.c.
 */
#include <errno.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "socket_manager.h"
#include "socket_manager_private.h"

static const char *TAG = "socket_manager";

/* What one client that is behind may have waiting: 64 KB, about 2.6 s of a
 * 1,100 frames/s CAN bus as slcan text (32 KB until 2026-11-18: the unit's
 * own search for its home network held a client on its access point for up
 * to 1.5 s on the bench, and 32 KB ran over in 4 of 28 searches). Not a
 * setting: it is external RAM, which is cheap, so it is sized with room
 * instead (Ali, 2026-11-18). A buffer is lent when a client first falls
 * behind and goes back when it has caught up, so the pool is the number of
 * clients that may be behind at the same time. */
#define SM_BACKLOG_BYTES 65536
#define SM_BACKLOG_POOL  4

/* A client that moved no byte for this long is gone (a phone that left the
 * network sends no FIN): its slot and its buffer are wanted back. */
#define SM_STALL_CLOSE_MS 10000

static uint8_t s_pool[SM_BACKLOG_POOL][SM_BACKLOG_BYTES] EXT_RAM_BSS_ATTR;
static bool s_pool_used[SM_BACKLOG_POOL] EXT_RAM_BSS_ATTR;

uint32_t sm_tx_buffer_bytes(void)
{
    return SM_BACKLOG_BYTES;
}

static void pool_lend(smp_backlog_t *b)
{
    for (int i = 0; i < SM_BACKLOG_POOL && b->buf == NULL; i++)
    {
        if (!s_pool_used[i])
        {
            s_pool_used[i] = true;
            b->buf = s_pool[i];
            b->cap = SM_BACKLOG_BYTES;
        }
    }
}

static void pool_return(smp_backlog_t *b)
{
    for (int i = 0; i < SM_BACKLOG_POOL; i++)
    {
        if (b->buf == s_pool[i])
        {
            s_pool_used[i] = false;
        }
    }

    b->buf = NULL;
    b->cap = 0;
}

void sm_tx_reset(smp_backlog_t *b)
{
    pool_return(b);
    smp_backlog_clear(b);
}

/** A send's result as bytes moved: >= 0, or -1 for an error that ends the
 *  connection. "Would block" is 0 bytes, not an error. */
static int send_some(int fd, const uint8_t *data, size_t len)
{
    int n = send(fd, data, len, 0);

    if (n >= 0)
    {
        return n;
    }

    return (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS ||
            errno == ENOMEM || errno == EINTR) ? 0 : -1;
}

bool sm_tx_push(int fd, smp_backlog_t *b, const uint8_t *data, size_t len,
                const char *name, uint32_t *bytes_out, uint32_t *tx_drops)
{
    const uint8_t *head;
    size_t waiting;
    bool moved = false;
    bool refused = false;

    /* what waits goes first, or the stream would leave out of order */
    while (!refused && (waiting = smp_backlog_peek(b, &head)) > 0)
    {
        int n = send_some(fd, head, waiting);

        if (n < 0)
        {
            return false;
        }

        smp_backlog_took(b, (size_t)n);
        *bytes_out += (uint32_t)n;
        moved = moved || n > 0;
        refused = (size_t)n < waiting;
    }

    if (len > 0 && !refused)
    {
        int n = send_some(fd, data, len);

        if (n < 0)
        {
            return false;
        }

        *bytes_out += (uint32_t)n;
        moved = moved || n > 0;
        refused = (size_t)n < len;
        data += n;
        len -= (size_t)n;
    }

    if (len > 0)
    {
        if (b->buf == NULL)
        {
            pool_lend(b);
        }

        if (!smp_backlog_put(b, data, len))
        {
            if (b->dropped == 0)
            {
                ESP_LOGW(TAG, "%s: a client is behind (%lu bytes waiting), "
                         "what does not fit is dropped", name,
                         (unsigned long)b->len);
            }

            b->dropped++;
            (*tx_drops)++;
        }
    }

    if (b->len == 0)
    {
        if (b->dropped > 0)
        {
            ESP_LOGW(TAG, "%s: the client caught up, %lu chunk(s) were "
                     "dropped meanwhile", name, (unsigned long)b->dropped);
            b->dropped = 0;
        }

        pool_return(b);
    }

    smp_backlog_attempt(b, moved, refused, esp_timer_get_time());
    return true;
}

bool sm_tx_gone(const smp_backlog_t *b, const char *name)
{
    if (!smp_backlog_expired(b, esp_timer_get_time(), SM_STALL_CLOSE_MS))
    {
        return false;
    }

    ESP_LOGW(TAG, "%s: client dropped (took no data for %d s, %lu bytes "
             "and %lu chunk(s) lost)", name, SM_STALL_CLOSE_MS / 1000,
             (unsigned long)b->len, (unsigned long)b->dropped);
    return true;
}
