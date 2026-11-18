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
 * in a backlog (the tx_backlog component: the ring, the pool that lends its
 * storage, the push that never blocks) for as long as the client is behind.
 * A chunk that finds the backlog full, or no buffer left in the pool, is
 * dropped whole and counted. A client that moved no byte for
 * SM_STALL_CLOSE_MS is gone.
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
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

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

static uint8_t s_pool_mem[SM_BACKLOG_POOL * SM_BACKLOG_BYTES] EXT_RAM_BSS_ATTR;

/* internal: 24 bytes, and an initialised object cannot sit in PSRAM .bss */
static tx_backlog_pool_t s_pool =
{
    .mem = s_pool_mem,
    .buf_bytes = SM_BACKLOG_BYTES,
    .count = SM_BACKLOG_POOL,
    .pace_us = SM_PACE_GAP_US,
    .pace_after = SM_PACE_AFTER,
};

uint32_t sm_tx_buffer_bytes(void)
{
    return SM_BACKLOG_BYTES;
}

void sm_tx_reset(tx_backlog_t *b)
{
    tx_backlog_return(&s_pool, b);
    tx_backlog_clear(b);
}

bool sm_tx_push(int fd, tx_backlog_t *b, const uint8_t *data, size_t len,
                const char *name, uint32_t *bytes_out, uint32_t *tx_drops)
{
    uint32_t dropped_before = b->dropped;
    tx_backlog_result_t r =
        tx_backlog_sock_push(fd, 0, b, &s_pool, NULL, 0, data, len,
                             esp_timer_get_time(), bytes_out);

    if (r == TX_BACKLOG_ERROR)
    {
        return false;
    }

    /* a chunk cut on the wire (the pool was empty while this client fell
       behind) is counted like a dropped one: slcan and the other line
       protocols resync at the next line end */
    if (r == TX_BACKLOG_DROPPED || r == TX_BACKLOG_CUT)
    {
        if (dropped_before == 0)
        {
            ESP_LOGW(TAG, "%s: a client is behind (%lu bytes waiting), "
                     "what does not fit is dropped", name,
                     (unsigned long)b->len);
        }

        (*tx_drops)++;
    }

    if (b->len == 0 && b->dropped > 0)
    {
        ESP_LOGW(TAG, "%s: the client caught up, %lu chunk(s) were "
                 "dropped meanwhile", name, (unsigned long)b->dropped);
        b->dropped = 0;
    }

    return true;
}

bool sm_tx_gone(const tx_backlog_t *b, const char *name)
{
    if (!tx_backlog_expired(b, esp_timer_get_time(), SM_STALL_CLOSE_MS))
    {
        return false;
    }

    ESP_LOGW(TAG, "%s: client dropped (took no data for %d s, %lu bytes "
             "and %lu chunk(s) lost)", name, SM_STALL_CLOSE_MS / 1000,
             (unsigned long)b->len, (unsigned long)b->dropped);
    return true;
}
