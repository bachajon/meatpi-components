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
 * @file websocket_manager_rx.c
 * @brief A channel's receive side: what its clients write, on to the
 *        subscriber (a bridge), and a place for it to wait while the
 *        subscriber's queue is full.
 *
 * The server's task hands a message over and never waits for room. A
 * message that does not fit the subscriber's queue waits here, in order,
 * and the flush task (websocket_manager_tx.c) moves it on as the queue
 * empties.
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "websocket_manager_internal.h"

static const char *TAG = "websocket_manager";

/* What the clients of a channel wrote and the subscriber (a bridge) has no
 * room for yet waits here, in order, and the flush task moves it on as the
 * bridge takes it. A client can write faster than the far end of the bridge
 * carries (a burst of slcan lines against a CAN bus): until 2026-11-18 what
 * found the bridge's queue (64 chunks) full was dropped chunk by chunk, so
 * a burst of 6000 lines put 2,167 frames on the bus. A buffer is lent when
 * a channel first has to wait and goes back when it has caught up: the
 * pool is the number of channels that may be behind at once. One buffer
 * of 128 KB (5,900 slcan lines of 22 bytes), not a setting. It is not
 * larger because the unit's free external RAM has a floor to keep: two
 * buffers of 256 KB took its lowest point to 723 KB where the stack audit
 * asks for 1 MB. A message that does not fit any more is dropped whole
 * and counted (rx_drops). */
#define WSM_RX_WAIT_BYTES 131072
#define WSM_RX_WAIT_POOL  1

static uint8_t s_rx_pool_mem[WSM_RX_WAIT_POOL * WSM_RX_WAIT_BYTES] EXT_RAM_BSS_ATTR;

/* internal: 24 bytes, and an initialised object cannot sit in PSRAM .bss */
static tx_backlog_pool_t s_rx_pool =
{
    .mem = s_rx_pool_mem,
    .buf_bytes = WSM_RX_WAIT_BYTES,
    .count = WSM_RX_WAIT_POOL,
};

void wsm_rx_reset(wsm_channel_t *ch)
{
    tx_backlog_return(&s_rx_pool, &ch->rx_wait);
    tx_backlog_clear(&ch->rx_wait);
}

/** Under wsm_lock: @p len bytes into the subscriber's queue as chunks, as
 *  many as it has room for. Returns the bytes that went in. */
static size_t rx_queue(wsm_channel_t *ch, const uint8_t *data, size_t len)
{
    websocket_chunk_t chunk;
    size_t off = 0;

    while (off < len)
    {
        size_t n = len - off;

        if (n > WEBSOCKET_MANAGER_CHUNK_SIZE)
        {
            n = WEBSOCKET_MANAGER_CHUNK_SIZE;
        }

        chunk.len = (uint16_t)n;
        memcpy(chunk.data, data + off, n);

        if (xQueueSend(ch->sub, &chunk, 0) != pdTRUE)
        {
            break; /* never block: the rest waits */
        }

        off += n;
    }

    return off;
}

/** A message to the subscriber: into its queue while that has room, behind
 *  what waits otherwise. The server's task never waits for room. */
void wsm_rx_deliver(wsm_channel_t *ch, const char *name,
                    const uint8_t *payload, size_t len)
{
    bool waiting = false;

    xSemaphoreTake(wsm_lock, portMAX_DELAY);
    ch->stats.frames_in++;
    ch->stats.bytes_in += len;

    if (ch->sub != NULL)
    {
        /* in order: once something waits, everything goes behind it */
        size_t in = (ch->rx_wait.len == 0) ? rx_queue(ch, payload, len) : 0;

        if (in < len)
        {
            if (ch->rx_wait.buf == NULL)
            {
                tx_backlog_lend(&s_rx_pool, &ch->rx_wait);
            }

            if (!tx_backlog_put(&ch->rx_wait, payload + in, len - in))
            {
                /* no room, or every buffer lent: the message, or what was
                   left of it, is lost */
                if (ch->rx_wait.dropped++ == 0)
                {
                    ESP_LOGW(TAG, "%s: the clients write faster than the connection "
                             "behind takes it (%lu bytes waiting): what does not fit "
                             "is dropped", name, (unsigned long)ch->rx_wait.len);
                }

                ch->stats.rx_drops++;
            }

            waiting = ch->rx_wait.len > 0;
        }
    }

    xSemaphoreGive(wsm_lock);

    if (waiting)
    {
        wsm_tx_kick(); /* the flush task moves it on */
    }
}

bool wsm_rx_feed(void)
{
    bool waiting = false;

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CHANNELS; i++)
    {
        wsm_channel_t *ch = &wsm_ch[i];
        const uint8_t *front;
        size_t n;

        if (ch->rx_wait.len == 0)
        {
            continue;
        }

        if (ch->sub == NULL)
        {
            wsm_rx_reset(ch); /* the bridge is gone: so is what waited for it */
            continue;
        }

        while ((n = tx_backlog_peek(&ch->rx_wait, &front)) > 0)
        {
            size_t in = rx_queue(ch, front, n);

            tx_backlog_took(&ch->rx_wait, in);

            if (in < n)
            {
                break; /* the queue is full again */
            }
        }

        if (ch->rx_wait.len == 0)
        {
            const wsm_channel_cfg_t *cfg = wsm_core_config(i);

            if (ch->rx_wait.dropped > 0)
            {
                ESP_LOGW(TAG, "%s: the connection behind caught up, %lu message(s) "
                         "were dropped meanwhile", (cfg != NULL) ? cfg->name : "?",
                         (unsigned long)ch->rx_wait.dropped);
            }

            wsm_rx_reset(ch);
        }
        else
        {
            waiting = true;
        }
    }

    return waiting;
}
