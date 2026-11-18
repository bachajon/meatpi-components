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
 * @file websocket_manager_tx.c
 * @brief A channel's send side: one frame to every client of the channel,
 *        never blocking, never holding one client back for another.
 *
 * Until 2026-11-18 a frame went out through httpd_ws_send_frame_async() in
 * the caller's task, one client after the other, each send blocking for up
 * to the server's send timeout (5 s). The caller is a bridge's pump. On the
 * bench (a CAN bus at 1,100 frames a second bridged to /ws/can): beside one
 * client that stopped reading, a client that kept reading lost 11,367 of
 * about 31,000 frames; beside one that took 2 KB a second it got a third of
 * them; and of 749 lines written the other way 353 reached the bus, because
 * the pump serves both directions.
 *
 * Now the frame (its header built here, never masked) is pushed to each
 * client's socket with MSG_DONTWAIT. What a client does not take waits in
 * its backlog (tx_backlog: a buffer lent from a pool of four for as long as
 * it is behind) and leaves in order; a frame that no longer fits is dropped
 * whole and counted in tx_drops; a client that took no byte for
 * WSM_STALL_CLOSE_MS is closed. A task of its own pushes the backlogs every
 * few ms, so they leave on a quiet channel too.
 *
 * The socket belongs to the web server, which reads it and may close it.
 * Every send here happens under wsm_lock, and the server's session close
 * hook (websocket_manager_ws.c) takes that lock before the socket closes.
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include "http_server_manager.h"

#include "websocket_manager_internal.h"

static const char *TAG = "websocket_manager";

/* What one client that is behind may have waiting: 64 KB, 2.6 s of a 1,100
 * frames/s CAN bus as slcan text, the size the TCP servers use. External
 * RAM is cheap, so it is sized with room and is not a setting. A buffer is
 * lent when a client first falls behind and goes back when it has caught
 * up: the pool is the number of clients that may be behind at once. */
#define WSM_BACKLOG_BYTES 65536
#define WSM_BACKLOG_POOL  4

/* A client that moved no byte for this long is gone (a browser tab that
 * was put to sleep, a phone that left the network): its slot and its buffer
 * are wanted back. A slow client that still takes data stays. */
#define WSM_STALL_CLOSE_MS 10000

#define WSM_FLUSH_TICK_MS 5    /* how often a waiting backlog is pushed */

/* How often the web server is told that a client that takes data is in
 * use. The server has seven places; when an eighth connection arrives it
 * closes the one used longest ago, and it counts a use only when a
 * connection SENDS something. A client that only listens (the CAN monitor,
 * the log, the live values) was therefore always the one to go: on the
 * bench a reader of /ws/can was closed as soon as seven plain requests
 * had come in after its handshake. */
#define WSM_IN_USE_EVERY_MS 1000
#define WSM_IN_USE_WITHIN_MS 2000 /* it took a byte this recently */

/* The pace of a client's stream (tx_backlog.h): frames that come faster
 * than one every WSM_PACE_GAP_US leave together on the flush tick, once
 * WSM_PACE_AFTER in a row came that fast: a busy CAN bus on /ws/can is a
 * couple of hundred TCP segments a second, not one per CAN frame. The gap
 * is a little under the tick so that a tick always finds the gap over. */
#define WSM_PACE_GAP_US 4000
#define WSM_PACE_AFTER  8

/* A frame up to this payload leaves in ONE send: its header and payload are
 * put together first. As two sends it was two small TCP segments and two
 * round trips to the TCP/IP task for every frame (the pump hands over at
 * most 1 KB at a time, so that is every frame of a bridged channel). */
#define WSM_ONE_SEND_MAX 1280
#define WSM_TX_STACK      3072
#define WSM_TX_PRIO       10   /* the bridges' pumps run at 10 too      */

static uint8_t s_pool_mem[WSM_BACKLOG_POOL * WSM_BACKLOG_BYTES] EXT_RAM_BSS_ATTR;

/* internal: 24 bytes, and an initialised object cannot sit in PSRAM .bss */
static tx_backlog_pool_t s_pool =
{
    .mem = s_pool_mem,
    .buf_bytes = WSM_BACKLOG_BYTES,
    .count = WSM_BACKLOG_POOL,
    .pace_us = WSM_PACE_GAP_US,
    .pace_after = WSM_PACE_AFTER,
};

/* header + payload of the frame being sent; under wsm_lock */
static uint8_t s_one[WSM_FRAME_HEAD_MAX + WSM_ONE_SEND_MAX] EXT_RAM_BSS_ATTR;

static TaskHandle_t s_task;
static StaticTask_t s_tcb; /* internal: FreeRTOS object */
static StackType_t s_stack[WSM_TX_STACK] EXT_RAM_BSS_ATTR;

typedef enum
{
    WSM_KEEP = 0,
    WSM_GONE_ERROR,  /* a hard socket error: the client left          */
    WSM_GONE_SILENT, /* took no byte for WSM_STALL_CLOSE_MS           */
    WSM_GONE_CUT,    /* a frame was cut on the wire: no buffer for it */
} wsm_verdict_t;

/* on the server's own task (httpd_queue_work): the only place that call
   is meant for */
static void mark_in_use(void *arg)
{
    httpd_handle_t server = http_server_manager_handle();

    if (server != NULL)
    {
        (void)httpd_sess_update_lru_counter(server, (int)(intptr_t)arg);
    }
}

void wsm_tx_reset(tx_backlog_t *b)
{
    tx_backlog_return(&s_pool, b);
    tx_backlog_clear(b);
}

tx_backlog_result_t wsm_tx_push_raw(wsm_channel_t *ch, int c,
                                    const uint8_t *head, size_t head_len,
                                    const uint8_t *data, size_t len)
{
    /* a control frame (a pong, the close frame before a close) does not
       wait for the pace: the session may end right after it */
    tx_backlog_pace_now(&ch->backlog[c]);
    return tx_backlog_sock_push(ch->fds[c], MSG_DONTWAIT, &ch->backlog[c],
                                &s_pool, head, head_len, data, len,
                                esp_timer_get_time(), NULL);
}

void wsm_tx_kick(void)
{
    if (s_task != NULL)
    {
        xTaskNotifyGive(s_task);
    }
}

/** Under wsm_lock: what waits for client @p c, then the new frame (none when
 *  @p head_len and @p len are 0). The verdict says whether the client stays. */
static wsm_verdict_t push_one(const wsm_channel_cfg_t *cfg, wsm_channel_t *ch,
                              int c, const uint8_t *head, size_t head_len,
                              const uint8_t *data, size_t len, int64_t now)
{
    tx_backlog_t *b = &ch->backlog[c];
    uint32_t dropped_before = b->dropped;
    tx_backlog_result_t r =
        tx_backlog_sock_push(ch->fds[c], MSG_DONTWAIT, b, &s_pool, head,
                             head_len, data, len, now, NULL);

    if (r == TX_BACKLOG_ERROR)
    {
        return WSM_GONE_ERROR;
    }

    if (r == TX_BACKLOG_CUT)
    {
        return WSM_GONE_CUT;
    }

    if (r == TX_BACKLOG_DROPPED)
    {
        if (dropped_before == 0)
        {
            ESP_LOGW(TAG, "%s: a client is behind (%lu bytes waiting), "
                     "what does not fit is dropped", cfg->name,
                     (unsigned long)b->len);
        }

        ch->stats.tx_drops++;
    }

    if (b->len == 0 && b->dropped > 0)
    {
        ESP_LOGW(TAG, "%s: the client caught up, %lu frame(s) were "
                 "dropped meanwhile", cfg->name, (unsigned long)b->dropped);
        b->dropped = 0;
    }

    return tx_backlog_expired(b, now, WSM_STALL_CLOSE_MS) ? WSM_GONE_SILENT
                                                          : WSM_KEEP;
}

/** Under wsm_lock: say why client @p c goes, and forget it. true when the
 *  caller must close its session once the lock is released: a client that
 *  is silent or whose stream is broken will not go by itself. A client whose
 *  send failed is leaving: its session stays the server's, which first
 *  reads what that client still sent (closing it here threw those frames
 *  away: the last lines a client wrote before it left never reached the
 *  bus) and then ends the session, where the close hook finds nothing. */
static bool let_go(const wsm_channel_cfg_t *cfg, wsm_channel_t *ch, int c,
                   wsm_verdict_t why)
{
    const tx_backlog_t *b = &ch->backlog[c];

    if (why == WSM_GONE_SILENT)
    {
        ESP_LOGW(TAG, "%s: client dropped (took no data for %d s, %lu bytes "
                 "and %lu frame(s) lost)", cfg->name,
                 WSM_STALL_CLOSE_MS / 1000, (unsigned long)b->len,
                 (unsigned long)b->dropped);
    }
    else if (why == WSM_GONE_CUT)
    {
        ESP_LOGW(TAG, "%s: client dropped (it fell behind while %d others "
                 "held every buffer)", cfg->name, WSM_BACKLOG_POOL);
    }
    else
    {
        ESP_LOGI(TAG, "%s: client gone (fd %d)", cfg->name, ch->fds[c]);
    }

    wsm_client_drop(ch, c);
    return why != WSM_GONE_ERROR;
}

esp_err_t wsm_ws_send(int idx, const uint8_t *data, size_t len)
{
    const wsm_channel_cfg_t *cfg = wsm_core_config(idx);
    wsm_channel_t *ch = &wsm_ch[idx];
    httpd_handle_t server = http_server_manager_handle();

    if (server == NULL || cfg == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t head[WSM_FRAME_HEAD_MAX];
    size_t head_len = wsm_frame_header(cfg->text_mode, len, head);
    int closing[WEBSOCKET_MANAGER_MAX_CLIENTS];
    int n_closing = 0;
    int in_use[WEBSOCKET_MANAGER_MAX_CLIENTS];
    int n_in_use = 0;
    int delivered = 0;
    bool waiting = false;
    int64_t now = esp_timer_get_time();

    const uint8_t *part1 = head;
    size_t part1_len = head_len;
    const uint8_t *part2 = data;
    size_t part2_len = len;

    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    if (len <= WSM_ONE_SEND_MAX)
    {
        memcpy(s_one, head, head_len);
        memcpy(s_one + head_len, data, len);
        part1 = NULL;
        part1_len = 0;
        part2 = s_one;
        part2_len = head_len + len;
    }

    for (int c = 0; c < WEBSOCKET_MANAGER_MAX_CLIENTS; c++)
    {
        int fd = ch->fds[c];

        if (fd < 0)
        {
            continue;
        }

        if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET)
        {
            if (wsm_client_is_ghost(server, fd, ch->since[c]))
            {
                /* not ours to close: the session is gone, or the number is
                   another connection's by now */
                ESP_LOGI(TAG, "%s: client reaped (fd %d)", cfg->name, fd);
                wsm_client_drop(ch, c);
            }

            continue; /* else its handshake is in flight: not its frame */
        }

        wsm_verdict_t v = push_one(cfg, ch, c, part1, part1_len, part2,
                                   part2_len, now);

        if (v == WSM_KEEP)
        {
            delivered++;
            waiting = waiting || ch->backlog[c].len > 0;

            if (now - ch->used[c] >= WSM_IN_USE_EVERY_MS * 1000LL &&
                !tx_backlog_expired(&ch->backlog[c], now, WSM_IN_USE_WITHIN_MS))
            {
                ch->used[c] = now;
                in_use[n_in_use++] = fd;
            }

            continue;
        }

        if (let_go(cfg, ch, c, v))
        {
            closing[n_closing++] = fd;
        }
    }

    if (delivered > 0)
    {
        ch->stats.frames_out++;
        ch->stats.bytes_out += len;
    }

    xSemaphoreGive(wsm_lock);

    /* outside the lock: the close runs on the server's task, whose close
       hook takes the lock */
    for (int i = 0; i < n_closing; i++)
    {
        (void)httpd_sess_trigger_close(server, closing[i]);
    }

    for (int i = 0; i < n_in_use; i++)
    {
        (void)httpd_queue_work(server, mark_in_use, (void *)(intptr_t)in_use[i]);
    }

    if (waiting && s_task != NULL)
    {
        xTaskNotifyGive(s_task);
    }

    return ESP_OK;
}

/* ---- the flush task ---------------------------------------------------------------- */

/** Push every backlog that holds something, and move on what the clients
 *  wrote and that waits for a subscriber. true while something still waits. */
static bool flush_all(void)
{
    httpd_handle_t server = http_server_manager_handle();
    int closing[WEBSOCKET_MANAGER_MAX_CHANNELS * WEBSOCKET_MANAGER_MAX_CLIENTS];
    int n_closing = 0;
    bool waiting = false;
    int64_t now = esp_timer_get_time();

    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CHANNELS; i++)
    {
        const wsm_channel_cfg_t *cfg = wsm_core_config(i);
        wsm_channel_t *ch = &wsm_ch[i];

        for (int c = 0; cfg != NULL && c < WEBSOCKET_MANAGER_MAX_CLIENTS; c++)
        {
            if (ch->fds[c] < 0 || ch->backlog[c].len == 0)
            {
                continue;
            }

            int fd = ch->fds[c];
            wsm_verdict_t v = push_one(cfg, ch, c, NULL, 0, NULL, 0, now);

            if (v == WSM_KEEP)
            {
                waiting = waiting || ch->backlog[c].len > 0;
                continue;
            }

            if (let_go(cfg, ch, c, v))
            {
                closing[n_closing++] = fd;
            }
        }
    }

    waiting = wsm_rx_feed() || waiting;

    xSemaphoreGive(wsm_lock);

    for (int i = 0; server != NULL && i < n_closing; i++)
    {
        (void)httpd_sess_trigger_close(server, closing[i]);
    }

    return waiting;
}

static void tx_task(void *arg)
{
    bool waiting = false;

    (void)arg;

    for (;;)
    {
        /* asleep until a send leaves something waiting, then one push every
           few ms for as long as something waits: never one push per frame,
           whatever the senders signal meanwhile */
        if (waiting)
        {
            vTaskDelay(pdMS_TO_TICKS(WSM_FLUSH_TICK_MS) > 0
                           ? pdMS_TO_TICKS(WSM_FLUSH_TICK_MS) : 1);
            (void)ulTaskNotifyTake(pdTRUE, 0);
        }
        else
        {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        waiting = flush_all();
    }
}

esp_err_t wsm_tx_start(void)
{
    if (s_task != NULL)
    {
        return ESP_OK;
    }

    s_task = xTaskCreateStatic(tx_task, "ws_tx", WSM_TX_STACK, NULL,
                               WSM_TX_PRIO, s_stack, &s_tcb);
    return (s_task != NULL) ? ESP_OK : ESP_FAIL;
}
