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
 * @file websocket_manager_ws.c
 * @brief The httpd WebSocket layer: route registration on the ONE server
 *        (http_server_manager), handshake gating at max_clients, the client
 *        table, frame RX into the subscriber queue. The send side is
 *        websocket_manager_tx.c.
 *
 * The control frames come to the handler here (handle_ws_control_frames =
 * true since 2026-11-18): the answer to a PING and the echo of a CLOSE go
 * out through the client's own send path, behind what already waits for
 * it. The server's own PONG was written from the server's task into a
 * socket another task was sending frames on.
 *
 * A client leaves the table in the server's session close hook, BEFORE its
 * socket is closed (http_server_manager_register_close_hook), so a socket
 * number that a later connection takes is never mistaken for it.
 */
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

#include "http_server_manager.h"

#include "websocket_manager_internal.h"

static const char *TAG = "websocket_manager";

#define WSM_FRAME_MAX 4096 /* per-frame RX cap (request-scoped PSRAM) */

/* A message above WSM_FRAME_MAX is read off the socket before the close
 * frame that refuses it goes out, up to this much: a socket closed with
 * unread data sends a reset, and a reset can overtake the close frame. */
#define WSM_DRAIN_MAX 65536

/* A client is registered by the pre-handshake hook, BEFORE its 101 goes
 * out, so for a moment it reads HTTPD_WS_CLIENT_HTTP: a fan-out send in
 * that moment used to reap it (2026-10-21, the release test: "client
 * reaped" and "client connected" in the same millisecond, the client then
 * got nothing and a third one took its slot). Within this grace an HTTP
 * state means "handshake in flight"; past it, a ghost (a silently closed
 * fd that an HTTP session reused). */
#define WSM_HANDSHAKE_GRACE_US (3LL * 1000 * 1000)

wsm_channel_t wsm_ch[WEBSOCKET_MANAGER_MAX_CHANNELS] EXT_RAM_BSS_ATTR;

SemaphoreHandle_t wsm_lock;
static StaticSemaphore_t s_lock_buf; /* internal: FreeRTOS object */

static bool s_close_hooked; /* the session close hook, registered once */

/* route table handed to httpd: must outlive the server */
static httpd_uri_t s_uris[WEBSOCKET_MANAGER_MAX_CHANNELS] EXT_RAM_BSS_ATTR;

/* ---- client table helpers (under wsm_lock) ------------------------------------- */

static int client_count(const wsm_channel_t *ch)
{
    int n = 0;

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CLIENTS; i++)
    {
        n += (ch->fds[i] >= 0) ? 1 : 0;
    }

    return n;
}

void wsm_client_drop(wsm_channel_t *ch, int c)
{
    if (ch->fds[c] >= 0)
    {
        ch->fds[c] = -1;
        ch->stats.clients--;
    }

    wsm_tx_reset(&ch->backlog[c]);
}

static bool client_remove(wsm_channel_t *ch, int fd)
{
    bool found = false;

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CLIENTS; i++)
    {
        if (ch->fds[i] == fd)
        {
            wsm_client_drop(ch, i);
            found = true;
        }
    }

    return found;
}

static bool client_add(wsm_channel_t *ch, int fd)
{
    (void)client_remove(ch, fd); /* fd reuse after a silent close */

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CLIENTS; i++)
    {
        if (ch->fds[i] < 0)
        {
            wsm_tx_reset(&ch->backlog[i]);
            ch->fds[i] = fd;
            ch->since[i] = esp_timer_get_time();
            ch->used[i] = ch->since[i]; /* the handshake was a use */
            ch->stats.clients++;
            return true;
        }
    }

    return false;
}

bool wsm_client_is_ghost(httpd_handle_t server, int fd, int64_t since)
{
    httpd_ws_client_info_t info = httpd_ws_get_fd_info(server, fd);

    if (info == HTTPD_WS_CLIENT_WEBSOCKET)
    {
        return false;
    }

    return info != HTTPD_WS_CLIENT_HTTP ||
           esp_timer_get_time() - since > WSM_HANDSHAKE_GRACE_US;
}

/* ---- the session close hook ---------------------------------------------------------
 * On the server's task, for every session that ends, before its socket is
 * closed. Until 2026-11-18 a client that left a quiet channel stayed in the
 * table until the next send failed on it, and a socket number is given out
 * again at once: the next connection that got the number (a client of
 * another channel, or a plain page request) was then sent this channel's
 * frames. */
static void on_session_close(int fd)
{
    if (wsm_lock == NULL)
    {
        return;
    }

    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CHANNELS; i++)
    {
        if (client_remove(&wsm_ch[i], fd))
        {
            const wsm_channel_cfg_t *cfg = wsm_core_config(i);

            ESP_LOGI(TAG, "%s: client disconnected (fd %d)",
                     (cfg != NULL) ? cfg->name : "?", fd);
        }
    }

    xSemaphoreGive(wsm_lock);
}

/* ---- handshake gate (IDF v6: the uri handler is NOT called at handshake,
 * the pre-handshake callback is the sanctioned hook, and it can refuse
 * BEFORE the 101 goes out) --------------------------------------------------------- */

static esp_err_t ws_pre_handshake(httpd_req_t *req)
{
    int idx = (int)(intptr_t)req->user_ctx;
    const wsm_channel_cfg_t *cfg = wsm_core_config(idx);
    wsm_channel_t *ch = &wsm_ch[idx];

    if (cfg == NULL)
    {
        return ESP_FAIL;
    }

    int fd = httpd_req_to_sockfd(req);
    httpd_handle_t server = http_server_manager_handle();

    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    /* reap ghosts first: belt and braces behind the close hook, which takes
     * a client out the moment its session ends */
    for (int i = 0; server != NULL && i < WEBSOCKET_MANAGER_MAX_CLIENTS; i++)
    {
        if (ch->fds[i] >= 0 &&
            wsm_client_is_ghost(server, ch->fds[i], ch->since[i]))
        {
            ESP_LOGD(TAG, "%s: stale client reaped (fd %d)", cfg->name,
                     ch->fds[i]);
            wsm_client_drop(ch, i);
        }
    }

    bool accept = client_count(ch) < cfg->max_clients &&
                  client_add(ch, fd);

    if (!accept)
    {
        ch->stats.refused++;
    }

    xSemaphoreGive(wsm_lock);

    if (!accept)
    {
        ESP_LOGD(TAG, "%s: upgrade refused (max %u)", cfg->name,
                 (unsigned)cfg->max_clients);
        return ESP_FAIL; /* handshake rejected, socket closed */
    }

    /* a frame leaves as two writes (header, payload) when the socket takes
     * them one by one; with Nagle the payload stalls on the peer's delayed
     * ACK of the header (~40 ms/frame measured). Interactive channels need
     * TCP_NODELAY. */
    int on = 1;

    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));

    ESP_LOGI(TAG, "%s: client connected (fd %d)", cfg->name, fd);
    return ESP_OK;
}

/* ---- the frame handler --------------------------------------------------------------- */

/** One control frame to the client on @p fd, behind what waits for it. A
 *  control frame that finds no room is not worth a client (the peer asks
 *  again, or is closing anyway); one that was cut on the wire is: the
 *  stream is no longer WebSocket, and that client goes. */
static void send_control(int idx, int fd, uint8_t opcode,
                         const uint8_t *payload, size_t len)
{
    wsm_channel_t *ch = &wsm_ch[idx];
    uint8_t head[2] = { (uint8_t)(0x80 | opcode), (uint8_t)len }; /* <= 125 */
    bool waiting = false;
    bool broken = false;

    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    for (int c = 0; c < WEBSOCKET_MANAGER_MAX_CLIENTS; c++)
    {
        if (ch->fds[c] != fd)
        {
            continue;
        }

        tx_backlog_result_t r =
            wsm_tx_push_raw(ch, c, head, sizeof(head), payload, len);

        if (r == TX_BACKLOG_CUT || r == TX_BACKLOG_ERROR)
        {
            wsm_client_drop(ch, c);
            broken = true;
        }
        else
        {
            waiting = ch->backlog[c].len > 0;
        }
    }

    xSemaphoreGive(wsm_lock);

    if (broken)
    {
        httpd_handle_t server = http_server_manager_handle();

        if (server != NULL)
        {
            (void)httpd_sess_trigger_close(server, fd);
        }
    }
    else if (waiting)
    {
        wsm_tx_kick();
    }
}

/** PING, PONG, CLOSE (RFC 6455: at most 125 bytes of payload). */
static esp_err_t ws_control(httpd_req_t *req, int idx, httpd_ws_frame_t *frame)
{
    const wsm_channel_cfg_t *cfg = wsm_core_config(idx);
    uint8_t payload[125];
    int fd = httpd_req_to_sockfd(req);

    if (frame->len > sizeof(payload))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    if (frame->len > 0)
    {
        frame->payload = payload;

        esp_err_t err = httpd_ws_recv_frame(req, frame, frame->len);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    if (frame->type == HTTPD_WS_TYPE_PING)
    {
        send_control(idx, fd, 0x0A, payload, frame->len);
        return ESP_OK;
    }

    if (frame->type != HTTPD_WS_TYPE_CLOSE)
    {
        return ESP_OK; /* a PONG: read off the socket, nothing to do */
    }

    /* echo the status code, forget the client, and let the server end the
       session: the close hook then finds nothing left to do */
    send_control(idx, fd, 0x08, payload, (frame->len >= 2) ? 2 : 0);

    xSemaphoreTake(wsm_lock, portMAX_DELAY);
    bool was_ours = client_remove(&wsm_ch[idx], fd);
    xSemaphoreGive(wsm_lock);

    if (was_ours)
    {
        ESP_LOGI(TAG, "%s: client closed the channel (fd %d)",
                 (cfg != NULL) ? cfg->name : "?", fd);
    }

    return ESP_FAIL;
}

/** Read @p len payload bytes off the socket and throw them away. */
static bool drain_payload(httpd_req_t *req, size_t len)
{
    char scrap[256];

    while (len > 0)
    {
        size_t want = (len < sizeof(scrap)) ? len : sizeof(scrap);
        int n = httpd_socket_recv(req->handle, httpd_req_to_sockfd(req),
                                  scrap, want, 0);

        if (n <= 0)
        {
            return false;
        }

        len -= (size_t)n;
    }

    return true;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    int idx = (int)(intptr_t)req->user_ctx;
    const wsm_channel_cfg_t *cfg = wsm_core_config(idx);
    wsm_channel_t *ch = &wsm_ch[idx];

    if (cfg == NULL)
    {
        return ESP_FAIL;
    }

    /* the length probe, then the payload (request-scoped PSRAM) */
    httpd_ws_frame_t frame = { 0 };
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);

    if (err != ESP_OK)
    {
        return err;
    }

    if (frame.type == HTTPD_WS_TYPE_PING || frame.type == HTTPD_WS_TYPE_PONG ||
        frame.type == HTTPD_WS_TYPE_CLOSE)
    {
        return ws_control(req, idx, &frame);
    }

    if (frame.len == 0)
    {
        return ESP_OK;
    }

    if (frame.len > WSM_FRAME_MAX)
    {
        /* say why, with the close code for it (1009, message too big): a
           bare end of the connection told the client nothing */
        static const uint8_t TOO_BIG[2] = { 0x03, 0xF1 };

        ESP_LOGW(TAG, "%s: a message of %u bytes is more than the %d this "
                 "channel takes: client closed", cfg->name,
                 (unsigned)frame.len, WSM_FRAME_MAX);

        if (frame.len <= WSM_DRAIN_MAX && drain_payload(req, frame.len))
        {
            send_control(idx, httpd_req_to_sockfd(req), 0x08, TOO_BIG,
                         sizeof(TOO_BIG));
        }

        xSemaphoreTake(wsm_lock, portMAX_DELAY);
        ch->stats.rx_drops++;
        xSemaphoreGive(wsm_lock);
        return ESP_ERR_INVALID_SIZE; /* the server ends the session */
    }

    uint8_t *payload = heap_caps_malloc(frame.len,
                                        MALLOC_CAP_SPIRAM |
                                            MALLOC_CAP_8BIT);

    if (payload == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    frame.payload = payload;
    err = httpd_ws_recv_frame(req, &frame, frame.len);

    if (err == ESP_OK)
    {
        wsm_rx_deliver(ch, cfg->name, payload, frame.len);
    }

    free(payload);
    return err;
}

/* ---- internal API ------------------------------------------------------------------ */

void wsm_ws_reset(void)
{
    if (wsm_lock == NULL)
    {
        wsm_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    for (int i = 0; i < WEBSOCKET_MANAGER_MAX_CHANNELS; i++)
    {
        for (int c = 0; c < WEBSOCKET_MANAGER_MAX_CLIENTS; c++)
        {
            wsm_ch[i].fds[c] = -1;
            wsm_tx_reset(&wsm_ch[i].backlog[c]);
        }

        wsm_ch[i].stats.clients = 0;
        wsm_ch[i].sub = NULL;
        wsm_rx_reset(&wsm_ch[i]);
    }

    xSemaphoreGive(wsm_lock);
}

esp_err_t wsm_ws_register_routes(void)
{
    if (!s_close_hooked)
    {
        esp_err_t herr =
            http_server_manager_register_close_hook(on_session_close);

        if (herr != ESP_OK)
        {
            return herr; /* without it a reused socket number gets frames */
        }

        s_close_hooked = true;
    }

    for (int i = 0; wsm_core_config(i) != NULL; i++)
    {
        const wsm_channel_cfg_t *cfg = wsm_core_config(i);

        if (!cfg->enabled)
        {
            continue;
        }

        s_uris[i] = (httpd_uri_t)
        {
            .uri = cfg->path,
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = (void *)(intptr_t)i,
            .is_websocket = true,
            .handle_ws_control_frames = true,
            .ws_pre_handshake_cb = ws_pre_handshake,
        };

        esp_err_t err = http_server_manager_register_uri(&s_uris[i]);

        if (err != ESP_OK)
        {
            return err;
        }

        ESP_LOGI(TAG, "%s: channel at %s (%s, max %u clients)", cfg->name,
                 cfg->path, cfg->text_mode ? "text" : "binary",
                 (unsigned)cfg->max_clients);
    }

    return ESP_OK;
}

esp_err_t wsm_ws_subscribe(int idx, void *queue)
{
    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    esp_err_t err = ESP_OK;

    if (wsm_ch[idx].sub != NULL)
    {
        err = ESP_ERR_INVALID_STATE; /* one subscriber = one bridge */
    }
    else
    {
        wsm_ch[idx].sub = (QueueHandle_t)queue;
    }

    xSemaphoreGive(wsm_lock);
    return err;
}

esp_err_t wsm_ws_unsubscribe(int idx, void *queue)
{
    xSemaphoreTake(wsm_lock, portMAX_DELAY);

    esp_err_t err = ESP_ERR_NOT_FOUND;

    if (wsm_ch[idx].sub == (QueueHandle_t)queue)
    {
        wsm_ch[idx].sub = NULL;
        wsm_rx_reset(&wsm_ch[idx]); /* what waited was for that queue */
        err = ESP_OK;
    }

    xSemaphoreGive(wsm_lock);
    return err;
}

esp_err_t wsm_ws_stats(int idx, void *stats_out)
{
    xSemaphoreTake(wsm_lock, portMAX_DELAY);
    *(websocket_stats_t *)stats_out = wsm_ch[idx].stats;
    xSemaphoreGive(wsm_lock);
    return ESP_OK;
}
