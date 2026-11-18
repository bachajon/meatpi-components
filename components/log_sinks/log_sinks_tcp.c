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
 * @file log_sinks_tcp.c
 * @brief The TCP log tail server (`nc <unit> 5515`): its listener, its
 *        clients and what each of them has not taken yet. Runs in the
 *        network flusher task of log_sinks.c. Out of log_sinks.c since
 *        2026-11-18 (700-line rule).
 *
 * A client's socket is non-blocking. What it does not take waits in a
 * backlog of its own (the tx_backlog component, as the TCP servers and the
 * WebSocket channels do since 2026-11-17), a batch of lines that no longer
 * fits is dropped whole and counted, and only a client that took nothing
 * for LS_TCP_STALL_CLOSE_MS is closed.
 *
 * Until 2026-11-18 a send was blocking with a 200 ms limit and a send that
 * did not finish closed the client, with a D line. lwIP holds 20 KB
 * unacknowledged, so a client whose ACKs came a second late while the unit
 * logged busily (a phone, a laptop on a weak link) lost its tail, and the
 * flusher stood still for those 200 ms for the other client and for the
 * UDP and WebSocket sinks behind it.
 *
 * Recursion rule (log_sinks.c): nothing here logs per line or per batch.
 * The W lines are one per episode of one client (it fell behind, it caught
 * up, it is gone) and come from the flusher task, never from the log task.
 */
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "tx_backlog.h"

#include "log_sinks_private.h"

static const char *TAG = "log_sinks";

/* What one client that is behind may have waiting: 64 KB, more than a
 * second of the busiest log (1000 lines a second are 55 KB). External RAM,
 * sized with room, no setting. One buffer per client slot, so a batch is
 * never cut: a client always finds a buffer. */
#define LS_TCP_BACKLOG_BYTES 65536

/* A client that moved no byte for this long is gone (a phone that left the
 * network sends no FIN): its slot is wanted back. */
#define LS_TCP_STALL_CLOSE_MS 10000

/* The flusher runs a lap for every log line. Two things in a lap are a
 * round trip to the TCP/IP task each and are asked on a clock instead: the
 * listener (a new client waits this long at most), and a client whose last
 * attempt moved nothing. Asked on every line, a stalled client cost the
 * log pipeline its lines: the flusher has the log task's priority, and at
 * a line a millisecond the pipeline (64 lines deep) lost 1,175 of 18,000
 * while one client's ACKs came a second late (bench 2026-11-18). */
#define LS_TCP_ACCEPT_MS 100
#define LS_TCP_ASK_MS    5

/* What leaves the ring in one go: whole lines, up to this many bytes. It is
 * the unit a client's backlog takes whole or not at all, so a line is never
 * cut. */
#define LS_TCP_BATCH 2048

typedef struct
{
    int          fd;        /* -1 = free                                   */
    tx_backlog_t bl;        /* what it has not taken yet                   */
    uint32_t     behind;    /* lines dropped for it since it fell behind   */
} ls_tcp_client_t;

static ls_tcp_client_t s_cl[LS_TCP_MAX_CLIENTS];
static uint8_t s_pool_mem[LS_TCP_MAX_CLIENTS * LS_TCP_BACKLOG_BYTES] EXT_RAM_BSS_ATTR;
static uint8_t s_batch[LS_TCP_BATCH] EXT_RAM_BSS_ATTR;

/* internal: 24 bytes, and an initialised object cannot sit in PSRAM .bss */
static tx_backlog_pool_t s_pool =
{
    .mem = s_pool_mem,
    .buf_bytes = LS_TCP_BACKLOG_BYTES,
    .count = LS_TCP_MAX_CLIENTS,
};

static int s_listen_fd = -1;
static volatile int s_clients;
static TickType_t s_accept_at;          /* the listener's next look          */
static TickType_t s_ask_at;             /* a stalled client's next attempt   */

uint32_t ls_tcp_clients(void)
{
    return (uint32_t)s_clients;
}

bool ls_tcp_waiting(void)
{
    for (int i = 0; i < LS_TCP_MAX_CLIENTS; i++)
    {
        if (s_cl[i].fd >= 0 && s_cl[i].bl.len > 0)
        {
            return true;
        }
    }

    return false;
}

void ls_tcp_reset(void)
{
    for (int i = 0; i < LS_TCP_MAX_CLIENTS; i++)
    {
        s_cl[i].fd = -1;
        s_cl[i].behind = 0;
        tx_backlog_return(&s_pool, &s_cl[i].bl);
        tx_backlog_clear(&s_cl[i].bl);
    }

    s_clients = 0;
}

static void client_close(int slot)
{
    ls_tcp_client_t *c = &s_cl[slot];

    if (c->fd < 0)
    {
        return;
    }

    close(c->fd);
    c->fd = -1;
    c->behind = 0;
    tx_backlog_return(&s_pool, &c->bl);
    tx_backlog_clear(&c->bl);
    s_clients--;
    ESP_LOGD(TAG, "tcp tail client gone (%d left)", s_clients);
}

static void accept_poll(void)
{
    if (s_listen_fd < 0)
    {
        return;
    }

    for (;;)
    {
        int fd = accept(s_listen_fd, NULL, NULL);

        if (fd < 0)
        {
            return; /* EWOULDBLOCK: nobody waiting */
        }

        int slot = -1;

        for (int i = 0; i < LS_TCP_MAX_CLIENTS; i++)
        {
            if (s_cl[i].fd < 0)
            {
                slot = i;
                break;
            }
        }

        if (slot < 0)
        {
            close(fd); /* table full: refuse quietly */
            continue;
        }

        int one = 1;

        /* never wait for a client: what it does not take waits in its
           backlog (the sink's 200 ms send limit ended here, 2026-11-18) */
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        s_cl[slot].fd = fd;
        s_clients++;
        ESP_LOGD(TAG, "tcp tail client attached (%d)", s_clients);
    }
}

/** What waits for the client, then @p n new bytes (@p lines whole lines; 0
 *  bytes = only what waits). False when the new batch was dropped for this
 *  client: it is behind by more than its backlog holds. */
static bool client_push(int slot, const uint8_t *data, uint32_t n, uint32_t lines)
{
    ls_tcp_client_t *c = &s_cl[slot];
    tx_backlog_result_t r =
        tx_backlog_sock_push(c->fd, 0, &c->bl, &s_pool, NULL, 0, data, n,
                             esp_timer_get_time(), NULL);

    if (r == TX_BACKLOG_ERROR)
    {
        client_close(slot);    /* reset or closed by the client */
        return true;
    }

    if (r == TX_BACKLOG_DROPPED || r == TX_BACKLOG_CUT)
    {
        if (c->behind == 0)
        {
            ESP_LOGW(TAG, "a tcp tail client is behind (%lu bytes waiting), the "
                     "lines that do not fit are dropped for it",
                     (unsigned long)c->bl.len);
        }

        c->behind += lines;
        return false;
    }

    if (c->bl.len == 0 && c->behind > 0)
    {
        ESP_LOGW(TAG, "the tcp tail client caught up, %lu line(s) were dropped "
                 "meanwhile", (unsigned long)c->behind);
        c->behind = 0;
        c->bl.dropped = 0;
    }

    return true;
}

void ls_tcp_poll(void)
{
    TickType_t now = xTaskGetTickCount();
    bool ask = (int32_t)(now - s_ask_at) >= 0;

    if ((int32_t)(now - s_accept_at) >= 0)
    {
        s_accept_at = now + pdMS_TO_TICKS(LS_TCP_ACCEPT_MS);
        accept_poll();
    }

    if (ask)
    {
        s_ask_at = now + pdMS_TO_TICKS(LS_TCP_ASK_MS);
    }

    /* What waits goes first, with or without a new line. A client that
       takes what it is given is pushed on every lap; a stalled one is
       asked here, on the clock, and nowhere else (tx_backlog does not ask
       it again for every new batch). A client that takes nothing is found
       here too. */
    for (int i = 0; i < LS_TCP_MAX_CLIENTS; i++)
    {
        ls_tcp_client_t *c = &s_cl[i];

        if (c->fd < 0)
        {
            continue;
        }

        if (c->bl.len > 0 && (ask || !c->bl.stalled))
        {
            (void)client_push(i, NULL, 0, 0);
        }

        if (c->fd >= 0 &&
            tx_backlog_expired(&c->bl, esp_timer_get_time(), LS_TCP_STALL_CLOSE_MS))
        {
            ESP_LOGW(TAG, "tcp tail client dropped (took no data for %d s, %lu "
                     "bytes and %lu line(s) lost)", LS_TCP_STALL_CLOSE_MS / 1000,
                     (unsigned long)c->bl.len, (unsigned long)c->behind);
            client_close(i);
        }
    }

    while (s_clients > 0)
    {
        uint32_t lines = 0;
        uint32_t n = ls_tcp_ring_pop(s_batch, sizeof(s_batch), &lines);

        if (n == 0)
        {
            return;
        }

        bool all = true;

        for (int i = 0; i < LS_TCP_MAX_CLIENTS; i++)
        {
            if (s_cl[i].fd >= 0 && !client_push(i, s_batch, n, lines))
            {
                all = false;
            }
        }

        /* out = every client took the batch (on the wire or waiting for
           it); dropped = at least one client did not get it */
        ls_tcp_ring_count(all ? lines : 0, all ? 0 : lines);
    }
}

void ls_tcp_close_all(void)
{
    if (s_listen_fd >= 0)
    {
        close(s_listen_fd);
        s_listen_fd = -1;
    }

    for (int i = 0; i < LS_TCP_MAX_CLIENTS; i++)
    {
        client_close(i);
    }
}

esp_err_t ls_tcp_open(uint16_t port)
{
    struct sockaddr_in addr =
    {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(port),
    };
    int one = 1;

    s_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (s_listen_fd < 0)
    {
        return ESP_FAIL;
    }

    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    if (bind(s_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(s_listen_fd, LS_TCP_MAX_CLIENTS) != 0)
    {
        close(s_listen_fd);
        s_listen_fd = -1;
        return ESP_FAIL;
    }

    /* non-blocking accepts: polled from the flusher lap */
    fcntl(s_listen_fd, F_SETFL,
          fcntl(s_listen_fd, F_GETFL, 0) | O_NONBLOCK);
    return ESP_OK;
}
