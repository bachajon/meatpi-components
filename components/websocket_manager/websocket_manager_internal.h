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
 * @file websocket_manager_internal.h
 * @brief What the device files of websocket_manager share: the channel
 *        table with its lock (websocket_manager_ws.c) and the send side
 *        (websocket_manager_tx.c). Not for the host tests: they build
 *        websocket_manager_private.h alone, without FreeRTOS and the server.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "tx_backlog.h"

#include "websocket_manager.h"
#include "websocket_manager_private.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int fds[WEBSOCKET_MANAGER_MAX_CLIENTS];       /* -1 = free            */
    int64_t since[WEBSOCKET_MANAGER_MAX_CLIENTS]; /* registered at (us)   */
    int64_t used[WEBSOCKET_MANAGER_MAX_CLIENTS];  /* server told: in use  */
    tx_backlog_t backlog[WEBSOCKET_MANAGER_MAX_CLIENTS]; /* not taken yet */
    tx_backlog_t rx_wait; /* written by clients, not yet in sub's queue  */
    QueueHandle_t sub;
    websocket_stats_t stats;
} wsm_channel_t;

/** The channel table, indexed like the config. Every read and write of a
 *  client slot, and every send on a client's socket, happens under
 *  wsm_lock: the session close hook takes the same lock BEFORE the server
 *  closes the socket, so nothing is ever sent on a socket that is closed or
 *  whose number another connection took. */
extern wsm_channel_t wsm_ch[WEBSOCKET_MANAGER_MAX_CHANNELS];
extern SemaphoreHandle_t wsm_lock;

/** Under wsm_lock: forget the client in slot @p c of @p ch (its backlog's
 *  buffer goes back to the pool). Does not close anything. */
void wsm_client_drop(wsm_channel_t *ch, int c);

/** A registered fd that is no longer a client: the server knows no session
 *  for it, or it stayed plain HTTP past the handshake grace. A handshake
 *  still in flight is NOT a ghost. */
bool wsm_client_is_ghost(httpd_handle_t server, int fd, int64_t since);

/* ---- the send side (websocket_manager_tx.c) -------------------------------------- */

/** Under wsm_lock: a client slot starts or ends: an empty backlog, its
 *  buffer returned. */
void wsm_tx_reset(tx_backlog_t *b);

/** Under wsm_lock: one ready-made frame (@p head is its header) to client
 *  @p c alone, behind what waits for it. The caller acts on the result:
 *  TX_BACKLOG_CUT and TX_BACKLOG_ERROR mean the client must go. */
tx_backlog_result_t wsm_tx_push_raw(wsm_channel_t *ch, int c,
                                    const uint8_t *head, size_t head_len,
                                    const uint8_t *data, size_t len);

/** Wake the flush task: something waits in a backlog. */
void wsm_tx_kick(void);

/* ---- the receive side (websocket_manager_ws.c) ----------------------------------- */

/** A message a client wrote, to the channel's subscriber: into its queue
 *  while that has room, behind what waits otherwise (dropped whole and
 *  counted in rx_drops when that does not fit either). Takes wsm_lock. */
void wsm_rx_deliver(wsm_channel_t *ch, const char *name,
                    const uint8_t *payload, size_t len);

/** Under wsm_lock: forget what waits for a channel's subscriber. */
void wsm_rx_reset(wsm_channel_t *ch);

/** Under wsm_lock: move what the clients of every channel wrote and that
 *  waits (rx_wait) into the subscribers' queues, as far as they have room.
 *  true while something still waits. The flush task calls it on its tick. */
bool wsm_rx_feed(void);

#ifdef __cplusplus
}
#endif
