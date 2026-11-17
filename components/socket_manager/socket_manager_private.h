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
 * @file socket_manager_private.h
 * @brief Internal API between socket_manager translation units, and the pure
 *        policy layer (host-testable: no lwIP dependency).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- pure policy (socket_manager_policy.c: host-tested) ------------------- */

/** Listener retry backoff progression: 1 s → 2 s → 4 s → 8 s (cap).
 *  prev_ms == 0 (fresh failure) yields the first step. */
uint32_t smp_backoff_next_ms(uint32_t prev_ms);

typedef enum
{
    SMP_ACCEPT = 0,        /* take the client                         */
    SMP_REJECT,            /* at max_clients: accept-then-close (§2)  */
} smp_accept_verdict_t;

smp_accept_verdict_t smp_accept_decision(int connected_count, int max_clients);

/**
 * Validate the "servers" array (already schema-validated for types/ranges):
 * cross-item rules, unique names, unique ports among enabled servers,
 * max_clients within the static table. Pure; used by on_validate.
 */
esp_err_t smp_validate_servers(const cJSON *servers, char *err,
                               size_t err_len);

/* ---- parsed per-server config (filled by on_apply) -------------------------- */

typedef struct
{
    bool     enabled;
    bool     is_udp;
    char     name[16];
    uint16_t port;
    uint8_t  max_clients;
    uint16_t keepalive_s;
} smp_server_cfg_t;

/** Parse one settings array item into @p out (code defaults for optional
 *  keys: max_clients=2, keepalive_s=30, enabled=false). Pure. */
esp_err_t smp_parse_server(const cJSON *item, smp_server_cfg_t *out);

/* ---- a client's backlog (socket_manager_policy.c: host-tested) ------------ */

/** What one TCP client has not taken yet, and since when it takes nothing.
 *  The storage is lent (buf NULL = none held). Bytes go in whole or not at
 *  all, so a dropped chunk never cuts a text protocol's line. */
typedef struct
{
    uint8_t *buf;         /* lent storage; NULL while the client keeps up   */
    uint32_t cap;
    uint32_t off;         /* first byte still to send                       */
    uint32_t len;         /* bytes waiting, from off around the end (ring)  */
    bool     stalled;     /* the last send attempts moved no byte           */
    int64_t  stalled_us;  /* when that began                                */
    uint32_t dropped;     /* chunks refused since the backlog was last empty */
} smp_backlog_t;

/** Empty it and stop the stall clock. Leaves buf and cap alone. */
void smp_backlog_clear(smp_backlog_t *b);

/** Queue @p len bytes behind what waits. false (nothing stored) when no
 *  storage is held or they do not all fit. */
bool smp_backlog_put(smp_backlog_t *b, const uint8_t *data, size_t len);

/** The first run of waiting bytes (up to the end of the storage): its
 *  length, *out at its first byte. What wraps is the next peek's, after
 *  smp_backlog_took(). 0 and NULL when nothing waits. */
size_t smp_backlog_peek(const smp_backlog_t *b, const uint8_t **out);

/** The socket took the first @p n waiting bytes. */
void smp_backlog_took(smp_backlog_t *b, size_t n);

/** One send attempt's result. @p moved: at least one byte left. @p refused:
 *  the socket would not take everything. Nothing moved and refused starts
 *  the stall clock; anything else stops it. */
void smp_backlog_attempt(smp_backlog_t *b, bool moved, bool refused,
                         int64_t now_us);

/** The client moved no byte for @p limit_ms: it is gone. */
bool smp_backlog_expired(const smp_backlog_t *b, int64_t now_us,
                         uint32_t limit_ms);

/* ---- one TCP client's send side (socket_manager_tx.c) ----------------------
 * Callers hold the net lock: the pool and the sockets are shared. */

/** What one client that is behind may have waiting, in bytes. */
uint32_t sm_tx_buffer_bytes(void);

/** A client's slot starts or ends: an empty backlog, its buffer returned. */
void sm_tx_reset(smp_backlog_t *b);

/** Send what waits, then @p data (may be NULL / 0: only what waits). Never
 *  blocks: what the socket does not take waits in the backlog, a chunk the
 *  backlog cannot hold is dropped and counted in @p tx_drops. Sent bytes add
 *  to @p bytes_out. false: a hard socket error, the caller closes the
 *  client. @p name is the server's, for the log. */
bool sm_tx_push(int fd, smp_backlog_t *b, const uint8_t *data, size_t len,
                const char *name, uint32_t *bytes_out, uint32_t *tx_drops);

/** The client has taken nothing for the stall limit (logs why). */
bool sm_tx_gone(const smp_backlog_t *b, const char *name);

/* ---- net layer (socket_manager_net.c): indexed like the config array ------ */

esp_err_t sm_net_start(void);
void      sm_net_stop(void);
esp_err_t sm_net_send(int idx, const uint8_t *data, size_t len);
esp_err_t sm_net_subscribe(int idx, void *queue_handle);
esp_err_t sm_net_unsubscribe(int idx, void *queue_handle);
esp_err_t sm_net_stats(int idx, void *stats_out);

/* ---- settings (socket_manager_settings.c) ----------------------------------- */

/** Register the "socket_manager" descriptor with settings_manager. */
esp_err_t sm_settings_register(void);

/** Boot-applied config for slot @p idx; NULL past the configured count. */
const smp_server_cfg_t *sm_core_config(int idx);

int  sm_settings_count(void);         /* configured server slots           */
bool sm_settings_is_configured(void); /* boot apply ran (standard §4.3)    */

#ifdef __cplusplus
}
#endif
