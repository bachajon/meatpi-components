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
 * @file socket_manager_policy.c
 * @brief Pure policy: backoff progression, accept decisions, config parsing
 *        and cross-item validation. No lwIP: host-tests on the linux target.
 */
#include <stdio.h>
#include <string.h>

#include "socket_manager.h"
#include "socket_manager_private.h"

#define SMP_BACKOFF_FIRST_MS 1000
#define SMP_BACKOFF_CAP_MS   8000

uint32_t smp_backoff_next_ms(uint32_t prev_ms)
{
    if (prev_ms == 0)
    {
        return SMP_BACKOFF_FIRST_MS;
    }

    uint32_t next = prev_ms * 2;

    return (next > SMP_BACKOFF_CAP_MS) ? SMP_BACKOFF_CAP_MS : next;
}

smp_accept_verdict_t smp_accept_decision(int connected_count, int max_clients)
{
    return (connected_count >= max_clients) ? SMP_REJECT : SMP_ACCEPT;
}

/* ---- a client's backlog ------------------------------------------------------ */

void smp_backlog_clear(smp_backlog_t *b)
{
    b->off = 0;
    b->len = 0;
    b->stalled = false;
    b->stalled_us = 0;
    b->dropped = 0;
}

/* A ring: the waiting bytes run from off, around the end of the storage.
 * Nothing is ever moved. A first cut closed the gap in front with a memmove
 * of the whole backlog, which a client that stays behind paid on every
 * chunk, under the net lock: up to 32 KB of PSRAM, 1,100 times a second. */
bool smp_backlog_put(smp_backlog_t *b, const uint8_t *data, size_t len)
{
    if (b->buf == NULL || len > (size_t)(b->cap - b->len))
    {
        return false;
    }

    uint32_t tail = (b->off + b->len) % b->cap;
    size_t first = (len < (size_t)(b->cap - tail)) ? len : (size_t)(b->cap - tail);

    memcpy(b->buf + tail, data, first);
    memcpy(b->buf, data + first, len - first);
    b->len += (uint32_t)len;
    return true;
}

size_t smp_backlog_peek(const smp_backlog_t *b, const uint8_t **out)
{
    if (b->len == 0)
    {
        *out = NULL;
        return 0;
    }

    *out = b->buf + b->off;
    return (b->len < b->cap - b->off) ? b->len : b->cap - b->off;
}

void smp_backlog_took(smp_backlog_t *b, size_t n)
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

void smp_backlog_attempt(smp_backlog_t *b, bool moved, bool refused,
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

bool smp_backlog_expired(const smp_backlog_t *b, int64_t now_us,
                         uint32_t limit_ms)
{
    return b->stalled &&
           (now_us - b->stalled_us) >= (int64_t)limit_ms * 1000;
}

static bool name_is_valid(const char *s)
{
    size_t len = strlen(s);

    if (len == 0 || len >= sizeof(((smp_server_cfg_t *)0)->name))
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        char c = s[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';

        if (!ok)
        {
            return false;
        }
    }

    return true;
}

esp_err_t smp_parse_server(const cJSON *item, smp_server_cfg_t *out)
{
    const cJSON *name = cJSON_GetObjectItemCaseSensitive(item, "name");
    const cJSON *proto = cJSON_GetObjectItemCaseSensitive(item, "proto");
    const cJSON *port = cJSON_GetObjectItemCaseSensitive(item, "port");
    const cJSON *maxc = cJSON_GetObjectItemCaseSensitive(item, "max_clients");
    const cJSON *keep = cJSON_GetObjectItemCaseSensitive(item, "keepalive_s");
    const cJSON *ena = cJSON_GetObjectItemCaseSensitive(item, "enabled");

    if (!cJSON_IsString(name) || !cJSON_IsString(proto) ||
        !cJSON_IsNumber(port) || !name_is_valid(name->valuestring))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    strncpy(out->name, name->valuestring, sizeof(out->name) - 1);
    out->is_udp = (strcmp(proto->valuestring, "udp") == 0);
    out->port = (uint16_t)port->valuedouble;
    out->max_clients = cJSON_IsNumber(maxc) ? (uint8_t)maxc->valuedouble : 2;
    out->keepalive_s = cJSON_IsNumber(keep) ? (uint16_t)keep->valuedouble : 30;
    out->enabled = cJSON_IsTrue(ena);
    return ESP_OK;
}

esp_err_t smp_validate_servers(const cJSON *servers, char *err,
                               size_t err_len)
{
    smp_server_cfg_t cfg[SOCKET_MANAGER_MAX_SERVERS];
    int count = 0;
    const cJSON *item = NULL;

    cJSON_ArrayForEach(item, servers)
    {
        if (count >= SOCKET_MANAGER_MAX_SERVERS)
        {
            snprintf(err, err_len, "more than %d servers",
                     SOCKET_MANAGER_MAX_SERVERS);
            return ESP_ERR_INVALID_ARG;
        }

        if (smp_parse_server(item, &cfg[count]) != ESP_OK)
        {
            snprintf(err, err_len, "servers[%d]: bad name/proto/port", count);
            return ESP_ERR_INVALID_ARG;
        }

        if (cfg[count].max_clients < 1 ||
            cfg[count].max_clients > SOCKET_MANAGER_MAX_CLIENTS)
        {
            snprintf(err, err_len, "servers[%d]: max_clients 1..%d", count,
                     SOCKET_MANAGER_MAX_CLIENTS);
            return ESP_ERR_INVALID_ARG;
        }

        /* the web server's port: lwip SO_REUSEADDR lets a second LISTEN
           pcb bind it, and incoming SYNs then ALTERNATE between httpd
           and the socket server: a nondeterministically dead UI/API
           (found live 2026-07-26, system_bench degraded leg). Refuse
           at validation; the device httpd is always :80. */
        if (cfg[count].enabled && !cfg[count].is_udp &&
            cfg[count].port == 80)
        {
            snprintf(err, err_len,
                     "servers[%d]: port 80 is the web server's", count);
            return ESP_ERR_INVALID_ARG;
        }

        for (int i = 0; i < count; i++)
        {
            if (strcmp(cfg[i].name, cfg[count].name) == 0)
            {
                snprintf(err, err_len, "duplicate server name '%s'",
                         cfg[count].name);
                return ESP_ERR_INVALID_ARG;
            }

            /* two ENABLED servers can't share a port (disabled ones may
               keep a port parked) */
            if (cfg[i].enabled && cfg[count].enabled &&
                cfg[i].port == cfg[count].port)
            {
                snprintf(err, err_len, "port %u used twice",
                         (unsigned)cfg[count].port);
                return ESP_ERR_INVALID_ARG;
            }
        }

        count++;
    }

    return ESP_OK;
}
