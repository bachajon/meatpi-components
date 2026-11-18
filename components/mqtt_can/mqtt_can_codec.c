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
 * @file mqtt_can_codec.c
 * @brief PURE codec (no I/O, no FreeRTOS): legacy-JSON batch encoder,
 *        fragmented-stream object reassembler, tx parser, and (2026-11-18)
 *        the translator's two steps whole, mc_batch_feed and mc_tx_feed,
 *        with the endpoint's chunking (mc_tx_chunk_len): the host suite
 *        runs the logic the bridge runs. Host-tested.
 *
 * Encode is hand-rolled snprintf (deterministic, heap-free: this runs
 * per batch at CAN rates); the low-rate tx parse uses cJSON.
 */
#include "mqtt_can_codec.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"

#include "can_frame_wire.h"

/* ---- rx: batcher ----------------------------------------------------------- */

void mc_batch_init(mc_batch_t *b, uint16_t batch_frames)
{
    memset(b, 0, sizeof(*b));

    if (batch_frames < 1)
    {
        batch_frames = 1;
    }

    if (batch_frames > MC_BATCH_MAX)
    {
        batch_frames = MC_BATCH_MAX;
    }

    b->limit = batch_frames;
}

size_t mc_batch_add_chunk(mc_batch_t *b, const uint8_t *chunk, size_t len,
                          int *added)
{
    size_t off = 0;
    size_t taken = len;
    int n = 0;

    while (off + CAN_WIRE_HDR <= len)
    {
        can_core_frame_t f;
        size_t used = can_wire_decode_next(chunk + off, len - off, &f);

        if (used == 0)
        {
            break;              /* malformed tail: stop, keep what fits */
        }

        if (b->n >= MC_BATCH_MAX)
        {
            /* full with a frame in hand: it stays in the chunk for the
               caller's next call (until 2026-11-18 it was dropped here) */
            taken = off;
            break;
        }

        b->frames[b->n++] = f;
        n++;
        off += used;
    }

    if (added != NULL)
    {
        *added = n;
    }

    return taken;
}

bool mc_batch_ready(const mc_batch_t *b)
{
    return b->n >= b->limit;
}

/* a batch that does not fit its JSON buffer: its frames are counted and the
   batch is emptied (it would never fit) */
static size_t batch_lost(mc_batch_t *b)
{
    b->dropped += b->n;
    b->n = 0;
    return 0;
}

size_t mc_batch_json(mc_batch_t *b, int64_t ts_ms, char *out, size_t cap)
{
    size_t w = 0;

    if (b->n == 0)
    {
        return 0;
    }

    int n = snprintf(out + w, cap - w,
                     "{\"bus\":\"0\",\"type\":\"rx\",\"ts\":%lld,"
                     "\"frame\":[", (long long)ts_ms);

    if (n < 0 || (size_t)n >= cap - w)
    {
        return batch_lost(b);
    }

    w += (size_t)n;

    for (uint16_t i = 0; i < b->n; i++)
    {
        const can_core_frame_t *f = &b->frames[i];
        uint8_t dlc = (f->dlc > 8) ? 8 : f->dlc;

        n = snprintf(out + w, cap - w,
                     "%s{\"id\":%lu,\"dlc\":%u,\"rtr\":%s,\"extd\":%s,"
                     "\"data\":[",
                     (i > 0) ? "," : "", (unsigned long)f->id, dlc,
                     f->rtr ? "true" : "false",
                     f->ext ? "true" : "false");

        if (n < 0 || (size_t)n >= cap - w)
        {
            return batch_lost(b);
        }

        w += (size_t)n;

        for (uint8_t d = 0; d < dlc; d++)
        {
            n = snprintf(out + w, cap - w, "%s%u",
                         (d > 0) ? "," : "", f->data[d]);

            if (n < 0 || (size_t)n >= cap - w)
            {
                return batch_lost(b);
            }

            w += (size_t)n;
        }

        n = snprintf(out + w, cap - w, "]}");

        if (n < 0 || (size_t)n >= cap - w)
        {
            return batch_lost(b);
        }

        w += (size_t)n;
    }

    n = snprintf(out + w, cap - w, "]}");

    if (n < 0 || (size_t)n >= cap - w)
    {
        return batch_lost(b);
    }

    w += (size_t)n;
    b->n = 0;
    return w;
}

esp_err_t mc_batch_feed(mc_batch_t *b, const uint8_t *chunk, size_t len,
                        int64_t ts_ms, char *json, size_t cap,
                        mc_out_cb_t emit, void *arg, int *added)
{
    esp_err_t ret = ESP_OK;
    size_t off = 0;

    *added = 0;

    for (;;)
    {
        int n = 0;

        off += mc_batch_add_chunk(b, chunk + off, len - off, &n);
        *added += n;

        if (off >= len && !mc_batch_ready(b))
        {
            break;              /* the chunk is in, the batch waits     */
        }

        /* due: full with frames of this chunk still to come, or at its
           limit with the chunk in */
        size_t out = mc_batch_json(b, ts_ms, json, cap);

        if (out > 0)
        {
            esp_err_t err = emit(arg, (const uint8_t *)json, out);

            if (ret == ESP_OK)
            {
                ret = err;
            }
        }

        if (off >= len)
        {
            break;
        }
    }

    return ret;
}

/* ---- rx: what the endpoint publishes ---------------------------------------- */

int mc_pub_each(const uint8_t *data, size_t len, mc_out_cb_t emit, void *arg,
                esp_err_t *err)
{
    esp_err_t first = ESP_OK;
    size_t start = 0;
    int calls = 0;

    while (start < len)
    {
        size_t end = len;       /* what is no closed object goes out whole */

        if (data[start] == MC_TX_OPEN)
        {
            int depth = 0;
            bool in_str = false;
            bool esc = false;

            for (size_t i = start; i < len; i++)
            {
                char ch = (char)data[i];

                if (in_str)
                {
                    if (esc)
                    {
                        esc = false;
                    }
                    else if (ch == '\\')
                    {
                        esc = true;
                    }
                    else if (ch == '"')
                    {
                        in_str = false;
                    }
                }
                else if (ch == '"')
                {
                    in_str = true;
                }
                else if (ch == '{')
                {
                    depth++;
                }
                else if (ch == '}' && --depth == 0)
                {
                    end = i + 1;
                    break;
                }
            }
        }

        esp_err_t e = emit(arg, data + start, end - start);

        if (first == ESP_OK)
        {
            first = e;
        }

        calls++;
        start = end;
    }

    if (err != NULL)
    {
        *err = first;
    }

    return calls;
}

/* ---- tx: stream reassembler ------------------------------------------------ */

void mc_txasm_init(mc_txasm_t *a)
{
    memset(a, 0, sizeof(*a));
}

int mc_txasm_feed(mc_txasm_t *a, const uint8_t *in, size_t len,
                  mc_obj_cb_t cb, void *arg)
{
    int done = 0;

    for (size_t i = 0; i < len; i++)
    {
        char ch = (char)in[i];

        if (a->depth == 0)
        {
            /* between objects: ignore everything until an opener */
            if (ch != '{')
            {
                continue;
            }

            a->len = 0;
            a->overflow = false;
        }

        if (a->len < MC_TXBUF_MAX)
        {
            a->buf[a->len++] = ch;
        }
        else
        {
            a->overflow = true;
        }

        if (a->in_str)
        {
            if (a->esc)
            {
                a->esc = false;
            }
            else if (ch == '\\')
            {
                a->esc = true;
            }
            else if (ch == '"')
            {
                a->in_str = false;
            }

            continue;
        }

        if (ch == '"')
        {
            a->in_str = true;
        }
        else if (ch == '{')
        {
            a->depth++;
        }
        else if (ch == '}')
        {
            a->depth--;

            if (a->depth <= 0)
            {
                a->depth = 0;

                if (a->overflow)
                {
                    a->oversize++;
                }
                else if (cb != NULL)
                {
                    cb(arg, a->buf, a->len);
                    done++;
                }

                a->len = 0;
                a->overflow = false;
            }
        }
    }

    return done;
}

/* ---- tx: parser ------------------------------------------------------------ */

/* one element of the "frame" array into @p f; false when it is malformed */
static bool tx_frame(const cJSON *fr, can_core_frame_t *f)
{
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(fr, "id");
    const cJSON *dlc = cJSON_GetObjectItemCaseSensitive(fr, "dlc");
    const cJSON *rtr = cJSON_GetObjectItemCaseSensitive(fr, "rtr");
    const cJSON *ext = cJSON_GetObjectItemCaseSensitive(fr, "extd");
    const cJSON *data = cJSON_GetObjectItemCaseSensitive(fr, "data");

    if (!cJSON_IsNumber(id) || id->valuedouble < 0 ||
        id->valuedouble > 0x1FFFFFFF ||
        !cJSON_IsNumber(dlc) || dlc->valueint < 0 || dlc->valueint > 8)
    {
        return false;
    }

    memset(f, 0, sizeof(*f));
    f->id = (uint32_t)id->valuedouble;
    f->dlc = (uint8_t)dlc->valueint;
    f->rtr = cJSON_IsTrue(rtr);
    f->ext = cJSON_IsTrue(ext) || f->id > 0x7FF;

    for (uint8_t d = 0; d < f->dlc; d++)
    {
        const cJSON *byte = cJSON_IsArray(data)
            ? cJSON_GetArrayItem(data, d) : NULL;

        if (cJSON_IsNumber(byte))
        {
            f->data[d] = (uint8_t)byte->valueint;
        }
    }

    return true;
}

int mc_tx_parse(const char *json, size_t len, mc_frame_cb_t cb, void *arg)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    const cJSON *fr = NULL;
    can_core_frame_t f;
    int count = 0;

    if (!cJSON_IsObject(root))
    {
        cJSON_Delete(root);
        return -1;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "frame");

    if (!cJSON_IsString(type) || strcmp(type->valuestring, "tx") != 0 ||
        !cJSON_IsArray(arr))
    {
        cJSON_Delete(root);
        return -1;
    }

    /* all or nothing: every frame is looked at before the first goes out */
    cJSON_ArrayForEach(fr, arr)
    {
        if (!tx_frame(fr, &f))
        {
            cJSON_Delete(root);
            return -1;
        }

        count++;
    }

    cJSON_ArrayForEach(fr, arr)
    {
        if (cb != NULL && tx_frame(fr, &f))
        {
            cb(arg, &f);
        }
    }

    cJSON_Delete(root);
    return count;
}

/* ---- tx: one encode step ---------------------------------------------------- */

size_t mc_tx_chunk_len(const uint8_t *data, size_t left, size_t cap)
{
    size_t n = (left > cap) ? cap : left;

    /* data[n] would begin the chunk behind this one */
    while (n > 1 && n < left && data[n] == MC_TX_OPEN)
    {
        n--;
    }

    return n;
}

typedef struct
{
    mc_out_cb_t emit;
    void *arg;
    int frames;
    uint32_t *parse_errors;
} tx_step_t;

static void tx_frame_out(void *arg, const can_core_frame_t *f)
{
    tx_step_t *s = arg;
    uint8_t wire[CAN_WIRE_MAX];
    size_t w = can_wire_encode(f, wire);

    if (s->emit(s->arg, wire, w) == ESP_OK)
    {
        s->frames++;
    }
}

static void tx_object(void *arg, const char *json, size_t len)
{
    tx_step_t *s = arg;

    if (mc_tx_parse(json, len, tx_frame_out, s) < 0)
    {
        (*s->parse_errors)++;
    }
}

int mc_tx_feed(mc_txasm_t *a, const uint8_t *in, size_t len,
               mc_out_cb_t emit, void *arg, uint32_t *parse_errors)
{
    tx_step_t s = { .emit = emit, .arg = arg, .frames = 0,
                    .parse_errors = parse_errors };

    if (len > 0 && in[0] == MC_TX_OPEN && a->depth != 0)
    {
        /* a new message (mc_tx_chunk_len: only the first chunk of one
           begins with '{') while an object is still open: the message
           that object came in ended, or lost its tail to a full queue,
           before it closed. Dropped and counted. Until 2026-11-18 the
           reassembler waited for the closing brace for ever, took every
           later message for more of the cut one, and nothing reached the
           bus again before a restart */
        a->cut++;
        a->depth = 0;
        a->in_str = false;
        a->esc = false;
        a->overflow = false;
        a->len = 0;
    }

    (void)mc_txasm_feed(a, in, len, tx_object, &s);
    return s.frames;
}
