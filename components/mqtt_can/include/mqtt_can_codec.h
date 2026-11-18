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
 * @file mqtt_can_codec.h
 * @brief PURE legacy-JSON CAN⇄MQTT codec (host-tested; TASK_mqtt_can.md).
 *
 * Byte-parity with the legacy contract (legacy main/mqtt.c):
 *   rx: {"bus":"0","type":"rx","ts":1234,"frame":[{"id":123,"dlc":8,
 *        "rtr":false,"extd":false,"data":[1,...,8]},...]}
 *   tx: {"bus":0,"type":"tx","frame":[...]}   (parsed, "ts" ignored)
 *
 * Sizing: everything lives inside the translator ctx (BM_CTX_MAX 4096;
 * NEVER on the 4 KB pump stack: the 2026-07-22 stack-audit lesson).
 * Worst-case frame JSON ≈ 91 B → 24-frame batch ≈ 2.2 KB.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "can_core.h" /* can_core_frame_t */

#ifdef __cplusplus
extern "C" {
#endif

#define MC_BATCH_MAX 24   /* frames per publish (settings cap)          */
#define MC_JSON_MAX  2304 /* worst-case 24-frame batch + wrapper        */
#define MC_TXBUF_MAX 1024 /* inbound tx-object reassembly cap           */
#define MC_TX_OPEN   '{'  /* what a tx message begins with              */

/** Where a codec step puts what it produces: the shape of bridge_manager's
 *  bridge_sink_fn_t, so the translator hands its sink straight through. */
typedef esp_err_t (*mc_out_cb_t)(void *arg, const uint8_t *out, size_t len);

/* ---- rx side: frame batcher -> legacy JSON --------------------------------- */

typedef struct
{
    uint16_t n;
    uint16_t limit;                 /* flush threshold (1..MC_BATCH_MAX) */
    uint32_t dropped;               /* frames of a batch that did not fit
                                       its JSON buffer (0 by sizing)     */
    can_core_frame_t frames[MC_BATCH_MAX];
} mc_batch_t;

void mc_batch_init(mc_batch_t *b, uint16_t batch_frames);

/** Unpack a can_frame_wire chunk (concatenated frames) into the batch as
 *  far as the batch has room (MC_BATCH_MAX). Nothing is dropped here.
 *  @return the bytes of @p chunk taken: less than @p len only when the
 *  batch is full and a frame is left (the caller drains the batch and
 *  calls again with the rest); a malformed tail counts as taken, the
 *  frames in front of it are kept. *@p added (may be NULL) = frames
 *  added. */
size_t mc_batch_add_chunk(mc_batch_t *b, const uint8_t *chunk, size_t len,
                          int *added);

/** True when the configured limit is reached (caller emits). */
bool mc_batch_ready(const mc_batch_t *b);

/** Serialize + DRAIN the batch into legacy rx JSON. @return bytes
 *  written; 0 when the batch is empty (nothing emitted), and 0 when it
 *  does not fit @p cap: its frames are then counted in b->dropped
 *  (cannot happen with MC_JSON_MAX, the host suite proves the sizing). */
size_t mc_batch_json(mc_batch_t *b, int64_t ts_ms, char *out, size_t cap);

/** One decode step (the canmqtt translator's decode). Unpacks @p chunk into
 *  the batch and hands every batch that is due to @p emit as ONE legacy rx
 *  JSON object (built in @p json, @p cap bytes):
 *   - a batch that is FULL while the chunk still has frames: it goes out
 *     and the rest of the chunk starts the next batch. Until 2026-11-18
 *     that rest was dropped: the can jack packs the frames that wait into
 *     one chunk (7 frames of 8 data bytes), so the fourth chunk of a batch
 *     lost 4 of its 7 frames, 4 of every 28 on a busy bus;
 *   - the batch the chunk ends on, when it has reached its limit.
 *  The limit is looked at when the chunk is in, as before: the frames of
 *  one chunk stay in one publish unless the batch is full, so a step emits
 *  at most ONE object of fewer than MC_BATCH_MAX frames.
 *  @return ESP_OK, or the first refusal of @p emit (the batch is drained
 *  either way). *@p added = frames taken into batches. */
esp_err_t mc_batch_feed(mc_batch_t *b, const uint8_t *chunk, size_t len,
                        int64_t ts_ms, char *json, size_t cap,
                        mc_out_cb_t emit, void *arg, int *added);

/** How the mqtt0 endpoint publishes what a bridge hands it in one send().
 *  Since 2026-11-17 the bridge gathers what a codec emits during one pass
 *  into one send() of up to 1 KB, so with a small batch_frames on a busy
 *  bus a send() holds two or more rx objects back to back, and an MQTT
 *  message with two is refused by every JSON parser (found 2026-11-18;
 *  the default batch of 24 frames is longer than 1 KB and always left
 *  alone). Each top-level JSON object of @p data goes to @p emit on its
 *  own (brace depth, string and escape aware); bytes that do not begin
 *  with '{', or an object that does not close, go out as they are in one
 *  call (a raw bridge on mqtt0, as before).
 *  @return the number of @p emit calls. *@p err (may be NULL) = the first
 *  refusal of @p emit, ESP_OK otherwise; a refusal does not hold the
 *  objects behind it. */
int mc_pub_each(const uint8_t *data, size_t len, mc_out_cb_t emit, void *arg,
                esp_err_t *err);

/* ---- tx side: stream reassembler + parser ---------------------------------- */

/** Extracts complete top-level JSON objects from a fragmented byte
 *  stream (brace depth, string/escape aware). An object longer than
 *  MC_TXBUF_MAX is discarded to its closing boundary and counted. */
typedef struct
{
    uint16_t len;
    int16_t  depth;
    bool     in_str;
    bool     esc;
    bool     overflow;
    uint32_t oversize;              /* objects discarded for length      */
    uint32_t cut;                   /* objects discarded: a new message
                                       began before they closed
                                       (mc_tx_feed)                      */
    char     buf[MC_TXBUF_MAX];
} mc_txasm_t;

typedef void (*mc_obj_cb_t)(void *arg, const char *json, size_t len);

void mc_txasm_init(mc_txasm_t *a);

/** Feed bytes; @p cb fires once per COMPLETE object. @return objects
 *  completed in this call. */
int mc_txasm_feed(mc_txasm_t *a, const uint8_t *in, size_t len,
                  mc_obj_cb_t cb, void *arg);

typedef void (*mc_frame_cb_t)(void *arg, const can_core_frame_t *f);

/** Parse one legacy tx object and hand EVERY frame it holds to @p cb, in
 *  order: no cap (the object itself is bounded by MC_TXBUF_MAX: some 55
 *  frames without data). Until 2026-11-18 the caller gave room for 8 and
 *  the frames behind the eighth were dropped without a count. All or nothing:
 *  a malformed frame anywhere refuses the whole object before the first
 *  @p cb. Tolerant of "ts" and of "bus" as number or string; requires
 *  "type":"tx" and a "frame" array. @return frame count (0 allowed), or
 *  -1 malformed. */
int mc_tx_parse(const char *json, size_t len, mc_frame_cb_t cb, void *arg);

/** How the mqtt0 endpoint cuts an MQTT message into bridge chunks: of the
 *  @p left bytes at @p data the next chunk takes this many (1..@p cap).
 *  A chunk is ended one byte early when the chunk behind it would begin
 *  with '{', so of all the chunks of a message only the FIRST can begin
 *  with one: that is what tells mc_tx_feed a new message from the rest of
 *  the one before. The bytes and their order are untouched (a raw bridge
 *  on mqtt0 sees the same stream). */
size_t mc_tx_chunk_len(const uint8_t *data, size_t left, size_t cap);

/** One encode step (the canmqtt translator's encode): one bridge chunk of
 *  an MQTT message. The chunk goes through the reassembler, every complete
 *  tx object is parsed (mc_tx_parse) and each of its frames goes to
 *  @p emit as a can_frame_wire record. A chunk that begins with '{' while
 *  an object is still open begins a NEW message (mc_tx_chunk_len): the
 *  open object is dropped and counted in a->cut, where until 2026-11-18 it
 *  swallowed every later message. A malformed object counts in
 *  *@p parse_errors. @return frames @p emit took.
 *  The source must be message-framed like mqtt0: a byte stream cut at
 *  random (a TCP socket) can begin a chunk with the '{' of a frame object
 *  and loses that message. */
int mc_tx_feed(mc_txasm_t *a, const uint8_t *in, size_t len,
               mc_out_cb_t emit, void *arg, uint32_t *parse_errors);

#ifdef __cplusplus
}
#endif
