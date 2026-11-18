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

/** @file test_steps.c: the canmqtt translator's two steps as the bridge
 *  runs them (mc_batch_feed, mc_tx_feed, the endpoint's chunking
 *  mc_tx_chunk_len): the loss at a full batch, the message cut short,
 *  every frame of a message (2026-11-18). */
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "cJSON.h"

#include "can_frame_wire.h"
#include "mqtt_can_codec.h"

#include "test_frames.h"

/* ---- one decode step: chunks from the can jack -> rx publishes -------------- */

#define MAXID 96
#define MAXPUB 8

static uint32_t g_ids[MAXID];           /* published frame ids, in order */
static int g_nids;
static int g_pub[MAXPUB];               /* frames per publish            */
static int g_npub;
static int g_bad_json;

/* what the translator's sink receives: one rx JSON object per call */
static esp_err_t rx_sink(void *arg, const uint8_t *out, size_t len)
{
    cJSON *root = cJSON_ParseWithLength((const char *)out, len);
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "frame");
    const cJSON *fr = NULL;
    int n = 0;

    (void)arg;

    if (!cJSON_IsArray(arr))
    {
        g_bad_json++;
    }

    cJSON_ArrayForEach(fr, arr)
    {
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(fr, "id");

        if (cJSON_IsNumber(id) && g_nids < MAXID)
        {
            g_ids[g_nids++] = (uint32_t)id->valuedouble;
        }

        n++;
    }

    if (g_npub < MAXPUB)
    {
        g_pub[g_npub] = n;
    }

    g_npub++;
    cJSON_Delete(root);
    return ESP_OK;
}

static void rx_reset(void)
{
    g_nids = 0;
    g_npub = 0;
    g_bad_json = 0;
}

/* @p n frames of @p dlc data bytes as ONE chunk into the batch, ids counting
   up from *@p id. @return frames the step took */
static int rx_chunk(mc_batch_t *b, int n, uint8_t dlc, uint32_t *id)
{
    static char json[MC_JSON_MAX];
    can_core_frame_t fr[12];
    uint8_t chunk[128];                 /* BRIDGE_MANAGER_CHUNK_SIZE */
    int added = 0;

    for (int i = 0; i < n; i++)
    {
        fr[i] = mk_frame((*id)++, dlc, false, false);
    }

    TEST_ASSERT_EQUAL(ESP_OK,
                      mc_batch_feed(b, chunk, wire_chunk(fr, n, chunk), 1,
                                    json, sizeof(json), rx_sink, NULL,
                                    &added));
    return added;
}

/* the batch_ms flush: what waits in the batch goes out */
static void rx_flush(mc_batch_t *b)
{
    static char json[MC_JSON_MAX];
    size_t n = mc_batch_json(b, 2, json, sizeof(json));

    if (n > 0)
    {
        rx_sink(NULL, (const uint8_t *)json, n);
    }
}

void test_batch_feed_chunk_past_a_full_batch_loses_nothing(void)
{
    /* the can jack packs the frames that wait into one chunk: 7 frames of
       8 data bytes are 126 bytes. Four such chunks are 28 frames for a
       batch of 24. Until 2026-11-18 the last 4 of the fourth chunk were
       dropped: 4 of every 28 frames of a busy bus */
    mc_batch_t b;
    uint32_t id = 0;
    int taken = 0;

    rx_reset();
    mc_batch_init(&b, MC_BATCH_MAX);

    for (int c = 0; c < 4; c++)
    {
        taken += rx_chunk(&b, 7, 8, &id);
    }

    rx_flush(&b);

    TEST_ASSERT_EQUAL(28, taken);
    TEST_ASSERT_EQUAL(28, g_nids);

    for (int i = 0; i < 28; i++)
    {
        TEST_ASSERT_EQUAL_UINT32(i, g_ids[i]);
    }

    TEST_ASSERT_EQUAL(2, g_npub);
    TEST_ASSERT_EQUAL(24, g_pub[0]);
    TEST_ASSERT_EQUAL(4, g_pub[1]);
    TEST_ASSERT_EQUAL(0, g_bad_json);
    TEST_ASSERT_EQUAL(0, b.dropped);
}

void test_batch_feed_limit_below_the_batch_size(void)
{
    /* batch_frames 20: 14 frames wait, then a chunk of 12 short frames
       (dlc 0, 10 bytes each) arrives. 26 frames, the batch holds 24:
       frames 25 and 26 were dropped */
    mc_batch_t b;
    uint32_t id = 0;
    int taken = 0;

    rx_reset();
    mc_batch_init(&b, 20);
    taken += rx_chunk(&b, 7, 8, &id);
    taken += rx_chunk(&b, 7, 8, &id);
    TEST_ASSERT_EQUAL(0, g_npub);       /* 14: below the limit          */
    taken += rx_chunk(&b, 12, 0, &id);
    rx_flush(&b);

    TEST_ASSERT_EQUAL(26, taken);
    TEST_ASSERT_EQUAL(26, g_nids);

    for (int i = 0; i < 26; i++)
    {
        TEST_ASSERT_EQUAL_UINT32(i, g_ids[i]);
    }

    TEST_ASSERT_EQUAL(2, g_npub);
    TEST_ASSERT_EQUAL(24, g_pub[0]);
    TEST_ASSERT_EQUAL(2, g_pub[1]);
    TEST_ASSERT_EQUAL(0, b.dropped);
}

void test_batch_feed_a_chunk_stays_in_one_publish(void)
{
    /* batch_frames 2 and a chunk of 7: ONE publish of 7, as it always was.
       The limit is looked at when a chunk is in; a decode step emits a
       second object only behind a FULL batch */
    mc_batch_t b;
    uint32_t id = 0;

    rx_reset();
    mc_batch_init(&b, 2);
    TEST_ASSERT_EQUAL(7, rx_chunk(&b, 7, 8, &id));
    TEST_ASSERT_EQUAL(1, g_npub);
    TEST_ASSERT_EQUAL(7, g_pub[0]);
    TEST_ASSERT_EQUAL(0, b.n);
}

void test_batch_json_that_does_not_fit_is_counted(void)
{
    /* cannot happen with MC_JSON_MAX (test_batch_worstcase_fits); should a
       buffer ever be too small, the frames of the batch are counted, not
       gone without a trace */
    mc_batch_t b;
    char small[64];
    uint32_t id = 0;

    rx_reset();
    mc_batch_init(&b, 4);
    TEST_ASSERT_EQUAL(3, rx_chunk(&b, 3, 8, &id));
    TEST_ASSERT_EQUAL(0, mc_batch_json(&b, 1, small, sizeof(small)));
    TEST_ASSERT_EQUAL(0, b.n);
    TEST_ASSERT_EQUAL(3, b.dropped);
}

/* ---- one encode step: MQTT messages -> frames for the bus ------------------- */

#define CHUNK 128                       /* BRIDGE_MANAGER_CHUNK_SIZE */
#define MAXTX 80

static can_core_frame_t g_tx[MAXTX];
static int g_ntx;

/* what the translator's sink receives: one can_frame_wire record per call */
static esp_err_t tx_sink(void *arg, const uint8_t *out, size_t len)
{
    (void)arg;

    if (g_ntx < MAXTX && can_wire_decode(out, len, &g_tx[g_ntx]))
    {
        g_ntx++;
    }

    return ESP_OK;
}

/* one MQTT message the way the mqtt0 endpoint hands it to the bridge: cut
   into chunks, one encode step per chunk. @return frames the sink took */
static int tx_message(mc_txasm_t *a, const char *msg, size_t len,
                      uint32_t *parse_errors)
{
    int frames = 0;
    size_t off = 0;

    while (off < len)
    {
        size_t n = mc_tx_chunk_len((const uint8_t *)msg + off, len - off,
                                   CHUNK);

        TEST_ASSERT_TRUE(n >= 1 && n <= CHUNK && n <= len - off);
        frames += mc_tx_feed(a, (const uint8_t *)msg + off, n, tx_sink,
                             NULL, parse_errors);
        off += n;
    }

    return frames;
}

/* a tx message of @p k frames (ids @p id0 up, dlc 1, the data byte = the
   frame's number), @p pad characters of a string in front of them.
   @p bad (1-based, 0 = none) gets a dlc of 9. @return its length */
static size_t tx_build(char *msg, size_t cap, int k, uint32_t id0, int pad,
                       int bad)
{
    size_t w = (size_t)snprintf(msg, cap, "{\"p\":\"");

    memset(msg + w, 'x', (size_t)pad);
    w += (size_t)pad;
    w += (size_t)snprintf(msg + w, cap - w,
                          "\",\"bus\":0,\"type\":\"tx\",\"frame\":[");

    for (int i = 0; i < k; i++)
    {
        w += (size_t)snprintf(msg + w, cap - w,
                              "%s{\"id\":%lu,\"dlc\":%d,\"data\":[%d]}",
                              (i > 0) ? "," : "",
                              (unsigned long)(id0 + (uint32_t)i),
                              (i + 1 == bad) ? 9 : 1, i);
    }

    w += (size_t)snprintf(msg + w, cap - w, "]}");
    TEST_ASSERT_TRUE(w < cap && w <= MC_TXBUF_MAX);
    return w;
}

void test_tx_cut_message_does_not_swallow_the_next(void)
{
    /* a message that ends before its object closes: a publisher's mistake,
       or the tail of a message lost to a full bridge queue. Until
       2026-11-18 the reassembler waited for the closing brace for ever:
       every later message was read as more of the cut one and nothing
       reached the bus again before a restart */
    static const char *const CUT[] =
    {
        "{\"bus\":0,\"type\":\"tx\",\"frame\":[",           /* top object */
        "{\"bus\":0,\"type\":\"tx\",\"frame\":[{\"id\":1,", /* a frame    */
        "{\"bus\":0,\"ty",                                  /* a string   */
        NULL,                           /* the first chunk of a long one */
    };
    static char lng[MC_TXBUF_MAX + 1];
    static char good[160];
    size_t lng_len = tx_build(lng, sizeof(lng), 9, 0x400, 0, 0);
    size_t good_len = tx_build(good, sizeof(good), 1, 0x123, 0, 0);

    TEST_ASSERT_TRUE(lng_len > 2 * CHUNK);

    for (int c = 0; c < 4; c++)
    {
        mc_txasm_t a;
        uint32_t perr = 0;
        const char *cut = (CUT[c] != NULL) ? CUT[c] : lng;
        size_t cut_len = (CUT[c] != NULL)
            ? strlen(cut)
            : mc_tx_chunk_len((const uint8_t *)lng, lng_len, CHUNK);

        mc_txasm_init(&a);
        g_ntx = 0;
        TEST_ASSERT_EQUAL(0, tx_message(&a, cut, cut_len, &perr));

        for (int i = 0; i < 20; i++)    /* the publisher goes on */
        {
            TEST_ASSERT_EQUAL(1, tx_message(&a, good, good_len, &perr));
        }

        TEST_ASSERT_EQUAL(20, g_ntx);
        TEST_ASSERT_EQUAL_HEX32(0x123, g_tx[0].id);
        TEST_ASSERT_EQUAL(1, a.cut);    /* counted, once */
        TEST_ASSERT_EQUAL(0, perr);
    }
}

void test_tx_whole_messages_at_every_chunk_alignment(void)
{
    /* what tells a new message from the rest of the one before must never
       take a whole message for a cut one: 1, 5 and 9 frames behind every
       length of padding, so a frame object begins at every offset of a
       128 byte chunk, one reassembler for the lot */
    static char msg[MC_TXBUF_MAX + 1];
    mc_txasm_t a;
    uint32_t perr = 0;

    mc_txasm_init(&a);

    for (int pad = 0; pad <= 2 * CHUNK; pad++)
    {
        for (int k = 1; k <= 9; k += 4)
        {
            size_t len = tx_build(msg, sizeof(msg), k, 0x200, pad, 0);

            g_ntx = 0;
            TEST_ASSERT_EQUAL(k, tx_message(&a, msg, len, &perr));
            TEST_ASSERT_EQUAL_HEX32(0x200 + k - 1, g_tx[k - 1].id);
        }
    }

    TEST_ASSERT_EQUAL(0, a.cut);
    TEST_ASSERT_EQUAL(0, a.oversize);
    TEST_ASSERT_EQUAL(0, perr);
}

void test_tx_every_frame_of_a_message_reaches_the_bus(void)
{
    /* 12 frames in one message: until 2026-11-18 the first 8 were sent and
       the other 4 dropped without a count */
    static char msg[MC_TXBUF_MAX + 1];
    mc_txasm_t a;
    uint32_t perr = 0;
    size_t len = tx_build(msg, sizeof(msg), 12, 0x101, 0, 0);
    int k = 0;

    mc_txasm_init(&a);
    g_ntx = 0;
    TEST_ASSERT_EQUAL(12, tx_message(&a, msg, len, &perr));

    for (int i = 0; i < 12; i++)
    {
        TEST_ASSERT_EQUAL_HEX32(0x101 + i, g_tx[i].id);
        TEST_ASSERT_EQUAL_HEX8(i, g_tx[i].data[0]);
    }

    /* and as many as the reassembly buffer holds (MC_TXBUF_MAX) */
    len = (size_t)snprintf(msg, sizeof(msg), "{\"type\":\"tx\",\"frame\":[");

    for (;;)
    {
        char one[32];
        size_t n = (size_t)snprintf(one, sizeof(one),
                                    "%s{\"id\":%d,\"dlc\":0}",
                                    (k > 0) ? "," : "", k + 1);

        if (len + n + 2 > MC_TXBUF_MAX)
        {
            break;
        }

        memcpy(msg + len, one, n);
        len += n;
        k++;
    }

    memcpy(msg + len, "]}", 2);
    len += 2;

    g_ntx = 0;
    TEST_ASSERT_TRUE(k > 50 && k <= MAXTX);
    TEST_ASSERT_EQUAL(k, tx_message(&a, msg, len, &perr));
    TEST_ASSERT_EQUAL_HEX32(k, g_tx[k - 1].id);
    TEST_ASSERT_EQUAL(0, perr);
    TEST_ASSERT_EQUAL(0, a.oversize);
}

void test_tx_bad_frame_anywhere_refuses_the_message(void)
{
    /* all or nothing: the eleventh of 12 frames has a dlc of 9. Until
       2026-11-18 nothing looked past the ninth frame, and the first 8 of
       this message went to the bus */
    static char msg[MC_TXBUF_MAX + 1];
    mc_txasm_t a;
    uint32_t perr = 0;
    size_t len = tx_build(msg, sizeof(msg), 12, 0x101, 0, 11);

    mc_txasm_init(&a);
    g_ntx = 0;
    TEST_ASSERT_EQUAL(0, tx_message(&a, msg, len, &perr));
    TEST_ASSERT_EQUAL(0, g_ntx);
    TEST_ASSERT_EQUAL(1, perr);

    len = tx_build(msg, sizeof(msg), 2, 0x300, 0, 0);
    TEST_ASSERT_EQUAL(2, tx_message(&a, msg, len, &perr));
    TEST_ASSERT_EQUAL(1, perr);
}

void run_step_tests(void)
{
    RUN_TEST(test_batch_feed_a_chunk_stays_in_one_publish);
    RUN_TEST(test_tx_whole_messages_at_every_chunk_alignment);
    RUN_TEST(test_batch_feed_chunk_past_a_full_batch_loses_nothing);
    RUN_TEST(test_batch_feed_limit_below_the_batch_size);
    RUN_TEST(test_batch_json_that_does_not_fit_is_counted);
    RUN_TEST(test_tx_cut_message_does_not_swallow_the_next);
    RUN_TEST(test_tx_every_frame_of_a_message_reaches_the_bus);
    RUN_TEST(test_tx_bad_frame_anywhere_refuses_the_message);
}
