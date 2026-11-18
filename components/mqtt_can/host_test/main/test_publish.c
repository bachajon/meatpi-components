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

/** @file test_publish.c: what the mqtt0 endpoint publishes when the bridge
 *  hands it the output of several codec calls in one send (mc_pub_each,
 *  2026-11-18): one MQTT message is one JSON object. */
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "cJSON.h"

#include "can_frame_wire.h"
#include "mqtt_can_codec.h"

#include "test_frames.h"

#define MAXMSG 8

static char g_msg[MAXMSG][512];
static size_t g_len[MAXMSG];
static int g_nmsg;
static int g_refuse_at;                 /* this publish is refused (-1: none) */

static void pub_reset(void)
{
    g_nmsg = 0;
    g_refuse_at = -1;
}

/* the endpoint's publish: one call = one MQTT message */
static esp_err_t pub_sink(void *arg, const uint8_t *out, size_t len)
{
    (void)arg;

    int i = g_nmsg++;

    if (i < MAXMSG && len < sizeof(g_msg[0]))
    {
        memcpy(g_msg[i], out, len);
        g_len[i] = len;
    }

    return (i == g_refuse_at) ? ESP_ERR_NO_MEM : ESP_OK;
}

/* how a broker client reads a message: the whole payload is one JSON
   document, nothing behind it. Frames in it, -1 when it is not one. */
static int frames_of_one_document(const char *msg, size_t len)
{
    const char *end = NULL;
    cJSON *root = cJSON_ParseWithLengthOpts(msg, len, &end, false);
    int n = -1;

    if (root != NULL && end == msg + len)
    {
        const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "frame");

        n = cJSON_IsArray(arr) ? cJSON_GetArraySize(arr) : -1;
    }

    cJSON_Delete(root);
    return n;
}

/* the bridge's batch: what the codec emits during one pass, back to back */
typedef struct
{
    uint8_t buf[1024];
    size_t len;
} gather_t;

static esp_err_t gather_sink(void *arg, const uint8_t *out, size_t len)
{
    gather_t *g = arg;

    TEST_ASSERT_TRUE(g->len + len <= sizeof(g->buf));
    memcpy(g->buf + g->len, out, len);
    g->len += len;
    return ESP_OK;
}

void test_pub_two_batches_of_one_pass_are_two_messages(void)
{
    static mc_batch_t b;
    static char json[MC_JSON_MAX];
    can_core_frame_t fr[2];
    uint8_t chunk[2 * CAN_WIRE_MAX];
    gather_t g = { .len = 0 };
    int added = 0;

    /* batch_frames 2 on a busy bus: two chunks of two frames wait for the
       bridge, it runs the codec on both and sends what came out at once */
    mc_batch_init(&b, 2);

    for (int c = 0; c < 2; c++)
    {
        fr[0] = mk_frame(0x100 + 2 * c, 8, false, false);
        fr[1] = mk_frame(0x101 + 2 * c, 8, false, false);

        size_t n = wire_chunk(fr, 2, chunk);

        TEST_ASSERT_EQUAL(ESP_OK, mc_batch_feed(&b, chunk, n, 1000 + c, json, sizeof(json),
                                                gather_sink, &g, &added));
    }

    /* two objects back to back, as the bridge hands them over */
    TEST_ASSERT_EQUAL_INT(-1, frames_of_one_document((const char *)g.buf, g.len));

    esp_err_t err = ESP_FAIL;

    pub_reset();
    TEST_ASSERT_EQUAL_INT(2, mc_pub_each(g.buf, g.len, pub_sink, NULL, &err));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_INT(2, g_nmsg);
    TEST_ASSERT_EQUAL_INT(2, frames_of_one_document(g_msg[0], g_len[0]));
    TEST_ASSERT_EQUAL_INT(2, frames_of_one_document(g_msg[1], g_len[1]));
    TEST_ASSERT_EQUAL_size_t(g.len, g_len[0] + g_len[1]);
}

void test_pub_one_object_is_one_message_untouched(void)
{
    static const char ONE[] =
        "{\"bus\":\"0\",\"type\":\"rx\",\"ts\":7,\"frame\":[{\"id\":291,\"dlc\":1,"
        "\"rtr\":false,\"extd\":false,\"data\":[1]}]}";

    pub_reset();
    TEST_ASSERT_EQUAL_INT(1, mc_pub_each((const uint8_t *)ONE, strlen(ONE), pub_sink, NULL, NULL));
    TEST_ASSERT_EQUAL_size_t(strlen(ONE), g_len[0]);
    TEST_ASSERT_EQUAL_MEMORY(ONE, g_msg[0], strlen(ONE));
}

void test_pub_braces_inside_strings_do_not_cut(void)
{
    static const char TWO[] = "{\"a\":\"}{\",\"b\":{\"c\":\"\\\"}\"}}{\"d\":1}";

    pub_reset();
    TEST_ASSERT_EQUAL_INT(2, mc_pub_each((const uint8_t *)TWO, strlen(TWO), pub_sink, NULL, NULL));
    TEST_ASSERT_EQUAL_size_t(strlen("{\"a\":\"}{\",\"b\":{\"c\":\"\\\"}\"}}"), g_len[0]);
    TEST_ASSERT_EQUAL_size_t(strlen("{\"d\":1}"), g_len[1]);
    TEST_ASSERT_EQUAL_MEMORY("{\"d\":1}", g_msg[1], g_len[1]);
}

void test_pub_bytes_that_are_no_object_go_out_as_they_are(void)
{
    /* a raw bridge on mqtt0: no JSON, one message as before */
    pub_reset();
    TEST_ASSERT_EQUAL_INT(1, mc_pub_each((const uint8_t *)"ATZ\r41 0C 1A F8\r>", 17, pub_sink,
                                         NULL, NULL));
    TEST_ASSERT_EQUAL_size_t(17, g_len[0]);

    /* an object that does not close: the rest, once */
    pub_reset();
    TEST_ASSERT_EQUAL_INT(2, mc_pub_each((const uint8_t *)"{\"a\":1}{\"b\":", 12, pub_sink, NULL,
                                         NULL));
    TEST_ASSERT_EQUAL_size_t(7, g_len[0]);
    TEST_ASSERT_EQUAL_size_t(5, g_len[1]);

    pub_reset();
    TEST_ASSERT_EQUAL_INT(0, mc_pub_each((const uint8_t *)"", 0, pub_sink, NULL, NULL));
}

void test_pub_a_refused_publish_does_not_hold_the_rest(void)
{
    static const char THREE[] = "{\"a\":1}{\"b\":2}{\"c\":3}";
    esp_err_t err = ESP_OK;

    pub_reset();
    g_refuse_at = 1;
    TEST_ASSERT_EQUAL_INT(3, mc_pub_each((const uint8_t *)THREE, strlen(THREE), pub_sink, NULL,
                                         &err));
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, err);
    TEST_ASSERT_EQUAL_MEMORY("{\"c\":3}", g_msg[2], 7);
}

void run_publish_tests(void)
{
    RUN_TEST(test_pub_two_batches_of_one_pass_are_two_messages);
    RUN_TEST(test_pub_one_object_is_one_message_untouched);
    RUN_TEST(test_pub_braces_inside_strings_do_not_cut);
    RUN_TEST(test_pub_bytes_that_are_no_object_go_out_as_they_are);
    RUN_TEST(test_pub_a_refused_publish_does_not_hold_the_rest);
}
