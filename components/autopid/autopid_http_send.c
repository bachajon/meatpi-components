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
 * @file autopid_http_send.c
 * @brief POST /api/can/send (2026-11-21, GitHub issue #1072) and the rule
 *        action `can.send`: the two surfaces of autopid_can_send()
 *        (autopid_send.c) that live in this component. The third, the
 *        script binding `can_send()`, is script_engine's. The route table
 *        stays in autopid_http.c.
 *
 * The route answers 200 whenever frames were tried, with what became of
 * each one (`ok` false when one was not acknowledged); 400 for a request
 * that is malformed and 409 for one that cannot be served right now, with
 * the reason, and nothing sent in either case.
 */
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "event_manager.h"
#include "obd_gate.h"

#include "autopid.h"
#include "autopid_http_private.h"
#include "autopid_send.h"

static const char *TAG = "autopid";

esp_err_t ap_send_post_handler(httpd_req_t *req)
{
    /* PSRAM: handlers run on the one web server task */
    static char s_body[2048] EXT_RAM_BSS_ATTR;
    char err[200];

    if (req->content_len == 0 || req->content_len >= sizeof(s_body))
    {
        return ap_http_send_error(req, "400 Bad Request",
                                  "a JSON body under 2 KB is expected");
    }

    size_t got = 0;

    while (got < req->content_len)
    {
        int n = httpd_req_recv(req, s_body + got, req->content_len - got);

        if (n <= 0)
        {
            return ap_http_send_error(req, "400 Bad Request",
                                      "body read failed");
        }

        got += (size_t)n;
    }

    s_body[got] = '\0';

    cJSON *root = cJSON_Parse(s_body);

    if (root == NULL)
    {
        return ap_http_send_error(req, "400 Bad Request", "invalid json");
    }

    cJSON *reply = NULL;
    esp_err_t serr = autopid_can_send(root, &reply, err, sizeof(err));

    cJSON_Delete(root);

    if (reply != NULL)
    {
        /* frames went to the bus, acknowledged or not: the answer says
           which, frame by frame */
        return ap_http_send_json(req, reply);
    }

    return ap_http_send_error(req, (serr == ESP_ERR_INVALID_ARG)
                                       ? "400 Bad Request" : "409 Conflict",
                              err);
}

static esp_err_t act_send(const cJSON *with, const em_event_t *ev)
{
    char err[200];

    (void)ev;

    if (obd_gate_diag_held())
    {
        /* a diagnostic tool has the bus to itself (the UDS Tool, J2534, a
           script, with Exclusive bus): a rule is background, like the
           polling that hold keeps off the bus. The route and a script's
           can_send() are somebody's own hand and are not held */
        ESP_LOGD(TAG, "can.send: not sent, a diagnostic tool holds the bus");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t serr = autopid_can_send(with, NULL, err, sizeof(err));

    if (serr != ESP_OK)
    {
        /* counted by the rules engine as an action error; the reason is
           here for whoever turns this log on (a rule can fire every few
           seconds: not a line per failure at a level that is on) */
        ESP_LOGD(TAG, "can.send: %s", err);
    }

    return serr;
}

void ap_send_events_register(void)
{
    static const em_action_t SEND =
    {
        .name = "can.send",
        .params_schema =
            "{\"type\":\"object\",\"properties\":{"
            "\"bus\":{\"type\":\"string\","
            "\"enum\":[\"obd\",\"can\",\"ms\",\"sw\",\"ch\"],"
            "\"description\":\"obd = the car's OBD protocol through the OBD "
            "chip, can = WiCAN's CAN controller, ms / sw / ch = MS, "
            "single-wire and CH CAN\"},"
            "\"id\":{\"type\":\"string\",\"description\":\"CAN id, hex\"},"
            "\"data\":{\"type\":\"string\","
            "\"description\":\"data bytes, hex\"},"
            "\"ext\":{\"type\":\"boolean\","
            "\"description\":\"a 29-bit id\"},"
            "\"wake\":{\"type\":\"boolean\","
            "\"description\":\"single-wire CAN: high-voltage wake-up\"},"
            "\"bus_kbps\":{\"type\":\"number\"}},"
            "\"required\":[\"bus\"]}",
        .run = act_send,
        .blocking = true, /* a chip round-trip: off the dispatcher */
    };

    (void)event_manager_register_action(&SEND);
}
