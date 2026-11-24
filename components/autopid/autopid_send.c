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
 * @file autopid_send.c
 * @brief The device half of can.send (autopid_send.h, GitHub issue #1072):
 *        autopid_can_send() behind the route POST /api/can/send, the rule
 *        action `can.send` and the script binding `can_send()`.
 *
 * A frame for `can` goes to WiCAN's own CAN controller (can_manager). A
 * frame for any other bus goes through the OBD chip as a chip job: the
 * poller is parked, the chip is taken to the bus (a slot in its RAM,
 * autopid_bus.c, or the car's own protocol before CAN), the frames go out,
 * and the chip is taken home with every word of the send undone. Nothing of
 * it writes the chip's memory.
 *
 * Callers: the web server's task, a worker of the rules engine, the script
 * task. None of them touches the flash here (PSRAM stacks are fine), and
 * the chip job serializes them with each other and with every other job
 * (the Test route, a trouble code scan, the standard PID scan).
 */
#include "autopid_send.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can_manager.h"

#include "autopid.h"
#include "autopid_bus.h"
#include "autopid_private.h"
#include "autopid_proto.h"
#include "autopid_runner_private.h"
#include "autopid_transport.h"

static const char *TAG = "autopid";

#define SEND_TR_LEN       2048 /* the transcript of a chip send              */
#define SEND_JOB_WAIT_MS  3000 /* for another chip job to end                */
#define SEND_CAN_TIMEOUT  pdMS_TO_TICKS(1500)
#define SEND_BUS_TIMEOUT  pdMS_TO_TICKS(6000) /* a K-line bus may start first */
#define SEND_CTRL_WAIT_MS 150  /* for the controller to report the frame sent */
#define SEND_REPLY_LEN    48
#define SEND_RESP_LEN     256  /* what the chip said to one frame            */
#define SEND_LINE_LEN     64   /* one line of it, in `replies`               */

typedef struct
{
    ap_send_result_t result;
    int32_t          at_ms;                 /* from the first frame's start */
    char             reply[SEND_REPLY_LEN]; /* short: what the chip said, or
                                               why not (`err`, and `reply`
                                               of a frame not kept whole)   */
} send_out_t;

/* the transcript of the chip send in flight; under the chip job */
static char s_tr[SEND_TR_LEN] EXT_RAM_BSS_ATTR;

/* what the chip said to each frame, whole (2026-11-21: `reply` was the at
   most 47 characters of send_out_t, and the answers of a request to 7DF
   that seven ECUs answered were cut off); under the chip job, read by
   build_reply() before the job is released. "" = the chip said nothing to
   that frame or it was never sent. */
static char s_resp[AP_SEND_FRAMES_MAX][SEND_RESP_LEN] EXT_RAM_BSS_ATTR;

static esp_err_t refuse(esp_err_t code, char *err, size_t err_len,
                        const char *msg)
{
    if (err != NULL && err_len > 0)
    {
        snprintf(err, err_len, "%s", msg);
    }

    return code;
}

/** Wait until @p due_us (esp_timer), when that is still ahead. */
static void wait_until(int64_t due_us)
{
    int64_t left_ms = (due_us - esp_timer_get_time()) / 1000;

    if (left_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)left_ms));
    }
}

/* ---- WiCAN's own CAN controller ------------------------------------------- */

static esp_err_t send_controller(const ap_send_req_t *req, send_out_t *out,
                                 char *err, size_t err_len)
{
    can_manager_status_t st;

    if (!can_manager_tx_ready())
    {
        bool known = can_manager_status(&st) == ESP_OK;

        return refuse(ESP_ERR_INVALID_STATE, err, err_len,
                      (!known || !st.enabled)
                          ? "WiCAN's CAN controller is off (Settings, CAN): "
                            "bus obd sends through the OBD chip"
                      : st.silent
                          ? "WiCAN's CAN controller is set to listen only"
                          : "WiCAN's CAN controller is not on the bus yet "
                            "(no bit rate proven, or the bus is down)");
    }

    int64_t t0 = 0;
    int64_t due = 0;

    for (int i = 0; i < req->n; i++)
    {
        const ap_send_frame_t *f = &req->f[i];
        uint32_t done = 0;
        uint32_t lost = 0;

        if (can_manager_status(&st) == ESP_OK)
        {
            done = st.stats.tx_done;
            lost = st.stats.tx_lost;
        }

        wait_until(due);

        int64_t now = esp_timer_get_time();

        if (i == 0)
        {
            t0 = now;
        }

        out[i].at_ms = (int32_t)((now - t0) / 1000);
        due = now + (int64_t)f->gap_ms * 1000;

        if (can_manager_send(f->id, f->ext, false, f->data, f->len) != ESP_OK)
        {
            out[i].result = AP_SEND_ERROR;
            snprintf(out[i].reply, sizeof(out[i].reply),
                     "the controller did not take the frame");
            continue;
        }

        /* the controller says when the frame is out; one it gave up on
           after its retries found nobody to acknowledge it */
        out[i].result = AP_SEND_NO_ACK;

        for (int ms = 0; ms < SEND_CTRL_WAIT_MS; ms += 2)
        {
            if (can_manager_status(&st) != ESP_OK)
            {
                break;
            }

            if (st.stats.tx_done != done)
            {
                out[i].result = AP_SEND_SENT;
                break;
            }

            if (st.stats.tx_lost != lost)
            {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    return ESP_OK;
}

/* ---- the OBD chip ------------------------------------------------------------ */

/** The chip on the car's own protocol with its baseline in effect, for a
 *  message before CAN. @return false = the bus guard has no verdict for
 *  that protocol right now: nothing may go out. */
static bool home_ready(ap_tr_t *tr)
{
    ap_base_mem_t *mem = ap_base_mem();
    uint32_t gen = ap_base_chip_gen();

    ap_bus_leave(tr); /* a row may have left the chip on another bus */

    if (!mem->baseline_sent || mem->chip_gen != gen || mem->moved != 0 ||
        mem->hdr_borrowed || mem->rxheader_set || mem->init_pin != '\0')
    {
        if (!ap_base_restore(tr))
        {
            return false;
        }

        mem->baseline_sent = true;
        mem->chip_gen = gen;
    }

    return true;
}

/** Every frame of @p req through the chip. Under the chip job, the poller
 *  parked. */
static void send_chip(const ap_send_req_t *req, char proto, send_out_t *out,
                      ap_tr_t *tr)
{
    bool precan = req->precan;
    ap_base_mem_t *mem = ap_base_mem();
    uint16_t at_key = UINT16_MAX; /* where our setup words are in effect */
    uint32_t hdr_id = UINT32_MAX; /* the id the chip's header is set to   */
    bool hdr_ext = false;
    bool stop = false;
    int64_t t0 = 0;
    int64_t due = 0;

    for (int i = 0; i < req->n; i++)
    {
        const ap_send_frame_t *f = &req->f[i];
        uint16_t key = ap_send_key(req, i, proto);
        char words[40];
        char *resp = s_resp[i];

        resp[0] = '\0';
        out[i].result = AP_SEND_NOT_SENT;

        if (stop)
        {
            continue;
        }

        /* the chip where this frame goes, with the setup words behind every
           arrival (the way there resets the header and the format) */
        ap_bus_stats_t before;
        ap_bus_stats_t after;

        ap_bus_get_stats(&before);

        bool there = precan ? ((at_key == 0) || home_ready(tr))
                            : ap_bus_row_setup(NULL, key, -1, NULL, NULL, tr);

        ap_bus_get_stats(&after);

        if (!there)
        {
            out[i].result = AP_SEND_ERROR;
            snprintf(out[i].reply, sizeof(out[i].reply), "%s",
                     precan ? "the bus guard holds the car's protocol"
                            : "the chip did not take that bus");
            stop = true;
            continue;
        }

        if (key != at_key || after.changes != before.changes)
        {
            ap_base_send_init(ap_send_setup(precan), tr);
            at_key = key;
            hdr_id = UINT32_MAX;
        }

        if (f->id != hdr_id || f->ext != hdr_ext)
        {
            if (ap_send_header(f, precan, words, sizeof(words)) > 0)
            {
                ap_base_send_init(words, tr);
            }

            hdr_id = f->id;
            hdr_ext = f->ext;
        }

        (void)ap_send_data(f, words, sizeof(words));
        wait_until(due);

        int64_t now = esp_timer_get_time();

        if (i == 0)
        {
            t0 = now;
        }

        out[i].at_ms = (int32_t)((now - t0) / 1000);
        due = now + (int64_t)f->gap_ms * 1000;

        esp_err_t err = ap_base_request_tr(words, resp, SEND_RESP_LEN,
                                           precan ? SEND_BUS_TIMEOUT
                                                  : SEND_CAN_TIMEOUT, tr);

        if (err != ESP_OK)
        {
            resp[0] = '\0'; /* the sentence below is the reply */
            out[i].result = AP_SEND_ERROR;
            snprintf(out[i].reply, sizeof(out[i].reply), "%s",
                     (err == ESP_ERR_INVALID_STATE)
                         ? "the chip is busy (an update or a monitor)"
                         : "no reply from the chip");
            stop = true; /* the chip is not ours to go on with */
            continue;
        }

        out[i].result = ap_send_classify(resp);
        ap_send_reply_join(resp, out[i].reply, sizeof(out[i].reply));
    }

    /* home, with what the send set put back: the baseline prelude carries
       the undo of every word (ap_chain_restore), and the slots go */
    if (precan)
    {
        (void)ap_base_restore(tr);
    }
    else
    {
        ap_bus_leave(tr);
    }

    if (mem->moved != 0)
    {
        /* no verdict of the bus guard for the prelude right now: the words
           that are not the prelude's own are undone all the same, the
           prelude is due before the next row */
        ap_runner_undo_tables();
        mem->baseline_sent = false;
    }
}

/** May the chip be taken for a send right now? */
static esp_err_t chip_free(const ap_send_req_t *req, char proto, char *err,
                           size_t err_len)
{
    if (ap_proto(proto) == NULL && req->where == AP_SEND_OBD)
    {
        (void)ap_send_check_car(req, proto, err, err_len);
        return ESP_ERR_INVALID_STATE;
    }

    if (!ap_send_check_car(req, proto, err, err_len))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (ap_be()->client_idle_ms != NULL &&
        ap_sched_client_hold(ap_be()->client_idle_ms()))
    {
        /* (2026-11-21, GitHub issue #1078) or when a monitor the app
           opened ends: obd_chip_client_write() */
        return refuse(ESP_ERR_INVALID_STATE, err, err_len,
                      AP_CLIENT_BUSY_TEXT ". Bus can does not need the chip");
    }

    if (req->where == AP_SEND_OBD)
    {
        /* the bus guard listens on the pins of the car's protocol: it has
           no say about the other buses */
        char reason[176];

        if (!ap_guard_job_ok(reason, sizeof(reason)))
        {
            return refuse(ESP_ERR_INVALID_STATE, err, err_len, reason);
        }
    }

    return ESP_OK;
}

static bool job_acquire_wait(void)
{
    for (int ms = 0; ms <= SEND_JOB_WAIT_MS; ms += 20)
    {
        if (ap_core_job_acquire())
        {
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return false;
}

/* ---- the result ------------------------------------------------------------- */

/** A frame's `reply` and `replies` out of what the chip said to it, whole
 *  (s_resp, under the chip job): `reply` one line as before, `replies` one
 *  answer a line. */
static void add_chip_reply(cJSON *fo, const char *resp)
{
    char text[SEND_RESP_LEN];
    char line[SEND_LINE_LEN];

    if (ap_send_reply_join(resp, text, sizeof(text)) == 0)
    {
        return;
    }

    cJSON_AddStringToObject(fo, "reply", text);

    cJSON *lines = cJSON_AddArrayToObject(fo, "replies");

    if (lines == NULL)
    {
        return; /* no memory: `reply` carries the answer */
    }

    for (const char *p = ap_send_reply_next(resp, line, sizeof(line));
         p != NULL; p = ap_send_reply_next(p, line, sizeof(line)))
    {
        cJSON *entry = cJSON_CreateString(line);

        if (entry != NULL && !cJSON_AddItemToArray(lines, entry))
        {
            cJSON_Delete(entry);
        }
    }
}

static cJSON *build_reply(const ap_send_req_t *req, const send_out_t *out,
                          int64_t elapsed_us, const char *transcript)
{
    cJSON *o = cJSON_CreateObject();
    cJSON *frames = cJSON_AddArrayToObject(o, "frames");
    int sent = 0;

    for (int i = 0; i < req->n; i++)
    {
        const ap_send_frame_t *f = &req->f[i];
        cJSON *fo = cJSON_CreateObject();
        char text[32];

        snprintf(text, sizeof(text),
                 req->precan ? "%06lX" : f->ext ? "%08lX" : "%03lX",
                 (unsigned long)f->id);
        cJSON_AddStringToObject(fo, "id", text);
        cJSON_AddStringToObject(fo, "result",
                                ap_send_result_word(out[i].result));
        cJSON_AddNumberToObject(fo, "at_ms", out[i].at_ms);

        /* a chip send (transcript set) keeps what the chip said whole; the
           sentence of a frame the chip or the controller refused stays */
        if (transcript != NULL && s_resp[i][0] != '\0')
        {
            add_chip_reply(fo, s_resp[i]);
        }
        else if (out[i].reply[0] != '\0')
        {
            cJSON_AddStringToObject(fo, "reply", out[i].reply);
        }

        cJSON_AddItemToArray(frames, fo);
        sent += (out[i].result == AP_SEND_SENT) ? 1 : 0;
    }

    cJSON_AddBoolToObject(o, "ok", sent == req->n);
    cJSON_AddStringToObject(o, "bus", ap_send_where_word(req->where));
    cJSON_AddStringToObject(o, "via", (req->where == AP_SEND_CAN)
                                          ? "can_controller" : "obd_chip");
    cJSON_AddNumberToObject(o, "sent", sent);
    cJSON_AddNumberToObject(o, "elapsed_ms", (double)elapsed_us / 1000.0);

    if (transcript != NULL)
    {
        cJSON_AddStringToObject(o, "transcript", transcript);
    }

    return o;
}

esp_err_t autopid_can_send(const cJSON *request, cJSON **reply, char *err,
                           size_t err_len)
{
    ap_send_req_t req;
    send_out_t out[AP_SEND_FRAMES_MAX];
    int64_t t0 = esp_timer_get_time();
    bool chip = false;

    if (reply != NULL)
    {
        *reply = NULL;
    }

    if (err != NULL && err_len > 0)
    {
        err[0] = '\0';
    }

    if (!ap_send_parse(request, &req, err, err_len))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(out));

    /* A diagnostic tool's hold of the bus (obd_gate) is not asked here: it
       keeps the BACKGROUND off the bus, and the caller may be the holder
       itself (a script that spoke UDS first). The rule action asks it. */
    if (req.where == AP_SEND_CAN)
    {
        esp_err_t cerr = send_controller(&req, out, err, err_len);

        if (cerr != ESP_OK)
        {
            return cerr;
        }
    }
    else
    {
        if (!ap_core_started())
        {
            return refuse(ESP_ERR_INVALID_STATE, err, err_len,
                          "AutoPID is not up yet");
        }

        char proto = ap_base_proto();
        esp_err_t ferr = chip_free(&req, proto, err, err_len);

        if (ferr != ESP_OK)
        {
            return ferr;
        }

        req.precan = req.where == AP_SEND_OBD && ap_proto_precan(proto);

        if (!job_acquire_wait())
        {
            return refuse(ESP_ERR_INVALID_STATE, err, err_len,
                          "another chip job runs (a test, a scan)");
        }

        ap_tr_t tr = { s_tr, sizeof(s_tr), 0 };

        s_tr[0] = '\0';
        chip = true;
        ap_core_scan_pause(true); /* park the poller around the send */
        send_chip(&req, proto, out, &tr);
        ap_core_scan_pause(false);
    }

    int sent = 0;

    for (int i = 0; i < req.n; i++)
    {
        sent += (out[i].result == AP_SEND_SENT) ? 1 : 0;
    }

    ESP_LOGD(TAG, "can.send on %s: %d of %d frame(s) on the bus",
             ap_send_where_word(req.where), sent, req.n);

    if (reply != NULL)
    {
        *reply = build_reply(&req, out, esp_timer_get_time() - t0,
                             chip ? s_tr : NULL);
    }

    if (chip)
    {
        ap_core_job_release(); /* the transcript is copied by now */
    }

    if (sent != req.n && err != NULL && err_len > 0)
    {
        /* the first frame that did not make it says why */
        for (int i = 0; i < req.n; i++)
        {
            if (out[i].result != AP_SEND_SENT)
            {
                snprintf(err, err_len, "frame %d: %s%s%s", i + 1,
                         (out[i].result == AP_SEND_NO_ACK)
                             ? "nobody on the bus acknowledged it"
                             : "not sent",
                         (out[i].reply[0] != '\0') ? ": " : "", out[i].reply);
                break;
            }
        }
    }

    return (sent == req.n) ? ESP_OK : ESP_FAIL;
}
