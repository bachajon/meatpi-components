/*
 * This file is part of the MeatPi components project.
 * Copyright (C) 2022-2026 MeatPi Electronics.
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
 * @file script_engine_flash_bind.c
 * @brief The flash_* bindings: bytes in, bytes out, so a script can run an
 *        ECU's programming sequence (UDS 34/36/37 or a native loader) with
 *        the per-frame work in C. Documented in script_engine_doc_flash.c.
 *
 * Everything that transmits needs the allow_reflash setting and takes the
 * exclusive-bus hold. flash_commit() marks the point of no return: from
 * then the device is locked (script_engine_flash.c) and the run's
 * max_runtime_ms no longer applies.
 */
#include "script_engine_flash.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "script_engine_private.h"

#define FL_ERR        "flash_error"

static void need_reflash(bvm *vm)
{
    se_check_budget(vm);

    if (!se_settings_allow_reflash())
    {
        be_raise(vm, FL_ERR, "needs Allow reflash (script_engine.allow_reflash)");
    }

    se_bus_hold();
}

const uint8_t *se_fl_bytes(bvm *vm, int idx, size_t *len)
{
    if (be_top(vm) < idx || !be_isbytes(vm, idx))
    {
        be_raise(vm, FL_ERR, "expected bytes");
    }

    return be_tobytes(vm, idx, len);
}

uint32_t se_fl_int(bvm *vm, int idx, uint32_t def)
{
    return (be_top(vm) >= idx && be_isint(vm, idx)) ? (uint32_t)be_toint(vm, idx)
                                                    : def;
}

const char *se_fl_str(bvm *vm, int idx, const char *what)
{
    if (be_top(vm) < idx || !be_isstring(vm, idx))
    {
        be_raise(vm, FL_ERR, what);
    }

    return be_tostring(vm, idx);
}

/* opts[key] from a map at @p idx (0: none), as an int; @p def if absent. */
static bint opt_int(bvm *vm, int idx, const char *key, bint def)
{
    bint v = def;

    if (idx > 0 && be_top(vm) >= idx && be_ismapinstance(vm, idx))
    {
        be_getmember(vm, idx, ".p");
        be_pushstring(vm, key);
        be_getindex(vm, -2);

        if (be_isint(vm, -1))
        {
            v = be_toint(vm, -1);
        }
        else if (be_isbool(vm, -1))
        {
            v = be_tobool(vm, -1) ? 1 : 0;
        }

        be_pop(vm, 3);
    }

    return v;
}

/* A list of CAN ids at @p idx into @p ids; returns the count. */
static size_t ids_arg(bvm *vm, int idx, uint32_t *ids, size_t cap)
{
    size_t n = 0;

    if (be_top(vm) < idx || !be_islistinstance(vm, idx))
    {
        return 0;
    }

    be_getmember(vm, idx, ".p");

    int size = be_data_size(vm, -1);

    for (int i = 0; i < size && n < cap; i++)
    {
        be_pushint(vm, i);
        be_getindex(vm, -2);

        if (be_isint(vm, -1))
        {
            ids[n++] = (uint32_t)be_toint(vm, -1);
        }

        be_pop(vm, 2);
    }

    be_pop(vm, 1);
    return n;
}

static int handle_arg(bvm *vm)
{
    if (be_top(vm) < 1 || !be_isint(vm, 1))
    {
        be_raise(vm, FL_ERR, "expected a session handle");
    }

    return (int)be_toint(vm, 1);
}

/* ---- ISO-TP sessions ---------------------------------------------------------- */

/* flash_isotp(tx, rx[, {ext, ae_tx, ae_rx, pad, bs, stmin, gap_us}]) -> h */
static int b_isotp(bvm *vm)
{
    need_reflash(vm);

    if (be_top(vm) < 2 || !be_isint(vm, 1) || !be_isint(vm, 2))
    {
        be_raise(vm, FL_ERR, "flash_isotp(tx_id, rx_id[, opts])");
    }

    se_tp_opts_t o;

    se_tp_opts_default(&o);

    bint pad = opt_int(vm, 3, "pad", 0);

    o.ae_tx = (int16_t)opt_int(vm, 3, "ae_tx", -1);
    o.ae_rx = (int16_t)opt_int(vm, 3, "ae_rx", -1);
    o.pad = pad >= 0;
    o.pad_byte = (uint8_t)((pad >= 0) ? pad : 0);
    o.bs = (uint8_t)opt_int(vm, 3, "bs", 0);
    o.stmin = (uint8_t)opt_int(vm, 3, "stmin", 0);

    int h = se_tp_open((uint32_t)be_toint(vm, 1), (uint32_t)be_toint(vm, 2),
                       opt_int(vm, 3, "ext", 0) != 0, &o,
                       (uint32_t)opt_int(vm, 3, "gap_us", 0));

    if (h < 0)
    {
        be_raise(vm, FL_ERR, "no free session (4 at most) or the bus is down");
    }

    be_pushint(vm, h);
    be_return(vm);
}

static int b_close(bvm *vm)
{
    se_tp_close(handle_arg(vm));
    be_return_nil(vm);
}

/* flash_send(h, bytes[, timeout_ms]) -> true */
static int b_send(bvm *vm)
{
    need_reflash(vm);

    int h = handle_arg(vm);
    size_t n = 0;
    const uint8_t *d = se_fl_bytes(vm, 2, &n);
    esp_err_t err = se_tp_send(h, d, n, se_fl_int(vm, 3, 1000));

    if (err != ESP_OK)
    {
        be_raise(vm, FL_ERR, esp_err_to_name(err));
    }

    be_pushbool(vm, true);
    be_return(vm);
}

/* flash_recv(h[, timeout_ms]) -> bytes, or nil */
static int b_recv(bvm *vm)
{
    se_check_budget(vm);

    int h = handle_arg(vm);
    const uint8_t *d = NULL;
    size_t n = 0;
    esp_err_t err = se_tp_recv(h, &d, &n, se_fl_int(vm, 2, 1000));

    if (err == ESP_ERR_TIMEOUT)
    {
        be_return_nil(vm);
    }

    if (err != ESP_OK)
    {
        be_raise(vm, FL_ERR, esp_err_to_name(err));
    }

    be_pushbytes(vm, d, n);
    be_return(vm);
}

/* flash_request(h, bytes[, timeout_ms[, {pending_ms, retries, raw}]])
 * -> the reply; waits through 7F xx 78; a 7F reply raises "nrc" unless raw;
 * nil when nothing came (after the retries). */
static int b_request(bvm *vm)
{
    need_reflash(vm);

    int h = handle_arg(vm);
    size_t n = 0;
    const uint8_t *d = se_fl_bytes(vm, 2, &n);
    uint32_t timeout = se_fl_int(vm, 3, 1000);
    uint32_t pending = (uint32_t)opt_int(vm, 4, "pending_ms", 5000);
    int retries = (int)opt_int(vm, 4, "retries", 0);
    bool raw = opt_int(vm, 4, "raw", 0) != 0;

    for (int attempt = 0; attempt <= retries; attempt++)
    {
        se_tp_flush(h);

        esp_err_t err = se_tp_send(h, d, n, timeout);

        if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
        {
            be_raise(vm, FL_ERR, esp_err_to_name(err));
        }

        const uint8_t *r = NULL;
        size_t rn = 0;
        uint32_t wait = timeout;

        while (err == ESP_OK &&
               (err = se_tp_recv(h, &r, &rn, wait)) == ESP_OK)
        {
            bool pending_reply = rn >= 3 && r[0] == 0x7F && r[2] == 0x78;

            if (!pending_reply)
            {
                if (!raw && rn >= 3 && r[0] == 0x7F)
                {
                    char msg[32];

                    snprintf(msg, sizeof(msg), "NRC 0x%02X for 0x%02X", r[2], r[1]);
                    be_raise(vm, "nrc", msg);
                }

                be_pushbytes(vm, r, rn);
                be_return(vm);
            }

            wait = pending;
            se_check_budget(vm);
        }

        if (err != ESP_ERR_TIMEOUT)
        {
            be_raise(vm, FL_ERR, esp_err_to_name(err));
        }
    }

    be_return_nil(vm);
}

/* ---- raw frames ---------------------------------------------------------------- */

/* flash_can_send(id, bytes[, ext]) -> true */
static int b_can_send(bvm *vm)
{
    need_reflash(vm);

    if (be_top(vm) < 2 || !be_isint(vm, 1))
    {
        be_raise(vm, FL_ERR, "flash_can_send(id, bytes[, ext])");
    }

    size_t n = 0;
    const uint8_t *d = se_fl_bytes(vm, 2, &n);
    bool ext = be_top(vm) >= 3 && be_tobool(vm, 3);

    if (n > 8)
    {
        be_raise(vm, FL_ERR, "a frame carries 8 bytes at most");
    }

    esp_err_t err = se_raw_send((uint32_t)be_toint(vm, 1), ext, d, (uint8_t)n);

    if (err != ESP_OK)
    {
        be_raise(vm, FL_ERR, esp_err_to_name(err));
    }

    be_pushbool(vm, true);
    be_return(vm);
}

/* flash_can_recv(timeout_ms[, [ids]]) -> [id, bytes], or nil */
static int b_can_recv(bvm *vm)
{
    se_check_budget(vm);

    uint32_t ids[16];
    size_t n = ids_arg(vm, 2, ids, 16);
    can_core_frame_t f;

    if (se_raw_recv(ids, n, se_fl_int(vm, 1, 1000), &f) != ESP_OK)
    {
        be_return_nil(vm);
    }

    be_newobject(vm, "list");
    be_pushint(vm, (bint)f.id);
    be_data_push(vm, -2);
    be_pop(vm, 1);
    be_pushbytes(vm, f.data, f.dlc);
    be_data_push(vm, -2);
    be_pop(vm, 1);
    be_pop(vm, 1);
    be_return(vm);
}

/* flash_can_quiet(window_ms, timeout_ms[, [ids]]) -> bool */
static int b_can_quiet(bvm *vm)
{
    se_check_budget(vm);

    uint32_t ids[16];
    size_t n = ids_arg(vm, 3, ids, 16);

    be_pushbool(vm, se_raw_quiet(ids, n, se_fl_int(vm, 1, 1000),
                                 se_fl_int(vm, 2, 10000)));
    be_return(vm);
}

/* ---- the run ------------------------------------------------------------------- */

static int b_commit(bvm *vm)
{
    need_reflash(vm);

    const char *label = (be_top(vm) >= 1 && be_isstring(vm, 1))
                            ? be_tostring(vm, 1) : "flash";

    if (se_flash_commit(label) != ESP_OK)
    {
        be_raise(vm, FL_ERR, "the commit could not be recorded: stop here");
    }

    be_pushbool(vm, true);
    be_return(vm);
}

static int b_abort_requested(bvm *vm)
{
    be_pushbool(vm, se_flash_abort_requested());
    be_return(vm);
}

static int b_state(bvm *vm)
{
    const char *s = se_fl_str(vm, 1, "flash_state(\"untouched\"|\"loader\"|\"unknown\")");

    if (strcmp(s, "untouched") != 0 && strcmp(s, "loader") != 0 &&
        strcmp(s, "unknown") != 0)
    {
        be_raise(vm, FL_ERR, "state is untouched, loader or unknown");
    }

    se_flash_set_ecu_state(s);
    be_return_nil(vm);
}

static int b_phase(bvm *vm)
{
    se_check_budget(vm);
    se_flash_set_phase(se_fl_str(vm, 1, "flash_phase(name)"));
    be_return_nil(vm);
}

static int b_progress(bvm *vm)
{
    se_check_budget(vm);
    se_flash_set_progress(se_fl_int(vm, 1, 0), se_fl_int(vm, 2, 0));
    be_return_nil(vm);
}

static int b_log(bvm *vm)
{
    se_flash_log(se_fl_str(vm, 1, "flash_log(msg)"));
    be_return_nil(vm);
}

typedef struct
{
    bvm *vm;
} ask_ctx_t;

static bool ask_alive(void *ctx)
{
    (void)ctx;
    return !se_flash_abort_requested() && !se_run_killed();
}

static int b_ask(bvm *vm)
{
    se_check_budget(vm);

    const char *text = se_fl_str(vm, 1, "flash_ask(text[, choices])");
    const char *choices = (be_top(vm) >= 2 && be_isstring(vm, 2))
                              ? be_tostring(vm, 2) : "";
    char answer[96];
    ask_ctx_t ctx = { vm };

    if (se_flash_ask(text, choices, ask_alive, &ctx, answer,
                     sizeof(answer)) != ESP_OK)
    {
        se_check_budget(vm); /* raises when killed */
        be_raise(vm, FL_ERR, "prompt abandoned: the run was asked to stop");
    }

    be_pushstring(vm, answer);
    be_return(vm);
}

static int b_param(bvm *vm)
{
    const char *name = se_fl_str(vm, 1, "flash_param(name)");
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(se_flash_params(), name);

    if (cJSON_IsString(v))
    {
        be_pushstring(vm, v->valuestring);
    }
    else if (cJSON_IsBool(v))
    {
        be_pushbool(vm, cJSON_IsTrue(v));
    }
    else if (cJSON_IsNumber(v))
    {
        be_pushint(vm, (bint)v->valuedouble);
    }
    else
    {
        be_pushnil(vm);
    }

    be_return(vm);
}

static int b_micros(bvm *vm)
{
    be_pushint(vm, (bint)esp_timer_get_time());
    be_return(vm);
}

/* ---- registration -------------------------------------------------------------- */

static const struct
{
    const char *name;
    bntvfunc    fn;
} FLASH_BINDINGS[] =
{
    { "flash_isotp",           b_isotp },
    { "flash_send",            b_send },
    { "flash_recv",            b_recv },
    { "flash_request",         b_request },
    { "flash_close",           b_close },
    { "flash_can_send",        b_can_send },
    { "flash_can_recv",        b_can_recv },
    { "flash_can_quiet",       b_can_quiet },
    { "flash_commit",          b_commit },
    { "flash_abort_requested", b_abort_requested },
    { "flash_state",           b_state },
    { "flash_phase",           b_phase },
    { "flash_progress",        b_progress },
    { "flash_log",             b_log },
    { "flash_ask",             b_ask },
    { "flash_param",           b_param },
    { "flash_micros",          b_micros },
    { "flash_file_size",       se_fb_file_size },
    { "flash_file_read",       se_fb_file_read },
    { "flash_file_write",      se_fb_file_write },
    { "flash_file_sha256",     se_fb_file_sha256 },
    { "flash_crc",             se_fb_crc },
    { "flash_adler32",         se_fb_adler32 },
    { "flash_sum",             se_fb_sum },
    { "flash_sha256",          se_fb_sha256 },
};

#define N_FLASH (sizeof(FLASH_BINDINGS) / sizeof(FLASH_BINDINGS[0]))

void se_flash_bindings_register(bvm *vm)
{
    for (size_t i = 0; i < N_FLASH; i++)
    {
        be_pushntvfunction(vm, FLASH_BINDINGS[i].fn);
        be_setglobal(vm, FLASH_BINDINGS[i].name);
        be_pop(vm, 1);
    }
}

size_t se_flash_binding_count(void)
{
    return N_FLASH;
}

const char *se_flash_binding_name(size_t i)
{
    return (i < N_FLASH) ? FLASH_BINDINGS[i].name : NULL;
}
