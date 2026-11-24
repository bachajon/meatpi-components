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
 * @file autopid_send_core.c
 * @brief PURE half of can.send (autopid_send.h, host-tested): a request's
 *        JSON into frames, every check that needs no hardware, the words the
 *        OBD chip is given, and how its reply reads.
 */
#include "autopid_send.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "autopid_bus.h"
#include "autopid_proto.h"

static const char *const WHERE[AP_SEND_WHERE_COUNT] =
{
    [AP_SEND_CAN] = "can",
    [AP_SEND_OBD] = "obd",
    [AP_SEND_MS]  = "ms",
    [AP_SEND_SW]  = "sw",
    [AP_SEND_CH]  = "ch",
};

/* the bus of a row a place is (autopid_bus.h); AP_BUS_OBD for the two that
   are none */
static const uint8_t ROW_BUS[AP_SEND_WHERE_COUNT] =
{
    [AP_SEND_CAN] = AP_BUS_OBD,
    [AP_SEND_OBD] = AP_BUS_OBD,
    [AP_SEND_MS]  = AP_BUS_MS,
    [AP_SEND_SW]  = AP_BUS_SW,
    [AP_SEND_CH]  = AP_BUS_CH,
};

static bool fail(char *err, size_t err_len, const char *fmt, ...)
{
    if (err != NULL && err_len > 0)
    {
        va_list ap;

        va_start(ap, fmt);
        vsnprintf(err, err_len, fmt, ap);
        va_end(ap);
    }

    return false;
}

const char *ap_send_where_word(ap_send_where_t where)
{
    return (where < AP_SEND_WHERE_COUNT) ? WHERE[where] : "?";
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }

    c = (char)toupper((unsigned char)c);
    return (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
}

/* why hex text is refused (2026-11-21: one sentence said all three) */
#define HEX_HALF_BYTE "has half a byte: two hex digits a byte (\"80 01 FF\")"
#define HEX_NOT_HEX   "is hex digits (\"80 01 FF\") and has another character"
#define HEX_TOO_LONG  "is 8 bytes at most"

/** ap_send_hex() with the reason of a refusal in @p why (NULL = taken). */
static bool hex_scan(const char *text, uint8_t *out, size_t cap, size_t *n,
                     const char **why)
{
    size_t count = 0;
    int high = -1;

    *n = 0;
    *why = NULL;

    for (const char *p = (text != NULL) ? text : ""; *p != '\0'; p++)
    {
        if (*p == ' ' || *p == ',' || *p == '\t')
        {
            if (high >= 0)
            {
                *why = HEX_HALF_BYTE; /* before the separator */
                return false;
            }

            continue;
        }

        if (high < 0 && p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
        {
            p++; /* the prefix of one byte */
            continue;
        }

        int v = hex_val(*p);

        if (v < 0)
        {
            *why = HEX_NOT_HEX;
            return false;
        }

        if (high < 0)
        {
            high = v;
            continue;
        }

        if (count >= cap)
        {
            *why = HEX_TOO_LONG;
            return false;
        }

        out[count++] = (uint8_t)((high << 4) | v);
        high = -1;
    }

    if (high >= 0)
    {
        *why = HEX_HALF_BYTE; /* an odd number of digits */
        return false;
    }

    *n = count;
    return true;
}

bool ap_send_hex(const char *text, uint8_t *out, size_t cap, size_t *n)
{
    const char *why = NULL;

    return hex_scan(text, out, cap, n, &why);
}

/** An id as hex text ("7DF", "0x1024E097") or as a JSON number. */
static bool parse_id(const cJSON *v, uint32_t *id)
{
    if (cJSON_IsNumber(v))
    {
        double d = v->valuedouble;

        if (d < 0 || d > 0x1FFFFFFF || d != (double)(uint32_t)d)
        {
            return false; /* negative, too wide, or not a whole number */
        }

        *id = (uint32_t)d;
        return true;
    }

    if (!cJSON_IsString(v))
    {
        return false;
    }

    const char *s = v->valuestring;

    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
    {
        s += 2;
    }

    uint32_t out = 0;
    size_t digits = 0;

    for (; *s != '\0'; s++)
    {
        if (*s == ' ')
        {
            continue;
        }

        int h = hex_val(*s);

        if (h < 0 || ++digits > 8)
        {
            return false;
        }

        out = (out << 4) | (uint32_t)h;
    }

    if (digits == 0 || out > 0x1FFFFFFF)
    {
        return false;
    }

    *id = out;
    return true;
}

/* the keys of a frame, and of a request beside its frame or its frames
   (2026-11-21): a key outside them is refused. Until then it was ignored,
   and a request that meant something by it was answered ok: a "gap_ms"
   beside "frames", an "at_ms" or a "repeat" in a frame, each sent as if
   it were not there. */
static const char *const FRAME_KEYS[] =
{
    "id", "ext", "data", "wake", "gap_ms",
};

static const char *const REQ_KEYS[] =
{
    "bus", "bus_kbps", "frames",
};

static bool key_in(const char *key, const char *const *keys, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        if (strcmp(key, keys[i]) == 0)
        {
            return true;
        }
    }

    return false;
}

#define KEYS_N(k) (sizeof(k) / sizeof((k)[0]))

/** The first key of @p o that is neither a frame's (with @p frame) nor a
 *  request's (with @p req); NULL when there is none. */
static const char *unknown_key(const cJSON *o, bool frame, bool req)
{
    const cJSON *item = NULL;

    cJSON_ArrayForEach(item, o)
    {
        const char *k = (item->string != NULL) ? item->string : "";

        if (!(frame && key_in(k, FRAME_KEYS, KEYS_N(FRAME_KEYS))) &&
            !(req && key_in(k, REQ_KEYS, KEYS_N(REQ_KEYS))))
        {
            return k;
        }
    }

    return NULL;
}

/** One frame out of the object that carries it. @p n: its number in the
 *  request, for the sentence. */
static bool parse_frame(const cJSON *o, ap_send_where_t where, int n,
                        ap_send_frame_t *f, char *err, size_t err_len)
{
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(o, "id");
    const cJSON *ext = cJSON_GetObjectItemCaseSensitive(o, "ext");
    const cJSON *data = cJSON_GetObjectItemCaseSensitive(o, "data");
    const cJSON *wake = cJSON_GetObjectItemCaseSensitive(o, "wake");
    const cJSON *gap = cJSON_GetObjectItemCaseSensitive(o, "gap_ms");
    size_t len = 0;

    memset(f, 0, sizeof(*f));

    if (!parse_id(id, &f->id))
    {
        return fail(err, err_len, "frame %d: id is hex text (\"7DF\", "
                    "\"1024E097\") or a number, 0 to 1FFFFFFF", n);
    }

    if (ext != NULL && !cJSON_IsBool(ext))
    {
        return fail(err, err_len, "frame %d: ext is true or false", n);
    }

    f->ext = (ext != NULL) ? cJSON_IsTrue(ext) : (f->id > 0x7FF);

    if (!f->ext && f->id > 0x7FF)
    {
        return fail(err, err_len, "frame %d: id %lX does not fit 11 bits "
                    "(ext false)", n, (unsigned long)f->id);
    }

    if (data != NULL && !cJSON_IsString(data))
    {
        return fail(err, err_len, "frame %d: data is hex text (\"80 01 FF\")",
                    n);
    }

    const char *why = NULL;

    if (!hex_scan(cJSON_IsString(data) ? data->valuestring : "", f->data,
                  sizeof(f->data), &len, &why))
    {
        return fail(err, err_len, "frame %d: data %s", n, why);
    }

    f->len = (uint8_t)len;

    if (where != AP_SEND_CAN && f->len == 0)
    {
        return fail(err, err_len, "frame %d: the OBD chip sends no frame "
                    "without data (bus can does)", n);
    }

    if (wake != NULL && !cJSON_IsBool(wake))
    {
        return fail(err, err_len, "frame %d: wake is true or false", n);
    }

    f->wake = cJSON_IsTrue(wake);

    if (f->wake && where != AP_SEND_SW)
    {
        return fail(err, err_len, "frame %d: wake is the single-wire CAN "
                    "transceiver's high-voltage mode (bus sw alone)", n);
    }

    if (gap != NULL)
    {
        if (!cJSON_IsNumber(gap) || gap->valuedouble < 0 ||
            gap->valuedouble > AP_SEND_GAP_MAX_MS)
        {
            return fail(err, err_len, "frame %d: gap_ms is 0 to %d", n,
                        AP_SEND_GAP_MAX_MS);
        }

        f->gap_ms = (uint16_t)gap->valuedouble;
    }

    return true;
}

bool ap_send_parse(const cJSON *body, ap_send_req_t *out, char *err,
                   size_t err_len)
{
    const cJSON *bus = cJSON_GetObjectItemCaseSensitive(body, "bus");
    const cJSON *kbps = cJSON_GetObjectItemCaseSensitive(body, "bus_kbps");
    const cJSON *frames = cJSON_GetObjectItemCaseSensitive(body, "frames");
    bool single = cJSON_GetObjectItemCaseSensitive(body, "id") != NULL ||
                  cJSON_GetObjectItemCaseSensitive(body, "data") != NULL;

    memset(out, 0, sizeof(*out));

    if (!cJSON_IsObject(body))
    {
        return fail(err, err_len, "a JSON object is expected");
    }

    int where = -1;

    for (int w = 0; cJSON_IsString(bus) && w < AP_SEND_WHERE_COUNT; w++)
    {
        if (strcmp(bus->valuestring, WHERE[w]) == 0)
        {
            where = w;
        }
    }

    if (where < 0)
    {
        return fail(err, err_len, "bus is one of can (WiCAN's CAN controller)"
                    ", obd (the car's OBD protocol), ms, sw, ch");
    }

    out->where = (ap_send_where_t)where;

    /* 0 = the bus's own speed, as in a row */
    if (kbps != NULL && !(cJSON_IsNumber(kbps) && kbps->valuedouble == 0))
    {
        if (ROW_BUS[where] == AP_BUS_OBD || !cJSON_IsNumber(kbps) ||
            kbps->valuedouble < 0 ||
            !ap_bus_rate_parse(ROW_BUS[where], kbps->valuedouble, &out->rate))
        {
            return fail(err, err_len, "bus_kbps needs bus ms, sw or ch and a "
                        "speed the chip can make (500 over 1 to 64, or that "
                        "times 8/7)");
        }
    }

    if (frames != NULL && single)
    {
        return fail(err, err_len, "give id and data, or frames, not both");
    }

    /* a key that would mean nothing is refused, with the keys that do */
    const char *bad = unknown_key(body, frames == NULL, true);

    if (bad != NULL && frames != NULL &&
        key_in(bad, FRAME_KEYS, KEYS_N(FRAME_KEYS)))
    {
        return fail(err, err_len, "%s is a key of each frame in frames", bad);
    }

    if (bad != NULL)
    {
        return fail(err, err_len, "unknown key \"%s\" (%s)", bad,
                    (frames != NULL)
                        ? "bus, bus_kbps, frames"
                        : "bus, bus_kbps, id, ext, data, wake, gap_ms");
    }

    if (frames == NULL)
    {
        out->n = 1;
        return parse_frame(body, out->where, 1, &out->f[0], err, err_len);
    }

    int count = cJSON_IsArray(frames) ? cJSON_GetArraySize(frames) : 0;

    if (count < 1 || count > AP_SEND_FRAMES_MAX)
    {
        return fail(err, err_len, "frames is a list of 1 to %d frames",
                    AP_SEND_FRAMES_MAX);
    }

    const cJSON *item = NULL;
    uint32_t total = 0;

    cJSON_ArrayForEach(item, frames)
    {
        bad = cJSON_IsObject(item) ? unknown_key(item, true, false) : NULL;

        if (bad != NULL)
        {
            return fail(err, err_len, "frame %d: unknown key \"%s\" (id, ext, "
                        "data, wake, gap_ms)", out->n + 1, bad);
        }

        if (!cJSON_IsObject(item) ||
            !parse_frame(item, out->where, out->n + 1, &out->f[out->n], err,
                         err_len))
        {
            return cJSON_IsObject(item)
                       ? false
                       : fail(err, err_len, "frame %d: an object is expected",
                              out->n + 1);
        }

        total += out->f[out->n].gap_ms;
        out->n++;
    }

    if (total > AP_SEND_TOTAL_MAX_MS)
    {
        return fail(err, err_len, "the gaps add up to %lu ms, %d at most",
                    (unsigned long)total, AP_SEND_TOTAL_MAX_MS);
    }

    return true;
}

bool ap_send_check_car(const ap_send_req_t *req, char proto, char *err,
                       size_t err_len)
{
    if (req->where != AP_SEND_OBD)
    {
        return true;
    }

    const ap_proto_t *p = ap_proto(proto);

    if (p == NULL)
    {
        return fail(err, err_len, "the car's OBD protocol is not known yet: "
                    "no car was found, and the protocol setting is "
                    "Automatic");
    }

    if (p->kind == AP_PK_CAN || p->kind == AP_PK_J1939)
    {
        uint8_t code = 0;

        return (p->bps > 0 &&
                ap_bus_rate_parse(AP_BUS_HS, (double)p->bps / 1000.0, &code))
                   ? true
                   : fail(err, err_len, "no bit rate for protocol %c", proto);
    }

    if (!ap_proto_precan(proto))
    {
        return fail(err, err_len, "bus obd cannot send on protocol %c (%s)",
                    proto, p->name);
    }

    for (int i = 0; i < req->n; i++)
    {
        const ap_send_frame_t *f = &req->f[i];

        if (f->id > 0xFFFFFF)
        {
            return fail(err, err_len, "frame %d: this car's OBD protocol is "
                        "%s, where the id is the message's three header "
                        "bytes (six hex digits)", i + 1, p->name);
        }

        if (f->len < 1 || f->len > AP_SEND_PRECAN_MAX)
        {
            return fail(err, err_len, "frame %d: a message on %s carries 1 "
                        "to %d data bytes", i + 1, p->name,
                        AP_SEND_PRECAN_MAX);
        }
    }

    return true;
}

uint16_t ap_send_key(const ap_send_req_t *req, int i, char proto)
{
    const ap_send_frame_t *f = &req->f[i];
    uint16_t flags = (uint16_t)(f->ext ? AP_BUS_KEY_EXT : 0u);

    switch (req->where)
    {
        case AP_SEND_MS:
        case AP_SEND_CH:
            return (uint16_t)(AP_BUS_KEY(ROW_BUS[req->where], req->rate) |
                              flags);

        case AP_SEND_SW:
            return (uint16_t)(AP_BUS_KEY(AP_BUS_SW, req->rate) | flags |
                              (f->wake ? AP_BUS_KEY_WAKE : 0u));

        case AP_SEND_OBD:
        {
            const ap_proto_t *p = ap_proto(proto);
            uint8_t code = 0;

            if (p == NULL || (p->kind != AP_PK_CAN && p->kind != AP_PK_J1939))
            {
                return 0; /* a car before CAN: on its own protocol */
            }

            (void)ap_bus_rate_parse(AP_BUS_HS, (double)p->bps / 1000.0,
                                    &code);
            return (uint16_t)(AP_BUS_KEY(AP_BUS_HS, code) | flags);
        }

        default:
            return 0;
    }
}

const char *ap_send_setup(bool precan)
{
    /* ATCAF0: the bytes as given, no ISO 15765 length byte in front.
       ATV1: the data length as given, not padded to eight. ATAT0 + ATST19:
       100 ms for an answer nobody asked for (bench 2026-11-21: the chip
       waits that long at least, and a frame nobody acknowledges reads CAN
       ERROR after 60 ms). */
    return precan ? "ATAT0;ATST19" : "ATCAF0;ATV1;ATAT0;ATST19";
}

size_t ap_send_header(const ap_send_frame_t *f, bool precan, char *out,
                      size_t cap)
{
    int n;

    if (precan)
    {
        n = snprintf(out, cap, "ATSH%06lX", (unsigned long)(f->id & 0xFFFFFF));
    }
    else if (f->ext)
    {
        /* the classic pair: the five bits above the three header bytes, then
           those. Every firmware of the chip takes it. */
        n = snprintf(out, cap, "ATCP%02lX;ATSH%06lX",
                     (unsigned long)((f->id >> 24) & 0x1F),
                     (unsigned long)(f->id & 0xFFFFFF));
    }
    else
    {
        n = snprintf(out, cap, "ATSH%03lX", (unsigned long)(f->id & 0x7FF));
    }

    if (n < 0 || (size_t)n >= cap)
    {
        if (cap > 0)
        {
            out[0] = '\0';
        }

        return 0;
    }

    return (size_t)n;
}

size_t ap_send_data(const ap_send_frame_t *f, char *out, size_t cap)
{
    size_t n = 0;

    if (cap == 0)
    {
        return 0;
    }

    for (uint8_t i = 0; i < f->len && n + 3 <= cap; i++)
    {
        n += (size_t)snprintf(out + n, cap - n, "%02X", f->data[i]);
    }

    out[n] = '\0';
    return n;
}

ap_send_result_t ap_send_classify(const char *reply)
{
    static const char *const REFUSED[] =
    {
        "ERROR", "UNABLE", "BUSY", "STOPPED", "BUFFER FULL", "ERR", "?",
        "LV RESET", "ACT ALERT",
    };

    if (reply == NULL)
    {
        return AP_SEND_ERROR;
    }

    if (strstr(reply, "CAN ERROR") != NULL)
    {
        return AP_SEND_NO_ACK;
    }

    for (size_t i = 0; i < sizeof(REFUSED) / sizeof(REFUSED[0]); i++)
    {
        if (strstr(reply, REFUSED[i]) != NULL)
        {
            return AP_SEND_ERROR;
        }
    }

    /* NO DATA (it went out, nobody answered, which is what a send expects)
       or an answer that came within the wait */
    for (const char *p = reply; *p != '\0'; p++)
    {
        if (!isspace((unsigned char)*p) && *p != '>')
        {
            return AP_SEND_SENT;
        }
    }

    return AP_SEND_ERROR; /* nothing at all */
}

const char *ap_send_result_word(ap_send_result_t r)
{
    switch (r)
    {
        case AP_SEND_SENT:
            return "sent";

        case AP_SEND_NO_ACK:
            return "no_ack";

        case AP_SEND_NOT_SENT:
            return "not_sent";

        default:
            return "error";
    }
}
