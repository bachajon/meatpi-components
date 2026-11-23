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
 * @file data_logger_cursor.c
 * @brief The export route's pure rules (2026-11-06, store and forward):
 *        where in a stream's file list a GET /api/logger/export request
 *        starts for its `since` value (and, since 2026-11-21, how that
 *        value is read off the query string), which lines of a file window go
 *        out (the name filter, the clock-step marker) and how a window is
 *        walked against the page's budgets. Host-tested with the file-name
 *        rules; data_logger_export.c applies them. Kept apart from
 *        data_logger_files.c, which sits at the 700-line rule.
 */
#include <stdint.h>
#include <string.h>

#include "data_logger_private.h"

/* A query value as a client sends it. A browser's encodeURIComponent and a
 * script's URL library write the cursor's colon as %3A and a space in a
 * parameter name as %20; httpd hands the value over as it came. Until
 * 2026-11-22 the route read it raw: an encoded cursor was a plain epoch
 * (the first page again, for ever), an encoded name matched no row. A '%'
 * that two hex digits do not follow stays as it is. */
static int hex_digit(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }

    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }

    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }

    return -1;
}

void dl_export_unescape(char *s)
{
    char *out = s;

    if (s == NULL)
    {
        return;
    }

    while (*s != '\0')
    {
        int hi = (s[0] == '%') ? hex_digit(s[1]) : -1;
        int lo = (hi >= 0) ? hex_digit(s[2]) : -1;

        if (lo >= 0 && (hi != 0 || lo != 0)) /* never a NUL into the text */
        {
            *out++ = (char)(hi * 16 + lo);
            s += 3;
        }
        else
        {
            *out++ = *s++;
        }
    }

    *out = '\0';
}

int dl_export_start(const int64_t *epochs, int n, int64_t since_epoch,
                    bool exact, long *off)
{
    if (since_epoch <= 0 || n <= 0)
    {
        *off = 0;
        return 0;
    }

    if (exact)
    {
        int idx = 0;

        while (idx < n && epochs[idx] < since_epoch)
        {
            idx++;
        }

        if (idx >= n || epochs[idx] != since_epoch)
        {
            *off = 0; /* the file is gone (retired) or never existed */
        }

        return idx;
    }

    /* the newest file opened at or before the moment; the oldest when the
       moment predates every file (the rows the client asks for may sit in
       any of them) */
    int idx = 0;

    for (int i = 0; i < n; i++)
    {
        if (epochs[i] <= since_epoch)
        {
            idx = i;
        }
    }

    *off = 0;
    return idx;
}

/* one run of decimal digits at *p (at most 18: no int64 overflow);
   false when there is none or it is longer */
static bool since_digits(const char **p, int64_t *out)
{
    int64_t v = 0;
    int n = 0;

    while (**p >= '0' && **p <= '9')
    {
        if (++n > 18)
        {
            return false;
        }

        v = v * 10 + (**p - '0');
        (*p)++;
    }

    *out = v;
    return n > 0;
}

/* `since` as it came off the query string (2026-11-21). The HTTP server
 * hands a query value over still percent-encoded, and a client that
 * encodes its query (Python's urlencode, JavaScript's encodeURIComponent)
 * sends the cursor's colon as "%3A". The route looked for a ':' in the raw
 * value, found none, read the cursor as a plain epoch and started over at
 * that file: the same first page and the same _cursor again, so a client
 * that followed _cursor looped forever (seen on a WiCAN PRO, 6.0.0-beta.12
 * and beta.14: one backfill read the same rows about 600 times). Now
 * the value is decoded first and must be one of the two forms; anything
 * else is refused (400) instead of guessed at. */
bool dl_export_parse_since(const char *raw, int64_t *epoch, long *off,
                           bool *exact)
{
    char buf[48];
    size_t o = 0;

    *epoch = 0;
    *off = 0;
    *exact = false;

    for (const char *p = raw; *p != '\0'; p++)
    {
        char c = *p;

        if (c == '%')
        {
            int hi = hex_digit(p[1]);
            int lo = (hi < 0) ? -1 : hex_digit(p[2]);

            if (lo < 0)
            {
                return false;
            }

            c = (char)((hi << 4) | lo);
            p += 2;

            if (c == '\0')
            {
                return false; /* "%00": would end the value early */
            }
        }

        if (o + 1 >= sizeof(buf))
        {
            return false;
        }

        buf[o++] = c;
    }

    buf[o] = '\0';

    if (o == 0)
    {
        return true; /* "since=": nothing, the oldest file */
    }

    const char *p = buf;
    int64_t e = 0;
    int64_t f = 0;
    bool cursor = false;

    if (!since_digits(&p, &e))
    {
        return false;
    }

    if (*p == ':')
    {
        p++;

        /* a file offset; bounded to what a 32-bit long holds on the
           device */
        if (!since_digits(&p, &f) || f > INT32_MAX)
        {
            return false;
        }

        cursor = true;
    }

    if (*p != '\0')
    {
        return false; /* outputs untouched: still 0 / false */
    }

    if (!cursor && e > 100000000000LL)
    {
        e /= 1000; /* milliseconds given: a client's watermark */
    }

    *epoch = e;
    *off = (long)f;
    *exact = cursor;
    return true;
}

/* ?name=<param> filter (dashboard history, 2026-09-06): a record's
 * "param":"<source>.<name>" matches on the full value or on the part
 * after its last '.', so the UI can ask by parameter name without knowing
 * the sink's source prefix. Plain substring scan: no JSON parse. */
bool dl_export_line_matches(const char *line, size_t len,
                                const char *name, bool csv)
{
    static const char KEY[] = "\"param\":\"";
    const size_t klen = sizeof(KEY) - 1;
    const char *p = NULL;
    const char *q = NULL;

    if (csv)
    {
        /* "ts_ms,source.name,value": the param is the second field (the
         * header row "ts_ms,param,value" never matches a real name) */
        p = memchr(line, ',', len);

        if (p == NULL)
        {
            return false;
        }

        p++;
        q = memchr(p, ',', len - (size_t)(p - line));
    }
    else
    {
        for (size_t i = 0; i + klen <= len; i++)
        {
            if (line[i] == '"' && memcmp(line + i, KEY, klen) == 0)
            {
                p = line + i + klen;
                break;
            }
        }

        if (p == NULL)
        {
            return false;
        }

        q = memchr(p, '"', len - (size_t)(p - line));
    }

    if (q == NULL)
    {
        return false;
    }

    size_t vlen = (size_t)(q - p);
    size_t nlen = strlen(name);

    if (vlen == nlen)
    {
        return memcmp(p, name, nlen) == 0;
    }

    return vlen > nlen && p[vlen - nlen - 1] == '.' &&
           memcmp(p + vlen - nlen, name, nlen) == 0;
}

/* the in-band clock-step marker (2026-10-13): not a record, never a point
 * for a consumer of this route (the journal and the file itself keep it) */
bool dl_export_line_is_marker(const char *line, size_t len, bool csv)
{
    static const char JSONL[] = "\"event\":\"clock_step\"";
    static const char CSV[] = ",logger.clock_step,";
    const char *key = csv ? CSV : JSONL;
    size_t klen = (csv ? sizeof(CSV) : sizeof(JSONL)) - 1;

    for (size_t i = 0; i + klen <= len; i++)
    {
        if (line[i] == key[0] && memcmp(line + i, key, klen) == 0)
        {
            return true;
        }
    }

    return false;
}

size_t dl_export_walk(const char *buf, size_t n, const char *name, bool csv,
                      int limit, int *lines_left, int *bytes_left,
                      dl_export_send_fn send, void *ctx, bool *stop,
                      bool *aborted)
{
    size_t consumed = 0;

    *stop = false;

    for (size_t i = 0; i < n && !*stop; i++)
    {
        if (buf[i] != '\n')
        {
            continue;
        }

        const char *line = buf + consumed;
        size_t len = i + 1 - consumed;

        if (!dl_export_line_is_marker(line, len, csv) &&
            (name[0] == '\0' || dl_export_line_matches(line, len, name, csv)))
        {
            /* the first line of a request always goes out, however small
               the budget; a later one that does not fit opens the next
               request instead (nothing consumed: the caller must not read
               that as a window without a line boundary, run 5) */
            if ((int)len > *bytes_left && *lines_left < limit)
            {
                *stop = true;
                break;
            }

            if (!send(ctx, line, len))
            {
                *aborted = true;
                *stop = true;
                break;
            }

            *bytes_left -= (int)len;
            (*lines_left)--;
        }

        consumed = i + 1;

        if (*lines_left == 0 || *bytes_left <= 0)
        {
            *stop = true;
        }
    }

    return consumed;
}
