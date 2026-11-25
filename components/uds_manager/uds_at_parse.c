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
 * @file uds_at_parse.c
 * @brief The PURE half of the obd_chip transport (host-tested, no I/O): the
 *        parser of the chip's printed answer, and what the transaction
 *        derives from an answer's size: its line count, the first
 *        response-count digit, the wait behind an answer that was cut.
 *        Moved out of uds_transport_at.c on 2026-11-26 (700-line rule).
 */
#include "uds_transport.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

/* ---- pure response parser --------------------------------------------------- */

static bool is_error_token(const char *tok, size_t len)
{
    /* ELM/MIC error phrases. Only match on a NON-hex char being present
     * (a pure hex+space line is data, never an error) so byte values
     * like FB/AC can't false-trigger. */
    bool has_alpha_word = false;

    for (size_t i = 0; i < len; i++)
    {
        char c = tok[i];

        if (c == '?')
        {
            return true; /* the ELM "did not understand" marker */
        }

        /* G-Z (letters that are NOT hex digits) => not a data line */
        char u = (char)toupper((unsigned char)c);

        if (u >= 'G' && u <= 'Z')
        {
            has_alpha_word = true;
        }
    }

    if (!has_alpha_word)
    {
        return false; /* pure hex/space/colon: it's data */
    }

    /* it has non-hex letters: confirm it's a known error phrase */
    static const char *ERR[] = { "NO DATA", "ERROR", "UNABLE", "BUFFER",
                                 "STOPPED", "SEARCHING", "BUS", "TIMEOUT",
                                 "RX", "TX", "CAN" };
    char up[32];
    size_t n = 0;

    for (size_t i = 0; i < len && n < sizeof(up) - 1; i++)
    {
        up[n++] = (char)toupper((unsigned char)tok[i]);
    }

    up[n] = '\0';

    for (size_t i = 0; i < sizeof(ERR) / sizeof(ERR[0]); i++)
    {
        if (strstr(up, ERR[i]) != NULL)
        {
            return true;
        }
    }

    /* has non-hex letters but no known phrase: treat as junk, reject */
    return true;
}

/* True if the line contains an ISO-TP frame index "<hex>:" (e.g. "0:"). */
static bool line_has_index(const char *p, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        if (p[i] == ':')
        {
            return true;
        }
    }

    return false;
}

/* Collect 2-hex-digit byte tokens from a line into out (after the ':'
 * if the line has a frame index). Returns false on overflow. @p keep is
 * the announced size of the message the line belongs to (0 = not known):
 * a byte behind it is the last frame's padding and is dropped. Until
 * 2026-11-26 the padding counted against @p cap before it was cut off, and
 * the largest message (4095 bytes, 4101 with its padding) overflowed the
 * 4096 bytes every caller has for it. */
static bool collect_line_bytes(const char *p, size_t len, uint8_t *out,
                               size_t cap, size_t *n, size_t keep)
{
    /* skip up to and including a frame-index ':' */
    for (size_t i = 0; i < len; i++)
    {
        if (p[i] == ':')
        {
            p += i + 1;
            len -= i + 1;
            break;
        }
    }

    char accum[3];
    size_t acc = 0;

    for (size_t i = 0; i <= len; i++)
    {
        char c = (i < len) ? p[i] : ' ';

        if (isxdigit((unsigned char)c))
        {
            accum[acc < 2 ? acc : 2] = c;
            acc = (acc < 2) ? acc + 1 : 3; /* >2 = not a byte pair */
            continue;
        }

        if (acc == 2 && keep > 0 && *n >= keep)
        {
            acc = 0; /* padding behind the message */
            continue;
        }

        if (acc == 2)
        {
            if (*n >= cap)
            {
                return false;
            }

            accum[2] = '\0';
            unsigned v = 0;
            sscanf(accum, "%2x", &v);
            out[(*n)++] = (uint8_t)v;
        }

        acc = 0;
    }

    return true;
}

uint8_t uds_at_lines_for(size_t total)
{
    /* first frame = 6 payload bytes, every consecutive frame 7 */
    if (total <= 7)
    {
        return 1;
    }

    size_t lines = 1 + (total - 6 + 6) / 7;

    return (lines > 255) ? 255 : (uint8_t)lines;
}

char uds_at_first_digit(uint8_t sid)
{
    /* "1" hands a single-frame answer over at once, and a longer answer is
     * cut and asked for AGAIN. Services that change the server's state and
     * may answer in more than one frame must not go out twice (a second
     * requestSeed, a second routine start): no digit, the chip prints every
     * frame and waits for more (bench 2026-10-02: +85 ms at ATST32, 7F xx
     * 78 lines are ridden out the same way). */
    switch (sid)
    {
    case 0x27: /* SecurityAccess          */
    case 0x29: /* Authentication          */
    case 0x31: /* RoutineControl          */
    case 0x38: /* RequestFileTransfer     */
    case 0x84: /* SecuredDataTransmission */
        return '\0';

    default:
        return '1';
    }
}

uint32_t uds_at_tail_wait_ms(size_t announced, size_t got, int cuts)
{
    /* frames of a message: the first carries 6 bytes, each further one 7 */
    size_t all = (announced <= 7) ? 1 : 1 + announced / 7;
    size_t had = (got <= 6) ? 1 : 1 + (got - 6) / 7;
    size_t left = (all > had) ? all - had : 0;

    /* The chip's flow control asks for no pause, the ECU takes its own time
     * per frame: 2 ms a frame and 10 ms on top the first time (the bench's
     * ECU takes 0.25 ms), four times that the second, an ECU of 8 ms a frame
     * is through by then. Capped: an answer of 4095 bytes is 585 frames. */
    uint32_t wait = 10 + 2 * (uint32_t)left;

    if (cuts > 0)
    {
        wait *= 4;
    }

    uint32_t cap = (cuts > 0) ? 3000u : 1200u;

    return (wait > cap) ? cap : wait;
}

/** The ISO-TP total of a multi-frame length line ("014"), 0 = not one. */
static size_t length_line_value(const char *p, size_t len)
{
    size_t v = 0;

    if (len == 0 || len > 4)
    {
        return 0;
    }

    for (size_t i = 0; i < len; i++)
    {
        if (!isxdigit((unsigned char)p[i]))
        {
            return 0;
        }

        char c = (char)toupper((unsigned char)p[i]);

        v = (v << 4) | (size_t)((c >= 'A') ? (c - 'A' + 10) : (c - '0'));
    }

    return v;
}

bool uds_at_parse_response_len(const char *resp, uint8_t *out, size_t cap,
                               size_t *out_len, uint8_t *pending_out,
                               size_t *announced_out)
{
    if (announced_out != NULL)
    {
        *announced_out = 0;
    }

    if (resp == NULL || out == NULL || out_len == NULL)
    {
        return false;
    }

    *out_len = 0;

    if (pending_out != NULL)
    {
        *pending_out = 0;
    }

    /* The chip prints one MESSAGE per line: a single frame is one line of
     * bytes; a multi-frame (ISO-TP) message is a hex length line followed
     * by indexed lines ("0: ..", "1: .."). With the response-count digit
     * the chip still prints every '7F xx 78' responsePending it saw before
     * the final answer (bench 2026-09-16, ELM327 v2.3 core): those are
     * counted and dropped, the LAST remaining message is the answer.
     * The length line is the message's size: the last frame's padding is
     * cut off, and fewer bytes than announced means the chip stopped
     * printing early (the response-count digit counts LINES; bench
     * 2026-10-02: every multi-frame answer came back as its first 6
     * bytes and was handed over as complete). */
    size_t n = 0;          /* bytes of the message being collected        */
    bool have = false;     /* a complete non-pending message sits in out  */
    bool in_multi = false; /* collecting indexed lines                    */
    size_t total = 0;      /* announced size of the multi-frame message   */
    size_t announced = 0;  /* ... of the message that sits in out         */
    uint8_t pending = 0;

    const char *p = resp;

    while (*p != '\0')
    {
        const char *eol = p;

        while (*eol != '\0' && *eol != '\r' && *eol != '\n') eol++;

        size_t len = (size_t)(eol - p);

        while (len > 0 && (p[len - 1] == ' ' || p[len - 1] == '>')) len--;
        while (len > 0 && *p == ' ') { p++; len--; }

        if (len > 0)
        {
            if (is_error_token(p, len))
            {
                return false;
            }

            if (line_has_index(p, len))
            {
                /* a frame of the current multi-frame message */
                if (!in_multi)
                {
                    in_multi = true; /* (no length line seen: tolerate) */
                    n = 0;
                    have = false;
                    total = 0;
                }

                if (!collect_line_bytes(p, len, out, cap, &n, total))
                {
                    return false;
                }
            }
            else
            {
                /* look ahead: is the next non-empty line indexed? then this
                   line is a multi-frame length prefix, not a message */
                const char *q = (*eol == '\0') ? eol : eol + 1;

                while (*q == '\r' || *q == '\n' || *q == ' ') q++;

                const char *qe = q;

                while (*qe != '\0' && *qe != '\r' && *qe != '\n') qe++;

                bool next_indexed = line_has_index(q, (size_t)(qe - q));

                /* a finished multi-frame message before this line */
                if (in_multi)
                {
                    in_multi = false;

                    if (total > 0 && n > total)
                    {
                        n = total; /* the last frame's padding */
                    }

                    have = (n > 0);
                    announced = total;
                }

                if (next_indexed)
                {
                    in_multi = true;
                    n = 0;
                    have = false;
                    total = length_line_value(p, len);
                }
                else
                {
                    /* one single-frame message on this line */
                    size_t m = 0;
                    uint8_t sf[8];

                    if (!collect_line_bytes(p, len, sf, sizeof(sf), &m, 0))
                    {
                        return false;
                    }

                    if (m == 3 && sf[0] == 0x7F && sf[2] == 0x78)
                    {
                        if (pending < 255) pending++; /* responsePending */
                    }
                    else if (m > 0)
                    {
                        if (m > cap)
                        {
                            return false;
                        }

                        memcpy(out, sf, m);
                        n = m;
                        have = true;
                        announced = 0; /* a single frame */
                    }
                }
            }
        }

        p = (*eol == '\0') ? eol : eol + 1;
    }

    if (in_multi)
    {
        if (total > 0 && n > total)
        {
            n = total;
        }

        have = (n > 0);
        announced = total;
    }

    if (pending_out != NULL)
    {
        *pending_out = pending;
    }

    if (!have)
    {
        n = 0;
        announced = 0;
    }

    if (announced_out != NULL)
    {
        *announced_out = announced;
    }

    *out_len = n;
    return have && n > 0;
}

bool uds_at_parse_response_ex(const char *resp, uint8_t *out, size_t cap,
                              size_t *out_len, uint8_t *pending_out)
{
    size_t announced = 0;

    if (!uds_at_parse_response_len(resp, out, cap, out_len, pending_out,
                                   &announced))
    {
        return false;
    }

    if (announced > *out_len)
    {
        *out_len = 0;   /* cut short: never a message */
        return false;
    }

    return true;
}

bool uds_at_parse_response(const char *resp, uint8_t *out, size_t cap,
                           size_t *out_len)
{
    return uds_at_parse_response_ex(resp, out, cap, out_len, NULL);
}
