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
 * @file autopid_resp.c
 * @brief PURE: ELM327 response text -> bytes (host-tested).
 *
 * Input is exactly what obd_chip_request() returns (echo + prompt already
 * stripped; '\r'/'\n' line endings preserved). Two outputs:
 *
 *   ap_resp_to_payload()  the framing-free PAYLOAD from the service echo
 *                         (B0 = 0x41 for a mode-01 response): what the
 *                         parsers (VIN, DTC, the detection) and the
 *                         cross-talk guard read.
 *   ap_resp_row_frames()  the reply AS THE CHIP PRINTS IT with headers on
 *                         (every line's bytes after the CAN id, PCI and
 *                         extended address bytes included, ONE responder's
 *                         lines in arrival order): what user expressions
 *                         index since 2026-10-24 (issue #1014). The legacy
 *                         firmware's parse_elm327_response() read the same
 *                         bytes, so the published vehicle profiles count
 *                         this way: `04 41 0C 1A F8` is the RPM reply,
 *                         `[B3:B4]/4`. ap_resp_frames() is the same view
 *                         of the lowest responder, with nothing compared.
 *
 * Which responder, when several ECUs answer one request (a row that sets no
 * header of its own goes out on the functional id, and every ECU that knows
 * the identifier replies): a STANDARD row reads the lowest id (7E8, the
 * engine ECU: the legacy firmware's priority rule for its standard table); a
 * CUSTOM or VEHICLE row reads the first ECU, in arrival order, whose reply
 * answers the request (2026-11-27). The legacy firmware evaluated such a row
 * over every line in arrival order, so a published expression counts into
 * the answer that came first; until that day v6 read the lowest id for
 * every row, and an odometer row (`222006`, `[B4:B6]`) answered by `7EC 06
 * 62 20 06 01 E2 40` and then by `7EA 06 62 20 06 FF FF FF` (an ECU that
 * holds no mileage) read 16777215 where 4.52p read 123456, and nothing at
 * all when the lower id refused (`7E8 03 7F 22 31`).
 *
 * Accepted shapes (auto-detected per line):
 *   41 0C 1A F8                headers off, single frame
 *   014 / 0: 49 02 01 .. / 1:  headers off, ISO-TP multi-line (length,
 *                              then indexed 8-byte rows; trimmed to length)
 *   7E8 06 41 00 BE 7F B8 13   headers on, single frame (PCI 0x0L)
 *   7E8 10 14 49 02 01 ..      headers on, ISO-TP first (0x1L LL) +
 *   7E8 21 ..                  consecutive (0x2N): reassembled from ONE
 *                              responder id only (the lowest, or the one a
 *                              row picks: see above)
 *   607 F1 10 09 62 E5 CE ..   headers on, extended addressing: the target
 *                              address byte before the PCI (take_ea)
 *   18 DA F1 58 07 62 F4 00 .. headers on, 29-bit: the chip prints the id
 *                              as FOUR byte tokens (bench 2026-10-03). The
 *                              legislated answers 18DAF1xx are taken as an
 *                              id, and the row's own receive filter id or
 *                              any id on a 29-bit protocol with the hint
 *                              (ap_resp_set_id29_hint, 2026-10-24)
 *
 * A `7F <service> 78` line (response pending: the ECU needs more time and
 * the chip keeps waiting) is not an answer and is dropped wherever it
 * stands; the lines after it are. The legacy firmware counted such a
 * frame into the bytes, which is why the i3 and Mini SOC rows of the
 * published file read `[B11:B12]` where the answer alone puts the SOC at
 * `[B6:B7]`.
 *
 * Headers off is the fallback (a K-line protocol's prelude keeps them
 * off): ap_resp_frames() rebuilds the frames from the rows, taking them
 * in ARRIVAL order (the chip prints `0:` to `F:` and wraps, so a reply
 * over 16 rows repeats the index: ordering by it interleaved the rows 16..
 * with 0.., the Leaf ZE1's 28-row reply, bench 2026-10-21).
 *
 * The line tokenizer (one chip line -> bytes, header, row index) lives in
 * autopid_resp_lines.c since 2026-10-03 (700-line rule) and the cross-talk
 * guard in autopid_resp_echo.c since 2026-11-27; this file assembles the
 * lines into payloads and picks the responder a row reads.
 */
#include "autopid_private.h"
#include "autopid_resp_private.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* The line table is ~5.6 KB: far too big for a task stack (a 6144-byte
   job stack silently overflowed on it, 2026-07-22; the 2026-07-22 static
   stack audit then showed EVERY caller was budgeting for it). One shared
   PSRAM table behind a mutex instead: response assembly is a
   parse-after-transaction step and the chip transactions are already
   serialized, so the lock never contends in practice. */
static ap_line_t s_lines[AP_MAX_LINES] EXT_RAM_BSS_ATTR;
/* the ECUs of a pre-CAN reply (autopid_resp_precan.c), under the same lock */
static ap_resp_ecu_t s_precan[AP_RESP_ECUS_MAX] EXT_RAM_BSS_ATTR;
static SemaphoreHandle_t s_lines_lock;
static StaticSemaphore_t s_lines_lock_buf;

static void lines_lock(void)
{
    /* lazy create: first callers (boot provisioning, host tests) run
       single-threaded, every later caller is past init */
    if (s_lines_lock == NULL)
    {
        s_lines_lock = xSemaphoreCreateMutexStatic(&s_lines_lock_buf);
    }

    xSemaphoreTake(s_lines_lock, portMAX_DELAY);
}

static void lines_unlock(void)
{
    xSemaphoreGive(s_lines_lock);
}

/** ISO-TP reassembly over the frames of ONE responder id (PCI in byte
 *  0; non-ISO-TP lines taken raw). @return assembled byte count. */
static size_t assemble_header_frames(const ap_line_t *lines, int n_lines,
                                     uint32_t header, uint8_t *payload,
                                     size_t payload_max)
{
    size_t n = 0;
    long expect = -1;

    for (int i = 0; i < n_lines; i++)
    {
        if (!lines[i].has_header || lines[i].header != header ||
            lines[i].n <= lines[i].ea)
        {
            continue;
        }

        /* behind the extended address byte when the line carries one */
        const uint8_t *b = lines[i].bytes + lines[i].ea;
        size_t bn = lines[i].n - lines[i].ea;
        uint8_t pci = b[0] >> 4;
        size_t start, count;

        if (pci == 0x0)
        {
            count = b[0] & 0x0F;
            start = 1;

            if (count > bn - 1)
            {
                count = bn - 1;
            }
        }
        else if (pci == 0x1)
        {
            if (bn < 2)
            {
                continue;
            }

            expect = ((long)(b[0] & 0x0F) << 8) | b[1];
            start = 2;
            count = bn - 2;
        }
        else if (pci == 0x2)
        {
            start = 1;
            count = bn - 1;
        }
        else
        {
            /* no PCI (11-bit non-ISO-TP data): take the raw bytes */
            start = 0;
            count = bn;
        }

        for (size_t j = 0; j < count && n < payload_max; j++)
        {
            payload[n++] = b[start + j];
        }
    }

    if (expect >= 0 && (size_t)expect < n)
    {
        n = (size_t)expect;
    }

    return n;
}

/** Headers-off assembly: ISO-TP rows in arrival order (trimmed to the
 *  bare length line) or plain concatenation. @return byte count. */
static size_t assemble_headerless(const ap_line_t *lines, int n_lines,
                                  long iso_total, uint8_t *payload,
                                  size_t payload_max)
{
    size_t n = 0;

    if (lines[0].iso_index >= 0 || iso_total >= 0)
    {
        /* headers off, ISO-TP rows: in ARRIVAL order. The printed index is
           the ISO-TP sequence number, `0:` to `F:` and round again, so a
           reply over 16 rows repeats it: ordering the rows BY that index
           interleaved the rows 16.. with 0.. (the Leaf ZE1's 28-row
           cell-voltage DID, bench 2026-10-21). It only tells a missing
           row: the assembly stops at the first gap rather than decode
           bytes that moved. */
        int expect = 0;

        for (int i = 0; i < n_lines; i++)
        {
            if (lines[i].iso_index < 0)
            {
                continue;       /* a stray plain line among the rows */
            }

            if (lines[i].iso_index != (expect & 0x0F))
            {
                break;          /* a row is missing: what follows moved */
            }

            for (size_t j = 0; j < lines[i].n && n < payload_max; j++)
            {
                payload[n++] = lines[i].bytes[j];
            }

            expect++;
        }

        if (iso_total >= 0 && (size_t)iso_total < n)
        {
            n = (size_t)iso_total;
        }
    }
    else
    {
        /* headers off, plain line(s): concatenate */
        for (int i = 0; i < n_lines; i++)
        {
            for (size_t j = 0; j < lines[i].n && n < payload_max; j++)
            {
                payload[n++] = lines[i].bytes[j];
            }
        }
    }

    return n;
}

esp_err_t ap_resp_to_payload(const char *resp, uint8_t *payload,
                             size_t payload_max, size_t *out_len)
{
    if (resp == NULL || payload == NULL || out_len == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_len = 0;

    lines_lock();

    ap_line_t *lines = s_lines;
    bool saw_error = false;
    long iso_total = -1; /* headers-off multi-line length */
    int n_lines = ap_resp_collect_lines(resp, lines, AP_MAX_LINES,
                                        &saw_error, &iso_total);

    if (saw_error || n_lines == 0)
    {
        lines_unlock();
        return saw_error ? ESP_FAIL : ESP_ERR_NOT_FOUND;
    }

    size_t n = 0;

    if (ap_resp_precan_ecus(lines, n_lines, iso_total, s_precan,
                            AP_RESP_ECUS_MAX) > 0)
    {
        /* a pre-CAN protocol: the lowest ECU's data, as on CAN */
        n = (s_precan[0].len < payload_max) ? s_precan[0].len : payload_max;
        memcpy(payload, s_precan[0].payload, n);
    }
    else if (lines[0].has_header)
    {
        /* headers on: keep only the LOWEST responder id (legacy rule) */
        uint32_t lowest = UINT32_MAX;

        for (int i = 0; i < n_lines; i++)
        {
            if (lines[i].has_header && lines[i].header < lowest)
            {
                lowest = lines[i].header;
            }
        }

        n = assemble_header_frames(lines, n_lines, lowest, payload,
                                   payload_max);
    }
    else
    {
        n = assemble_headerless(lines, n_lines, iso_total, payload,
                                payload_max);
    }

    lines_unlock();

    if (n == 0)
    {
        return ESP_ERR_NOT_FOUND;
    }

    *out_len = n;
    return ESP_OK;
}

/* ap_resp_frames, headers off: copy what fits, keep counting */
static size_t frames_put(uint8_t *out, size_t out_max, size_t n,
                         const uint8_t *src, size_t count)
{
    for (size_t i = 0; i < count && n < out_max; i++)
    {
        out[n++] = src[i];
    }

    return n;
}

/** The frames the chip would have printed with headers on for a reply it
 *  gave headers off (normal addressing): `0L` + the bytes for one that
 *  fits a frame, else `1L LL` + six bytes then `2N` + seven per
 *  consecutive frame, the last padded with 0x00 (the ECU's padding is not
 *  in the rows). @return bytes written. */
static size_t frames_from_payload(const uint8_t *payload, size_t len,
                                  uint8_t *out, size_t out_max)
{
    size_t n = 0;

    if (len == 0)
    {
        return 0;
    }

    if (len <= 7)
    {
        uint8_t pci = (uint8_t)len;

        n = frames_put(out, out_max, n, &pci, 1);
        return frames_put(out, out_max, n, payload, len);
    }

    uint8_t ff[2] =
    {
        (uint8_t)(0x10 | ((len >> 8) & 0x0F)), (uint8_t)(len & 0xFF),
    };

    n = frames_put(out, out_max, n, ff, 2);
    n = frames_put(out, out_max, n, payload, 6);

    static const uint8_t zeros[7] = { 0 };
    size_t at = 6;
    unsigned sn = 1;

    while (at < len)
    {
        uint8_t pci = (uint8_t)(0x20 | (sn & 0x0F));
        size_t take = len - at;

        if (take > 7)
        {
            take = 7;
        }

        n = frames_put(out, out_max, n, &pci, 1);
        n = frames_put(out, out_max, n, payload + at, take);
        n = frames_put(out, out_max, n, zeros, 7 - take);
        at += take;
        sn++;
    }

    return n;
}

ap_resp_pick_t ap_resp_pick_for(ap_pid_type_t type)
{
    return (type == AP_PID_STD) ? AP_RESP_PICK_LOWEST : AP_RESP_PICK_FIRST;
}

/**
 * Headers on, and more than one ECU may have answered: the id whose lines a
 * row reads (ap_resp_pick_t), and whether that ECU's reply answers
 * @p request (NULL = nothing to compare). Under the lines lock.
 */
static uint32_t pick_header(const ap_line_t *lines, int n_lines,
                            const char *request, ap_resp_pick_t pick,
                            bool *answered)
{
    static uint8_t s_try[AP_PAYLOAD_MAX] EXT_RAM_BSS_ATTR;
    uint32_t first = 0;
    uint32_t lowest = UINT32_MAX;
    bool have_first = false;

    for (int i = 0; i < n_lines; i++)
    {
        if (!lines[i].has_header)
        {
            continue;
        }

        if (!have_first)
        {
            first = lines[i].header;
            have_first = true;
        }

        if (lines[i].header < lowest)
        {
            lowest = lines[i].header;
        }
    }

    if (pick == AP_RESP_PICK_LOWEST || request == NULL)
    {
        uint32_t id = (pick == AP_RESP_PICK_LOWEST) ? lowest : first;
        size_t n = assemble_header_frames(lines, n_lines, id, s_try,
                                          sizeof(s_try));

        *answered = (request == NULL) ||
                    ap_payload_matches_cmd(request, s_try, n);
        return id;
    }

    /* every id once, in the order of its first line: the first one whose
       reply answers the request. An ECU that refuses (`7F 22 31`) or one
       whose frame belongs to another request is passed over */
    for (int i = 0; i < n_lines; i++)
    {
        if (!lines[i].has_header)
        {
            continue;
        }

        bool seen = false;

        for (int k = 0; k < i && !seen; k++)
        {
            seen = lines[k].has_header && lines[k].header == lines[i].header;
        }

        if (seen)
        {
            continue;
        }

        size_t n = assemble_header_frames(lines, n_lines, lines[i].header,
                                          s_try, sizeof(s_try));

        if (ap_payload_matches_cmd(request, s_try, n))
        {
            *answered = true;
            return lines[i].header;
        }
    }

    *answered = false;
    return first;   /* nobody answered it: what came first, for a test to show */
}

esp_err_t ap_resp_row_frames(const char *resp, const char *request,
                             ap_resp_pick_t pick, uint8_t *out,
                             size_t out_max, size_t *out_len, bool *answered)
{
    if (resp == NULL || out == NULL || out_len == NULL || answered == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_len = 0;
    *answered = false;

    lines_lock();

    ap_line_t *lines = s_lines;
    bool saw_error = false;
    long iso_total = -1;
    int n_lines = ap_resp_collect_lines(resp, lines, AP_MAX_LINES,
                                        &saw_error, &iso_total);

    if (saw_error || n_lines == 0)
    {
        lines_unlock();
        return saw_error ? ESP_FAIL : ESP_ERR_NOT_FOUND;
    }

    size_t n = 0;

    if (ap_resp_precan_ecus(lines, n_lines, iso_total, s_precan,
                            AP_RESP_ECUS_MAX) > 0)
    {
        /* a pre-CAN protocol: the lowest ECU's data as the frame a CAN ECU
           would have sent it in (its ECUs are known by address, not by
           arrival: the lowest address is the row's, whatever the pick) */
        n = frames_from_payload(s_precan[0].payload, s_precan[0].len, out,
                                out_max);
        *answered = (request == NULL) ||
                    ap_payload_matches_cmd(request, s_precan[0].payload,
                                           s_precan[0].len);
    }
    else if (lines[0].has_header)
    {
        /* headers on: the picked responder's lines, verbatim (PCI and
           extended address bytes included), in arrival order */
        uint32_t id = pick_header(lines, n_lines, request, pick, answered);

        for (int i = 0; i < n_lines; i++)
        {
            if (lines[i].has_header && lines[i].header == id)
            {
                n = frames_put(out, out_max, n, lines[i].bytes, lines[i].n);
            }
        }
    }
    else
    {
        /* headers off (a K-line protocol's prelude, or a chip left so by
           a reset): the framing-free payload, then the frames it would
           have printed with headers on. Under the lines lock. */
        static uint8_t s_flat[AP_PAYLOAD_MAX] EXT_RAM_BSS_ATTR;
        size_t flat = assemble_headerless(lines, n_lines, iso_total, s_flat,
                                          sizeof(s_flat));

        n = frames_from_payload(s_flat, flat, out, out_max);
        *answered = (request == NULL) ||
                    ap_payload_matches_cmd(request, s_flat, flat);
    }

    lines_unlock();

    if (n == 0)
    {
        *answered = false;
        return ESP_ERR_NOT_FOUND;
    }

    *out_len = n;
    return ESP_OK;
}

esp_err_t ap_resp_frames(const char *resp, uint8_t *out, size_t out_max,
                         size_t *out_len)
{
    bool answered;

    return ap_resp_row_frames(resp, NULL, AP_RESP_PICK_LOWEST, out, out_max,
                              out_len, &answered);
}

int ap_resp_to_payloads(const char *resp, ap_resp_ecu_t *out,
                        int max_ecus)
{
    if (resp == NULL || out == NULL || max_ecus <= 0)
    {
        return 0;
    }

    if (max_ecus > AP_RESP_ECUS_MAX)
    {
        max_ecus = AP_RESP_ECUS_MAX;
    }

    lines_lock();

    ap_line_t *lines = s_lines;
    bool saw_error = false;
    long iso_total = -1;
    int n_lines = ap_resp_collect_lines(resp, lines, AP_MAX_LINES,
                                        &saw_error, &iso_total);

    if (saw_error || n_lines == 0)
    {
        lines_unlock();
        return saw_error ? -1 : 0;
    }

    int n_pre = ap_resp_precan_ecus(lines, n_lines, iso_total, out, max_ecus);

    if (n_pre > 0)
    {
        lines_unlock();
        return n_pre; /* a pre-CAN protocol: its ECUs by address */
    }

    if (!lines[0].has_header)
    {
        /* headers off: responders are indistinguishable; assemble the
           single-payload shapes from the lines already collected (NOT
           a nested ap_resp_to_payload call: its ~5.6 KB line table on
           top of ours is exactly the stack-overflow shape that
           corrupted the 2026-07-22 scans) */
        out[0].header = UINT32_MAX;
        out[0].len = assemble_headerless(lines, n_lines, iso_total,
                                         out[0].payload,
                                         sizeof(out[0].payload));
        lines_unlock();
        return (out[0].len > 0) ? 1 : 0;
    }

    /* distinct responder ids, ascending; over-cap keeps the lowest */
    uint32_t hdrs[AP_RESP_ECUS_MAX];
    int n_hdrs = 0;

    for (int i = 0; i < n_lines; i++)
    {
        if (!lines[i].has_header)
        {
            continue;
        }

        uint32_t h = lines[i].header;
        int at = 0;

        while (at < n_hdrs && hdrs[at] < h)
        {
            at++;
        }

        if (at < n_hdrs && hdrs[at] == h)
        {
            continue;
        }

        if (n_hdrs < max_ecus)
        {
            n_hdrs++;
        }
        else if (at >= max_ecus)
        {
            continue;           /* full and larger than everything kept  */
        }

        for (int k = n_hdrs - 1; k > at; k--)
        {
            hdrs[k] = hdrs[k - 1];
        }

        hdrs[at] = h;
    }

    int n_out = 0;

    for (int i = 0; i < n_hdrs; i++)
    {
        size_t len = assemble_header_frames(lines, n_lines, hdrs[i],
                                            out[n_out].payload,
                                            sizeof(out[n_out].payload));

        if (len > 0)
        {
            out[n_out].header = hdrs[i];
            out[n_out].len = len;
            n_out++;
        }
    }

    lines_unlock();
    return n_out;
}

