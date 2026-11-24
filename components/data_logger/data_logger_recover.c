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
 * @file data_logger_recover.c
 * @brief PURE recovery helpers (no IO, no RTOS, host-tested), the
 *        decision logic behind ROBUSTNESS.md:
 *        - torn tails: how much of a text log / a .wdl log is intact
 *        - set-aside names for corrupt files (*.corrupt) and their parse
 *        - the PSRAM salvage envelope: ring index and record sanity, CRC
 *        - an open against the internal heap: the floor under which none
 *          is tried, and what the latched fault says about it
 */
#include <string.h>

#include "data_logger_private.h"

/* ---- torn tails ---------------------------------------------------------- */

size_t dl_recover_text_keep(const char *buf, size_t len)
{
    while (len > 0)
    {
        if (buf[len - 1] == '\n')
        {
            return len;
        }

        len--;
    }

    return 0;
}

size_t dl_recover_wdl_scan(const uint8_t *buf, size_t len, bool *bad)
{
    size_t off = 0;

    if (bad != NULL)
    {
        *bad = false;
    }

    while (off < len)
    {
        uint8_t t = buf[off];
        size_t need;

        if (t == 0x01)               /* def: type, u16 id, u8 len, name */
        {
            if (off + 4 > len)
            {
                break;               /* header itself is cut */
            }

            need = 4 + buf[off + 3];
        }
        else if (t == 0x02)          /* param: type, i64 ts, u16 id, f64 */
        {
            need = 19;
        }
        else if (t == 0x03)          /* frame: type, i64 ts, u32 id, dlc, data */
        {
            if (off + 14 > len)
            {
                break;
            }

            if (buf[off + 13] > 8)
            {
                if (bad != NULL)
                {
                    *bad = true;
                }

                break;
            }

            need = 14 + buf[off + 13];
        }
        else if (t == 0x04)          /* clock step: type, i64 ts, i64 delta, u8 */
        {
            need = DL_WDL_STEP_LEN;
        }
        else
        {
            if (bad != NULL)
            {
                *bad = true;         /* not a record start: garbage from here */
            }

            break;
        }

        if (off + need > len)
        {
            break;                   /* partial record at the end */
        }

        off += need;
    }

    return off;
}

/* ---- set-aside names ------------------------------------------------------ */

bool dl_recover_corrupt_name(const char *fname, char *out, size_t cap)
{
    size_t n = (fname != NULL) ? strlen(fname) : 0;

    if (n == 0 || n + sizeof(DL_CORRUPT_SUFFIX) > cap)
    {
        return false;
    }

    memcpy(out, fname, n);
    memcpy(out + n, DL_CORRUPT_SUFFIX, sizeof(DL_CORRUPT_SUFFIX));
    return true;
}

bool dl_recover_is_corrupt_name(const char *fname, const char *prefix,
                                int64_t *epoch_out)
{
    const size_t sl = sizeof(DL_CORRUPT_SUFFIX) - 1;
    size_t n = (fname != NULL) ? strlen(fname) : 0;
    char base[48];

    if (n <= sl || strcmp(fname + n - sl, DL_CORRUPT_SUFFIX) != 0 ||
        n - sl >= sizeof(base))
    {
        return false;
    }

    memcpy(base, fname, n - sl);
    base[n - sl] = '\0';

    if (prefix != NULL && strncmp(base, prefix, strlen(prefix)) != 0)
    {
        return false;
    }

    return dl_files_parse(base, epoch_out);
}

/* ---- PSRAM salvage sanity --------------------------------------------------- */

bool dl_recover_record_sane(const dl_record_t *r)
{
    /* random PSRAM almost never lands inside these ranges; the clock may
       be unset (1970) so the floor is 0, the ceiling is year 2096 */
    if (r == NULL || r->ts_ms < 0 || r->ts_ms > 4000000000000LL ||
        (r->flags & (uint8_t)~DL_REC_ABS) != 0)
    {
        return false;
    }

    if (r->kind == DL_REC_PARAM)
    {
        return r->u.p.param >= 0 && r->u.p.param < DL_MAX_PARAMS;
    }

    if (r->kind == DL_REC_FRAME)
    {
        return r->u.f.dlc <= 8 && r->u.f.id <= 0x1FFFFFFFu &&
               (r->u.f.flags & (uint8_t)~(DL_FRAME_EXT | DL_FRAME_RTR)) == 0;
    }

    return false;
}

bool dl_recover_ring_sane(uint32_t cap, uint32_t cap_limit, uint32_t head,
                          uint32_t tail, uint32_t *fill)
{
    if (cap == 0 || cap > cap_limit || head >= cap || tail >= cap ||
        fill == NULL)
    {
        return false;
    }

    uint32_t span = (head + cap - tail) % cap;

    /* the three indices are updated one after another: a reset between
       two of them leaves fill off by one, trust head/tail then. A full
       ring (head == tail, fill == cap) is the one legitimate mismatch. */
    if (*fill > cap || (*fill != span && !(*fill == cap && span == 0)))
    {
        *fill = span;
    }

    return true;
}

/* The rings live in PSRAM behind the CPU's data cache. A reset drops the
 * cache lines that were not written to the chip yet, one by one: the
 * newest record can be there with its first half only (its stamp and its
 * value new, the parameter number still the one of the record this slot
 * held a ring length ago), or not at all while the ring's indices already
 * count it. Range checks pass such a record; this check does not: it runs
 * over the fields of the record's kind and over the number of the push. */
static uint32_t check_mix(uint32_t h, uint32_t v)
{
    h ^= v;
    h *= 0x01000193u;
    return h ^ (h >> 15);
}

uint16_t dl_recover_record_check(const dl_record_t *r, uint32_t seq)
{
    uint32_t h = check_mix(0x811C9DC5u, seq);
    uint64_t ts = (uint64_t)r->ts_ms;

    h = check_mix(h, (uint32_t)ts);
    h = check_mix(h, (uint32_t)(ts >> 32));
    h = check_mix(h, (uint32_t)r->kind | ((uint32_t)r->flags << 8));
    h = check_mix(h, r->mono_ms);

    if (r->kind == DL_REC_PARAM)
    {
        uint64_t bits;

        memcpy(&bits, &r->u.p.value, sizeof(bits));
        h = check_mix(h, (uint32_t)bits);
        h = check_mix(h, (uint32_t)(bits >> 32));
        h = check_mix(h, (uint16_t)r->u.p.param);
    }
    else
    {
        uint32_t d[2];

        memcpy(d, r->u.f.data, sizeof(d));
        h = check_mix(h, r->u.f.id);
        h = check_mix(h, d[0]);
        h = check_mix(h, d[1]);
        h = check_mix(h, (uint32_t)r->u.f.dlc | ((uint32_t)r->u.f.flags << 8));
    }

    return (uint16_t)(h ^ (h >> 16));
}

uint32_t dl_recover_ring_keep(dl_record_t *buf, uint32_t cap, uint32_t tail,
                              uint32_t fill, uint32_t next_seq, int kind,
                              const dl_param_entry_t *params)
{
    if (buf == NULL || cap == 0 || tail >= cap || fill > cap)
    {
        return 0;
    }

    uint32_t first_seq = next_seq - fill; /* modulo 2^32, like the counter */
    uint32_t kept = 0;

    for (uint32_t i = 0; i < fill; i++)
    {
        const dl_record_t *rec = &buf[(tail + i) % cap];

        if (rec->kind != kind || !dl_recover_record_sane(rec) ||
            rec->check != dl_recover_record_check(rec, first_seq + i))
        {
            continue;
        }

        if (kind == DL_REC_PARAM && params != NULL &&
            !params[rec->u.p.param].used)
        {
            continue; /* the registry is older than this record */
        }

        if (kept != i)
        {
            buf[(tail + kept) % cap] = *rec;
        }

        kept++;
    }

    for (uint32_t i = 0; i < kept; i++)
    {
        dl_record_t *rec = &buf[(tail + i) % cap];

        rec->flags |= DL_REC_ABS;
        rec->check = dl_recover_record_check(rec, next_seq - kept + i);
    }

    return kept;
}

uint32_t dl_recover_crc32_update(uint32_t crc, const void *data, size_t len)
{
    const uint8_t *p = data;

    crc = ~crc;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= p[i];

        for (int b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }

    return ~crc;
}

/* ---- an open and the internal heap (2026-11-14) -------------------------- */

bool dl_recover_open_ram_ok(size_t largest_internal_block)
{
    return largest_internal_block >= DL_OPEN_RAM_MIN;
}

bool dl_recover_corrupt_counts(bool engine_says_corrupt,
                               size_t largest_internal_block)
{
    return engine_says_corrupt &&
           dl_recover_open_ram_ok(largest_internal_block);
}

int dl_recover_fault_detail(char *out, size_t cap, bool frames, bool no_mem)
{
    int n = snprintf(out, cap, "%s%s", frames ? "can" : "params",
                     no_mem ? ": out of internal RAM" : "");

    if (n < 0 || (size_t)n >= cap)
    {
        /* never a cut text that reads as the other reason */
        if (cap > 0)
        {
            out[0] = 0;
        }

        return 0;
    }

    return n;
}
