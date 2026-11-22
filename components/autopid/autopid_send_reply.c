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
 * @file autopid_send_reply.c
 * @brief PURE: what the OBD chip said to a frame of can.send, as `reply`
 *        (one line) and as `replies` (one line an entry), host-tested
 *        (autopid_send.h). Apart from autopid_send_core.c, to keep that
 *        file under the 700-line rule (2026-11-21).
 */
#include "autopid_send.h"

#include <stddef.h>

/* reply_line() of autopid_send.c until 2026-11-21, moved here to be
   host-tested with the line walk below */
size_t ap_send_reply_join(const char *resp, char *out, size_t cap)
{
    size_t n = 0;

    if (out == NULL || cap == 0)
    {
        return 0;
    }

    for (const char *p = resp; p != NULL && *p != '\0' && n + 1 < cap; p++)
    {
        if (*p == '\r' || *p == '\n')
        {
            if (n > 0 && out[n - 1] != ' ')
            {
                out[n++] = ' ';
            }
        }
        else if (*p != '>')
        {
            out[n++] = *p;
        }
    }

    while (n > 0 && out[n - 1] == ' ')
    {
        n--;
    }

    out[n] = '\0';
    return n;
}

/* The answers one per line (2026-11-21). `reply` held what the chip said
   joined into one line of at most 47 characters: a request to 7DF that
   seven ECUs answered (27 characters each) came back cut in the middle of
   the second answer, the rest in the transcript only. */
const char *ap_send_reply_next(const char *p, char *line, size_t cap)
{
    if (line == NULL || cap == 0)
    {
        return NULL;
    }

    line[0] = '\0';

    while (p != NULL && *p != '\0')
    {
        size_t n = 0;

        for (; *p != '\0' && *p != '\r' && *p != '\n'; p++)
        {
            if (*p != '>' && n + 1 < cap)
            {
                line[n++] = *p;
            }
        }

        while (n > 0 && line[n - 1] == ' ')
        {
            n--;
        }

        line[n] = '\0';

        while (*p == '\r' || *p == '\n')
        {
            p++;
        }

        if (n > 0)
        {
            return p;
        }
    }

    return NULL;
}
