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
 * @file autopid_resp_echo.c
 * @brief PURE: does a reply answer the request (host-tested). The cross-talk
 *        guard (ap_payload_matches_cmd: another chip master's response, or an
 *        ECU's late answer to the request before, is never taken for ours) and
 *        the request a raw row carries behind its ISO-TP length byte
 *        (ap_raw_cmd_request). It is also what tells the ECU that answered
 *        from one that refused when several reply to one request
 *        (ap_resp_row_frames in autopid_resp.c). Split out of autopid_resp.c
 *        2026-11-27 (700-line rule); behaviour unchanged.
 */
#include "autopid_private.h"

#include <ctype.h>
#include <stdlib.h>

bool ap_payload_matches_cmd(const char *cmd, const uint8_t *payload,
                            size_t payload_len)
{
    /* parse the leading hex byte pairs of the command ("010C", "22202A",
       "03"; a trailing response-count hint digit like "010C1" is ignored
       by pair alignment). Non-hex commands (AT/ST/VT) can't be checked. */
    uint8_t want[3];
    size_t n_want = 0;
    const char *p = cmd;

    while (n_want < 3)
    {
        while (*p == ' ')
        {
            p++;
        }

        if (!isxdigit((unsigned char)p[0]) ||
            !isxdigit((unsigned char)p[1]))
        {
            break;
        }

        char pair[3] = { p[0], p[1], '\0' };

        want[n_want++] = (uint8_t)strtol(pair, NULL, 16);
        p += 2;
    }

    if (n_want == 0)
    {
        return true; /* AT/ST/VT: nothing to verify */
    }

    /* positive responses echo (service | 0x40) then the identifier */
    if (payload_len < 1 || payload[0] != (uint8_t)(want[0] + 0x40))
    {
        return false;
    }

    if (n_want > 1 && payload_len > 1 && payload[1] != want[1])
    {
        return false;
    }

    /* ReadDataByIdentifier echoes a 2-byte identifier: both bytes count,
       or `62 F4 0D` would pass as the answer to `22F40C` */
    if (want[0] == 0x22 && n_want > 2 && payload_len > 2 &&
        payload[2] != want[2])
    {
        return false;
    }

    return true;
}

const char *ap_raw_cmd_request(const char *cmd)
{
    /* the frame's bytes as the chip reads them: hex pairs, spaces ignored */
    const char *p = cmd;
    const char *service_at = NULL;
    uint8_t pci = 0;
    uint8_t service = 0;
    size_t n_pairs = 0;

    for (;;)
    {
        while (*p == ' ')
        {
            p++;
        }

        if (!isxdigit((unsigned char)p[0]) ||
            !isxdigit((unsigned char)p[1]))
        {
            break;
        }

        char pair[3] = { p[0], p[1], '\0' };
        uint8_t byte = (uint8_t)strtol(pair, NULL, 16);

        if (n_pairs == 0)
        {
            pci = byte;
        }
        else if (n_pairs == 1)
        {
            service = byte;
            service_at = p;
        }

        n_pairs++;
        p += 2;
    }

    /* an ISO-TP single frame: length 1..7 in byte 0, that many bytes
       after it. Anything else (a raw frame of another kind, an AT/ST
       command) carries no service to compare */
    if (pci < 0x01 || pci > 0x07 || n_pairs < (size_t)pci + 1)
    {
        return NULL;
    }

    /* GMLAN readDataByPacketIdentifier: the answers are UUDT frames that
       start with the packet's number, no response service (GMW3110 $AA) */
    if (service == 0xAA)
    {
        return NULL;
    }

    return service_at;
}
