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
 * @file j2534_proto_session.c
 * @brief What a session's wire version decides (2026-11-18). PURE, host
 *        tested: no IDF types.
 *
 * Until this day the seven return codes from ERR_MSG_PROTOCOL_ID on were
 * numbered one too low against SAE J2534-1 (0x14 to 0x19 where the standard
 * has 0x15 to 0x1A) and ERR_DEVICE_IN_USE was 0x1A where the standard has
 * 0x0E, in this firmware and in the Windows driver alike, so a standard
 * application read "wrong bit rate" as ERR_NOT_UNIQUE. The numbers in
 * j2534_proto.h are the standard's now. A client that was written against
 * the old ones must keep getting them, so the numbers on the wire follow
 * the session: a client that asks for wire version 2 in its HELLO gets the
 * standard's, every other client (version 1: every driver up to 1.9, the
 * benches, a phone application) gets what it always got.
 *
 * Version 2 also turns the EVENT frame on, which no version 1 client was
 * ever sent: the device's word that received messages were lost.
 */
#include "j2534_proto.h"

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

uint32_t j2534_hello_version(const uint8_t *payload, uint32_t len)
{
    uint32_t asked = 1;

    if (payload != NULL && len >= 4)
    {
        asked = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
    }

    if (asked < 2)
    {
        return 1;
    }

    return J2534_SESSION_VERSION;
}

uint32_t j2534_status_wire(uint32_t status, uint32_t version)
{
    if (version >= 2)
    {
        return status;
    }

    if (status == J2534_ERR_DEVICE_IN_USE)
    {
        return 0x1Au;
    }

    if (status >= J2534_ERR_MSG_PROTOCOL_ID &&
        status <= J2534_ERR_INVALID_DEVICE_ID)
    {
        return status - 1u;
    }

    return status;
}

size_t j2534_event_encode(uint8_t *buf, size_t cap, uint16_t channel,
                          uint32_t event, uint32_t value)
{
    if (buf == NULL || cap < J2534_EVENT_FRAME_SIZE)
    {
        return 0;
    }

    j2534_hdr_encode(buf, J2534_MT_EVENT, 0, channel, 8);
    put_u32(buf + J2534_HDR_SIZE, event);
    put_u32(buf + J2534_HDR_SIZE + 4, value);
    return J2534_EVENT_FRAME_SIZE;
}

uint32_t j2534_lost_since(uint32_t drops_now, uint32_t *seen)
{
    uint32_t n;

    if (seen == NULL)
    {
        return 0;
    }

    /* a counter that went back is a new subscription (zeroed when it was
       made): everything it counts is new */
    n = (drops_now >= *seen) ? drops_now - *seen : drops_now;
    *seen = drops_now;
    return n;
}
