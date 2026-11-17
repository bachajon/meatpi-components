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
 * @file j2534_iso_cfg.c
 * @brief ISO15765_BS and ISO15765_STMIN of an ISO15765 channel: the list a
 *        tool sends is checked and taken here, and the list it reads is
 *        filled here. No I/O, no lock: j2534_channel_iso.c calls in with
 *        the channel lock held and applies the result to the sessions.
 *
 * 2026-11-17, on top of Elias Kotlyar's pull request 3 (the ISO-TP stack
 * and the provider take a block size and a separation time per session):
 * the values are checked against SAE J2534-1's ranges, a refused list
 * changes nothing, and GET_CONFIG is answered by the device, so a driver
 * reports what the bus sees on any firmware.
 */
#include "j2534_iso_cfg.h"

#include <stddef.h>

#include "j2534_proto.h"

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

bool j2534_iso_stmin_to_us(uint32_t stmin, uint32_t *us)
{
    uint32_t value;

    if (stmin <= 0x7Fu)
    {
        value = stmin * 1000u;
    }
    else if (stmin >= 0xF1u && stmin <= 0xF9u)
    {
        value = (stmin - 0xF0u) * 100u;
    }
    else
    {
        return false;
    }

    if (us != NULL)
    {
        *us = value;
    }

    return true;
}

uint32_t j2534_iso_us_to_stmin(uint32_t us)
{
    if (us >= 100u && us <= 900u)
    {
        return 0xF0u + us / 100u;
    }

    if (us / 1000u > 0x7Fu)
    {
        return 0x7Fu;
    }

    return us / 1000u;
}

/* The count of a well-formed list, or -1. */
static int list_count(const uint8_t *body, uint32_t body_len)
{
    uint32_t count;

    if (body_len < 4)
    {
        return -1;
    }

    count = rd_u32(body);

    if (count > J2534_ISO_CFG_MAX || body_len != 4u + count * 8u)
    {
        return -1;
    }

    return (int)count;
}

uint32_t j2534_iso_cfg_set(const uint8_t *body, uint32_t body_len,
                           j2534_iso_flow_t *flow, bool *changed)
{
    j2534_iso_flow_t next;
    bool named = false;
    int count;

    if (changed != NULL)
    {
        *changed = false;
    }

    if (flow == NULL)
    {
        return J2534_ERR_NULL_PARAMETER;
    }

    if (body == NULL || body_len == 0)
    {
        return J2534_STATUS_NOERROR;
    }

    count = list_count(body, body_len);

    if (count < 0)
    {
        return J2534_ERR_INVALID_IOCTL_VALUE;
    }

    next = *flow;

    for (int i = 0; i < count; i++)
    {
        uint32_t param = rd_u32(body + 4 + i * 8);
        uint32_t value = rd_u32(body + 8 + i * 8);

        if (param == J2534_CFG_ISO15765_BS)
        {
            if (value > 0xFFu)
            {
                return J2534_ERR_INVALID_IOCTL_VALUE;
            }

            next.bs = (uint8_t)value;
            named = true;
        }
        else if (param == J2534_CFG_ISO15765_STMIN)
        {
            if (!j2534_iso_stmin_to_us(value, &next.st_us))
            {
                return J2534_ERR_INVALID_IOCTL_VALUE;
            }

            named = true;
        }
    }

    *flow = next;

    if (changed != NULL)
    {
        *changed = named;
    }

    return J2534_STATUS_NOERROR;
}

uint32_t j2534_iso_cfg_get(const uint8_t *body, uint32_t body_len,
                           const j2534_iso_flow_t *flow,
                           uint8_t *out, uint32_t out_cap, uint32_t *out_len)
{
    uint32_t n = 0;
    int count;

    if (out_len != NULL)
    {
        *out_len = 0;
    }

    if (flow == NULL)
    {
        return J2534_ERR_NULL_PARAMETER;
    }

    if (body == NULL || body_len == 0)
    {
        return J2534_STATUS_NOERROR;
    }

    count = list_count(body, body_len);

    if (count < 0)
    {
        return J2534_ERR_INVALID_IOCTL_VALUE;
    }

    if (out == NULL || out_len == NULL || out_cap < 4u + (uint32_t)count * 8u)
    {
        return J2534_ERR_INVALID_MSG;
    }

    for (int i = 0; i < count; i++)
    {
        uint32_t param = rd_u32(body + 4 + i * 8);
        uint32_t value;

        if (param == J2534_CFG_ISO15765_BS)
        {
            value = flow->bs;
        }
        else if (param == J2534_CFG_ISO15765_STMIN)
        {
            value = j2534_iso_us_to_stmin(flow->st_us);
        }
        else
        {
            continue;
        }

        wr_u32(out + 4 + n * 8, param);
        wr_u32(out + 8 + n * 8, value);
        n++;
    }

    wr_u32(out, n);
    *out_len = 4u + n * 8u;
    return J2534_STATUS_NOERROR;
}
