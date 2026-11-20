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
 * @file j2534_iso_ae.c
 * @brief ISO15765 extended addressing on a J2534 channel, the pure part
 *        (j2534_iso_ae.h). The layout came with pull request 5 (bachajon,
 *        2026-11-19) inside j2534_channel_iso.c and j2534_channel_filter.c;
 *        it is here since 2026-11-20 so the host suite can run it.
 */
#include "j2534_iso_ae.h"

#include <string.h>

bool j2534_iso_filter_ae(const j2534_msg_t *pattern,
                         const j2534_msg_t *flow_control,
                         j2534_iso_ae_t *ae)
{
    if (pattern == NULL || flow_control == NULL || ae == NULL)
    {
        return false;
    }

    bool on = ((pattern->tx_flags | flow_control->tx_flags) &
               J2534_TX_ISO15765_ADDR_TYPE) != 0;
    uint32_t need = on ? 5u : 4u;

    if (pattern->data_size < need || flow_control->data_size < need ||
        pattern->data_size > J2534_MAX_DATA ||
        flow_control->data_size > J2534_MAX_DATA)
    {
        return false;
    }

    ae->on = on;
    ae->rx = on ? pattern->data[4] : 0;
    ae->tx = on ? flow_control->data[4] : 0;
    return true;
}

bool j2534_iso_ae_split(const j2534_msg_t *msg, uint32_t *id, uint8_t *ae,
                        const uint8_t **payload, size_t *len)
{
    if (msg == NULL || id == NULL || ae == NULL || payload == NULL ||
        len == NULL)
    {
        return false;
    }

    if ((msg->tx_flags & J2534_TX_ISO15765_ADDR_TYPE) == 0 ||
        msg->data_size < 5 || msg->data_size > J2534_MAX_DATA)
    {
        return false;
    }

    *id = ((uint32_t)msg->data[0] << 24) | ((uint32_t)msg->data[1] << 16) |
          ((uint32_t)msg->data[2] << 8) | (uint32_t)msg->data[3];
    *ae = msg->data[4];
    *payload = msg->data + 5;
    *len = msg->data_size - 5;
    return true;
}

uint8_t j2534_iso_ae_single_frame(uint8_t ae, const uint8_t *payload,
                                  size_t len, bool pad, uint8_t frame[8])
{
    if (payload == NULL || frame == NULL || len == 0 ||
        len > J2534_ISO_SF_MAX_AE)
    {
        return 0;
    }

    frame[0] = ae;              /* the address extension leads the frame   */
    frame[1] = (uint8_t)len;    /* the PCI counts the payload, not that    */
    memcpy(frame + 2, payload, len);

    uint8_t dlc = (uint8_t)(2 + len);

    if (pad && dlc < 8)
    {
        memset(frame + dlc, 0xAA, (size_t)(8 - dlc));
        dlc = 8;
    }

    return dlc;
}

bool j2534_iso_ae_join(j2534_msg_t *out, uint32_t id, bool ext, uint8_t ae,
                       const uint8_t *payload, size_t len, uint32_t ts_us)
{
    if (out == NULL || (payload == NULL && len > 0) ||
        len > (size_t)J2534_MAX_DATA - 5)
    {
        return false;
    }

    j2534_iso_indication(out, id, ext, 0, ts_us);   /* the id and the flags */
    j2534_iso_ae_indication(out, ae);

    if (len > 0)
    {
        memcpy(out->data + 5, payload, len);
    }

    out->data_size = (uint32_t)(5 + len);
    out->extra_data_index = out->data_size;
    return true;
}

void j2534_iso_ae_indication(j2534_msg_t *out, uint8_t ae)
{
    if (out == NULL)
    {
        return;
    }

    out->data[4] = ae;
    out->data_size = 5;
    out->extra_data_index = 5;
    out->rx_status |= J2534_RX_ISO15765_ADDR_TYPE;
}

bool j2534_iso_pdu_of_first_frame(const uint8_t *pdu, size_t n,
                                  const uint8_t *ff, uint8_t dlc)
{
    if (pdu == NULL || ff == NULL || dlc < 2 || dlc > 8 || (ff[0] >> 4) != 1)
    {
        return false;
    }

    size_t len = ((size_t)(ff[0] & 0x0F) << 8) | ff[1];
    uint8_t at = 2;

    if (len == 0)
    {
        if (dlc < 6)
        {
            return false;
        }

        /* the 32-bit length of a message above 4095 bytes */
        len = ((size_t)ff[2] << 24) | ((size_t)ff[3] << 16) |
              ((size_t)ff[4] << 8) | ff[5];
        at = 6;
    }

    return n == len && n >= (size_t)(dlc - at) &&
           memcmp(pdu, ff + at, (size_t)(dlc - at)) == 0;
}

uint32_t j2534_iso_frame_count(size_t len, bool has_ae)
{
    size_t sf = has_ae ? J2534_ISO_SF_MAX_AE : J2534_ISO_SF_MAX;
    size_t ff = sf - 1u;    /* a first frame gives one byte to the length */

    if (len <= sf)
    {
        return 1u;
    }

    return 1u + (uint32_t)((len - ff + sf - 1u) / sf);
}
