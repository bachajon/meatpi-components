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
 * @file j2534_channel_filter.c
 * @brief The filter table of a J2534 channel: adding a PASS, BLOCK or
 *        FLOW_CONTROL filter (the last one binds an ISO-TP session, on an
 *        ISO15765 channel only), stopping one, clearing them, and the
 *        PASS/BLOCK verdict the receive paths ask for. Out of
 *        j2534_channel.c (2026-11-18, the 700-line rule: it had grown to
 *        801 lines); the code is unchanged but for the names of what the
 *        two files now share. The j2534_channel_* entry points take the
 *        channel lock themselves; the j2534_ch_* helpers run with it
 *        held by the caller.
 */
#include "j2534_channel.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "j2534_channel_private.h"
#include "j2534_server.h"   /* J2534_MAX_CHANNELS */

static void filter_remove_at(j2534_channel_t *c, int i)
{
    c->filters[i] = c->filters[--c->n_filters];
}

uint32_t j2534_channel_add_filter(int slot, uint32_t type,
                                  const j2534_msg_t *mask,
                                  const j2534_msg_t *pattern,
                                  const j2534_msg_t *flow_control,
                                  uint32_t *filter_id)
{
    if (slot < 0 || slot >= J2534_MAX_CHANNELS || !j2534_ch_tbl[slot].in_use)
    {
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    if (filter_id == NULL)
    {
        return J2534_ERR_NULL_PARAMETER;
    }

    j2534_channel_t *c = &j2534_ch_tbl[slot];
    uint32_t st = J2534_STATUS_NOERROR;

    xSemaphoreTake(j2534_ch_lock, portMAX_DELAY);

    if (type == J2534_FLOW_CONTROL_FILTER)
    {
        /* ISO15765: pattern = the ECU's response id, flow-control message =
           the request id; its TxFlags name 29-bit ids and the padding of
           the frames this session sends (SAE J2534-1). With
           ISO15765_ADDR_TYPE (extended addressing) each carries a fifth
           byte, the address extension of that direction. */
        bool has_ae = pattern != NULL && flow_control != NULL &&
                      ((pattern->tx_flags | flow_control->tx_flags) &
                       J2534_TX_ISO15765_ADDR_TYPE) != 0;
        uint32_t need = has_ae ? 5u : 4u;

        if (c->protocol != J2534_PROT_ISO15765 ||
            pattern == NULL || flow_control == NULL ||
            pattern->data_size < need || flow_control->data_size < need)
        {
            st = J2534_ERR_INVALID_MSG;
        }
        else if (c->n_filters >= J2534_CH_MAX_FILTERS)
        {
            st = J2534_ERR_EXCEEDED_LIMIT;
        }
        else
        {
            uint32_t fl = pattern->tx_flags | flow_control->tx_flags;
            bool ext = (fl & J2534_TX_CAN_29BIT_ID) != 0;
            bool pad = (flow_control->tx_flags & J2534_TX_ISO15765_FRAME_PAD) != 0;
            uint32_t idmask = ext ? 0x1FFFFFFFu : 0x7FFu;
            uint32_t rx = j2534_ch_get_id_be(pattern->data) & idmask;
            uint32_t tx = j2534_ch_get_id_be(flow_control->data) & idmask;
            int sess = -1;
            j2534_iso_ae_t ae = { .on = has_ae };

            if (has_ae)
            {
                ae.rx = pattern->data[4];
                ae.tx = flow_control->data[4];
            }

            st = j2534_iso_bind(c, tx, rx, ext, pad, &ae, &sess);

            if (st == J2534_STATUS_NOERROR)
            {
                /* the tool re-arming the same ECU: its old record goes */
                for (int i = 0; i < c->n_filters; i++)
                {
                    if (c->filters[i].type == J2534_FLOW_CONTROL_FILTER &&
                        c->filters[i].session == sess)
                    {
                        filter_remove_at(c, i);
                        break;
                    }
                }

                j2534_ch_filter_t *f = &c->filters[c->n_filters++];

                memset(f, 0, sizeof(*f));
                f->id = c->next_filter_id++;
                f->type = type;
                f->session = sess;
                *filter_id = f->id;
            }
        }
    }
    else if (type == J2534_PASS_FILTER || type == J2534_BLOCK_FILTER)
    {
        if (mask == NULL || pattern == NULL)
        {
            st = J2534_ERR_NULL_PARAMETER;
        }
        else if (c->n_filters >= J2534_CH_MAX_FILTERS)
        {
            st = J2534_ERR_EXCEEDED_LIMIT;
        }
        else
        {
            j2534_ch_filter_t *f = &c->filters[c->n_filters];
            uint8_t n = (uint8_t)(mask->data_size < 8 ? mask->data_size : 8);

            memset(f, 0, sizeof(*f));
            f->id = c->next_filter_id++;
            f->type = type;
            f->session = -1;
            f->len = n;
            memcpy(f->mask, mask->data, n);
            memcpy(f->pattern, pattern->data,
                   pattern->data_size < n ? pattern->data_size : n);
            c->n_filters++;
            *filter_id = f->id;
        }
    }
    else
    {
        st = J2534_ERR_INVALID_MSG;
    }

    xSemaphoreGive(j2534_ch_lock);
    return st;
}

uint32_t j2534_channel_stop_filter(int slot, uint32_t filter_id)
{
    if (slot < 0 || slot >= J2534_MAX_CHANNELS || !j2534_ch_tbl[slot].in_use)
    {
        return J2534_ERR_INVALID_CHANNEL_ID;
    }

    j2534_channel_t *c = &j2534_ch_tbl[slot];
    uint32_t st = J2534_ERR_INVALID_FILTER_ID;

    xSemaphoreTake(j2534_ch_lock, portMAX_DELAY);
    for (int i = 0; i < c->n_filters; i++)
    {
        if (c->filters[i].id == filter_id)
        {
            /* A TP2.0 connection owns this slot until teardown. */
            if (c->filters[i].tp_owned)
            {
                break;
            }

            if (c->filters[i].type == J2534_FLOW_CONTROL_FILTER)
            {
                j2534_iso_unbind(c, c->filters[i].session);
            }

            filter_remove_at(c, i);
            st = J2534_STATUS_NOERROR;
            break;
        }
    }
    xSemaphoreGive(j2534_ch_lock);
    return st;
}

void j2534_channel_clear_filters(int slot)
{
    if (slot < 0 || slot >= J2534_MAX_CHANNELS)
    {
        return;
    }

    j2534_channel_t *c = &j2534_ch_tbl[slot];

    xSemaphoreTake(j2534_ch_lock, portMAX_DELAY);
    for (int i = c->n_filters - 1; i >= 0; i--)
    {
        if (c->filters[i].type == J2534_FLOW_CONTROL_FILTER)
        {
            j2534_iso_unbind(c, c->filters[i].session);
        }

        /* Implicit TP2.0 pass filters stay until the connection does. */
        if (!c->filters[i].tp_owned)
        {
            filter_remove_at(c, i);
        }
    }
    xSemaphoreGive(j2534_ch_lock);
}

bool j2534_ch_has_pass_filter(const j2534_channel_t *c)
{
    for (int i = 0; i < c->n_filters; i++)
    {
        if (c->filters[i].type == J2534_PASS_FILTER && !c->filters[i].tp_owned)
        {
            return true;
        }
    }

    return false;
}

/* apply PASS/BLOCK filters to a [id][data] view */
bool j2534_ch_filters_pass(const j2534_channel_t *c, const uint8_t *view,
                           size_t len)
{
    bool any_pass = false, has_pass = false;

    for (int i = 0; i < c->n_filters; i++)
    {
        const j2534_ch_filter_t *f = &c->filters[i];

        if (f->tp_owned || f->type == J2534_FLOW_CONTROL_FILTER)
        {
            continue; /* a session, not a user match rule */
        }

        bool m = j2534_filter_match(f->mask, f->len, f->pattern, f->len,
                                    view, len);

        if (f->type == J2534_BLOCK_FILTER && m)
        {
            return false;
        }
        if (f->type == J2534_PASS_FILTER)
        {
            has_pass = true;
            any_pass = any_pass || m;
        }
    }

    return has_pass ? any_pass : true; /* no PASS filter = pass all */
}
