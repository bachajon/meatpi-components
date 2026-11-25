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
 * @file bridge_manager_peers.c
 * @brief The peers at the far end of a bridge (2026-11-25, GitHub issue
 *        #1078). PURE: no FreeRTOS call, host-tested.
 *
 *        A bridge moves bytes and knew nothing of who was there to send
 *        them. The `obd` jack needs to know: an app that opened a monitor
 *        holds the OBD chip until the monitor ends, and an app that went
 *        away in the middle of one held it until the cap of two minutes.
 *        Three optional members of bridge_endpoint_t carry it:
 *
 *        - `peers`: an endpoint that can count who is connected to it (a
 *          TCP server, a WebSocket channel, Bluetooth) says how many.
 *        - `send_from`: an endpoint that wants to know where its bytes
 *          come from gets the far endpoint with them.
 *        - `far_gone`: it is told when that far endpoint had peers and has
 *          none left.
 *
 *        The far endpoint is named by the address of its descriptor: an
 *        identity, nothing to read through. An endpoint that cannot count
 *        (a UART, a UDP port) is never reported gone.
 */
#include "bridge_manager.h"

#include "bridge_manager_peers.h"

esp_err_t bm_peers_send(const bridge_endpoint_t *dst,
                        const bridge_endpoint_t *src, const uint8_t *data,
                        size_t len)
{
    return (dst->send_from != NULL) ? dst->send_from(src, data, len)
                                    : dst->send(data, len);
}

/** One direction: @p src is the end whose peers are counted, @p dst the
 *  end that is told. */
static void poll_one(bool *had, const bridge_endpoint_t *src,
                     const bridge_endpoint_t *dst)
{
    if (src->peers == NULL || dst->far_gone == NULL)
    {
        return;
    }

    bool has = (src->peers() > 0);

    if (*had && !has)
    {
        dst->far_gone(src);
    }

    *had = has;
}

void bm_peers_poll(bm_peers_t *st, const bridge_endpoint_t *a,
                   const bridge_endpoint_t *b)
{
    poll_one(&st->had[0], a, b);
    poll_one(&st->had[1], b, a);
}
