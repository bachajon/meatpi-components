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
 * @file j2534_channel_lost.c
 * @brief What a channel lost on the way in (2026-11-18). Runs with the
 *        channel lock held by the caller, like the rest of the channel
 *        files.
 *
 * A channel's bus queue holds 256 frames (J2534_CH_CAN_QLEN). The CAN node
 * fills it from its receive task; the RX pump empties it. A pump that is
 * held up (a tester that does not read its link, a link slower than the
 * bus) lets it run over: the node drops the oldest frame for every new one
 * and counts it for that subscription (`subscribers` on /api/can). Until
 * this day the count stopped there and the tester was never told. SAE
 * J2534-1 asks for the opposite: the application is told at the first
 * message it reads after the loss (PassThruReadMsgs returns
 * ERR_BUFFER_OVERFLOW for it). The pump asks here before it takes a
 * message and sends the tester an EVENT RX_LOST with the count in front of
 * it; the Windows driver turns that into the return value.
 *
 * The count is in bus frames. On a raw CAN channel a frame is a message;
 * some of the frames lost may be ones a filter of the channel would have
 * kept out. On an ISO15765 and a TP2.0 channel the queue is what the
 * channel reads beside its ISO-TP sessions (first frames looked at for the
 * START_OF_MESSAGE indication, single-frame answers to a functional
 * request, TP2.0 telegrams): the count says how many frames went there.
 *
 * 2026-11-20: pull request 5 made an ISO15765 and a TP2.0 channel report
 * nothing, because Toyota's Calibration Update Wizard gives up on
 * ERR_BUFFER_OVERFLOW. Not kept. What that tool met was the indication
 * ring of three (lost_pending, counted here): five TxDone of one write,
 * two dropped; the ring holds 63 since the same pull request. An
 * indication the ring drops is a message the tester does not get, and an
 * answer to a functional request comes off this very queue through the
 * PASS filters: saying nothing about those is the fault of before
 * 2026-11-18 again. What stays true of that change's reasoning: this
 * queue takes every frame of the bus, so on a busy bus a pump that is
 * held up reports frames the tester never asked for.
 */
#include "j2534_channel_private.h"

#include "can_manager.h"

uint32_t j2534_ch_take_lost(j2534_channel_t *c)
{
    uint32_t n = c->lost_pending;
    can_manager_subscriber_t sub;

    c->lost_pending = 0;

    if (c->can_sub >= 0 && can_manager_subscriber_get(c->can_sub, &sub))
    {
        n += j2534_lost_since(sub.drops, &c->sub_drops_seen);
    }

    return n;
}
