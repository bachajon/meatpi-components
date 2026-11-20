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
 * @file j2534_channel_iso_rx.c
 * @brief The receive side of an ISO15765 J2534 channel: the ring of TxDone
 *        and START_OF_MESSAGE indications, and the RX pump's poll, which
 *        hands the tester one indication or one received message at a time
 *        in the order of the bus. Out of j2534_channel_iso.c (2026-11-20,
 *        the 700-line rule: with pull request 5 it had grown to 848 lines);
 *        the code is that file's, but for the names the two now share and
 *        the layout helpers of j2534_iso_ae.c. Every function runs with the
 *        channel lock held by j2534_channel.c.
 *
 * What a poll looks at, in this order:
 *   1.  an indication waiting in the ring;
 *   1b. the message held back at the last poll (see 2.);
 *   2.  the channel's bus queue: a first frame on a session's id is the
 *       START_OF_MESSAGE indication, a single frame on an id no session
 *       owns is an answer to a functional request and goes through the
 *       PASS filters (normal addressing only: an extended-addressing
 *       answer is received through a FLOW_CONTROL filter);
 *   3.  what the sessions have reassembled.
 *
 * 2026-11-19 (pull request 5): a message a session finished BEFORE a first
 * frame on its id is handed over in front of that frame's
 * START_OF_MESSAGE, as it was on the bus. An ECU that acknowledges with a
 * single frame and then answers in several on the same id (the loader
 * login of Toyota's Calibration Update Wizard) was read the other way
 * round until then: the indication of the second message, then the first
 * message.
 */
#include "j2534_channel_private.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_timer.h"

#include "can_core.h"
#include "can_isotp.h"
#include "obd_gate.h"

/* the reassembled PDU scratch: PSRAM, one RX pump drains every channel */
EXT_RAM_BSS_ATTR static uint8_t s_pdu[J2534_MAX_DATA];

/* a PDU taken out of a session ahead of its turn (j2534_iso_poll: the
   message whose START_OF_MESSAGE is only now being told), kept for the
   next poll of its channel. One: there is one RX pump. */
EXT_RAM_BSS_ATTR static uint8_t s_held[J2534_MAX_DATA];
static const j2534_iso_session_t *s_held_sess;
static size_t s_held_len;

static uint32_t now_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

static void put_id_be(uint8_t *p, uint32_t id)
{
    p[0] = (uint8_t)(id >> 24);
    p[1] = (uint8_t)(id >> 16);
    p[2] = (uint8_t)(id >> 8);
    p[3] = (uint8_t)id;
}

void j2534_iso_rx_forget(const j2534_iso_session_t *s)
{
    if (s_held_sess == s)
    {
        s_held_sess = NULL;
    }
}

/* ---- indications ------------------------------------------------------ */

void j2534_iso_indicate(j2534_channel_t *c, uint32_t rx_status, uint32_t id,
                        bool ext, bool has_ae, uint8_t ae, uint32_t ts_us)
{
    uint8_t next = (uint8_t)((c->ind_head + 1) % J2534_CH_MAX_IND);

    if (next == c->ind_tail)
    {
        /* the pump is behind: the oldest indication goes, and the tester
           is told one is missing (j2534_ch_take_lost) */
        c->ind_tail = (uint8_t)((c->ind_tail + 1) % J2534_CH_MAX_IND);
        c->lost_pending++;
    }

    c->ind[c->ind_head].rx_status = rx_status;
    c->ind[c->ind_head].id = id;
    c->ind[c->ind_head].ext = ext;
    c->ind[c->ind_head].has_ae = has_ae;
    c->ind[c->ind_head].ae = ae;
    c->ind[c->ind_head].ts_us = ts_us;
    c->ind_head = next;
}

/* ---- poll ------------------------------------------------------------- */

/* The session a frame off the bus belongs to, -1 for none. *owned tells
   whether the frame's id has a session at all: with extended addressing a
   frame led by an address extension nobody here listens to is still not a
   functional reply to hand out through the PASS filters. */
static int find_by_frame(const j2534_channel_t *c, const can_core_frame_t *fr,
                         bool *owned)
{
    *owned = false;

    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        const j2534_iso_session_t *s = &c->sess[i];

        if (!s->in_use || s->rx_id != fr->id || s->ext != fr->ext)
        {
            continue;
        }

        *owned = true;

        if (!s->ae.on || (fr->dlc >= 1 && fr->data[0] == s->ae.rx))
        {
            return i;
        }
    }

    return -1;
}

/* a session's reassembled message for the tester: [id][payload], and
   [id][address extension][payload] with ISO15765_ADDR_TYPE for an
   extended-addressing session */
static bool session_join(j2534_msg_t *out, const j2534_iso_session_t *s,
                         const uint8_t *payload, size_t len, uint32_t ts_us)
{
    if (s->ae.on)
    {
        return j2534_iso_ae_join(out, s->rx_id, s->ext, s->ae.rx, payload,
                                 len, ts_us);
    }

    return j2534_iso_join(out, s->rx_id, s->ext, payload, len, ts_us);
}

/* A first frame came on the id of session @p s; @p pci is the frame from
   its PCI byte on (behind the address extension when there is one), @p n
   bytes. True with @p out filled: the START_OF_MESSAGE indication, or the
   message the session finished before this frame (the indication then
   waits in the ring for the next poll). */
static bool first_frame(j2534_channel_t *c, j2534_iso_session_t *s,
                        const can_core_frame_t *fr, const uint8_t *pci,
                        uint8_t n, uint32_t ts, j2534_msg_t *out)
{
    size_t got = 0;

    /* A message this session finished before this first frame came (an
       ECU's single-frame acknowledgement, then its multi-frame answer on
       the same id: Toyota's loader login) is read before the
       START_OF_MESSAGE of the next one, as it was on the bus. Until
       2026-11-19 the indication went first, whatever the session still
       held. */
    if (s_held_sess == NULL &&
        can_isotp()->recv(s->isotp, s_pdu, sizeof(s_pdu), &got, 0) == ESP_OK &&
        got > 0)
    {
        if (j2534_iso_pdu_of_first_frame(s_pdu, got, pci, n))
        {
            /* the pump is late and the message of this very first frame
               is whole already: the indication now, the message at the
               next poll */
            memcpy(s_held, s_pdu, got);
            s_held_len = got;
            s_held_sess = s;
        }
        else if (session_join(out, s, s_pdu, got, now_us()))
        {
            j2534_iso_indicate(c, J2534_RX_START_OF_MESSAGE, fr->id, fr->ext,
                               s->ae.on, s->ae.rx, ts);
            obd_gate_release(c);
            return true;
        }
    }

    j2534_iso_indication(out, fr->id, fr->ext, J2534_RX_START_OF_MESSAGE, ts);

    if (s->ae.on)
    {
        j2534_iso_ae_indication(out, s->ae.rx);
    }

    return true;
}

bool j2534_iso_poll(j2534_channel_t *c, j2534_msg_t *out)
{
    /* 1. a pending indication */
    if (c->ind_tail != c->ind_head)
    {
        const j2534_ch_ind_t *i = &c->ind[c->ind_tail];

        j2534_iso_indication(out, i->id, i->ext, i->rx_status, i->ts_us);

        if (i->has_ae)
        {
            j2534_iso_ae_indication(out, i->ae);
        }

        c->ind_tail = (uint8_t)((c->ind_tail + 1) % J2534_CH_MAX_IND);
        return true;
    }

    /* 1b. the message a START_OF_MESSAGE was just told for (see 2.) */
    if (s_held_sess != NULL && s_held_sess >= c->sess &&
        s_held_sess < c->sess + J2534_CH_MAX_SESSIONS)
    {
        const j2534_iso_session_t *s = s_held_sess;

        s_held_sess = NULL;

        if (s->in_use && session_join(out, s, s_held, s_held_len, now_us()))
        {
            obd_gate_release(c);
            return true;
        }
    }

    /* 2. the bus frames: a first frame on a session's id announces a
          message; a single frame on an id no session owns is a reply to
          a functional request, delivered through the PASS filters */
    can_core_frame_t fr;
    bool pass_armed = j2534_ch_has_pass_filter(c);

    while (c->can_q != NULL && xQueueReceive(c->can_q, &fr, 0) == pdTRUE)
    {
        if (fr.rtr || fr.dlc > 8)
        {
            continue;
        }

        uint32_t ts = fr.ts_us ? fr.ts_us : now_us();

        bool owned;
        int si = find_by_frame(c, &fr, &owned);

        if (owned)
        {
            if (si >= 0)
            {
                /* behind an address extension the PCI is the second byte */
                uint8_t off = c->sess[si].ae.on ? 1 : 0;
                uint8_t n = (uint8_t)(fr.dlc - off);

                if (j2534_iso_is_first_frame(fr.data + off, n))
                {
                    return first_frame(c, &c->sess[si], &fr, fr.data + off,
                                       n, ts, out);
                }
            }

            continue; /* the session reassembles the rest */
        }

        if (!pass_armed)
        {
            continue; /* ISO15765 delivers nothing it was not asked for */
        }

        uint8_t view[12];
        size_t n;

        put_id_be(view, fr.id);
        memcpy(view + 4, fr.data, fr.dlc);

        if (j2534_ch_filters_pass(c, view, 4u + fr.dlc) &&
            j2534_iso_parse_single_frame(fr.data, fr.dlc, &n) &&
            j2534_iso_join(out, fr.id, fr.ext, fr.data + 1, n, ts))
        {
            obd_gate_release(c);
            return true;
        }
    }

    /* 3. the sessions' reassembled PDUs */
    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        j2534_iso_session_t *s = &c->sess[i];
        size_t n = 0;

        if (!s->in_use)
        {
            continue;
        }

        if (can_isotp()->recv(s->isotp, s_pdu, sizeof(s_pdu), &n, 0) == ESP_OK &&
            n > 0 && session_join(out, s, s_pdu, n, now_us()))
        {
            obd_gate_release(c); /* the reply is in: conversation over */
            return true;
        }
    }

    return false;
}
