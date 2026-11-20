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
 * @file j2534_channel_iso.c
 * @brief The ISO15765 side of a J2534 channel: one ISO-TP session per
 *        FLOW_CONTROL filter on the registered provider (can_isotp.h),
 *        the SAE J2534-1 message layout [4-byte id][payload] both ways
 *        ([4-byte id][address extension][payload] with extended
 *        addressing, ISO15765_ADDR_TYPE),
 *        single frames without a session (functional addressing), and the
 *        TxDone and START_OF_MESSAGE indications. Every function runs with
 *        the channel lock held by j2534_channel.c; j2534_iso_write alone
 *        gives it up while the tester's own message is on the bus
 *        (2026-11-18, j2534_channel_private.h says why and what keeps
 *        that safe).
 *
 * 2026-10-19: a udsoncan tester's `10 03` reached the bus as the 6-byte
 * PDU `00 00 07 E0 10 03`, unpadded, because the device took the bare
 * payload and read the pad flag from CONNECT. Standard tools put the id
 * first and the pad flag on every message; the device does the same now.
 *
 * 2026-11-19: extended addressing in both directions. A FLOW_CONTROL
 * filter with ISO15765_ADDR_TYPE (5-byte pattern and flow-control message)
 * opens a provider session with the two address-extension bytes; the
 * tester's messages to it go through that session (single and segmented),
 * and what the ECU answers comes back as [id][address extension][payload]
 * with ISO15765_ADDR_TYPE in RxStatus. Toyota's central gateway (0x750 /
 * 0x758, one address extension per ECU behind it) speaks this; until now
 * only a single frame went out, around the provider, and nothing came
 * back.
 */
#include "j2534_channel_private.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "can_core.h"
#include "can_isotp.h"
#include "can_manager.h"
#include "obd_gate.h"

static const char *TAG = "j2534_server";

/* the reassembled PDU scratch: PSRAM, one RX pump drains every channel */
EXT_RAM_BSS_ATTR static uint8_t s_pdu[J2534_MAX_DATA];

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

bool j2534_iso_provider_ready(void)
{
    return can_isotp() != NULL && can_manager_core_handle() != NULL;
}

/* ---- the session table ------------------------------------------------ */

static int find_free(const j2534_channel_t *c)
{
    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        if (!c->sess[i].in_use)
        {
            return i;
        }
    }

    return -1;
}

/* the session of an rx id; with extended addressing, of the rx id and the
   address extension its frames arrive with */
static int find_by_rx(const j2534_channel_t *c, uint32_t rx_id, bool ext,
                      const j2534_iso_ae_t *ae)
{
    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        if (c->sess[i].in_use && c->sess[i].rx_id == rx_id &&
            c->sess[i].ext == ext && c->sess[i].ae.on == ae->on &&
            (!ae->on || c->sess[i].ae.rx == ae->rx))
        {
            return i;
        }
    }

    return -1;
}

/* a session on the rx id of the other addressing kind (normal when
   @p ae_on, extended when not): the two cannot share an id */
static int find_other_kind(const j2534_channel_t *c, uint32_t rx_id, bool ext,
                           bool ae_on)
{
    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        if (c->sess[i].in_use && c->sess[i].rx_id == rx_id &&
            c->sess[i].ext == ext && c->sess[i].ae.on != ae_on)
        {
            return i;
        }
    }

    return -1;
}

static int find_by_tx(const j2534_channel_t *c, uint32_t tx_id, bool ext,
                      bool has_ae, uint8_t ae)
{
    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        if (c->sess[i].in_use && c->sess[i].tx_id == tx_id &&
            c->sess[i].ext == ext && c->sess[i].ae.on == has_ae &&
            (!has_ae || c->sess[i].ae.tx == ae))
        {
            return i;
        }
    }

    return -1;
}

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

/* open the provider session for a record whose ids and padding are set */
static uint32_t session_open(j2534_channel_t *c, j2534_iso_session_t *s)
{
    uint32_t st_us = c->iso_st_us;
    uint8_t stmin_ms = 0;

    if (st_us >= 1000u && (st_us % 1000u) == 0 && st_us / 1000u <= 127u)
    {
        stmin_ms = (uint8_t)(st_us / 1000u);
    }

    can_isotp_cfg_t cfg = {
        .tx_id        = s->tx_id,
        .rx_id        = s->rx_id,
        .ext_id       = s->ext,
        .block_size   = c->iso_bs,
        .stmin_ms     = stmin_ms,
        .stmin_us     = st_us,
        .padding_byte = 0xAA,
        .use_padding  = s->pad,
        .addr_ext     = s->ae.on,
        .tx_ae        = s->ae.tx,
        .rx_ae        = s->ae.rx,
    };
    esp_err_t err = can_isotp()->open(&cfg, &s->isotp);

    if (err == ESP_OK)
    {
        s->in_use = true;

        if (s->ae.on)
        {
            ESP_LOGI(TAG, "ISO15765 session tx %03lX/%02X rx %03lX/%02X%s%s "
                          "(extended addressing)",
                     (unsigned long)s->tx_id, s->ae.tx,
                     (unsigned long)s->rx_id, s->ae.rx,
                     s->ext ? " 29-bit" : "", s->pad ? " padded" : "");
        }
        else
        {
            ESP_LOGI(TAG, "ISO15765 session tx %03lX rx %03lX%s%s",
                     (unsigned long)s->tx_id, (unsigned long)s->rx_id,
                     s->ext ? " 29-bit" : "", s->pad ? " padded" : "");
        }

        return J2534_STATUS_NOERROR;
    }

    s->isotp = NULL;
    s->in_use = false;

    if (err == ESP_ERR_NO_MEM)
    {
        ESP_LOGW(TAG, "ISO15765: the ISO-TP provider has no session left "
                      "for rx %03lX", (unsigned long)s->rx_id);
        return J2534_ERR_EXCEEDED_LIMIT;
    }

    /* expected refusals: the UDS terminal holds the id for 5 s after its
       last request, or the CAN node is down (the ACK carries the verdict) */
    ESP_LOGW(TAG, "ISO15765 session tx %03lX rx %03lX refused: %s",
             (unsigned long)s->tx_id, (unsigned long)s->rx_id,
             esp_err_to_name(err));
    return J2534_ERR_FAILED;
}

static void session_close(j2534_iso_session_t *s)
{
    if (s->in_use && s->isotp != NULL)
    {
        can_isotp()->close(s->isotp);
    }

    memset(s, 0, sizeof(*s));
}

uint32_t j2534_iso_bind(j2534_channel_t *c, uint32_t tx_id, uint32_t rx_id,
                        bool ext, bool pad, const j2534_iso_ae_t *ae,
                        int *slot)
{
    static const j2534_iso_ae_t NO_AE = { .on = false };

    if (ae == NULL || !ae->on)
    {
        ae = &NO_AE;
    }

    if (!j2534_iso_provider_ready())
    {
        ESP_LOGW(TAG, "ISO15765 not available (no ISO-TP provider, or the "
                      "CAN node is down)");
        return J2534_ERR_FAILED;
    }

    /* the same ECU again: the tool re-arms its filter, rebind in place.
       So does a session of the other addressing kind on the rx id: the
       provider keeps one kind per id. */
    int s = find_by_rx(c, rx_id, ext, ae);

    if (s < 0)
    {
        s = find_other_kind(c, rx_id, ext, ae->on);
    }

    if (s >= 0)
    {
        session_close(&c->sess[s]);
    }
    else
    {
        s = find_free(c);

        if (s < 0)
        {
            ESP_LOGW(TAG, "ISO15765: the channel's %d sessions are in use",
                     J2534_CH_MAX_SESSIONS);
            return J2534_ERR_EXCEEDED_LIMIT;
        }
    }

    c->sess[s].tx_id = tx_id;
    c->sess[s].rx_id = rx_id;
    c->sess[s].ext = ext;
    c->sess[s].ae = *ae;
    c->sess[s].pad = pad;

    uint32_t st = session_open(c, &c->sess[s]);

    if (st == J2534_STATUS_NOERROR && slot != NULL)
    {
        *slot = s;
    }

    return st;
}

uint32_t j2534_iso_config(j2534_channel_t *c, bool get,
                          const uint8_t *body, uint32_t body_len,
                          uint8_t *out, uint32_t out_cap, uint32_t *out_len)
{
    j2534_iso_flow_t flow = { .bs = c->iso_bs, .st_us = c->iso_st_us };
    bool changed = false;

    if (get)
    {
        return j2534_iso_cfg_get(body, body_len, &flow, out, out_cap, out_len);
    }

    uint32_t st = j2534_iso_cfg_set(body, body_len, &flow, &changed);

    if (st != J2534_STATUS_NOERROR || !changed)
    {
        return st;
    }

    c->iso_bs = flow.bs;
    c->iso_st_us = flow.st_us;

    const can_isotp_ops_t *ops = can_isotp();

    if (ops == NULL || ops->set_flow == NULL)
    {
        /* a provider without the call: the sessions opened from here on
           carry the values, the open ones keep theirs */
        return J2534_STATUS_NOERROR;
    }

    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        if (c->sess[i].in_use && c->sess[i].isotp != NULL)
        {
            (void)ops->set_flow(c->sess[i].isotp, flow.bs, flow.st_us);
        }
    }

    ESP_LOGD(TAG, "ISO15765 flow control: BS %u, STmin %lu us",
             (unsigned)flow.bs, (unsigned long)flow.st_us);
    return J2534_STATUS_NOERROR;
}

void j2534_iso_unbind(j2534_channel_t *c, int slot)
{
    if (slot >= 0 && slot < J2534_CH_MAX_SESSIONS)
    {
        session_close(&c->sess[slot]);
    }
}

void j2534_iso_unbind_all(j2534_channel_t *c)
{
    for (int i = 0; i < J2534_CH_MAX_SESSIONS; i++)
    {
        session_close(&c->sess[i]);
    }

    c->ind_head = 0;
    c->ind_tail = 0;
}

/* ---- indications ------------------------------------------------------ */

static void indicate(j2534_channel_t *c, uint32_t rx_status, uint32_t id,
                     bool ext, bool has_ae, uint8_t ae)
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
    c->ind[c->ind_head].ts_us = now_us();
    c->ind_head = next;
}

/* ---- write ------------------------------------------------------------ */

/* padding follows each message: a session opened the other way is reopened */
static bool pad_rebind(j2534_channel_t *c, j2534_iso_session_t *s, bool pad)
{
    if (s->pad == pad)
    {
        return true;
    }

    uint32_t tx = s->tx_id;
    uint32_t rx = s->rx_id;
    bool ext = s->ext;
    j2534_iso_ae_t ae = s->ae;

    session_close(s);
    s->tx_id = tx;
    s->rx_id = rx;
    s->ext = ext;
    s->ae = ae;
    s->pad = pad;
    return session_open(c, s) == J2534_STATUS_NOERROR;
}

/* the send's own wait: flow control after the first frame, then the
   consecutive frames at the ECU's STmin; capped under the DLL's 6 s ACK
   wait (a 4 KB block at STmin 5 ms needs about 3 s) */
static uint32_t send_timeout_ms(size_t len, bool has_ae)
{
    /* a frame carries one byte less behind an address extension */
    size_t sf = has_ae ? 6u : 7u;   /* single / consecutive frame */
    size_t ff = sf - 1u;            /* first frame                */
    uint32_t frames = (len <= sf) ? 1u
                                  : 1u + (uint32_t)((len - ff + sf - 1u) / sf);
    uint32_t t = 1500u + frames * 10u;

    return (t > 5500u) ? 5500u : t;
}

uint32_t j2534_iso_write(j2534_channel_t *c, const j2534_msg_t *msg,
                         bool periodic)
{
    uint32_t id;
    const uint8_t *pl;
    size_t n;
    bool    has_ae = false;   /* ISO 15765-2 extended/mixed addressing */
    uint8_t ae = 0;           /* the address-extension byte            */

    /* Extended addressing: the SAE J2534-1 message is
       [4-byte CAN id][1-byte address extension][payload], and every CAN
       frame is led by that address-extension byte, so a single frame holds
       6 payload bytes (not 7). Toyota's central gateway (0x750/0x758) speaks
       this, and the stock tools (Techstream, CUW, PCMflash) cannot reach an
       engine ECU behind the gateway without it. The message goes through
       the session its FLOW_CONTROL filter opened for that id and address
       extension (single or segmented; the answer comes back through it,
       j2534_iso_poll); without one a single frame still goes out directly,
       as a functional request does. */
    if ((msg->tx_flags & J2534_TX_ISO15765_ADDR_TYPE) != 0)
    {
        if (msg->data_size < 5 || msg->data_size > (uint32_t)J2534_MAX_DATA)
        {
            return J2534_ERR_INVALID_MSG;
        }

        id = ((uint32_t)msg->data[0] << 24) | ((uint32_t)msg->data[1] << 16) |
             ((uint32_t)msg->data[2] << 8) | (uint32_t)msg->data[3];
        ae = msg->data[4];
        pl = msg->data + 5;
        n  = (size_t)msg->data_size - 5u;
        has_ae = true;
    }
    else if (!j2534_iso_split(msg, &id, &pl, &n))
    {
        return J2534_ERR_INVALID_MSG;
    }

    if (n == 0)
    {
        return J2534_ERR_INVALID_MSG;
    }

    bool ext = (msg->tx_flags & J2534_TX_CAN_29BIT_ID) != 0;

    id &= ext ? 0x1FFFFFFFu : 0x7FFu;

    bool pad = (msg->tx_flags & J2534_TX_ISO15765_FRAME_PAD) != 0;

    int s = find_by_tx(c, id, ext, has_ae, ae);

    /* without a session only a single frame can go: 7 bytes, 6 behind an
       address extension */
    if (s < 0 && n > (has_ae ? 6u : 7u))
    {
        ESP_LOGD(TAG, "ISO15765 %u B to %03lX: no flow-control filter",
                 (unsigned)n, (unsigned long)id);
        return J2534_ERR_NO_FLOW_CONTROL;
    }

    if (!can_manager_tx_ready())
    {
        j2534_ch_say_refused("ISO15765");
        return J2534_ERR_FAILED;
    }

    j2534_iso_session_t *ss = (s >= 0) ? &c->sess[s] : NULL;

    if (ss != NULL)
    {
        /* the tester's own write to this ECU is on the bus (only a periodic
           message meets this: that write gave the lock up, see below). No
           periodic message into a transfer to the same address, and nobody
           reopens the session under it: not now. */
        if (ss->tx_busy)
        {
            return J2534_ERR_BUFFER_FULL;
        }

        if (!pad_rebind(c, ss, pad))
        {
            return J2534_ERR_FAILED;
        }
    }

    /* The tester's own write gives the channel lock up from here until the
       message is on the bus: the wait for the gate (up to 3 s) and a
       multi-frame transfer at the ECU's pace (up to 5.5 s). The RX pump
       and the periodic messages take the lock meanwhile. What they may not
       touch is this session (tx_busy); the channel itself stays, because
       the task that closes channels is the one running here. */
    can_isotp_session_t tp = (ss != NULL) ? ss->isotp : NULL;
    bool yield_lock = !periodic;

    if (yield_lock)
    {
        if (ss != NULL)
        {
            ss->tx_busy = true;
        }

        xSemaphoreGive(j2534_ch_lock);
    }

    /* an ESP-side requester on the shared bus: hold the conversation gate
       from the request until the reply is delivered (or the hold expires)
       so the MIC chip does not interleave; fail-open (obd_gate.h) */
    obd_gate_acquire(c, OBD_GATE_WAIT_MS);

    esp_err_t err;

    if (ss != NULL)
    {
        err = can_isotp()->send(tp, pl, n, send_timeout_ms(n, has_ae));
    }
    else
    {
        /* functional addressing, or an extended-addressed single frame to
           an address no FLOW_CONTROL filter names: one frame, no session */
        uint8_t fr[8];
        uint8_t dlc;

        if (has_ae)
        {
            fr[0] = ae;            /* address extension leads the frame      */
            fr[1] = (uint8_t)n;    /* single-frame PCI 0x0n (AE not counted) */
            memcpy(fr + 2, pl, n);
            dlc = (uint8_t)(2u + n);

            if (pad && dlc < 8)
            {
                memset(fr + dlc, 0xAA, (size_t)(8 - dlc));
                dlc = 8;
            }
        }
        else
        {
            dlc = j2534_iso_single_frame(pl, n, pad, fr);
        }

        err = can_manager_send(id, ext, false, fr, dlc);
    }

    if (yield_lock)
    {
        xSemaphoreTake(j2534_ch_lock, portMAX_DELAY);

        if (ss != NULL)
        {
            ss->tx_busy = false;
        }
    }

    if (err != ESP_OK)
    {
        obd_gate_release(c);

        if (err == ESP_ERR_INVALID_STATE)
        {
            j2534_ch_say_refused("ISO15765");
        }

        ESP_LOGD(TAG, "ISO15765 send to %03lX: %s", (unsigned long)id,
                 esp_err_to_name(err));
        return (err == ESP_ERR_TIMEOUT) ? J2534_ERR_TIMEOUT : J2534_ERR_FAILED;
    }

    j2534_ch_note_reflash(pl[0]); /* forwarded: counted on the status, never refused */

    if (!periodic)
    {
        indicate(c, J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION, id, ext,
                 has_ae, ae);
    }

    return J2534_STATUS_NOERROR;
}

/* ---- poll ------------------------------------------------------------- */

/* an indication of an extended-addressing session: the address extension
   follows the id (SAE J2534-1), ISO15765_ADDR_TYPE says so */
static void indication_add_ae(j2534_msg_t *out, uint8_t ae)
{
    out->data[4] = ae;
    out->data_size = 5;
    out->extra_data_index = 5;
    out->rx_status |= J2534_RX_ISO15765_ADDR_TYPE;
}

/* a session's reassembled message for the tester: [id][payload], and
   [id][address extension][payload] with ISO15765_ADDR_TYPE for an
   extended-addressing session */
static bool session_join(j2534_msg_t *out, const j2534_iso_session_t *s,
                         const uint8_t *payload, size_t len, uint32_t ts_us)
{
    if (!s->ae.on)
    {
        return j2534_iso_join(out, s->rx_id, s->ext, payload, len, ts_us);
    }

    if (len > (size_t)J2534_MAX_DATA - 5 ||
        !j2534_iso_join(out, s->rx_id, s->ext, NULL, 0, ts_us))
    {
        return false;
    }

    out->data[4] = s->ae.rx;
    memcpy(out->data + 5, payload, len);
    out->data_size = (uint32_t)(5 + len);
    out->extra_data_index = out->data_size;
    out->rx_status |= J2534_RX_ISO15765_ADDR_TYPE;
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
            indication_add_ae(out, i->ae);
        }

        c->ind_tail = (uint8_t)((c->ind_tail + 1) % J2534_CH_MAX_IND);
        return true;
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
            /* behind an address extension the PCI is the second byte */
            uint8_t off = (si >= 0 && c->sess[si].ae.on) ? 1 : 0;

            if (si >= 0 &&
                j2534_iso_is_first_frame(fr.data + off, (uint8_t)(fr.dlc - off)))
            {
                j2534_iso_indication(out, fr.id, fr.ext,
                                     J2534_RX_START_OF_MESSAGE, ts);

                if (off != 0)
                {
                    indication_add_ae(out, c->sess[si].ae.rx);
                }

                return true;
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
