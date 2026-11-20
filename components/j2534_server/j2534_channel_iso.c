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
 *
 * 2026-11-20: the receive side (the indications and the RX pump's poll) is
 * j2534_channel_iso_rx.c and the pure part of the extended addressing
 * layout is j2534_iso_ae.c (the 700-line rule: this file had grown to 848
 * lines). Here: the session table and the tester's write.
 */
#include "j2534_channel_private.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "can_isotp.h"
#include "can_manager.h"
#include "obd_gate.h"

static const char *TAG = "j2534_server";

static uint32_t now_us(void)
{
    return (uint32_t)esp_timer_get_time();
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

    j2534_iso_rx_forget(s); /* a message held for it goes with the session */

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
       The sessions of the other addressing kind on the rx id go too: the
       provider keeps one kind per id, and with extended addressing there
       can be several of them (2026-11-20: one was closed, and the open
       then failed against the next). The filter records of the ones that
       go are dropped by the caller (j2534_channel_filter.c). */
    int s = find_by_rx(c, rx_id, ext, ae);

    for (int o = find_other_kind(c, rx_id, ext, ae->on); o >= 0;
         o = find_other_kind(c, rx_id, ext, ae->on))
    {
        session_close(&c->sess[o]);

        if (s < 0)
        {
            s = o;
        }
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
    uint32_t t = 1500u + j2534_iso_frame_count(len, has_ae) * 10u;

    return (t > 5500u) ? 5500u : t;
}

uint32_t j2534_iso_write(j2534_channel_t *c, const j2534_msg_t *msg,
                         bool periodic)
{
    uint32_t id;
    const uint8_t *pl;
    size_t n;
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
    bool has_ae = (msg->tx_flags & J2534_TX_ISO15765_ADDR_TYPE) != 0;
    bool whole = has_ae ? j2534_iso_ae_split(msg, &id, &ae, &pl, &n)
                        : j2534_iso_split(msg, &id, &pl, &n);

    if (!whole || n == 0)
    {
        return J2534_ERR_INVALID_MSG;
    }

    bool ext = (msg->tx_flags & J2534_TX_CAN_29BIT_ID) != 0;

    id &= ext ? 0x1FFFFFFFu : 0x7FFu;

    bool pad = (msg->tx_flags & J2534_TX_ISO15765_FRAME_PAD) != 0;

    int s = find_by_tx(c, id, ext, has_ae, ae);

    /* without a session only a single frame can go: 7 bytes, 6 behind an
       address extension */
    if (s < 0 && n > (has_ae ? J2534_ISO_SF_MAX_AE : J2534_ISO_SF_MAX))
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
        uint8_t dlc = has_ae ? j2534_iso_ae_single_frame(ae, pl, n, pad, fr)
                             : j2534_iso_single_frame(pl, n, pad, fr);

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
        j2534_iso_indicate(c, J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION,
                           id, ext, has_ae, ae, now_us());
    }

    return J2534_STATUS_NOERROR;
}
