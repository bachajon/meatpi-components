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
 * @file j2534_channel_private.h
 * @brief The channel record shared by j2534_channel.c (lifecycle, CAN),
 *        j2534_channel_filter.c (the filter table) and
 *        j2534_channel_iso.c (the ISO15765 side). Every function here
 *        runs with the channel lock held by the caller.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "can_isotp.h"

#include "j2534_channel.h"
#include "j2534_iso_cfg.h"
#include "j2534_proto.h"

typedef struct j2534_tp20_state j2534_tp20_state_t;

/* A channel's bus queue takes what the CAN node's receive queue can release
 * at once: after a flash write on the unit the frames of that time arrive
 * in one go (can_core_driver.c, CAN_CORE_RXQ_DEPTH). At 32 a run at the
 * line rate with the flash written meanwhile dropped 201 frames here
 * (dispatch_drops, bench 2026-11-17) that the node itself had kept. PSRAM,
 * 16 bytes a frame, one queue per channel. */
#define J2534_CH_CAN_QLEN 256
/* pending TxDone / START_OF_MESSAGE indications. One PassThruWriteMsgs
   batch queues a TxDone for each of its messages before the RX pump takes
   the first: at 4 (three held) the two oldest of Toyota CUW's five-frame
   loader login were dropped, and the tester never saw them (2026-11-19). */
#define J2534_CH_MAX_IND  64

typedef struct
{
    uint32_t id;        /* filter id (1-based, per channel)                    */
    uint32_t type;      /* PASS / BLOCK / FLOW_CONTROL                         */
    uint8_t  mask[8];
    uint8_t  pattern[8];
    uint8_t  len;
    int      session;   /* FLOW_CONTROL: the ISO-TP session it bound, else -1 */
    bool     tp_owned;  /* TP2.0 implicit pass; CLEAR_MSG_FILTERS keeps it */
} j2534_ch_filter_t;

/* ISO 15765-2 extended addressing of a session (ISO15765_ADDR_TYPE, SAE
   J2534-1): the fifth byte of the FLOW_CONTROL filter's pattern (rx) and
   flow-control message (tx), which leads every frame on the bus. */
typedef struct
{
    bool    on;
    uint8_t tx;     /* in front of every frame sent to the ECU    */
    uint8_t rx;     /* in front of every frame the ECU sends back */
} j2534_iso_ae_t;

typedef struct
{
    bool                in_use;
    uint32_t            tx_id;
    uint32_t            rx_id;
    bool                ext;    /* 29-bit ids (not the address extension) */
    j2534_iso_ae_t      ae;     /* extended addressing, else .on false    */
    bool                pad;    /* frames padded to 8 bytes (the message TxFlags) */
    bool                tx_busy; /* the tester's write to this ECU is on the bus,
                                    the channel lock given up meanwhile
                                    (j2534_iso_write): nobody reopens the
                                    session or sends on it until it is back */
    can_isotp_session_t isotp;
} j2534_iso_session_t;

typedef struct
{
    uint32_t rx_status;
    uint32_t id;
    bool     ext;
    bool     has_ae;    /* extended addressing: the indication carries */
    uint8_t  ae;        /* this address extension after the id         */
    uint32_t ts_us;
} j2534_ch_ind_t;

typedef struct j2534_channel
{
    bool     in_use;
    uint32_t protocol;
    uint32_t flags;
    bool     ext;                    /* CONNECT: 29-bit ids                      */
    uint8_t  iso_bs;                 /* flow control BS this channel sends       */
    uint32_t iso_st_us;              /* flow control STmin, microseconds         */

    QueueHandle_t can_q;             /* CAN: every frame on the bus; ISO15765:   */
    int           can_sub;           /* first-frame peeks + frames no session owns */
    uint32_t      sub_drops_seen;    /* can_q's drop counter at the last look    */
    uint32_t      lost_pending;      /* lost otherwise (an indication the ring
                                        had no room for), not yet reported       */

    j2534_ch_filter_t filters[J2534_CH_MAX_FILTERS];
    int               n_filters;
    uint32_t          next_filter_id;

    j2534_iso_session_t sess[J2534_CH_MAX_SESSIONS];

    j2534_ch_ind_t ind[J2534_CH_MAX_IND];   /* ring, drained by the RX pump     */
    uint8_t        ind_head;
    uint8_t        ind_tail;

    uint8_t leg_hdr[J2534_J1850_HDR];  /* J1850 (legacy): the ATSH header cache  */
    bool    leg_have_hdr;              /* so a repeated header is not re-sent     */

    /* TP2.0 (protocol 0x800E) only. Raw CAN leaves this NULL. */
    j2534_tp20_state_t *tp20;
} j2534_channel_t;

#include "j2534_channel_tp20.h"

/* ---- j2534_channel.c, used by the ISO15765 file ---------------------- */

/** The once-per-CONNECT "write refused" line + the tx_refused counter. */
void j2534_ch_say_refused(const char *what);

/** A tester's request service byte: a UDS memory-transfer service
 *  (0x34..0x37) is counted on the status (`reflash_seen`), never refused
 *  (the content gate went on 2026-10-27). */
void j2534_ch_note_reflash(uint8_t sid);

/* ---- j2534_channel.c, used by the filter file ------------------------ */

/** The channel records, one per slot: J2534_MAX_CHANNELS of them (PSRAM). */
extern j2534_channel_t j2534_ch_tbl[];

/** The channel lock: one mutex for all the records, taken by every
 *  j2534_channel_* entry point around its work on a slot. */
extern SemaphoreHandle_t j2534_ch_lock;

/** The CAN id a message's data begins with: 4 bytes, big-endian. */
uint32_t j2534_ch_get_id_be(const uint8_t *p);

/* ---- j2534_channel_lost.c -------------------------------------------- */

/**
 * The received messages the channel lost since the last call: what its bus
 * queue dropped because it was full (the CAN node's count for that
 * subscription) and what the channel dropped itself. They are missing in
 * front of whatever the channel hands over next, so the caller asks BEFORE
 * it takes a message: a loss is then never reported in front of a message
 * that came before it (one that falls between this look and the take is
 * reported at the next look, one message late).
 */
uint32_t j2534_ch_take_lost(j2534_channel_t *c);

/* ---- j2534_channel_filter.c ------------------------------------------ */

/** PASS/BLOCK verdict over a [4-byte id][frame] view. No PASS filter at
 *  all passes everything (the CAN channel's rule). */
bool j2534_ch_filters_pass(const j2534_channel_t *c, const uint8_t *view,
                           size_t len);

/** True when at least one PASS filter is armed. */
bool j2534_ch_has_pass_filter(const j2534_channel_t *c);

/* ---- j2534_channel_iso.c -------------------------------------------- */

/** An ISO-TP provider is registered and the CAN node is up. */
bool j2534_iso_provider_ready(void);

/**
 * Open (or, for an rx id the channel already serves, reopen) the ISO-TP
 * session for an ECU. @p ae NULL (or .on false): normal addressing. With
 * extended addressing a session is one rx id AND rx address extension, so
 * several share an rx id; a session of the other addressing kind on the
 * rx id is replaced. J2534 status: NOERROR (*slot = the session index),
 * ERR_EXCEEDED_LIMIT (the channel's or the provider's table is full),
 * ERR_FAILED (no provider, the id held elsewhere, the bus down).
 */
uint32_t j2534_iso_bind(j2534_channel_t *c, uint32_t tx_id, uint32_t rx_id,
                        bool ext, bool pad, const j2534_iso_ae_t *ae,
                        int *slot);

void j2534_iso_unbind(j2534_channel_t *c, int slot);
void j2534_iso_unbind_all(j2534_channel_t *c);

/**
 * WRITE_MSGS on an ISO15765 channel (the J2534-1 layout).
 *
 * @p periodic false, the tester's own write: a TxDone indication is queued
 * on success, and THE CHANNEL LOCK IS GIVEN UP while the message is on the
 * bus (the wait for the conversation gate, then a transfer that can take
 * seconds at an ECU's STmin) and taken again before the return. Until
 * 2026-11-18 it was held throughout: the RX pump waited on it, so nothing
 * received on any channel reached the tester during a long write, what did
 * not fit a channel's queue meanwhile was lost, and a periodic message
 * waited too. That is safe because the task that writes is the session's
 * own, the only one that closes sessions and channels; the one other
 * writer, a periodic message, finds the session marked `tx_busy`.
 *
 * @p periodic true: no indication (SAE J2534-1), the lock stays held (a
 * periodic message is a frame or two), and a message to an ECU whose
 * session is `tx_busy` is not sent: J2534_ERR_BUFFER_FULL, "not now", for
 * the periodic task to try again (no periodic message into a segmented
 * transfer to the same address).
 */
uint32_t j2534_iso_write(j2534_channel_t *c, const j2534_msg_t *msg,
                         bool periodic);

/** One received message or indication, if any (the RX pump's poll). */
bool j2534_iso_poll(j2534_channel_t *c, j2534_msg_t *out);

/**
 * SET_CONFIG (@p get false) or GET_CONFIG on an ISO15765 channel: the list
 * rules are j2534_iso_cfg.h's. A list that names ISO15765_BS or
 * ISO15765_STMIN changes the flow control of every session the channel has
 * open (from the next first frame or block end on) and of the ones opened
 * after. J2534 status; a GET's pairs land in @p out.
 */
uint32_t j2534_iso_config(j2534_channel_t *c, bool get,
                          const uint8_t *body, uint32_t body_len,
                          uint8_t *out, uint32_t out_cap, uint32_t *out_len);

/* ---- j2534_channel_legacy.c (J1850 VPW over the OBD chip) ------------ */

/** The OBD chip is present and finished bring-up (J1850 needs it). */
bool j2534_legacy_provider_ready(void);

/**
 * Bring a J1850 channel up: run the VPW protocol init on the OBD chip
 * (ATTP2, headers on, auto-receive, fixed timing). J2534 status:
 * NOERROR, ERR_FAILED (chip not ready), ERR_NOT_SUPPORTED (a J1850
 * protocol other than VPW, not built yet).
 */
uint32_t j2534_legacy_connect(j2534_channel_t *c);

/**
 * WRITE_MSGS on a J1850 channel: data = [3-byte header][payload]. Sets
 * the chip header (cached), sends the payload (a bare hex line, or
 * VTFullyRequestCk for the chip to segment), parses the reply lines into
 * the slot's RX ring. @p indicate queues a TxDone echo first. The reflash
 * gate applies to the payload's first byte.
 */
uint32_t j2534_legacy_write(j2534_channel_t *c, int slot,
                            const j2534_msg_t *msg, bool indicate);

/** One buffered J1850 message (or TxDone echo) for @p slot, if any. */
bool j2534_legacy_poll(j2534_channel_t *c, int slot, j2534_msg_t *out);

/** Drop every buffered message for @p slot (channel teardown). */
void j2534_legacy_drop(int slot);
