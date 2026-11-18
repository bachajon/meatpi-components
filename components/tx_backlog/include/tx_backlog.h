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
 * @file tx_backlog.h
 * @brief What a stream client has not taken yet: a byte ring that takes a
 *        unit whole or not at all, a pool that lends the rings their
 *        storage, and (on the device) the socket send that never blocks and
 *        uses both.
 *
 * Born in socket_manager (2026-11-17, a field report: a phone on slcan over
 * TCP lost its connection whenever its ACKs came 150 ms late) and moved here
 * on 2026-11-18, when the WebSocket channels needed the same thing: a client
 * that is behind keeps its connection, what it does not take waits in
 * order, what no longer fits is dropped whole and counted, and only a
 * client that takes nothing at all for a while is let go.
 *
 * Nothing here locks: the owner of a backlog calls every function under its
 * own lock.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- the ring (tx_backlog.c: pure, host-tested) ------------------------------ */

/** What one client has not taken yet, and since when it takes nothing. The
 *  storage is lent (buf NULL = none held). */
typedef struct
{
    uint8_t *buf;         /* lent storage; NULL while the client keeps up   */
    uint32_t cap;
    uint32_t off;         /* first byte still to send                       */
    uint32_t len;         /* bytes waiting, from off around the end (ring)  */
    bool     stalled;     /* the last send attempts moved no byte           */
    int64_t  stalled_us;  /* when that began                                */
    uint32_t dropped;     /* units refused since the owner last zeroed it   */
    int64_t  sent_us;     /* when bytes last left for the socket            */
    int64_t  unit_us;     /* when the last unit came                        */
    uint8_t  run;         /* units in a row that came sooner than the pace  */
} tx_backlog_t;

/** Empty it, stop the stall clock, zero `dropped`. Leaves buf and cap alone. */
void tx_backlog_clear(tx_backlog_t *b);

/** Queue @p len bytes behind what waits. false (nothing stored) when no
 *  storage is held or they do not all fit, so a dropped unit never cuts a
 *  text protocol's line. */
bool tx_backlog_put(tx_backlog_t *b, const uint8_t *data, size_t len);

/** tx_backlog_put() for a unit in two parts (a frame's header and its
 *  payload): both go in, or neither. Either part may be empty. */
bool tx_backlog_put2(tx_backlog_t *b, const uint8_t *head, size_t head_len,
                     const uint8_t *data, size_t len);

/** The first run of waiting bytes (up to the end of the storage): its
 *  length, *out at its first byte. What wraps is the next peek's, after
 *  tx_backlog_took(). 0 and NULL when nothing waits. */
size_t tx_backlog_peek(const tx_backlog_t *b, const uint8_t **out);

/** The client took the first @p n waiting bytes. */
void tx_backlog_took(tx_backlog_t *b, size_t n);

/** One send attempt's result. @p moved: at least one byte left. @p refused:
 *  the socket would not take everything. Nothing moved and refused starts
 *  the stall clock; anything else stops it. */
void tx_backlog_attempt(tx_backlog_t *b, bool moved, bool refused,
                        int64_t now_us);

/** The client moved no byte for @p limit_ms: it is gone. */
bool tx_backlog_expired(const tx_backlog_t *b, int64_t now_us,
                        uint32_t limit_ms);

/* ---- the pace (tx_backlog.c: pure, host-tested) ---------------------------------
 * A stream of small units (a CAN bus as slcan text: a line of 22 bytes,
 * thousands a second) sent unit by unit is a TCP segment and a WiFi frame
 * per unit. On the bench (2026-11-18, a bus at 4,200 frames a second to one
 * slcan client) the TCP/IP task then used a whole CPU core and a plain web
 * request took 0.2 to 2.4 s instead of 25 ms. So a client that gets units
 * faster than one per @p gap_us gets them together: after @p after units
 * in a row that each came sooner than the gap after the unit before it, a
 * new unit waits in the backlog until the gap since the last send is
 * over. Then the next unit takes what has gathered with it, or the
 * owner's next push does when no unit comes. A unit that comes after a
 * pause of the gap or more leaves at once, and so do the next @p after:
 * a request and its answer in a few pieces are never held.
 *
 * The pause is measured from the last UNIT, not from the last send: the
 * pace itself stretches the time between sends, and a first cut that
 * looked at the send let most of a stream through unpaced (700 to 870
 * segments a second on the bench where about 330 were meant). */

/** A new unit at @p now_us: true = it waits (for the next unit that comes
 *  once the gap is over, or for the owner's next push). Notes when the
 *  unit came and counts the run. @p gap_us 0 = no pace. */
bool tx_backlog_pace_hold(tx_backlog_t *b, int64_t now_us, uint32_t gap_us,
                          uint8_t after);

/** A push with no new unit at @p now_us: true = what waits may leave (the
 *  gap since the last send is over, or there is no pace). */
bool tx_backlog_pace_due(const tx_backlog_t *b, int64_t now_us,
                         uint32_t gap_us);

/** The next unit leaves at once whatever came before it, and what waits
 *  goes with it: for a unit that must not wait (a frame that ends a
 *  connection, sent just before the close). */
void tx_backlog_pace_now(tx_backlog_t *b);

/* ---- the pool (tx_backlog.c) ---------------------------------------------------
 * @p count buffers of @p buf_bytes in one block of the owner's (PSRAM, a
 * static array). A buffer is lent when a client first falls behind and goes
 * back when it has caught up, so the pool is the number of clients that may
 * be behind at the same time. */

#define TX_BACKLOG_POOL_MAX 8

typedef struct
{
    uint8_t *mem;         /* count * buf_bytes, the owner's                 */
    uint32_t buf_bytes;
    uint8_t  count;       /* at most TX_BACKLOG_POOL_MAX                    */
    bool     used[TX_BACKLOG_POOL_MAX];
    uint32_t pace_us;     /* the pace of every client of this pool; 0 = none */
    uint8_t  pace_after;  /* units in a row before a stream is held          */
} tx_backlog_pool_t;

/** Give @p b a buffer when it holds none and one is free; nothing otherwise. */
void tx_backlog_lend(tx_backlog_pool_t *p, tx_backlog_t *b);

/** Take @p b 's buffer back (what waited in it is gone with it). Nothing
 *  when it holds none. */
void tx_backlog_return(tx_backlog_pool_t *p, tx_backlog_t *b);

/* ---- the socket send (tx_backlog_sock.c: device only, lwIP) --------------------- */

typedef enum
{
    TX_BACKLOG_SENT = 0, /* on the wire, or waiting in the backlog in order   */
    TX_BACKLOG_DROPPED,  /* the new unit did not fit: dropped WHOLE, counted
                            in dropped; the stream is intact                  */
    TX_BACKLOG_CUT,      /* a part of the new unit was on the wire and the
                            rest found no room (no buffer left in the pool):
                            counted in dropped; the unit is cut. A line
                            protocol resyncs, a framed one (WebSocket) is
                            broken and its owner closes the client            */
    TX_BACKLOG_ERROR,    /* a hard socket error: the owner closes the client  */
} tx_backlog_result_t;

/**
 * Send what waits, then the new unit (@p head, @p data: either may be NULL
 * / 0, both empty = only what waits), never blocking. What the socket does
 * not take waits in the backlog (a buffer is lent from @p pool when
 * needed), a unit the backlog cannot hold is dropped. The buffer goes back
 * to the pool once nothing waits. Updates the stall clock with @p now_us.
 * Sent bytes add to *@p bytes_out (may be NULL).
 *
 * While the client is stalled (its last attempt moved nothing) and something
 * waits, a call WITH a new unit does not touch the socket: the unit queues,
 * or is dropped. Only a call with no new unit asks the socket again, so the
 * owner must make such
 * calls for a client that is behind (on a tick, or when the socket reports
 * writable), or that client never catches up.
 *
 * With a pace in @p pool (pace_us > 0) the same holds for a client whose
 * units are held for it: a call with a new unit may only queue it, and a
 * call with no new unit sends what waits once the gap is over (before
 * that it does nothing). The owner makes those calls on a tick about as
 * long as the gap for as long as something waits. A held unit is not a
 * stall: the stall clock is left alone. With no buffer free in the pool a
 * unit is not held, it leaves at once.
 *
 * @p send_flags goes to send(): 0 for a socket that is non-blocking itself,
 * MSG_DONTWAIT for one that must stay blocking for its other user.
 */
tx_backlog_result_t tx_backlog_sock_push(int fd, int send_flags,
                                         tx_backlog_t *b,
                                         tx_backlog_pool_t *pool,
                                         const uint8_t *head, size_t head_len,
                                         const uint8_t *data, size_t len,
                                         int64_t now_us, uint32_t *bytes_out);

#ifdef __cplusplus
}
#endif
