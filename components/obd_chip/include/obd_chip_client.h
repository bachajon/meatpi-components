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
 * @file obd_chip_client.h
 * @brief An app on a bridge and the firmware's own requesters on the one
 *        OBD chip: how the chip changes hands. Part of obd_chip's public
 *        API (obd_chip.h includes it; its own file since 2026-11-25, the
 *        700-line rule).
 *
 * An app (TCP 35000, the web terminal, Bluetooth, USB) writes raw bytes to
 * the chip; AutoPID and the other requesters of the firmware ask through
 * obd_chip_request(). Both cannot talk to the chip at once: the chip stops
 * the request it is working on at the first byte that arrives (`STOPPED`)
 * and runs neither. The rules, each with the date it came:
 *
 * - The poller yields (2026-09-08): it stays off the chip while an app
 *   wrote within the last ten seconds (obd_chip_client_idle_ms).
 * - A monitor holds (2026-11-21, GitHub issue #1078): a monitor the app
 *   opened writes nothing while it runs and counts as activity until it
 *   ends.
 * - A hold ends when the app has gone (2026-11-25): the bridge says which
 *   connection the bytes come from and when that connection has nobody
 *   left (obd_chip_client_gone). The cap of two minutes stays for a
 *   transport that cannot tell and for an app that hangs.
 * - The app's bytes wait for the poller's turn to end (2026-11-25): the
 *   poller takes a turn for every pass (obd_chip_turn_begin), and an app's
 *   first command is no longer lost in the row that was on the wire.
 * - An app is not shown the firmware's own traffic (2026-11-25): a
 *   subscriber that stands for an app (obd_chip_subscribe_app) gets what
 *   the chip says, without the answers to the firmware's own requests.
 *
 * What these rules leave to the app: the chip is in the state the poller's
 * last row left it in (headers on, a header, a receive filter, on some
 * profiles another protocol of the same bus). An app that wants a clean
 * chip sends `ATZ` or `ATD` first, as ELM327 apps do; the reset reaches the
 * chip since the app's bytes wait for the poller's turn.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mark "an external app just wrote to the chip". NOT called by
 * obd_chip_send itself, because autopid's own monitor sends go through
 * that too and would pause autopid against itself. ESP-side pollers
 * (autopid) read obd_chip_client_idle_ms() and yield the chip while an
 * app is driving it (legacy DEV_AUTOPID_ELM327_APP_BIT parity). The
 * bridge glue calls obd_chip_client_write() in its place since
 * 2026-11-21.
 */
void obd_chip_client_touch(void);

/** How long a monitor an app opened counts as activity at most
 *  (obd_chip_client_write): for a transport that cannot tell when its app
 *  went away, and for an app that hangs while it is connected. */
#define OBD_CLIENT_MONITOR_HOLD_MS 120000

/** How long an app's bytes wait for a background turn to end before they
 *  go out anyway (obd_chip_client_write): the poller's pass is a row or a
 *  filter's window, a second at most; a requester that never ends its turn
 *  must not lock the apps out. */
#define OBD_CLIENT_TURN_WAIT_MS 3000

/**
 * The bytes an app is about to write to the chip (2026-11-21, GitHub
 * issue #1078): obd_chip_client_touch(), and the app's monitor. A monitor
 * (ATMA, STMA, ... obd_chip_is_monitor_cmd) writes nothing while it runs,
 * and the poller took the chip back ten seconds into it. A line that is a
 * monitor command opens one; the app's next byte (not LF) or the chip's
 * prompt ends it. The bridge glue calls this ahead of obd_chip_send().
 *
 * Since 2026-11-25 (the same issue):
 * @p who names the connection the bytes come from (the address of the
 * bridge's far endpoint, an identity and nothing else; NULL = not told),
 * for obd_chip_client_gone(). And the call BLOCKS while a background turn
 * is in progress (obd_chip_turn_begin), OBD_CLIENT_TURN_WAIT_MS at most:
 * when it returns, the chip is at its prompt, or the wait was given up
 * (counted, one W line) and the bytes stop the request on the wire as
 * they did before.
 */
void obd_chip_client_write(const void *who, const uint8_t *data, size_t len);

/** The connection @p who has nobody left (2026-11-25): when it is the one
 *  that wrote last, a monitor it opened no longer counts as activity, and
 *  the idle time runs from its last byte again. Any other connection's
 *  leaving changes nothing. */
void obd_chip_client_gone(const void *who);

/** Milliseconds since the last obd_chip_client_touch() or _write();
 *  0 while an app's monitor runs (OBD_CLIENT_MONITOR_HOLD_MS at most);
 *  UINT32_MAX when no external client ever wrote. */
uint32_t obd_chip_client_idle_ms(void);

/**
 * A background requester's turn on the chip (2026-11-25, GitHub issue
 * #1078), for the one that yields to apps: AutoPID's poller, once for
 * every pass. False = an app drives the chip (it wrote within @p yield_ms,
 * or its monitor runs): no turn, the caller stays off the chip. True = the
 * turn is the caller's until obd_chip_turn_end(), and an app's bytes that
 * arrive meanwhile wait for that. One turn at a time, not counted: the
 * caller ends its turn before it waits for anything but the chip.
 */
bool obd_chip_turn_begin(uint32_t yield_ms);
void obd_chip_turn_end(void);

/**
 * obd_chip_subscribe() for a queue that stands for an app on a bridge
 * (2026-11-25, GitHub issue #1078). It gets every chunk but the chip's
 * answers to the firmware's own requests: a chunk that arrives while a
 * claim is held (a request, a transaction, AutoPID's monitor window) is
 * kept from it and counted (`withheld`), unless a command of an app is
 * under way at that moment. Until then an app that stayed connected and
 * silent received the poller's lines.
 */
esp_err_t obd_chip_subscribe_app(QueueHandle_t q, const char *name);

typedef struct
{
    uint32_t turn_waits;       /* app writes that found a turn in progress  */
    uint32_t turn_wait_max_ms; /* the longest of those waits                */
    uint32_t turn_forced;      /* of them: gave up and wrote under the turn */
    uint32_t gone;             /* monitor holds ended by a connection that
                                  has nobody left                           */
} obd_chip_client_stats_t;

/** The counters of the rules above, since boot (`GET /api/obd_chip`
 *  "client"). */
void obd_chip_client_get_stats(obd_chip_client_stats_t *out);

#ifdef __cplusplus
}
#endif
