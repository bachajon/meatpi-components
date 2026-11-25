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
 * @file obd_chip_client_private.h
 * @brief Private: a bridged app beside the firmware's requesters
 *        (obd_chip_client.c, pure but for a spinlock, host-tested). Included
 *        by obd_chip_private.h; its own file since 2026-11-25 (the 700-line
 *        rule).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "obd_chip_client.h" /* obd_chip_client_stats_t */

#ifdef __cplusplus
extern "C" {
#endif

/* (2026-11-21, GitHub issue #1078) the activity clock behind
   obd_chip_client_touch() / _write() / _idle_ms(), with the clock passed
   in: an app's monitor counts as activity until it ends. */
void     obd_client_touch(int64_t now_us);
void     obd_client_rx(const uint8_t *data, size_t len); /* every RX chunk */
uint32_t obd_client_idle_ms(int64_t now_us);

/* (2026-11-25) @p who = the connection the bytes come from, NULL = not
   told. obd_client_sent(): they went out to the chip (behind the wait for
   the poller's turn); a CR in them = a command of the app is under way */
void     obd_client_sent(const uint8_t *data, size_t len);
void     obd_client_write_from(const void *who, const uint8_t *data,
                               size_t len, int64_t now_us);

static inline void obd_client_write(const uint8_t *data, size_t len,
                                    int64_t now_us)
{
    obd_client_write_from(NULL, data, len, now_us);
}

/** The connection @p who has nobody left. True when it wrote last and a
 *  monitor of its was open: that hold is over. */
bool     obd_client_gone(const void *who);

/* the poller's turn: refused while an app drives the chip (its idle time
   below @p yield_ms), under the lock an app's write stamps the clock with */
bool     obd_client_turn_begin(int64_t now_us, uint32_t yield_ms);
void     obd_client_turn_end(void);
bool     obd_client_turn(void);
void     obd_client_turn_waited(uint32_t waited_ms, bool forced); /* stats */

/** Is a chunk kept from the apps? @p claimed = a claim is held (a request
 *  of the firmware's is on the wire). Not while an app's own command is
 *  under way. Asked BEFORE obd_client_rx() sees the chunk: the prompt that
 *  ends an app's command still reaches the app. */
bool     obd_client_mute(bool claimed);
void     obd_client_get_stats(obd_chip_client_stats_t *out);
#ifdef OBD_CHIP_HOST_TEST
void     obd_client_test_reset(void);
#endif

#ifdef __cplusplus
}
#endif
