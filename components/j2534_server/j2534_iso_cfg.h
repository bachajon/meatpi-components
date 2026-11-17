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
 * @file j2534_iso_cfg.h
 * @brief SET_CONFIG and GET_CONFIG of an ISO15765 channel (pure, host-tested):
 *        ISO15765_BS (0x1E) and ISO15765_STMIN (0x1F), the block size and
 *        the separation time of the flow control the channel sends when an
 *        ECU answers in several frames. The list on the wire is
 *        `u32 count`, then `count` pairs `u32 parameter, u32 value`.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The most pairs one list carries (the ACK's room, as on a TP2.0 channel). */
#define J2534_ISO_CFG_MAX 24

/*
 * What a channel sends until the tool sets it: BS 0 / STmin 0, the defaults
 * SAE J2534-1 names (2026-11-17). Every firmware before sent BS 8 / STmin
 * 1 ms whatever the tool set, so this changes what a tool that sets nothing
 * gets: an ECU's long answer in one go, not eight frames at a time one
 * frame a millisecond (on the bench a 4095-byte answer in 0.16 s, 0.72 s
 * before). GET_CONFIG reports the values in force, and a tool that wants
 * the old pace sets 8 and 1.
 *
 * What BS 0 asks of the unit: an ECU may then send a whole message without
 * a pause, and a write to the unit's flash holds the CAN receive task about
 * 23 ms. With the node's receive queue at 64 frames that cost 28 and 30
 * frames and the 4095-byte answer that was arriving (2026-11-17, the
 * configuration backup written during it), which is why the default was
 * kept at 8 / 1 at first. The queue holds 256 since that day and the same
 * rate with the flash written on purpose loses nothing (the firmware
 * repo's stage j2534fc, leg W); the default followed.
 */
#define J2534_ISO_DEFAULT_BS     0u
#define J2534_ISO_DEFAULT_ST_US  0u

/** The flow control one channel sends as a receiver. */
typedef struct
{
    uint8_t  bs;     /**< frames per block; 0 = the rest of the message   */
    uint32_t st_us;  /**< separation time the sender must keep, microseconds */
} j2534_iso_flow_t;

/**
 * ISO15765_STMIN as SAE J2534-1 and ISO 15765-2 code it, to microseconds:
 * 0x00..0x7F are milliseconds, 0xF1..0xF9 are 100..900 us. Every other
 * value is reserved: false, @p us untouched.
 */
bool j2534_iso_stmin_to_us(uint32_t stmin, uint32_t *us);

/** The way back, for GET_CONFIG (exact for every value the first makes). */
uint32_t j2534_iso_us_to_stmin(uint32_t us);

/**
 * SET_CONFIG on an ISO15765 channel. No body (the id alone, drivers up to
 * 1.8) is NOERROR and changes nothing. A list is checked whole before
 * anything is taken: a count over J2534_ISO_CFG_MAX, a length that is not
 * the count's, ISO15765_BS over 0xFF or a reserved ISO15765_STMIN are
 * ERR_INVALID_IOCTL_VALUE with @p flow untouched. A parameter the channel
 * does not act on (DATA_RATE, LOOPBACK, ...) is passed over, as before.
 * @p changed says whether the list named one of the two.
 */
uint32_t j2534_iso_cfg_set(const uint8_t *body, uint32_t body_len,
                           j2534_iso_flow_t *flow, bool *changed);

/**
 * GET_CONFIG on an ISO15765 channel. No body is NOERROR without an answer.
 * A list is answered with `u32 count` and the pairs of the two parameters
 * it names, their values filled; a parameter the channel does not hold is
 * left out (the driver answers DATA_RATE and the like itself). A malformed
 * list is ERR_INVALID_IOCTL_VALUE, an answer that does not fit @p out_cap
 * ERR_INVALID_MSG.
 */
uint32_t j2534_iso_cfg_get(const uint8_t *body, uint32_t body_len,
                           const j2534_iso_flow_t *flow,
                           uint8_t *out, uint32_t out_cap, uint32_t *out_len);

#ifdef __cplusplus
}
#endif
