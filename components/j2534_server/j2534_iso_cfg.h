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
 * What a channel sends until the tool sets it: BS 8 / STmin 1 ms, what every
 * firmware before 2026-11-17 sent whatever the tool set. SAE J2534-1 names 0
 * and 0. The device keeps its old values because BS 0 lets an ECU send a
 * whole message without a pause, and the CAN node's receive queue holds 64
 * frames: a write to the unit's flash holds the receive task longer than
 * that lasts at the 500 kbit/s line rate (measured 2026-11-17: 28 and 30
 * frames lost beyond the queue, and with them one 4095-byte answer, when
 * the configuration backup was written during it; nothing lost in the 238
 * answers after it, nor in 100 with the flash at rest). GET_CONFIG reports
 * the values in force, and a tool that wants the speed sets 0 / 0.
 */
#define J2534_ISO_DEFAULT_BS     8u
#define J2534_ISO_DEFAULT_ST_US  1000u

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
