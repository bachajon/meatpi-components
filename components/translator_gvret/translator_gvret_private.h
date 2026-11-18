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
 * @file translator_gvret_private.h
 * @brief The one impure edge of the GVRET codec (2026-11-18): the device
 *        clock a TIME_SYNC answer carries. SavvyCAN lines its own clock up
 *        with that answer and reads the timestamps of the frame records
 *        against it, so it has to be the clock those records carry: the
 *        low 32 bits of the microseconds since boot (can_core's ts_us, taken
 *        in the CAN RX interrupt). The codec stays pure and host-tested;
 *        translator_gvret.c points the hook at esp_timer on the target, the
 *        host test at a stub. NULL = the answer carries 0, as it always did
 *        until that day.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t (*gvret_clock_us_hook)(void);

#ifdef __cplusplus
}
#endif
