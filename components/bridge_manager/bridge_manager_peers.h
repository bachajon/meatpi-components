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
 * @file bridge_manager_peers.h
 * @brief Private: the peers at the far end of a bridge
 *        (bridge_manager_peers.c, pure and host-tested).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "bridge_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/** What one bridge remembers between two looks: did end `a` ([0]) and end
 *  `b` ([1]) have peers at the last one. Zeroed when the bridge is built. */
typedef struct
{
    bool had[2];
} bm_peers_t;

/** A raw send to @p dst of bytes that came from @p src: send_from() when
 *  the destination wants to know where they come from, send() otherwise. */
esp_err_t bm_peers_send(const bridge_endpoint_t *dst,
                        const bridge_endpoint_t *src, const uint8_t *data,
                        size_t len);

/** One look at both ends of a bridge (the pump, about ten times a second):
 *  an end that had peers and has none now is reported to the other end
 *  (far_gone), once per time it empties. An end that cannot count its
 *  peers is never reported; an end that does not want to know is not
 *  told. */
void bm_peers_poll(bm_peers_t *st, const bridge_endpoint_t *a,
                   const bridge_endpoint_t *b);

#ifdef __cplusplus
}
#endif
