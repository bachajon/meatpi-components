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

/** @file test_frames.h: the frames the mqtt_can host suite feeds its
 *  codec, shared by test_main.c and test_steps.c. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "can_frame_wire.h"

/* ---- helpers ---------------------------------------------------------------- */

static inline can_core_frame_t mk_frame(uint32_t id, uint8_t dlc, bool ext,
                                        bool rtr)
{
    can_core_frame_t f;

    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    f.ext = ext;
    f.rtr = rtr;

    for (uint8_t i = 0; i < dlc; i++)
    {
        f.data[i] = (uint8_t)(i + 1);
    }

    return f;
}

static inline size_t wire_chunk(const can_core_frame_t *frames, int n,
                                uint8_t *out)
{
    size_t w = 0;

    for (int i = 0; i < n; i++)
    {
        w += can_wire_encode(&frames[i], out + w);
    }

    return w;
}
